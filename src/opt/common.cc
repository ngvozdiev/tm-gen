#include "common.h"

#include <ncode/common.h>
#include <ncode/event_queue.h>
#include <ncode/free_list.h>
#include <ncode/htsim/packet.h>
#include <ncode/lp/demand_matrix.h>
#include <ncode/lp/mc_flow.h>
#include <ncode/net/algorithm.h>
#include <ncode/net/net_common.h>
#include <ncode/perfect_hash.h>
#include <ncode/stats.h>
#include <ncode/strutil.h>
#include <ncode/substitute.h>
#include <ncode/viz/graph.h>
#include <stddef.h>
#include <initializer_list>
#include <random>
#include <ratio>
#include <set>
#include <string>
#include <tuple>

#include "path_provider.h"

namespace tm_gen {

constexpr char RoutingConfiguration::kDefaultOptimizerString[];

bool operator<(const AggregateId& a, const AggregateId& b) {
  return std::tie(a.src_, a.dst_) < std::tie(b.src_, b.dst_);
}

bool operator==(const AggregateId& a, const AggregateId& b) {
  return std::tie(a.src_, a.dst_) == std::tie(b.src_, b.dst_);
}

bool operator!=(const AggregateId& a, const AggregateId& b) {
  return std::tie(a.src_, a.dst_) != std::tie(b.src_, b.dst_);
}

std::string AggregateId::ToString(const nc::net::GraphStorage& graph) const {
  std::string src_str = graph.GetNode(src_)->id();
  std::string dst_str = graph.GetNode(dst_)->id();
  return nc::StrCat("<", src_str, ",", dst_str, ">");
}

nc::net::Delay AggregateId::GetSPDelay(
    const nc::net::GraphStorage& graph) const {
  return GetSP(graph)->delay();
}

std::unique_ptr<nc::net::Walk> AggregateId::GetSP(
    const nc::net::GraphStorage& graph) const {
  std::unique_ptr<nc::net::Walk> sp =
      nc::net::ShortestPathWithConstraints(src_, dst_, graph, {});
  CHECK(sp);
  return sp;
}

void TrafficMatrix::AddDemand(const AggregateId& aggregate_id,
                              const DemandAndFlowCount& demand_and_flow_count) {
  CHECK(!nc::ContainsKey(demands_, aggregate_id));
  demands_[aggregate_id] = demand_and_flow_count;
}

TrafficMatrix::TrafficMatrix(
    const nc::lp::DemandMatrix& demand_matrix,
    const std::map<nc::lp::SrcAndDst, size_t>& flow_counts)
    : graph_(demand_matrix.graph()) {
  for (const auto& element : demand_matrix.elements()) {
    size_t flow_count =
        nc::FindWithDefault(flow_counts, {element.src, element.dst}, 1ul);
    demands_[{element.src, element.dst}] = {element.demand, flow_count};
  }
}

std::unique_ptr<TrafficMatrix> TrafficMatrix::ProportionalFromDemandMatrix(
    const nc::lp::DemandMatrix& demand_matrix,
    size_t top_aggregate_flow_count) {
  nc::net::Bandwidth max_demand = nc::net::Bandwidth::Zero();
  for (const auto& element : demand_matrix.elements()) {
    max_demand = std::max(max_demand, element.demand);
  }

  // Will assign flow counts so that the max bandwidth aggregate has
  // kTopAggregateFlowCount, and all other aggregates proportionally less.
  std::map<AggregateId, DemandAndFlowCount> demands_and_counts;
  for (const auto& element : demand_matrix.elements()) {
    size_t flow_count =
        top_aggregate_flow_count * (element.demand / max_demand);
    flow_count = std::max(1ul, flow_count);

    AggregateId id(element.src, element.dst);
    demands_and_counts[id] = {element.demand, flow_count};
  }

  auto out =
      nc::make_unique<TrafficMatrix>(demand_matrix.graph(), demands_and_counts);
  out->properties_ = demand_matrix.properties();
  return out;
}

std::unique_ptr<TrafficMatrix> TrafficMatrix::DistributeFromDemandMatrix(
    const nc::lp::DemandMatrix& demand_matrix, size_t total_flow_count) {
  nc::net::Bandwidth total_demand = nc::net::Bandwidth::Zero();
  for (const auto& element : demand_matrix.elements()) {
    total_demand += element.demand;
  }

  std::map<AggregateId, DemandAndFlowCount> demands_and_counts;
  for (const auto& element : demand_matrix.elements()) {
    double f = element.demand / total_demand;
    size_t flow_count = total_flow_count * f;
    flow_count = std::max(1ul, flow_count);

    AggregateId id(element.src, element.dst);
    demands_and_counts[id] = {element.demand, flow_count};
  }

  auto out =
      nc::make_unique<TrafficMatrix>(demand_matrix.graph(), demands_and_counts);
  out->properties_ = demand_matrix.properties();
  return out;
}

std::unique_ptr<TrafficMatrix> TrafficMatrix::ConstantFromDemandMatrix(
    const nc::lp::DemandMatrix& demand_matrix, size_t common_flow_count) {
  std::map<AggregateId, DemandAndFlowCount> demands_and_counts;
  for (const auto& element : demand_matrix.elements()) {
    size_t flow_count = std::max(1ul, common_flow_count);

    AggregateId id(element.src, element.dst);
    demands_and_counts[id] = {element.demand, flow_count};
  }

  auto out =
      nc::make_unique<TrafficMatrix>(demand_matrix.graph(), demands_and_counts);
  out->properties_ = demand_matrix.properties();
  return out;
}

std::unique_ptr<nc::lp::DemandMatrix> TrafficMatrix::ToDemandMatrix() const {
  std::vector<nc::lp::DemandMatrixElement> elements;
  for (const auto& aggregate_and_demand : demands_) {
    const AggregateId& aggregate = aggregate_and_demand.first;
    nc::net::Bandwidth demand = aggregate_and_demand.second.first;

    elements.emplace_back(aggregate.src(), aggregate.dst(), demand);
  }

  auto out = nc::make_unique<nc::lp::DemandMatrix>(std::move(elements), graph_);
  for (const auto& k_and_v : properties_) {
    out->UpdateProperty(k_and_v.first, k_and_v.second);
  }
  return out;
}

std::pair<nc::net::Bandwidth, nc::net::Bandwidth>
TrafficMatrix::MinMaxAggregates() const {
  nc::net::Bandwidth min_bw = nc::net::Bandwidth::Max();
  nc::net::Bandwidth max_bw = nc::net::Bandwidth::Zero();

  for (const auto& aggregate_and_demand : demands_) {
    const DemandAndFlowCount& demand_and_flow_count =
        aggregate_and_demand.second;
    min_bw = std::min(min_bw, demand_and_flow_count.first);
    max_bw = std::max(max_bw, demand_and_flow_count.first);
  }

  return {min_bw, max_bw};
}

std::unique_ptr<TrafficMatrix> TrafficMatrix::ScaleDemands(
    double factor, const std::set<AggregateId>& to_scale) const {
  std::map<AggregateId, DemandAndFlowCount> new_demands;
  for (const auto& aggregate_and_demand : demands_) {
    const AggregateId& aggregate = aggregate_and_demand.first;
    const DemandAndFlowCount& demand_and_flow_count =
        aggregate_and_demand.second;
    if (!to_scale.empty() && !nc::ContainsKey(to_scale, aggregate)) {
      new_demands[aggregate] = demand_and_flow_count;
      continue;
    }

    new_demands[aggregate] = {demand_and_flow_count.first * factor,
                              demand_and_flow_count.second};
  }

  auto out = nc::make_unique<TrafficMatrix>(graph_, new_demands);
  out->properties_ = properties_;
  return out;
}

std::unique_ptr<TrafficMatrix> TrafficMatrix::AddToDemands(
    nc::net::Bandwidth value, const std::set<AggregateId>& to_extend) const {
  std::map<AggregateId, DemandAndFlowCount> new_demands;
  for (const auto& aggregate_and_demand : demands_) {
    const AggregateId& aggregate = aggregate_and_demand.first;
    const DemandAndFlowCount& demand_and_flow_count =
        aggregate_and_demand.second;
    if (!to_extend.empty() && !nc::ContainsKey(to_extend, aggregate)) {
      new_demands[aggregate] = demand_and_flow_count;
      continue;
    }

    new_demands[aggregate] = {demand_and_flow_count.first + value,
                              demand_and_flow_count.second};
  }

  auto out = nc::make_unique<TrafficMatrix>(graph_, new_demands);
  out->properties_ = properties_;
  return out;
}

static double Pick(double current, double fraction, std::mt19937* rnd) {
  double delta = current * fraction;
  double low = current - delta;
  double high = current + delta;
  std::uniform_real_distribution<double> dist(low, high);
  double new_demand = dist(*rnd);
  return std::max(new_demand, 1.0);
}

std::unique_ptr<TrafficMatrix> TrafficMatrix::Randomize(
    double demand_fraction, double flow_count_fraction, size_t aggregate_count,
    std::mt19937* rnd) const {
  std::vector<AggregateId> all_aggregates;
  for (const auto& aggregate_and_rest : demands_) {
    all_aggregates.emplace_back(aggregate_and_rest.first);
  }

  std::shuffle(all_aggregates.begin(), all_aggregates.end(), *rnd);

  std::map<AggregateId, DemandAndFlowCount> new_demands;
  for (size_t i = 0; i < all_aggregates.size(); ++i) {
    const AggregateId& aggregate = all_aggregates[i];
    const DemandAndFlowCount& demand_and_flow_count =
        nc::FindOrDieNoPrint(demands_, aggregate);

    if (i >= aggregate_count) {
      new_demands[aggregate] = demand_and_flow_count;
      continue;
    }

    nc::net::Bandwidth demand = demand_and_flow_count.first;
    double flow_count = demand_and_flow_count.second;

    nc::net::Bandwidth new_demand = nc::net::Bandwidth::FromBitsPerSecond(
        Pick(demand.bps(), demand_fraction, rnd));

    double new_flow_count = Pick(flow_count, flow_count_fraction, rnd);
    new_demands[aggregate] = {new_demand, new_flow_count};
  }

  auto new_tm = nc::make_unique<TrafficMatrix>(graph_);
  new_tm->demands_ = std::move(new_demands);
  new_tm->properties_ = properties_;
  return new_tm;
}

void RoutingConfiguration::AddRouteAndFraction(
    const AggregateId& aggregate_id,
    const std::vector<RouteAndFraction>& routes_and_fractions) {
  CHECK(!nc::ContainsKey(configuration_, aggregate_id));

  // Just to be on the safe side will check that the sum of all fractions is 1
  double total = 0.0;
  for (const auto& route_and_fraction : routes_and_fractions) {
    CHECK(route_and_fraction.second != 0);
    total += route_and_fraction.second;
  }
  CHECK(total <= 1.001 && total >= 0.999) << "Bad total " << total;
  configuration_[aggregate_id] = routes_and_fractions;
}

const std::vector<RouteAndFraction>& RoutingConfiguration::FindRoutesOrDie(
    const AggregateId& aggregate_id) const {
  return nc::FindOrDieNoPrint(configuration_, aggregate_id);
}

std::unique_ptr<RoutingConfiguration> RoutingConfiguration::ExcludeAggregates(
    const std::set<AggregateId>& aggregates) const {
  std::map<AggregateId, DemandAndFlowCount> new_demands;
  for (const auto& aggregate_and_demand : demands()) {
    if (nc::ContainsKey(aggregates, aggregate_and_demand.first)) {
      continue;
    }
    new_demands.insert(aggregate_and_demand);
  }

  TrafficMatrix new_tm(graph_, new_demands);
  auto new_rc = nc::make_unique<RoutingConfiguration>(new_tm);
  for (const auto& aggregate_and_paths : configuration_) {
    if (nc::ContainsKey(aggregates, aggregate_and_paths.first)) {
      continue;
    }

    new_rc->AddRouteAndFraction(aggregate_and_paths.first,
                                aggregate_and_paths.second);
  }
  return new_rc;
}

std::string RoutingConfiguration::ToString() const {
  std::string base_to_string = TrafficMatrix::ToString();
  std::vector<std::string> out;
  double total = 0;
  for (const auto& aggregate_and_routes : configuration_) {
    const AggregateId& aggregate = aggregate_and_routes.first;
    double aggregate_contribution;
    out.emplace_back(AggregateToString(aggregate, &aggregate_contribution));
    total += aggregate_contribution;
  }

  return nc::StrCat(base_to_string, "\n", nc::Join(out, "\n"), "\ntotal: ",
                    total, "\n");
}

std::string RoutingConfiguration::AggregateToString(
    const AggregateId& aggregate, double* aggregate_contribution) const {
  const std::vector<RouteAndFraction>& routes =
      nc::FindOrDieNoPrint(configuration_, aggregate);
  const DemandAndFlowCount& demand_and_flows =
      nc::FindOrDieNoPrint(demands(), aggregate);

  std::vector<std::string> routes_str;
  double total_contribution = 0;
  for (const RouteAndFraction& route_and_fraction : routes) {
    const nc::net::Walk* path = route_and_fraction.first;
    double fraction = route_and_fraction.second;

    double delay_ms =
        std::chrono::duration<double, std::milli>(path->delay()).count();
    double flow_count = demand_and_flows.second * fraction;
    double contribution = delay_ms * flow_count;
    total_contribution += contribution;

    std::string route_str =
        nc::StrCat(path->ToStringNoPorts(*graph_), " : ", fraction,
                   " (contribution ", contribution, "ms)");
    routes_str.emplace_back(route_str);
  }

  if (aggregate_contribution != nullptr) {
    *aggregate_contribution = total_contribution;
  }

  return nc::StrCat(aggregate.ToString(*graph_), " -> ",
                    nc::Join(routes_str, ","), " (total ", total_contribution,
                    "ms)");
}

std::map<AggregateId, std::vector<size_t>> RoutingConfiguration::GetKValues()
    const {
  std::map<AggregateId, std::vector<size_t>> out;
  size_t i = 0;
  for (const auto& id_and_routes : configuration_) {
    const AggregateId& id = id_and_routes.first;
    std::vector<size_t>& indices = out[id];

    nc::net::KShortestPathsGenerator ksp_gen(id.src(), id.dst(), *graph_, {});
    for (const auto& route : id_and_routes.second) {
      const nc::net::Walk& to_look_for = *route.first;
      indices.emplace_back(ksp_gen.KforPath(to_look_for));
    }

    ++i;
  }

  return out;
}

std::string TrafficMatrix::ToString() const {
  std::vector<std::string> out;
  std::unique_ptr<nc::lp::DemandMatrix> demand_matrix = ToDemandMatrix();
  std::vector<double> sp_utilizations;
  nc::net::GraphLinkMap<double> per_link_utilization =
      demand_matrix->SPUtilization();
  for (const auto& link_and_utilization : per_link_utilization) {
    sp_utilizations.emplace_back(*link_and_utilization.second);
  }
  std::sort(sp_utilizations.begin(), sp_utilizations.end());

  out.emplace_back(
      nc::StrCat("TM with ", static_cast<uint64_t>(demands_.size()),
                 " demands, scale factor ",
                 demand_matrix->MaxCommodityScaleFactor({}, 1.0),
                 " sp link utilizations: ", nc::Join(sp_utilizations, ",")));
  for (const auto& aggregate_and_demand : demands_) {
    const AggregateId& aggregate = aggregate_and_demand.first;
    out.emplace_back(AggregateToString(aggregate));
  }

  return nc::Join(out, "\n");
}

std::string TrafficMatrix::AggregateToString(
    const AggregateId& aggregate) const {
  const DemandAndFlowCount& demand_and_flows_count =
      nc::FindOrDieNoPrint(demands_, aggregate);
  nc::net::Bandwidth demand = demand_and_flows_count.first;
  size_t flow_count = demand_and_flows_count.second;

  return nc::StrCat(aggregate.ToString(*graph_), " -> ", demand.Mbps(), "Mbps ",
                    std::to_string(flow_count), " flows");
}

std::string TrafficMatrix::SummaryToString() const {
  std::string graph_summary = graph_->Stats().ToString();
  size_t aggregate_count = demands_.size();

  auto demand_matrix = ToDemandMatrix();
  double mcsf = demand_matrix->MaxCommodityScaleFactor({}, 1.0);

  std::vector<nc::net::Bandwidth> demands;
  std::vector<size_t> flow_counts;
  for (const auto& aggregate_and_demand : demands_) {
    const DemandAndFlowCount& demand_and_flow_count =
        aggregate_and_demand.second;
    nc::net::Bandwidth demand = demand_and_flow_count.first;
    size_t flow_count = demand_and_flow_count.second;

    demands.emplace_back(demand);
    flow_counts.emplace_back(flow_count);
  }

  std::vector<nc::net::Bandwidth> demands_percentiles =
      nc::Percentiles(&demands);
  std::string demands_str = nc::Substitute(
      "[min: $0 Mbps, 50p: $1 Mbps, 90p: $2 Mbps, max: $3 Mbps]",
      demands_percentiles[0].Mbps(), demands_percentiles[50].Mbps(),
      demands_percentiles[90].Mbps(), demands_percentiles[100].Mbps());

  std::vector<size_t> flows_percentiles = nc::Percentiles(&flow_counts);
  std::string flow_counts_str = nc::Substitute(
      "[min: $0, 50p: $1, 90p: $2, max: $3]", flows_percentiles[0],
      flows_percentiles[50], flows_percentiles[90], flows_percentiles[100]);

  double fraction_of_all = static_cast<double>(aggregate_count) /
                           (graph_->NodeCount() * (graph_->NodeCount() - 1));
  return nc::Substitute(
      "$0\nnumber of aggregates: $1 ($2%), scale factor: $3\naggregates: "
      "$4\nflow counts: $5\n",
      graph_summary, aggregate_count, fraction_of_all * 100, mcsf, demands_str,
      flow_counts_str);
}

const nc::net::Walk* PickPath(const std::vector<RouteAndFraction>& routes,
                              double init_p) {
  double p = init_p;
  for (const auto& route : routes) {
    double f = route.second;
    if (p < f) {
      return route.first;
    }

    p -= f;
  }

  LOG(FATAL) << "Should not happen";
  return nullptr;
}

nc::net::Delay RoutingConfiguration::TotalPerFlowDelay(bool sp) const {
  double total = 0;
  for (const auto& aggregate_id_and_routes : configuration_) {
    const AggregateId& aggregate_id = aggregate_id_and_routes.first;
    const DemandAndFlowCount& demand_and_flow_count =
        nc::FindOrDieNoPrint(demands(), aggregate_id);

    if (sp) {
      nc::net::Delay sp_delay = aggregate_id.GetSPDelay(*graph_);
      total += sp_delay.count() * demand_and_flow_count.second;
      continue;
    }

    for (const RouteAndFraction& route_and_fraction :
         aggregate_id_and_routes.second) {
      const nc::net::Walk* path = route_and_fraction.first;
      nc::net::Delay path_delay = path->delay();
      double fraction = route_and_fraction.second;
      double num_flows = demand_and_flow_count.second * fraction;

      total += path_delay.count() * num_flows;
    }
  }

  return nc::net::Delay(static_cast<size_t>(total));
}

nc::net::GraphLinkMap<double> RoutingConfiguration::LinkUtilizations() const {
  // A map from a link to the total load over the link.
  std::map<nc::net::GraphLinkIndex, nc::net::Bandwidth> link_to_total_load;
  for (const auto& aggregate_and_aggregate_output : configuration_) {
    const AggregateId& aggregate_id = aggregate_and_aggregate_output.first;
    const std::vector<RouteAndFraction>& routes =
        aggregate_and_aggregate_output.second;
    const DemandAndFlowCount& demand_and_flow_count =
        nc::FindOrDieNoPrint(demands(), aggregate_id);
    nc::net::Bandwidth total_aggregate_demand = demand_and_flow_count.first;

    for (const auto& route : routes) {
      const nc::net::Walk* path = route.first;
      double fraction = route.second;
      CHECK(fraction > 0);

      for (nc::net::GraphLinkIndex link : path->links()) {
        link_to_total_load[link] += total_aggregate_demand * fraction;
      }
    }
  }

  nc::net::GraphLinkMap<double> out;
  for (const auto& link_and_total_load : link_to_total_load) {
    nc::net::GraphLinkIndex link_index = link_and_total_load.first;
    const nc::net::GraphLink* link = graph_->GetLink(link_index);

    nc::net::Bandwidth total_load = link_and_total_load.second;
    double utilization = total_load / link->bandwidth();
    out[link_index] = utilization;
  }

  for (nc::net::GraphLinkIndex link : graph_->AllLinks()) {
    if (out.HasValue(link)) {
      continue;
    }

    out[link] = 0;
  }

  return out;
}

std::vector<std::pair<double, AggregateId>>
RoutingConfiguration::OverloadedAggregates() const {
  std::map<nc::net::GraphLinkIndex, nc::net::Bandwidth> link_to_total_load;
  nc::net::GraphLinkMap<std::set<AggregateId>> link_to_aggregate;
  for (const auto& aggregate_and_aggregate_output : configuration_) {
    const AggregateId& aggregate_id = aggregate_and_aggregate_output.first;
    const std::vector<RouteAndFraction>& routes =
        aggregate_and_aggregate_output.second;
    const DemandAndFlowCount& demand_and_flow_count =
        nc::FindOrDieNoPrint(demands(), aggregate_id);
    nc::net::Bandwidth total_aggregate_demand = demand_and_flow_count.first;

    for (const auto& route : routes) {
      const nc::net::Walk* path = route.first;
      double fraction = route.second;
      CHECK(fraction > 0);

      for (nc::net::GraphLinkIndex link : path->links()) {
        link_to_total_load[link] += total_aggregate_demand * fraction;
        link_to_aggregate[link].emplace(aggregate_id);
      }
    }
  }

  std::map<AggregateId, double> overloaded_aggregates;
  for (const auto& link_and_total_load : link_to_total_load) {
    nc::net::GraphLinkIndex link_index = link_and_total_load.first;
    const nc::net::GraphLink* link = graph_->GetLink(link_index);

    nc::net::Bandwidth total_load = link_and_total_load.second;
    double utilization = total_load / link->bandwidth();
    if (utilization > 1) {
      const std::set<AggregateId>& aggregates_crossing_link =
          link_to_aggregate.GetValueOrDie(link_index);
      for (AggregateId id : aggregates_crossing_link) {
        double& v = overloaded_aggregates[id];
        v = std::max(utilization, v);
      }
    }
  }

  std::vector<std::pair<double, AggregateId>> aggregates_by_overload;
  for (const auto& aggregate_and_overlad : overloaded_aggregates) {
    aggregates_by_overload.emplace_back(aggregate_and_overlad.second,
                                        aggregate_and_overlad.first);
  }

  std::sort(aggregates_by_overload.begin(), aggregates_by_overload.end());
  return aggregates_by_overload;
}

double RoutingConfiguration::MaxLinkUtilization() const {
  nc::net::GraphLinkMap<double> link_utilizations = LinkUtilizations();
  double max_utilization = 0;
  for (const auto& link_and_utilization : link_utilizations) {
    max_utilization = std::max(max_utilization, *link_and_utilization.second);
  }
  return max_utilization;
}

std::unique_ptr<RoutingConfiguration> RoutingConfiguration::Copy() const {
  auto out = nc::make_unique<RoutingConfiguration>(
      *static_cast<const TrafficMatrix*>(this));
  out->configuration_ = configuration_;
  out->time_to_compute_ = time_to_compute_;
  out->optimizer_string_ = optimizer_string_;
  return out;
}

static std::vector<nc::net::GraphNodeIndex> NodesFromPath(
    const nc::net::Walk& path, const nc::net::GraphStorage& graph) {
  std::vector<nc::net::GraphNodeIndex> out;
  for (nc::net::GraphLinkIndex link : path.links()) {
    nc::net::GraphNodeIndex node = graph.GetLink(link)->src();
    out.emplace_back(node);
  }

  nc::net::GraphLinkIndex last_link = path.links().back();
  nc::net::GraphNodeIndex last_node = graph.GetLink(last_link)->dst();
  out.emplace_back(last_node);
  return out;
}

std::string RoutingConfiguration::SerializeToText(
    const std::vector<std::string>& node_order) const {
  std::map<std::string, uint32_t> indices;
  for (size_t i = 0; i < node_order.size(); ++i) {
    indices[node_order[i]] = i;
  }

  std::string out;
  nc::StrAppend(&out, optimizer_string_, ",", time_to_compute_.count(), "\n");
  for (const auto& aggregate_and_paths : configuration_) {
    for (const auto& path_and_fraction : aggregate_and_paths.second) {
      const nc::net::Walk* path = path_and_fraction.first;
      double fraction = path_and_fraction.second;

      std::vector<nc::net::GraphNodeIndex> nodes =
          NodesFromPath(*path, *graph_);

      std::vector<std::string> strings;
      for (nc::net::GraphNodeIndex node : nodes) {
        const std::string& node_id = graph_->GetNode(node)->id();
        strings.emplace_back(nc::StrCat(nc::FindOrDie(indices, node_id)));
      }

      strings.emplace_back(nc::StrCat(fraction));
      nc::StrAppend(&out, nc::Join(strings, ","), "\n");
    }
  }

  return out;
}

std::unique_ptr<RoutingConfiguration>
RoutingConfiguration::LoadFromSerializedText(
    const TrafficMatrix& base_matrix,
    const std::vector<std::string>& node_order, const std::string& text,
    PathProvider* path_provider) {
  std::vector<std::string> lines = nc::Split(text, "\n");
  CHECK(!lines.empty());

  std::vector<std::string> first_line = nc::Split(lines[0], ",");
  CHECK(first_line.size() == 2);

  std::string optimizer_string = first_line[0];
  uint32_t time_to_compute;
  CHECK(nc::safe_strtou32(first_line[1], &time_to_compute));

  std::map<AggregateId, std::vector<RouteAndFraction>> routes;
  const nc::net::GraphStorage* graph = base_matrix.graph();

  for (size_t i = 1; i < lines.size(); ++i) {
    const std::string& line = lines[i];
    std::vector<std::string> line_split = nc::Split(line, ",");
    CHECK(line_split.size() >= 3);

    double fraction;
    CHECK(nc::safe_strtod(line_split.back(), &fraction));

    std::vector<nc::net::GraphNodeIndex> nodes_on_path;
    for (size_t j = 0; j < line_split.size() - 1; ++j) {
      uint32_t node_index;
      CHECK(nc::safe_strtou32(line_split[j], &node_index));
      CHECK(node_index < node_order.size());

      const std::string& node_id = node_order[node_index];
      nc::net::GraphNodeIndex node = graph->NodeFromStringOrDie(node_id);
      nodes_on_path.emplace_back(node);
    }
    CHECK(nodes_on_path.size() >= 2);

    nc::net::Links links_on_path;
    for (size_t j = 0; j < nodes_on_path.size() - 1; ++j) {
      auto link = graph->LinkOrDie(nodes_on_path[j], nodes_on_path[j + 1]);
      links_on_path.emplace_back(link);
    }

    auto new_path = nc::make_unique<nc::net::Walk>(links_on_path, *graph);
    const nc::net::Walk* path_ptr =
        path_provider->TakeOwnership(std::move(new_path));

    AggregateId id(nodes_on_path.front(), nodes_on_path.back());
    CHECK(nc::ContainsKey(base_matrix.demands(), id));
    routes[id].emplace_back(path_ptr, fraction);
  }

  auto out = nc::make_unique<RoutingConfiguration>(base_matrix);
  out->time_to_compute_ = std::chrono::milliseconds(time_to_compute);
  out->optimizer_string_ = optimizer_string;

  for (const auto& aggregate_id_and_routes : routes) {
    const AggregateId& id = aggregate_id_and_routes.first;
    out->AddRouteAndFraction(id, aggregate_id_and_routes.second);
  }

  return out;
}

}  // namespace tm_gen
