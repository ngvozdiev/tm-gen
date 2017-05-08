#include "tm_gen.h"

#include <stdint.h>
#include <algorithm>
#include <functional>
#include <iostream>
#include <set>
#include <string>
#include <tuple>

#include "ncode-src/src/common/file.h"
#include "ncode-src/src/common/logging.h"
#include "ncode-src/src/common/map_util.h"
#include "ncode-src/src/common/perfect_hash.h"
#include "ncode-src/src/common/strutil.h"
#include "ncode-src/src/common/substitute.h"
#include "ncode-src/src/grapher/grapher.h"
#include "ncode-src/src/lp/lp.h"
#include "ncode-src/src/lp/mc_flow.h"
#include "ncode-src/src/net/algorithm.h"
#include "ncode-src/src/net/path_cache.h"
#include "ncode-src/src/web/web_page.h"
#include "fubar/b4.h"
#include "fubar/fubar.h"
#include "fubar/fubar_common.h"
#include "fubar/minmax.h"
#include "fubar/sp.h"
#include "tldr/tldr_base.h"

namespace e2e {

using namespace std::chrono;

constexpr double TrafficMatrix::kDefaultB4FairShare;
constexpr double TrafficMatrix::kDefaultFUBARConstant;
constexpr double TrafficMatrix::kDefaultCapacityMultiplier;
constexpr double TrafficMatrix::kDefaultMinMaxRadiusFromSP;

ncode::net::GraphLinkMap<double> TrafficMatrix::SPUtilization() const {
  using namespace ncode::net;

  GraphLinkSet all_links = graph_storage_->AllLinks();
  GraphLinkMap<double> out;

  DirectedGraph directed_graph(graph_storage_);
  AllPairShortestPath sp({}, &directed_graph);
  for (const TrafficMatrixElement& element : elements_) {
    LinkSequence shortest_path = sp.GetPath(element.src, element.dst);
    for (GraphLinkIndex link : shortest_path.links()) {
      out[link] += element.load.Mbps();
    }
  }

  for (GraphLinkIndex link : all_links) {
    double capacity = graph_storage_->GetLink(link)->bandwidth().Mbps();
    double& link_utilization = out[link];
    link_utilization = link_utilization / capacity;
  }

  return out;
}

ncode::net::Bandwidth TrafficMatrix::TotalLoad() const {
  double load_mbps = 0;
  for (const TrafficMatrixElement& element : elements_) {
    load_mbps += element.load.Mbps();
  }

  return ncode::net::Bandwidth::FromMBitsPerSecond(load_mbps);
}

ncode::net::Bandwidth TrafficMatrix::Load(const NodePair& node_pair) const {
  for (const TrafficMatrixElement& element : elements_) {
    if (element.src == node_pair.first && element.dst == node_pair.second) {
      return element.load;
    }
  }

  return ncode::net::Bandwidth::FromMBitsPerSecond(0);
}

std::map<TrafficMatrix::NodePair, std::pair<size_t, ncode::net::Delay>>
TrafficMatrix::SPStats() const {
  using namespace ncode::net;

  std::map<TrafficMatrix::NodePair, std::pair<size_t, ncode::net::Delay>> out;
  DirectedGraph directed_graph(graph_storage_);
  AllPairShortestPath sp({}, &directed_graph);
  for (const TrafficMatrixElement& element : elements_) {
    LinkSequence shortest_path = sp.GetPath(element.src, element.dst);
    out[{element.src, element.dst}] = {shortest_path.size(),
                                       shortest_path.delay()};
  }

  return out;
}

double TrafficMatrix::SPGlobalUtilization() const {
  ncode::net::GraphLinkMap<double> sp_utilizaiton = SPUtilization();
  double total_load = 0;
  double total_capacity = 0;
  for (const auto& link_index_and_utilization : sp_utilizaiton) {
    ncode::net::GraphLinkIndex link_index = link_index_and_utilization.first;
    double utilization = *link_index_and_utilization.second;
    double capacity = graph_storage_->GetLink(link_index)->bandwidth().Mbps();
    double load = utilization * capacity;

    total_load += load;
    total_capacity += capacity;
  }

  return total_load / total_capacity;
}

static tldr::AggregateHistory GetHistory(ncode::net::Bandwidth mean,
                                         double spread, size_t bin_count,
                                         size_t flow_count,
                                         milliseconds bin_size,
                                         std::mt19937* gen) {
  double bytes_per_second = mean.bps() / 8.0;
  double bytes_per_bin = bytes_per_second * (bin_size.count() / 1000.0);
  uint64_t min = bytes_per_bin * (1 - spread);
  uint64_t max = bytes_per_bin * (1 + spread);

  std::vector<uint64_t> bins;
  std::uniform_int_distribution<uint64_t> dist(min, max);
  for (size_t i = 0; i < bin_count; ++i) {
    bins.emplace_back(dist(*gen));
  }

  return {bins, bin_size, flow_count};
}

fubar::Input TrafficMatrix::ToInput(size_t max_flow_count,
                                    size_t min_flow_count, size_t bin_count,
                                    double spread,
                                    std::chrono::milliseconds bin_size) const {
  double max_volume = 0;
  for (const TrafficMatrixElement& element : elements_) {
    double volume = element.load.Mbps();
    if (volume > max_volume) {
      max_volume = volume;
    }
  }

  // Provides randomness for the spread.
  std::mt19937 rnd(1);

  size_t init_cookie = 0;
  std::map<uint64_t, fubar::AggregateInput> cookie_to_input;
  for (const TrafficMatrixElement& element : elements_) {
    double volume = element.load.Mbps();
    double fraction_of_max = volume / max_volume;
    uint64_t flow_count = std::max(static_cast<double>(min_flow_count),
                                   max_flow_count * fraction_of_max);

    uint64_t cookie = ++init_cookie;
    tldr::AggregateHistory history =
        GetHistory(element.load, spread, bin_count, flow_count, bin_size, &rnd);
    fubar::AggregateId id = {cookie, element.src, element.dst};

    cookie_to_input.emplace(
        std::piecewise_construct, std::forward_as_tuple(cookie),
        std::forward_as_tuple(id, flow_count, element.load));
  }

  return fubar::Input(kDefaultCapacityMultiplier, std::move(cookie_to_input));
}

std::unique_ptr<TrafficMatrix> TrafficMatrix::Scale(double factor) const {
  auto tm = ncode::make_unique<TrafficMatrix>(elements_, graph_storage_);
  for (auto& element : tm->elements_) {
    double new_load = element.load.Mbps() * factor;
    element.load = ncode::net::Bandwidth::FromMBitsPerSecond(new_load);
  }

  return tm;
}

std::unique_ptr<TrafficMatrix> TrafficMatrix::IsolateLargest() const {
  ncode::net::Bandwidth max_rate = ncode::net::Bandwidth::Zero();
  const TrafficMatrixElement* element_ptr = nullptr;
  for (const TrafficMatrixElement& element : elements_) {
    if (element.load > max_rate) {
      element_ptr = &element;
      max_rate = element.load;
    }
  }
  CHECK(element_ptr != nullptr);

  std::vector<TrafficMatrixElement> new_elements = {*element_ptr};
  return ncode::make_unique<TrafficMatrix>(new_elements, graph_storage_);
}

std::pair<ncode::net::Bandwidth, double> TrafficMatrix::GetMaxFlow(
    const ncode::net::GraphLinkSet& to_exclude) const {
  ncode::lp::MaxFlowMCProblem max_flow_problem(to_exclude, graph_storage_,
                                               kDefaultCapacityMultiplier);
  for (const TrafficMatrixElement& element : elements_) {
    max_flow_problem.AddCommodity(element.src, element.dst, element.load);
  }

  ncode::net::Bandwidth max_flow = ncode::net::Bandwidth::Zero();
  max_flow_problem.GetMaxFlow(&max_flow);
  return std::make_pair(max_flow, max_flow_problem.MaxCommodityScaleFactor());
}

bool TrafficMatrix::IsFeasible(
    const ncode::net::GraphLinkSet& to_exclude) const {
  ncode::lp::MaxFlowMCProblem max_flow_problem(to_exclude, graph_storage_,
                                               kDefaultCapacityMultiplier);
  for (const TrafficMatrixElement& element : elements_) {
    max_flow_problem.AddCommodity(element.src, element.dst, element.load);
  }

  return max_flow_problem.IsFeasible();
}

double TrafficMatrix::MaxCommodityScaleFractor() const {
  ncode::lp::MaxFlowMCProblem max_flow_problem({}, graph_storage_,
                                               kDefaultCapacityMultiplier);
  for (const TrafficMatrixElement& element : elements_) {
    max_flow_problem.AddCommodity(element.src, element.dst, element.load);
  }

  return max_flow_problem.MaxCommodityScaleFactor();
}

bool TrafficMatrix::ResilientToFailures() const {
  for (ncode::net::GraphLinkIndex link : graph_storage_->AllLinks()) {
    if (!IsFeasible({link})) {
      return false;
    }
  }

  return true;
}

void TMGenerator::AddUtilizationConstraint(double fraction,
                                           double utilization) {
  CHECK(utilization >= 0.0);
  CHECK(fraction >= 0.0);
  utilization_constraints_.emplace_back(fraction, utilization);

  // Will sort the constraints so that the highest ones come first.
  std::sort(utilization_constraints_.begin(), utilization_constraints_.end(),
            std::greater<FractionAndUtilization>());

  // Both fractions and utilization should decrease.
  for (size_t i = 0; i < utilization_constraints_.size() - 1; ++i) {
    CHECK(utilization_constraints_[i].second >=
          utilization_constraints_[i + 1].second);
  }
}

void TMGenerator::AddHopCountLocalityConstraint(double fraction,
                                                size_t hop_count) {
  CHECK(hop_count > 0);
  CHECK(fraction >= 0.0);
  locality_hop_constraints_.emplace_back(fraction, hop_count);
}

void TMGenerator::AddDistanceLocalityConstraint(
    double fraction, std::chrono::milliseconds distance) {
  CHECK(distance > std::chrono::milliseconds::zero());
  CHECK(fraction >= 0.0);
  locality_delay_constraints_.emplace_back(fraction, distance);
}

void TMGenerator::AddOutgoingFractionConstraint(double fraction,
                                                double out_fraction) {
  CHECK(fraction >= 0);
  CHECK(out_fraction >= 0);
  outgoing_fraction_constraints_.emplace_back(fraction, out_fraction);
  std::sort(outgoing_fraction_constraints_.begin(),
            outgoing_fraction_constraints_.end(),
            std::greater<FractionAndOutgoingFraction>());

  for (size_t i = 0; i < outgoing_fraction_constraints_.size() - 1; ++i) {
    CHECK(outgoing_fraction_constraints_[i].second >=
          outgoing_fraction_constraints_[i + 1].second);
  }
}

static void AddLocalityConstraint(
    const std::vector<std::vector<ncode::lp::VariableIndex>>& vars,
    const std::vector<ncode::lp::VariableIndex>& all_variables, double fraction,
    size_t limit, ncode::lp::Problem* problem,
    std::vector<ncode::lp::ProblemMatrixElement>* problem_matrix) {
  // The constraint says that all paths of 'limit' or more need to receive
  // 'fraction' of the total load.
  std::set<ncode::lp::VariableIndex> affected_vars;
  for (size_t i = limit - 1; i < vars.size(); ++i) {
    affected_vars.insert(vars[i].begin(), vars[i].end());
  }

  ncode::lp::ConstraintIndex sum_constraint = problem->AddConstraint();
  problem->SetConstraintRange(sum_constraint, 0,
                              ncode::lp::Problem::kInifinity);
  for (ncode::lp::VariableIndex ie_variable : all_variables) {
    bool affected = ncode::ContainsKey(affected_vars, ie_variable);
    double coefficient = affected ? 1 - fraction : -fraction;
    problem_matrix->emplace_back(sum_constraint, ie_variable, coefficient);
  }
}

std::unique_ptr<TrafficMatrix> TMGenerator::GenerateMatrix(bool explore_alt,
                                                           double scale) {
  size_t tries = explore_alt ? kMaxTries : 1;

  double max_gu = 0;
  std::unique_ptr<TrafficMatrix> best_matrix;
  for (size_t i = 0; i < tries; ++i) {
    auto matrix = GenerateMatrixPrivate();
    if (!matrix) {
      LOG(ERROR) << "Not feasible";
      continue;
    }

    matrix = matrix->Scale(scale);
    if (!matrix->IsFeasible({})) {
      LOG(ERROR) << "Not feasible";
      continue;
    }

    double scale_factor = matrix->MaxCommodityScaleFractor();
    CHECK(scale_factor < 10.0);
    if (scale_factor < min_scale_factor_) {
      LOG(ERROR) << "Scale factor too low";
      continue;
    }

    double global_utilization = matrix->SPGlobalUtilization();
    LOG(ERROR) << "GU " << global_utilization << " aggregates "
               << matrix->elements().size() << " mcsf " << scale_factor;
    if (global_utilization > max_gu) {
      max_gu = global_utilization;
      best_matrix = std::move(matrix);
    }
  }

  return best_matrix;
}

std::unique_ptr<TrafficMatrix> TMGenerator::GenerateMatrixPrivate() {
  using namespace ncode::net;
  using namespace std::chrono;

  DirectedGraph directed_graph(graph_);
  GraphLinkSet all_links = graph_->AllLinks();
  ncode::lp::Problem problem(ncode::lp::MAXIMIZE);
  std::vector<ncode::lp::ProblemMatrixElement> problem_matrix;

  // First need to create a variable for each of the N^2 possible pairs, will
  // also compute for each link the pairs on whose shortest path the link is.
  // Will also group pairs based on hop count.
  GraphNodeSet all_nodes = graph_->AllNodes();
  std::map<std::pair<GraphNodeIndex, GraphNodeIndex>, ncode::lp::VariableIndex>
      ie_pair_to_variable;

  // The n-th element in this vector contains the variables for all pairs whose
  // shortest paths have length of n + 1 milliseconds.
  std::vector<std::vector<ncode::lp::VariableIndex>> by_ms_count;

  // The n-th element in this vector contains the variables for all pairs whose
  // shortest paths have length of n + 1 hops.
  std::vector<std::vector<ncode::lp::VariableIndex>> by_hop_count;

  // All variables.
  std::vector<ncode::lp::VariableIndex> ordered_variables;

  // For each ie-pair (variable) the total load that the source can emit. This
  // is the sum of the capacities of all links going out of the source. This is
  // redundant -- the value will be the same for all src, but it is convenient.
  std::map<ncode::lp::VariableIndex, double> variable_to_total_out_capacity;

  GraphLinkMap<std::vector<ncode::lp::VariableIndex>> link_to_variables;
  for (GraphNodeIndex src : all_nodes) {
    double total_out = 0;
    for (GraphLinkIndex link : directed_graph.AdjacencyList()[src]) {
      total_out += graph_->GetLink(link)->bandwidth().Mbps();
    }

    ShortestPath sp({}, src, &directed_graph);
    for (GraphNodeIndex dst : all_nodes) {
      if (src != dst) {
        ncode::lp::VariableIndex new_variable = problem.AddVariable();
        problem.SetVariableRange(new_variable, 0,
                                 ncode::lp::Problem::kInifinity);
        ie_pair_to_variable[{src, dst}] = new_variable;
        LinkSequence shortest_path = sp.GetPath(dst);

        size_t ms_count =
            duration_cast<milliseconds>(shortest_path.delay()).count() + 1;
        by_ms_count.resize(std::max(by_ms_count.size(), ms_count));
        by_ms_count[ms_count - 1].emplace_back(new_variable);

        size_t hop_count = shortest_path.size();
        CHECK(hop_count > 0);
        by_hop_count.resize(std::max(by_hop_count.size(), hop_count));
        by_hop_count[hop_count - 1].emplace_back(new_variable);

        variable_to_total_out_capacity[new_variable] = total_out;
        ordered_variables.emplace_back(new_variable);
        for (GraphLinkIndex link : shortest_path.links()) {
          link_to_variables[link].emplace_back(new_variable);
        }
      }
    }
  }

  std::vector<GraphLinkIndex> ordered_links;
  for (GraphLinkIndex link_index : all_links) {
    ordered_links.emplace_back(link_index);
  }

  std::shuffle(ordered_links.begin(), ordered_links.end(), rnd_);
  std::shuffle(ordered_variables.begin(), ordered_variables.end(), rnd_);

  for (const FractionAndUtilization& utilization_constraint :
       utilization_constraints_) {
    double fraction = utilization_constraint.first;
    size_t index_to = fraction * ordered_links.size();

    for (size_t i = 0; i < index_to; ++i) {
      GraphLinkIndex link_index = ordered_links[i];
      ncode::lp::ConstraintIndex set_constraint = problem.AddConstraint();
      Bandwidth link_capacity = graph_->GetLink(link_index)->bandwidth();
      problem.SetConstraintRange(
          set_constraint, 0,
          utilization_constraint.second * link_capacity.Mbps());

      for (ncode::lp::VariableIndex ie_variable :
           link_to_variables[link_index]) {
        problem_matrix.emplace_back(set_constraint, ie_variable, 1.0);
      }
    }
  }

  for (const auto& node_pair_and_variable : ie_pair_to_variable) {
    ncode::lp::VariableIndex variable = node_pair_and_variable.second;
    problem.SetObjectiveCoefficient(variable, 1.0);
  }

  for (const FractionAndHopCount& locality_constraint :
       locality_hop_constraints_) {
    double fraction = locality_constraint.first;
    size_t hop_count = locality_constraint.second;
    AddLocalityConstraint(by_hop_count, ordered_variables, fraction, hop_count,
                          &problem, &problem_matrix);
  }

  for (const FractionAndDistance& locality_constraint :
       locality_delay_constraints_) {
    double fraction = locality_constraint.first;
    std::chrono::milliseconds distance = locality_constraint.second;
    AddLocalityConstraint(by_ms_count, ordered_variables, fraction,
                          distance.count(), &problem, &problem_matrix);
  }

  // To enforce the global utilization constraint will have to collect for all
  // links the total load they see from all variables.
  double total_capacity = 0;
  std::map<ncode::lp::VariableIndex, double> variable_to_coefficient;
  for (ncode::net::GraphLinkIndex link_index : graph_->AllLinks()) {
    Bandwidth link_capacity = graph_->GetLink(link_index)->bandwidth();
    total_capacity += link_capacity.Mbps();
    for (ncode::lp::VariableIndex ie_variable : link_to_variables[link_index]) {
      variable_to_coefficient[ie_variable] += 1.0;
    }
  }

  ncode::lp::ConstraintIndex gu_constraint = problem.AddConstraint();
  problem.SetConstraintRange(gu_constraint, 0,
                             max_global_utilization_ * total_capacity);
  for (const auto& variable_and_coefficient : variable_to_coefficient) {
    ncode::lp::VariableIndex var = variable_and_coefficient.first;
    double coefficient = variable_and_coefficient.second;
    problem_matrix.emplace_back(gu_constraint, var, coefficient);
  }

  for (const FractionAndOutgoingFraction& out_fraction_constraint :
       outgoing_fraction_constraints_) {
    double fraction = out_fraction_constraint.first;
    size_t index_to = fraction * ordered_variables.size();

    for (size_t i = 0; i < index_to; ++i) {
      ncode::lp::VariableIndex variable = ordered_variables[i];
      ncode::lp::ConstraintIndex out_constraint = problem.AddConstraint();

      double out_from_variable = variable_to_total_out_capacity[variable];
      double limit = out_fraction_constraint.second;

      problem.SetConstraintRange(out_constraint, 0, limit * out_from_variable);
      problem_matrix.emplace_back(out_constraint, variable, 1);
    }
  }

  problem.SetMatrix(problem_matrix);
  //  problem.DumpToFile("out.lp");
  std::unique_ptr<ncode::lp::Solution> solution = problem.Solve();
  //  LOG(ERROR) << "ST " << solution->type();
  if (solution->type() == ncode::lp::INFEASIBLE_OR_UNBOUNDED) {
    return {};
  }

  //  LOG(ERROR) << "V " << solution->ObjectiveValue();

  std::vector<TrafficMatrixElement> out;
  for (const auto& node_pair_and_variable : ie_pair_to_variable) {
    const std::pair<GraphNodeIndex, GraphNodeIndex>& ie_pair =
        node_pair_and_variable.first;
    ncode::lp::VariableIndex variable = node_pair_and_variable.second;
    double value = solution->VariableValue(variable);
    if (value > 0) {
      //      LOG(ERROR) << graph_->GetNode(ie_pair.first)->id() << " -> "
      //                 << graph_->GetNode(ie_pair.second)->id() << " v " <<
      //                 value;
      out.emplace_back(ie_pair.first, ie_pair.second,
                       ncode::net::Bandwidth::FromMBitsPerSecond(value));
    }
  }

  return ncode::make_unique<TrafficMatrix>(std::move(out), graph_);
}

void TMGenerator::SetMaxGlobalUtilization(double fraction) {
  CHECK(fraction > 0.0);
  CHECK(fraction < 1.0);
  max_global_utilization_ = fraction;
}

void TMGenerator::SetMinScaleFactor(double factor) {
  CHECK(factor >= 1.0);
  min_scale_factor_ = factor;
}

struct PerOptimizerSummary {
  PerOptimizerSummary(uint64_t processing_time_ms,
                      uint64_t cached_processing_time_ms,
                      uint64_t oversubscribed_links, uint64_t aggregate_count,
                      std::vector<double>&& stretches,
                      std::vector<double>&& stretches_rel,
                      std::vector<std::pair<double, double>>&& link_loads,
                      std::vector<double>&& num_paths,
                      std::vector<double>&& unmet_demand)
      : path_stretches(std::move(stretches)),
        path_stretches_rel(std::move(stretches_rel)),
        link_loads(std::move(link_loads)),
        num_paths(std::move(num_paths)),
        unmet_demand(std::move(unmet_demand)),
        processing_time_ms(processing_time_ms),
        cached_processing_time_ms(cached_processing_time_ms),
        oversubscribed_links(oversubscribed_links),
        network_state_path_count(std::numeric_limits<uint64_t>::max()),
        aggregate_count(aggregate_count) {}

  void SaveStats(const std::string& file) const {
    std::string prefix = ncode::StrCat(aggregate_count, ",");

    std::string all_stretches = ncode::Join(path_stretches, ",");
    ncode::File::WriteStringToFileOrDie(ncode::StrCat(prefix, all_stretches),
                                        ncode::StrCat(file, "_stretches"));

    std::string all_stretches_rel = ncode::Join(path_stretches_rel, ",");
    ncode::File::WriteStringToFileOrDie(
        ncode::StrCat(prefix, all_stretches_rel),
        ncode::StrCat(file, "_stretches_rel"));

    std::function<std::string(
        const std::pair<double, double>& link_flow_and_capacity)> f =
        [](const std::pair<double, double>& link_flow_and_capacity) {
          return ncode::StrCat(link_flow_and_capacity.first, ",",
                               link_flow_and_capacity.second);
        };
    std::string all_link_utilization = ncode::Join(link_loads, ",", f);
    ncode::File::WriteStringToFileOrDie(
        ncode::StrCat(prefix, all_link_utilization),
        ncode::StrCat(file, "_link_utilization"));

    std::string all_unmet_demand = ncode::Join(unmet_demand, ",");
    ncode::File::WriteStringToFileOrDie(ncode::StrCat(prefix, all_unmet_demand),
                                        ncode::StrCat(file, "_unmet_demand"));

    std::string all_num_paths = ncode::Join(num_paths, ",");
    ncode::File::WriteStringToFileOrDie(ncode::StrCat(prefix, all_num_paths),
                                        ncode::StrCat(file, "_num_paths"));
  }

  void SaveProcessingTime(const std::string& file) const {
    std::string prefix = ncode::StrCat(aggregate_count, ",");

    ncode::File::WriteStringToFileOrDie(
        ncode::StrCat(prefix, std::to_string(processing_time_ms)),
        ncode::StrCat(file, "_processing_time"));

    ncode::File::WriteStringToFileOrDie(
        ncode::StrCat(prefix, std::to_string(cached_processing_time_ms)),
        ncode::StrCat(file, "_cached_processing_time"));

    if (network_state_path_count != std::numeric_limits<uint64_t>::max()) {
      ncode::File::WriteStringToFileOrDie(
          std::to_string(network_state_path_count),
          ncode::StrCat(file, "_network_state_path_count"));
    }
  }

  std::vector<double> LinkUtilization() {
    std::vector<double> out;
    for (const std::pair<double, double>& link_flow_and_capacity : link_loads) {
      out.emplace_back(link_flow_and_capacity.first /
                       link_flow_and_capacity.second);
    }

    return out;
  }

  // Per-flow path stretch.
  std::vector<double> path_stretches;
  std::vector<double> path_stretches_rel;

  // Link utilization, first is flow over link, second is link capacity.
  std::vector<std::pair<double, double>> link_loads;

  // Number of paths per aggregate.
  std::vector<double> num_paths;

  // Unmet demand
  std::vector<double> unmet_demand;

  // Time to run the optimizer.
  uint64_t processing_time_ms;

  // Time to get a good solution with paths in the cache.
  uint64_t cached_processing_time_ms;

  // The number of links that are oversubscribed.
  uint64_t oversubscribed_links;

  // Total number of paths that need to be installed in the network, this
  // includes alternative paths to protect against failures, as well as primary
  // paths. Paths that are the same as the shortest path are not included.
  size_t network_state_path_count;

  // Number of aggregates.
  uint64_t aggregate_count;
};

struct BaseConfigInfo {
  void PathStretchesToHtml(ncode::web::HtmlPage* out) const {
    ncode::grapher::HtmlGrapher html_grapher(out, "path_stretches");

    ncode::grapher::PlotParameters1D plot_params;
    plot_params.data_label = "milliseconds";
    plot_params.scale = 1000.0;
    plot_params.title = "Path stretch";

    std::vector<ncode::grapher::DataSeries1D> data = {};
    data.push_back({"FUBAR", fubar_summary->path_stretches});

    if (minmax_summary) {
      data.push_back({"MinMax", minmax_summary->path_stretches});
    }

    if (b4_summary) {
      data.push_back({"B4", b4_summary->path_stretches});
    }

    data.push_back({"SP", sp_summary->path_stretches});
    html_grapher.PlotCDF(plot_params, data);
  }

  void LinkLoadToHtml(ncode::web::HtmlPage* out) const {
    ncode::grapher::HtmlGrapher html_grapher(out, "link_load");

    ncode::grapher::PlotParameters1D plot_params;
    plot_params.data_label = "utilization";
    plot_params.title = "Link utilization";

    std::vector<ncode::grapher::DataSeries1D> data = {};
    data.push_back({"FUBAR", fubar_summary->LinkUtilization()});

    if (minmax_summary) {
      data.push_back({"MinMax", minmax_summary->LinkUtilization()});
    }

    if (b4_summary) {
      data.push_back({"B4", b4_summary->LinkUtilization()});
    }

    data.push_back({"SP", sp_summary->LinkUtilization()});
    html_grapher.PlotCDF(plot_params, data);
  }

  void PathCountToHtml(ncode::web::HtmlPage* out) const {
    ncode::grapher::HtmlGrapher html_grapher(out, "path_count");

    ncode::grapher::PlotParameters1D plot_params;
    plot_params.data_label = "number of paths";
    plot_params.title = "Path counts per aggregate";

    std::vector<ncode::grapher::DataSeries1D> data = {};
    data.push_back({"FUBAR", fubar_summary->num_paths});

    if (minmax_summary) {
      data.push_back({"MinMax", minmax_summary->num_paths});
    }

    if (b4_summary) {
      data.push_back({"B4", b4_summary->num_paths});
    }

    data.push_back({"SP", sp_summary->num_paths});
    html_grapher.PlotCDF(plot_params, data);
  }

  void SaveToHtml(const std::string& location) const {
    ncode::web::HtmlPage page;
    PathStretchesToHtml(&page);
    LinkLoadToHtml(&page);
    PathCountToHtml(&page);

    ncode::File::WriteStringToFileOrDie(page.Construct(), location);
  }

  void SaveStats(const std::string& prefix) const {
    if (sp_summary) {
      sp_summary->SaveStats(ncode::StrCat(prefix, "_sp"));
    }

    if (fubar_summary) {
      fubar_summary->SaveStats(ncode::StrCat(prefix, "_fubar"));
      fubar_summary->SaveProcessingTime(ncode::StrCat(prefix, "_fubar"));
    }

    if (minmax_summary) {
      minmax_summary->SaveStats(ncode::StrCat(prefix, "_minmax"));
    }

    if (b4_summary) {
      b4_summary->SaveStats(ncode::StrCat(prefix, "_b4"));
    }
  }

  double commodity_scale_factor;
  ncode::net::Bandwidth max_flow;
  ncode::net::Bandwidth total_load;
  uint64_t aggregate_count;
  std::unique_ptr<PerOptimizerSummary> sp_summary;
  std::unique_ptr<PerOptimizerSummary> fubar_summary;
  std::unique_ptr<PerOptimizerSummary> minmax_summary;
  std::unique_ptr<PerOptimizerSummary> b4_summary;
};

static std::unique_ptr<PerOptimizerSummary> GetOptimizerSummary(
    const fubar::Output& output, uint64_t optimal_processing_time_ms,
    ncode::net::PathCache* path_cache) {
  // How far away each flow is from the shortest path. In seconds.
  std::vector<double> path_stretches;
  std::vector<double> path_stretches_rel;

  // For each link the total flow over the link and the link's capacity.
  std::vector<std::pair<double, double>> link_load;

  // Number of paths per aggregate.
  std::vector<double> num_paths;

  // A map from a link to the total load over the link.
  std::map<ncode::net::GraphLinkIndex, uint64_t> link_to_total_load;

  // Unmet demand.
  std::vector<double> unmet_demand;

  for (const auto& cookie_and_aggregate_output : output.aggregates()) {
    const fubar::AggregateOutput& aggregate_output =
        cookie_and_aggregate_output.second;
    size_t total_num_flows = aggregate_output.total_num_flows();
    double total_aggregate_demand = aggregate_output.total_demand_bps();
    double required_per_flow_bps = total_aggregate_demand / total_num_flows;

    std::chrono::microseconds shortest_path_delay =
        aggregate_output.ShortestPath(path_cache)->delay();
    double sp_delay_sec =
        std::chrono::duration<double>(shortest_path_delay).count();
    sp_delay_sec = std::max(sp_delay_sec, 0.001);

    size_t path_count = 0;
    for (const auto& tag_and_path : aggregate_output.paths()) {
      const fubar::PathOutput& path_output = tag_and_path.second;
      std::chrono::microseconds path_delay = path_output.path()->delay();
      double per_flow_bps = path_output.per_flow_bps();
      CHECK(path_delay >= shortest_path_delay);

      if (path_output.bps_limit() > 0) {
        ++path_count;
      }

      double path_delay_sec = std::chrono::duration<double>(path_delay).count();
      path_delay_sec = std::max(path_delay_sec, 0.001);
      double delay_sec = path_delay_sec - sp_delay_sec;
      double unmet = std::max(0.0, required_per_flow_bps - per_flow_bps);
      double delta_rel = path_delay_sec / sp_delay_sec;

      for (size_t i = 0; i < path_output.fraction() * total_num_flows; ++i) {
        path_stretches.emplace_back(delay_sec);
        path_stretches_rel.emplace_back(delta_rel);
        unmet_demand.emplace_back(unmet);
      }

      for (ncode::net::GraphLinkIndex link :
           path_output.path()->link_sequence().links()) {
        link_to_total_load[link] +=
            total_aggregate_demand * path_output.fraction();
      }
    }

    num_paths.emplace_back(path_count);
  }

  ncode::net::GraphLinkSet links_seen;
  for (const auto& link_and_total_load : link_to_total_load) {
    ncode::net::GraphLinkIndex link_index = link_and_total_load.first;
    links_seen.Insert(link_index);
    const ncode::net::GraphLink* link =
        path_cache->graph_storage()->GetLink(link_index);

    double total_load = link_and_total_load.second;
    link_load.emplace_back(total_load, link->bandwidth().bps());
  }

  for (ncode::net::GraphLinkIndex link_index :
       path_cache->graph_storage()->AllLinks()) {
    if (!links_seen.Contains(link_index)) {
      const ncode::net::GraphLink* link =
          path_cache->graph_storage()->GetLink(link_index);
      link_load.emplace_back(0, link->bandwidth().bps());
    }
  }

  return ncode::make_unique<PerOptimizerSummary>(
      output.processing_time_ms(), optimal_processing_time_ms,
      output.OversubscribedLinks().Count(), output.aggregates().size(),
      std::move(path_stretches), std::move(path_stretches_rel),
      std::move(link_load), std::move(num_paths), std::move(unmet_demand));
}

static std::unique_ptr<PerOptimizerSummary> RunFUBAR(
    const fubar::Input& input, ncode::net::PathCache* cache) {
  fubar::FubarOptimizer fubar_optimizer(cache);
  fubar::FubarOutput output = fubar_optimizer.Optimize(input, {});

  fubar::FubarOptimizer cached_fubar_optimizer(cache);
  fubar::FubarOutput cached_output = cached_fubar_optimizer.Optimize(input, {});

  auto summary =
      GetOptimizerSummary(output, cached_output.processing_time_ms(), cache);
  summary->network_state_path_count = output.GetNetworkStatePaths(cache);
  return summary;
}

static std::unique_ptr<PerOptimizerSummary> RunMinMax(
    const fubar::Input& input, ncode::net::PathCache* cache, bool* fits) {
  fubar::MinMaxOptimizer min_max_optimizer(cache);
  fubar::MinMaxOutput output = min_max_optimizer.OptimizeMC(input);
  *fits = output.min_max_link_utilization() <= 1;

  return GetOptimizerSummary(output, output.processing_time_ms(), cache);
}

static std::unique_ptr<PerOptimizerSummary> RunB4(
    const fubar::Input& input, ncode::net::PathCache* cache) {
  fubar::B4Optimizer b4_optimizer(cache);
  fubar::B4Output output = b4_optimizer.Optimize(input);
  return GetOptimizerSummary(output, output.processing_time_ms(), cache);
}

static std::unique_ptr<PerOptimizerSummary> RunSP(
    const fubar::Input& input, ncode::net::PathCache* cache) {
  fubar::SPOptimizer sp_optimizer(cache);
  fubar::SPOutput output = sp_optimizer.Optimize(input);
  return GetOptimizerSummary(output, output.processing_time_ms(), cache);
}

void TMRun(const e2e::TrafficMatrix& tm, const std::string& out,
           ncode::net::GraphStorage* storage) {
  std::string summary_location = ncode::Substitute("$0_summary.html", out);
  ncode::net::PathCache cache(storage);

  // Each aggregate will get between 1000 and 10 flows.
  fubar::Input input =
      tm.ToInput(1000, 50, 600, 0.1, std::chrono::milliseconds(100));

  // Assumes that each data stream will have a single .pcap stream.
  BaseConfigInfo info;
  info.aggregate_count = input.aggregates().size();

  bool fits = false;
  info.minmax_summary = RunMinMax(input, &cache, &fits);
  //  if (!fits) {
  //    LOG(INFO) << "TM does not fit, will skip";
  //    return;
  //  }

  info.fubar_summary = RunFUBAR(input, &cache);

  //  std::tie(info.max_flow, info.commodity_scale_factor) = tm.GetMaxFlow({});
  info.total_load = tm.TotalLoad();

  info.sp_summary = RunSP(input, &cache);
  info.b4_summary = RunB4(input, &cache);

  info.SaveStats(out);
  info.SaveToHtml(summary_location);
}

}  // namespace e2e
