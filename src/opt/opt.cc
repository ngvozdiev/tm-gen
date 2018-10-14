#include "opt.h"

#include <gflags/gflags.h>
#include <stddef.h>
#include <initializer_list>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "ncode/common.h"
#include "ncode/lp/lp.h"
#include "ncode/lp/mc_flow.h"
#include "ncode/map_util.h"
#include "ncode/net/net_common.h"
#include "ncode/net/algorithm.h"
#include "ncode/perfect_hash.h"
#include "ncode/thread_runner.h"

DEFINE_bool(
    b4_break_on_congestion, false,
    "If true will cause B4 to stop as soon as an aggregate does not fit.");

namespace tm_gen {

std::unique_ptr<RoutingConfiguration> ShortestPathOptimizer::Optimize(
    const TrafficMatrix& tm) {
  auto out = nc::make_unique<RoutingConfiguration>(tm);
  for (const auto& aggregate_and_demand : tm.demands()) {
    const AggregateId& aggregate_id = aggregate_and_demand.first;

    const nc::net::Walk* path =
        path_provider_->AvoidingPathOrNull(aggregate_id, {});
    CHECK(path != nullptr) << "Unable to find a path for aggregate";
    out->AddRouteAndFraction(aggregate_id, {{path, 1.0}});
  }

  return out;
}

static nc::net::GraphLinkMap<double> GetCapacities(
    const nc::net::GraphStorage& graph, double multiplier,
    const nc::net::ExclusionSet& exclusion_set) {
  nc::net::GraphLinkMap<double> out;
  for (nc::net::GraphLinkIndex link : graph.AllLinks()) {
    const nc::net::GraphLink* link_ptr = graph.GetLink(link);
    bool should_exclude = exclusion_set.ShouldExcludeLink(link) ||
                          exclusion_set.ShouldExcludeNode(link_ptr->src()) ||
                          exclusion_set.ShouldExcludeNode(link_ptr->dst());
    double capacity =
        should_exclude ? 0 : graph.GetLink(link)->bandwidth().Mbps();

    out[link] = capacity * multiplier;
  }

  return out;
}

static std::unique_ptr<RoutingConfiguration> GetRoutingConfig(
    const TrafficMatrix& tm, PathProvider* path_provider,
    std::map<nc::lp::SrcAndDst, std::vector<nc::lp::FlowAndPath>>* paths) {
  auto out = nc::make_unique<RoutingConfiguration>(tm);
  for (const auto& aggregate_and_demand : tm.demands()) {
    const AggregateId& aggregate_id = aggregate_and_demand.first;
    double demand = aggregate_and_demand.second.first.Mbps();

    // All input aggregates should have an entry in the solution.
    std::vector<nc::lp::FlowAndPath>& flow_and_paths =
        nc::FindOrDieNoPrint(*paths, {aggregate_id.src(), aggregate_id.dst()});

    std::vector<RouteAndFraction> routes_for_aggregate;
    for (nc::lp::FlowAndPath& flow_and_path : flow_and_paths) {
      // Compute the fraction of the total demand that this path handles.
      double fraction = flow_and_path.flow() / demand;
      if (fraction == 0) {
        continue;
      }

      // Need to take ownership of the path and transfer it to the path
      // provider, the path provider will also return the same pointer for the
      // same path, avoiding creating new path objects for the same path every
      // time this runs.
      auto path = flow_and_path.TakeOwnershipOfPath();
      routes_for_aggregate.emplace_back(
          path_provider->TakeOwnership(std::move(path)), fraction);
    }

    out->AddRouteAndFraction(aggregate_id, routes_for_aggregate);
  }

  return out;
}

std::unique_ptr<RoutingConfiguration> MinMaxOptimizer::Optimize(
    const TrafficMatrix& tm) {
  nc::lp::MinMaxProblem problem(
      graph_, GetCapacities(*graph_, link_capacity_multiplier_, exclusion_set_),
      also_minimize_delay_);

  for (const auto& aggregate_and_demand : tm.demands()) {
    const AggregateId& aggregate_id = aggregate_and_demand.first;
    nc::net::Bandwidth demand = aggregate_and_demand.second.first;
    problem.AddDemand(aggregate_id.src(), aggregate_id.dst(), demand.Mbps());
  }

  std::map<nc::lp::SrcAndDst, std::vector<nc::lp::FlowAndPath>> paths;
  problem.Solve(&paths);

  return GetRoutingConfig(tm, path_provider_, &paths);
}

static nc::net::GraphLinkMap<double> GetCosts(
    const nc::net::GraphStorage& graph) {
  nc::net::GraphLinkMap<double> out;
  for (nc::net::GraphLinkIndex link : graph.AllLinks()) {
    nc::net::Delay link_delay = graph.GetLink(link)->delay();
    double cost = std::chrono::duration<double, std::milli>(link_delay).count();
    out[link] = cost;
  }

  return out;
}

std::unique_ptr<RoutingConfiguration> LDRLinkBased::Optimize(
    const TrafficMatrix& tm) {
  nc::lp::MinCostMultiCommodityFlowProblem problem(
      GetCapacities(*graph_, link_capacity_multiplier_, exclusion_set_),
      GetCosts(*graph_), graph_);

  for (const auto& aggregate_and_demand : tm.demands()) {
    const AggregateId& aggregate_id = aggregate_and_demand.first;
    const DemandAndFlowCount& demand_and_flow_count =
        aggregate_and_demand.second;
    nc::net::Bandwidth demand = demand_and_flow_count.first;
    double weight = demand_and_flow_count.second;

    problem.AddDemand(aggregate_id.src(), aggregate_id.dst(), demand.Mbps(),
                      weight);
  }

  std::map<nc::lp::SrcAndDst, std::vector<nc::lp::FlowAndPath>> paths;
  if (problem.Solve(&paths) == std::numeric_limits<double>::max()) {
    return {};
  }

  return GetRoutingConfig(tm, path_provider_, &paths);
}

struct MMPathAndAggregate {
  MMPathAndAggregate(nc::lp::VariableIndex variable, const nc::net::Walk* path,
                     const AggregateId& aggregate, nc::net::Bandwidth demand)
      : variable(variable), path(path), aggregate(aggregate), demand(demand) {}

  nc::lp::VariableIndex variable;
  const nc::net::Walk* path;
  AggregateId aggregate;
  nc::net::Bandwidth demand;
};

std::unique_ptr<RoutingConfiguration> MinMaxPathBasedOptimizer::Optimize(
    const TrafficMatrix& tm) {
  using namespace nc::lp;

  // Will build a mapping from a graph link to all paths that traverse that
  // link. For convenience each path is also paired with its aggregate.
  std::map<nc::net::GraphLinkIndex, std::vector<MMPathAndAggregate>>
      link_to_paths;

  // The main LP.
  Problem problem(MINIMIZE);

  // The matrix of variable coefficients.
  std::vector<ProblemMatrixElement> problem_matrix;
  const nc::net::GraphStorage* graph = path_provider_->graph();

  size_t num_paths = 0;
  for (const auto& id_and_demand : tm.demands()) {
    // Each aggregate is an IE pair.
    const AggregateId& id = id_and_demand.first;
    const DemandAndFlowCount& demand_and_flow_count = id_and_demand.second;

    // A per-aggregate constraint to make the variables that belong to each
    // aggregate sum up to 1.
    ConstraintIndex per_aggregate_constraint = problem.AddConstraint();
    problem.SetConstraintRange(per_aggregate_constraint, 1.0, 1.0);

    std::vector<const nc::net::Walk*> paths =
        path_provider_->KShorestPaths(id, 0, k_);
    CHECK(!paths.empty());
    for (const nc::net::Walk* path : paths) {
      // Each path in each aggregate will have a variable associated with it.
      VariableIndex variable = problem.AddVariable();
      problem.SetVariableRange(variable, 0, Problem::kInifinity);
      problem_matrix.emplace_back(per_aggregate_constraint, variable, 1.0);

      for (nc::net::GraphLinkIndex link : path->links()) {
        link_to_paths[link].emplace_back(variable, path, id,
                                         demand_and_flow_count.first);
      }
      ++num_paths;
    }
  }

  // There will be one max utilization variable.
  VariableIndex max_utilization_var = problem.AddVariable();
  problem.SetVariableRange(max_utilization_var, 0, Problem::kInifinity);
  problem.SetObjectiveCoefficient(max_utilization_var, 100000.0);

  // Will add per-link constraints.
  for (const auto& link_and_path : link_to_paths) {
    nc::net::GraphLinkIndex link_index = link_and_path.first;
    const nc::net::GraphLink* link = graph->GetLink(link_index);

    // All utilization variables will be less than max utilization.
    ConstraintIndex utilization_var_constraint = problem.AddConstraint();
    problem.SetConstraintRange(utilization_var_constraint,
                               Problem::kNegativeInifinity, 0);
    problem_matrix.emplace_back(utilization_var_constraint, max_utilization_var,
                                -1.0);

    VariableIndex link_utilization_var = problem.AddVariable();
    problem.SetVariableRange(link_utilization_var, 0, Problem::kInifinity);
    problem_matrix.emplace_back(utilization_var_constraint,
                                link_utilization_var, 1.0);

    ConstraintIndex constraint = problem.AddConstraint();
    problem.SetConstraintRange(constraint, 0, 0);

    double link_capacity = link->bandwidth().Mbps() * capacity_multiplier_;
    problem_matrix.emplace_back(constraint, link_utilization_var,
                                -link_capacity);

    for (const MMPathAndAggregate& path_and_aggregate : link_and_path.second) {
      VariableIndex variable = path_and_aggregate.variable;
      nc::net::Bandwidth demand = path_and_aggregate.demand;

      // The coefficient for the column is the total volume of the aggregate.
      double value = demand.Mbps();
      problem_matrix.emplace_back(constraint, variable, value);
    }

    double link_weight =
        std::chrono::duration<double, std::milli>(link->delay()).count();
    if (also_minimize_delay_) {
      problem.SetObjectiveCoefficient(link_utilization_var, link_weight);
    } else {
      problem.SetObjectiveCoefficient(link_utilization_var, -link_weight);
    }
  }

  // Solve the problem.
  problem.SetMatrix(problem_matrix);
  std::unique_ptr<Solution> solution = problem.Solve();
  CHECK(solution->type() == OPTIMAL || solution->type() == FEASIBLE);

  // Recover the solution and return it.
  std::map<AggregateId, std::vector<RouteAndFraction>> routes_and_fractions;
  std::set<const nc::net::Walk*> paths_added;
  for (const auto& link_and_path : link_to_paths) {
    for (const MMPathAndAggregate& path_and_aggregate : link_and_path.second) {
      const AggregateId& id = path_and_aggregate.aggregate;
      const nc::net::Walk* path = path_and_aggregate.path;

      if (nc::ContainsKey(paths_added, path)) {
        continue;
      }
      paths_added.insert(path);

      VariableIndex variable = path_and_aggregate.variable;
      double fraction = std::max(0.0, solution->VariableValue(variable));
      if (fraction == 0) {
        continue;
      }

      routes_and_fractions[id].emplace_back(path, fraction);
    }
  }

  auto out = nc::make_unique<RoutingConfiguration>(tm);
  for (const auto& id_and_route_and_fraction : routes_and_fractions) {
    const AggregateId& id = id_and_route_and_fraction.first;
    out->AddRouteAndFraction(id, id_and_route_and_fraction.second);
  }

  return out;
}

class B4AggregateState;

// State to associate with each link in B4.
struct B4LinkState {
 public:
  B4LinkState(nc::net::GraphLinkIndex link, double capacity)
      : link_(link), remaining_capacity_(capacity) {}

  // How much fair share would it take for all non-frozen aggregates over this
  // link to congest it.
  double FairShareToCongest();

  // Advances fair share and assigns capacity to aggregates.
  void AdvanceByFairShare(double fair_share);

  void AddAggregate(B4AggregateState* aggregate_state) {
    aggregates_over_link_.insert(aggregate_state);
  }

  const std::set<B4AggregateState*>& aggregates_over_link() const {
    return aggregates_over_link_;
  }

  nc::net::GraphLinkIndex link() const { return link_; }

  bool IsCongested() const { return remaining_capacity_ < 0.1; }

 private:
  nc::net::GraphLinkIndex link_;

  // Aggregates that go over this link.
  std::set<B4AggregateState*> aggregates_over_link_;

  // Capacity left on this link.
  double remaining_capacity_;
};

// State to associate with each aggregate.
class B4AggregateState {
 public:
  B4AggregateState(const AggregateId& aggregate_id,
                   const DemandAndFlowCount& demand_and_flow_count,
                   double fair_share)
      : aggregate_id_(aggregate_id),
        demand_(demand_and_flow_count.first),
        current_path_(nullptr),
        fair_share_(fair_share) {
    fair_share_ratio_ = demand_.bps() / fair_share_;
  }

  const AggregateId& aggregate_id() const { return aggregate_id_; }

  const nc::net::Walk* current_path() const { return current_path_; }
  void set_current_path(const nc::net::Walk* path) { current_path_ = path; }

  void freeze() { frozen_ = true; }
  bool frozen() const { return frozen_; }

  double fair_share_ratio() const { return fair_share_ratio_; }

  double fair_share() const { return fair_share_; }

  // Adds some capacity to the current path.
  void AdvanceByFairShare(double fair_share) {
    CHECK(current_path_ != nullptr);
    path_to_capacity_[current_path_] += fair_share_ratio_ * fair_share;
  }

  // Returns a map from path to the capacity that will be sent over it.
  const std::map<const nc::net::Walk*, double>& path_to_capacity() const {
    return path_to_capacity_;
  }

  bool CurrentPathContainsLink(const nc::net::GraphLinkIndex link_index) const {
    CHECK(current_path_ != nullptr);
    return current_path_->Contains(link_index);
  }

 private:
  // Identifies the aggregate.
  AggregateId aggregate_id_;

  // Total volume in the entire aggregate.
  nc::net::Bandwidth demand_;

  // All links that are on this aggregate's current path.
  const nc::net::Walk* current_path_;

  // Allocation of paths to capacity on those paths.
  std::map<const nc::net::Walk*, double> path_to_capacity_;

  // Set to true when the aggregate reaches capacity.
  bool frozen_ = false;

  // The ratio C / F, where C is the total required demand of the aggregate and
  // F is the fair share at which the aggregate achieves its total demand.
  double fair_share_ratio_;

  // This aggregate's fair share.
  double fair_share_;
};

double B4LinkState::FairShareToCongest() {
  double sum = 0;  // sum of fair share ratios
  for (const B4AggregateState* aggregate : aggregates_over_link_) {
    if (aggregate->frozen()) {
      continue;
    }

    if (!aggregate->CurrentPathContainsLink(link_)) {
      continue;
    }

    sum += aggregate->fair_share_ratio();
  }

  return remaining_capacity_ / sum;
}

void B4LinkState::AdvanceByFairShare(double fair_share) {
  double sum = 0;  // sum of capacities.
  for (const B4AggregateState* aggregate : aggregates_over_link_) {
    if (aggregate->frozen()) {
      continue;
    }

    if (!aggregate->CurrentPathContainsLink(link_)) {
      continue;
    }

    double capacity = aggregate->fair_share_ratio() * fair_share;
    sum += capacity;
  }

  remaining_capacity_ -= sum;
}

std::unique_ptr<RoutingConfiguration> B4Optimizer::Optimize(
    const TrafficMatrix& tm) {
  std::vector<B4AggregateState> aggregate_states;
  std::map<nc::net::GraphLinkIndex, B4LinkState> link_states;

  // Populate link states.
  for (nc::net::GraphLinkIndex link_index : graph_->AllLinks()) {
    const nc::net::GraphLink* link = graph_->GetLink(link_index);

    link_states.emplace(
        std::piecewise_construct, std::forward_as_tuple(link_index),
        std::forward_as_tuple(
            link_index, link->bandwidth().bps() * link_capacity_multiplier_));
  }

  // A constraint to avoid congested links.
  nc::net::GraphLinkSet to_avoid;

  // Populate aggregate states and initial paths.
  aggregate_states.reserve(tm.demands().size());
  for (const auto& aggregate_and_demand : tm.demands()) {
    const AggregateId& aggregate_id = aggregate_and_demand.first;
    const DemandAndFlowCount& demand_and_flow_count =
        aggregate_and_demand.second;

    double fair_share = 1.0;
    if (flow_count_as_fair_share_) {
      fair_share = demand_and_flow_count.second;
    }

    aggregate_states.emplace_back(aggregate_id, demand_and_flow_count,
                                  fair_share);
    B4AggregateState& aggregate_state = aggregate_states.back();

    const nc::net::Walk* path =
        path_provider_->AvoidingPathOrNull(aggregate_id, to_avoid);
    aggregate_state.set_current_path(path);

    for (nc::net::GraphLinkIndex link : path->links()) {
      B4LinkState& link_state = nc::FindOrDie(link_states, link);
      link_state.AddAggregate(&aggregate_state);
    }
  }

  size_t satisfied_aggregates = 0;
  double current_fair_share = 0;
  while (true) {
    // Figure out at which fair share the next event is -- either a link is
    // congested or an aggregate is satisfied.
    double fair_share_of_next_event = std::numeric_limits<double>::max();
    const B4LinkState* link_to_congest = nullptr;
    std::vector<B4AggregateState*> aggregates_to_satisfy;

    for (auto& link_and_state : link_states) {
      B4LinkState* link_state = &link_and_state.second;
      if (link_state->IsCongested()) {
        continue;
      }

      double fair_share_to_congest =
          current_fair_share + link_state->FairShareToCongest();
      if (fair_share_to_congest < fair_share_of_next_event) {
        fair_share_of_next_event = fair_share_to_congest;
        link_to_congest = link_state;
      }
    }

    for (B4AggregateState& aggregate_state : aggregate_states) {
      if (aggregate_state.frozen()) {
        continue;
      }

      double fair_share_to_satisfy = aggregate_state.fair_share();
      if (fair_share_to_satisfy < fair_share_of_next_event) {
        fair_share_of_next_event = fair_share_to_satisfy;
        aggregates_to_satisfy = {&aggregate_state};
        link_to_congest = nullptr;
      } else if (fair_share_to_satisfy == fair_share_of_next_event) {
        aggregates_to_satisfy.emplace_back(&aggregate_state);
        link_to_congest = nullptr;
      }
    }

    if (fair_share_of_next_event == std::numeric_limits<double>::max()) {
      // Done -- all links have been congested and all aggregates satisfied (or
      // we ran out of paths).
      break;
    }

    double fair_share_delta = fair_share_of_next_event - current_fair_share;
    for (auto& link_and_state : link_states) {
      B4LinkState* link_state = &link_and_state.second;
      link_state->AdvanceByFairShare(fair_share_delta);
    }
    for (B4AggregateState& aggregate_state : aggregate_states) {
      if (aggregate_state.frozen()) {
        continue;
      }

      aggregate_state.AdvanceByFairShare(fair_share_delta);
    }
    current_fair_share = fair_share_of_next_event;

    bool to_break = false;
    if (link_to_congest != nullptr) {
      // Will advance fair share to the congesting point. All aggregates that go
      // over the link will need new paths.
      to_avoid.Insert(link_to_congest->link());

      for (B4AggregateState* aggregate_state :
           link_to_congest->aggregates_over_link()) {
        const AggregateId& aggregate_id = aggregate_state->aggregate_id();
        const nc::net::Walk* path =
            path_provider_->AvoidingPathOrNull(aggregate_id, to_avoid);
        if (path == nullptr) {
          if (FLAGS_b4_break_on_congestion && !aggregate_state->frozen()) {
            LOG(INFO) << "Will break B4 on link "
                      << graph_->GetLink(link_to_congest->link())
                             ->ToStringNoPorts()
                      << " aggregate " << aggregate_id.ToString(*graph_);

            to_break = true;
            break;
          }

          aggregate_state->freeze();
          continue;
        }

        aggregate_state->set_current_path(path);
        for (nc::net::GraphLinkIndex link : path->links()) {
          B4LinkState& link_state = nc::FindOrDie(link_states, link);
          link_state.AddAggregate(aggregate_state);
        }
      }
    }

    if (to_break) {
      break;
    }

    for (B4AggregateState* aggregate_to_satisfy : aggregates_to_satisfy) {
      // Will freeze the aggregate, since it has satisfied its demand.
      aggregate_to_satisfy->freeze();
      ++satisfied_aggregates;
    }
  }

  auto out = nc::make_unique<RoutingConfiguration>(tm);
  for (const B4AggregateState& aggregate_state : aggregate_states) {
    const std::map<const nc::net::Walk*, double>& path_to_capacity =
        aggregate_state.path_to_capacity();

    // If we were unable to satisfy the demand the total capacity allocated to
    // paths may be less than the aggregate's total demand. Will normalize.
    double total_capacity = 0;

    for (const auto& path_and_capacity : path_to_capacity) {
      total_capacity += path_and_capacity.second;
    }

    std::vector<RouteAndFraction> routes_for_aggregate;
    for (const auto& path_and_capacity : path_to_capacity) {
      const nc::net::Walk* path = path_and_capacity.first;
      double capacity = path_and_capacity.second;
      if (capacity == 0) {
        continue;
      }

      double fraction = capacity / total_capacity;
      routes_for_aggregate.emplace_back(path, fraction);
    }
    out->AddRouteAndFraction(aggregate_state.aggregate_id(),
                             routes_for_aggregate);
  }

  return out;
}

static nc::net::Bandwidth GetCapacityAtDelay(const nc::net::GraphStorage& graph,
                                             const AggregateId& id,
                                             nc::net::Delay threshold,
                                             nc::net::Bandwidth min,
                                             nc::net::Bandwidth max,
                                             nc::net::Bandwidth accuracy) {
  // Will create a problem with a single aggregate and will increase the
  // aggregate's demand until a path with delay more than 'threshold' is used.
  // Will do a binary search.
  nc::net::Bandwidth candidate_bw = min;

  while (true) {
    if (max - min < accuracy) {
      break;
    }
    nc::net::Bandwidth bw = min + (max - min) / 2;

    TrafficMatrix tm(&graph);
    tm.AddDemand(id, DemandAndFlowCount(bw, 1ul));

    PathProvider path_provider(&graph);
    LDRLinkBased ctr(&path_provider, 1.0, {});
    auto result = ctr.Optimize(tm);
    if (!result) {
      // Not feasible.
      max = bw;
      continue;
    }

    const std::vector<RouteAndFraction>& routes_for_aggregate =
        nc::FindOrDieNoPrint(result->routes(), id);
    bool too_long = false;
    for (const auto& route_and_fraction : routes_for_aggregate) {
      if (route_and_fraction.first->delay() > threshold) {
        too_long = true;
        break;
      }
    }

    if (too_long) {
      max = bw;
    } else {
      min = bw;
      candidate_bw = std::max(candidate_bw, min);
    }
  }

  candidate_bw = std::min(candidate_bw, max);
  return candidate_bw;
}

nc::net::Bandwidth PathFlow(const nc::net::Walk& path,
                            const nc::net::GraphStorage& graph) {
  nc::net::Bandwidth flow = nc::net::Bandwidth::Max();
  for (const auto& link : path.links()) {
    nc::net::Bandwidth bw = graph.GetLink(link)->bandwidth();
    flow = std::min(flow, bw);
  }

  return flow;
}

nc::net::Bandwidth OutgoingFlow(nc::net::GraphNodeIndex node,
                                const nc::net::GraphStorage& graph) {
  const nc::net::AdjacencyList& adj_list = graph.AdjacencyList();
  const std::vector<nc::net::AdjacencyList::LinkInfo>& neighbors =
      adj_list.GetNeighbors(node);

  nc::net::Bandwidth total = nc::net::Bandwidth::Zero();
  for (const auto& link_info : neighbors) {
    nc::net::GraphLinkIndex link_index = link_info.link_index;
    nc::net::Bandwidth link_bw = graph.GetLink(link_index)->bandwidth();
    total += link_bw;
  }

  return total;
}

nc::net::Bandwidth MinLinkCapacity(const nc::net::GraphStorage& graph) {
  nc::net::Bandwidth capacity = nc::net::Bandwidth::Max();
  for (const nc::net::GraphLinkIndex link : graph.AllLinks()) {
    nc::net::Bandwidth bw = graph.GetLink(link)->bandwidth();
    capacity = std::min(capacity, bw);
  }

  return capacity;
}

std::map<AggregateId, double> GetCapacityAtDelay(
    const nc::net::GraphStorage& graph, double fraction) {
  std::map<AggregateId, double> out;
  nc::net::AllPairShortestPath sp({}, graph.AdjacencyList(), nullptr, nullptr);
  for (nc::net::GraphNodeIndex src : graph.AllNodes()) {
    LOG(INFO) << "Processing source " << graph.GetNode(src)->id();

    nc::net::Bandwidth outgoing_flow = OutgoingFlow(src, graph);
    for (nc::net::GraphNodeIndex dst : graph.AllNodes()) {
      if (src == dst) {
        continue;
      }

      AggregateId id(src, dst);
      auto path = sp.GetPath(src, dst);
      nc::net::Bandwidth sp_path_flow = PathFlow(*path, graph);
      if (sp_path_flow == outgoing_flow) {
        out[id] = 1.0;
        continue;
      }

      double threshold = path->delay().count() * fraction;
      nc::net::Delay delay_threshold =
          nc::net::Delay(static_cast<size_t>(threshold));
      nc::net::Bandwidth max_flow = GetCapacityAtDelay(
          graph, id, delay_threshold, sp_path_flow, outgoing_flow,
          nc::net::Bandwidth::FromMBitsPerSecond(1));

      double f = max_flow / PathFlow(*path, graph);
      CHECK(f >= 1.0) << max_flow.Mbps() << " vs "
                      << PathFlow(*path, graph).Mbps();
      out[id] = f;
    }
  }

  return out;
}

// Returns true if it is possible to fit 'bandwidth' traffic from the source of
// the aggregate identified by 'id' to its destination without using any paths
// with delay longer than 'threshold'.
static bool CanFitTraffic(const nc::net::GraphStorage& graph,
                          const AggregateId& id,
                          const nc::net::ExclusionSet& exclusion_set,
                          nc::net::Bandwidth bandwidth,
                          nc::net::Delay threshold) {
  PathProvider path_provider(&graph);
  LDRLinkBased ctr_link_based(&path_provider, 1.0, exclusion_set);

  std::map<AggregateId, DemandAndFlowCount> demands;
  TrafficMatrix tm(&graph);
  tm.AddDemand(id, {bandwidth, 1000});

  std::unique_ptr<RoutingConfiguration> routing_configuration =
      ctr_link_based.Optimize(tm);
  if (!routing_configuration) {
    return false;
  }

  const std::vector<RouteAndFraction>& routes =
      routing_configuration->FindRoutesOrDie(id);
  for (const auto& route_and_fraction : routes) {
    if (route_and_fraction.first->delay() > threshold) {
      return false;
    }
  }

  CHECK(routes.size() > 0);
  return true;
}

// Returns all links that have capacity less than 'bandwidth'.
nc::net::GraphLinkSet LinksWithCapacityLessThan(
    const nc::net::GraphStorage& graph, nc::net::Bandwidth bandwidth) {
  nc::net::GraphLinkSet out;
  for (nc::net::GraphLinkIndex link : graph.AllLinks()) {
    nc::net::Bandwidth link_bw = graph.GetLink(link)->bandwidth();
    if (link_bw < bandwidth) {
      out.insert(link);
    }
  }

  return out;
}

static bool HasSingleAlternativePath(
    const nc::net::GraphStorage& graph,
    const nc::net::GraphLinkSet& links_with_low_capacity,
    nc::net::GraphLinkIndex link, const AggregateId& id,
    nc::net::Delay threshold) {
  nc::net::GraphLinkSet to_exclude(links_with_low_capacity);
  to_exclude.Insert(link);

  nc::net::ConstraintSet constraints;
  constraints.Exclude().Links(to_exclude);

  auto alternative_path = nc::net::ShortestPathWithConstraints(
      id.src(), id.dst(), graph, constraints);

  if (!alternative_path) {
    return false;
  }

  if (alternative_path->delay() > threshold) {
    return false;
  }

  return true;
}

// Returns the fraction of links on the aggregate's shortest path that can be
// removed and still have the entire aggregate be routed along an alternative
// path.
static double GetLinkFractionAtDelay(const nc::net::GraphStorage& graph,
                                     const nc::net::Walk& shortest_path,
                                     const AggregateId& id,
                                     nc::net::Delay threshold, bool all_same) {
  nc::net::Bandwidth sp_path_flow = PathFlow(shortest_path, graph);
  nc::net::GraphLinkSet links_with_low_capacity =
      LinksWithCapacityLessThan(graph, sp_path_flow);

  double count = 0;
  for (nc::net::GraphLinkIndex link : shortest_path.links()) {
    if (HasSingleAlternativePath(graph, links_with_low_capacity, link, id,
                                 threshold)) {
      ++count;
      continue;
    }

    if (all_same) {
      continue;
    }

    nc::net::ExclusionSet exclusion_set;
    exclusion_set.Links({link});
    if (!CanFitTraffic(graph, id, exclusion_set, sp_path_flow, threshold)) {
      continue;
    }

    ++count;
  }

  return count / shortest_path.links().size();
}

size_t CapacityDiversity(const nc::net::GraphStorage& graph) {
  std::set<nc::net::Bandwidth> links_set;
  for (nc::net::GraphLinkIndex link_index : graph.AllLinks()) {
    const nc::net::GraphLink* link_ptr = graph.GetLink(link_index);
    links_set.insert(link_ptr->bandwidth());
  }

  return links_set.size();
}

std::map<AggregateId, double> GetLinkFractionAtDelay(
    const nc::net::GraphStorage& graph, double delay_fraction) {
  nc::net::AllPairShortestPath sp({}, graph.AdjacencyList(), nullptr, nullptr);
  bool all_same = CapacityDiversity(graph) == 1;

  std::map<AggregateId, double> out;
  for (nc::net::GraphNodeIndex src : graph.AllNodes()) {
    for (nc::net::GraphNodeIndex dst : graph.AllNodes()) {
      if (src == dst) {
        continue;
      }

      AggregateId id(src, dst);
      auto path = sp.GetPath(src, dst);
      double threshold = path->delay().count() * delay_fraction;
      nc::net::Delay delay_threshold =
          nc::net::Delay(static_cast<size_t>(threshold));

      double f =
          GetLinkFractionAtDelay(graph, *path, id, delay_threshold, all_same);
      out[id] = f;
    }
  }

  return out;
}

double GetFractionOfPairsAboveLinkFraction(const nc::net::GraphStorage& graph,
                                           double delay_fraction,
                                           double link_fraction) {
  std::map<AggregateId, double> link_fraction_at_delay =
      GetLinkFractionAtDelay(graph, delay_fraction);
  double count = 0;
  for (const auto& aggregate_and_fraction : link_fraction_at_delay) {
    double fraction = aggregate_and_fraction.second;
    if (fraction >= link_fraction) {
      ++count;
    }
  }

  return count / link_fraction_at_delay.size();
}

}  // namespace tm_gen
