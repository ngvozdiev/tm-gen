#ifndef LDR_LDR_H
#define LDR_LDR_H

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "ncode/net/net_common.h"
#include "common.h"
#include "opt.h"

namespace tm_gen {

// Paths to be used by the optimizer. For each aggregate a list of paths in
// increasing order of delay.
using LDRPathMap = std::map<AggregateId, std::vector<const nc::net::Walk*>>;

struct RunOutput {
  RunOutput()
      : obj_value(std::numeric_limits<double>::max()),
        max_oversubscription(0) {}

  std::map<AggregateId, std::vector<RouteAndFraction>> aggregate_outputs;
  double obj_value;
  double max_oversubscription;
};

// A single pass of the optimizer.
class LDROptimizerPass {
 public:
  LDROptimizerPass(double link_capacity_multiplier, bool ignore_flow_counts,
                   bool force_single_path_aggregates,
                   const TrafficMatrix* input, const LDRPathMap* paths,
                   const nc::net::GraphStorage* graph,
                   const RoutingConfiguration* base_solution);

  const nc::net::GraphLinkSet& links_with_no_capacity() const {
    return links_with_no_capacity_;
  }

  RunOutput& run_output() { return run_output_; }

 private:
  using FrozenCapacityMap = nc::net::GraphLinkMap<double>;

  // Performs a single pass by repeatedly calling
  // OptimizerMinLinkOversubscription.
  void Optimize();

  // Runs the optimization with a given set of paths. Frozen aggregates will be
  // excluded from the optimization. Frozen per-link capacity will be excluded
  // from a link's capacity. Aggregates that have all of their paths go over
  // links at max oversubscription will be frozen after the optimization. The
  // freeze_all argument will cause all aggregates to be frozen after the
  // optimization, regardless of where their paths go.
  double OptimizeMinLinkOversubscription();

  // Returns a limit on the fraction of an aggregate's traffic that can go on a
  // path. Will use base_solution_ to do so if available. If base_solution_ is
  // null will always return 1.
  double PathLimitFraction(const AggregateId& aggregate,
                           const nc::net::Walk* path) const;

  // The input. Not owned by this class.
  const TrafficMatrix* input_;

  // A map of paths that the solver can use. Not owned by this class.
  const LDRPathMap* paths_;

  // The graph.
  const nc::net::GraphStorage* graph_;

  // Aggregates which are frozen are not going to be optimized by the solver.
  std::set<AggregateId> frozen_aggregates_;

  // Per-link capacity that the solver should ignore.
  FrozenCapacityMap frozen_capacity_;

  // Value of max oversubscription after the most recent optimization run.
  double latest_run_max_oversubscription_;

  // Links that have no capacity left on them.
  nc::net::GraphLinkSet links_with_no_capacity_;

  // Stores the output.
  RunOutput run_output_;

  // Oversubscription after all single-path aggregates have been frozen.
  double initial_oversubscription_;

  // Objective function value after all single-path aggregates have been frozen.
  double initial_obj_;

  // Each aggregate's path fractions will be set to never exceed the ones in
  // this solution, if present. The exception is the aggregate's longest path in
  // base_solution_. Paths that are not in base_solution_ are not limited.
  const RoutingConfiguration* base_solution_;

  // Can scale all links' capacities by that fraction.
  double link_capacity_multiplier_;

  // If true will treat all aggregates the same, irrespective of their flow
  // count.
  bool ignore_flow_counts_;

  // If true all aggregates' variables will be binary.
  bool force_single_path_aggregates_;
};

class LDROptimizer : public Optimizer {
 public:
  LDROptimizer(PathProvider* path_provider, double link_capacity_multiplier,
               bool add_limits, bool ignore_flow_counts)
      : Optimizer(path_provider),
        add_limits_(add_limits),
        ignore_flow_counts_(ignore_flow_counts),
        link_capacity_multiplier_(link_capacity_multiplier),
        force_single_path_aggregates_(false) {}

  std::unique_ptr<RoutingConfiguration> Optimize(
      const TrafficMatrix& tm) override;

  // The first result returned is the same as from Optimize. The second is the
  // solution without stability limits applied (true optimal).
  std::pair<std::unique_ptr<RoutingConfiguration>,
            std::unique_ptr<RoutingConfiguration>>
  OptimizeAndReturnUnlimitedRun(const TrafficMatrix& tm);

  // Forces all aggregates to only take one path.
  void ForceSinglePathAggregates() { force_single_path_aggregates_ = true; }

 private:
  std::unique_ptr<RoutingConfiguration> LimitedUnlimitedDispatch(
      const TrafficMatrix& tm,
      std::unique_ptr<RoutingConfiguration>* unlimited_run);

  // Single run of the optimization for given input.
  double OptimizePrivate(const TrafficMatrix& input,
                         const std::vector<AggregateId>& aggregates_ordered,
                         bool use_previous, RoutingConfiguration* out);

  // Prioritizes the aggregates from an input. Aggregates that are more
  // important are at the start of the returned list.
  std::vector<AggregateId> PrioritizeAggregates(
      const TrafficMatrix& input) const;

  // For each aggregate will add all paths up to and including the lowest delay
  // path that has some free capacity. Returns whether it added any paths.
  bool AddFreePaths(const nc::net::GraphLinkSet& links_with_no_capacity,
                    const std::vector<AggregateId>& aggregates_ordered);

  // If the total number of paths that will go to the optimizer exceeds this
  // limit only the next best free path will be added, as opposed to all k
  // shortest paths until the free path.
  size_t soft_path_limit_ = 100000;

  // If the total number of paths that will go to the optimizer exceeds this
  // limit no more paths will be added (each aggregate after that will only have
  // one path -- its shortest).
  size_t hard_path_limit_ = 200000;

  // Each aggregate will only have up to this many K shortest paths added to it.
  // After this limit is reached, paths will be added in a way that does not
  // ensure optimality of the solution.
  size_t per_aggregate_path_limit_ = 1000;

  // How far away does the unlimited solution need to be from the limited one in
  // order to use the unlimited.
  double unlimited_threshold_ = 0.01;

  // Paths for the optimization. Potentially extended by calls to Optimize.
  LDRPathMap path_map_;

  // Need some place to remember where in the order of K shortest paths we have
  // gone up to for each aggregate.
  std::map<AggregateId, size_t> ksp_indices_;

  // Holds a copy of the most recent return value of Optimize.
  std::unique_ptr<RoutingConfiguration> previous_;

  // If true will run two different optimizations, one with limits to avoid
  // reordering and another one without.
  bool add_limits_;

  // Whether or not to ignore flow counts when optimizing.
  bool ignore_flow_counts_;

  // All links' capacities will be multiplied by this number.
  double link_capacity_multiplier_;

  // If true will force all aggregates to only take one path.
  bool force_single_path_aggregates_;
};

}  // namespace tm_gen

#endif
