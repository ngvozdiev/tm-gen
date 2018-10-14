#ifndef TM_GEN_OPT_H
#define TM_GEN_OPT_H

#include <algorithm>
#include <memory>

#include "ncode/logging.h"
#include "common.h"
#include "path_provider.h"

namespace tm_gen {

class Optimizer {
 public:
  explicit Optimizer(PathProvider* path_provider)
      : path_provider_(path_provider) {
    CHECK(path_provider_);
    graph_ = path_provider_->graph();
  }

  virtual ~Optimizer() {}

  virtual std::unique_ptr<RoutingConfiguration> Optimize(
      const TrafficMatrix& tm) = 0;

  // Same as Optimize, but will also time itself and set the time and optimizer
  // string variables in the returned RoutingConfiguration.
  std::unique_ptr<RoutingConfiguration> OptimizeWithTimeAndOptString(
      const TrafficMatrix& tm, const std::string& opt_string) {
    auto start = std::chrono::high_resolution_clock::now();
    auto rc = Optimize(tm);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    rc->set_time_to_compute(duration);
    rc->set_optimizer_string(opt_string);
    return rc;
  }

  const nc::net::GraphStorage* graph() { return path_provider_->graph(); }

  const PathProvider* path_provider() const { return path_provider_; }

 protected:
  // Provides paths to the optimizer.
  PathProvider* path_provider_;

  // The graph.
  const nc::net::GraphStorage* graph_;
};

// Only ever assigns aggregates to their single shortest path.
class ShortestPathOptimizer : public Optimizer {
 public:
  explicit ShortestPathOptimizer(PathProvider* path_provider)
      : Optimizer(path_provider) {}

  std::unique_ptr<RoutingConfiguration> Optimize(
      const TrafficMatrix& tm) override;
};

// Returns a solution that minimizes the maximum utilization of any link in the
// network. The solution will be optimal, and the paths that it uses will not
// come from the path provider (i.e., they will not conform to any policy).
// Should be considered a lower bound for how low link utilization can get with
// a given traffic matrix.
class MinMaxOptimizer : public Optimizer {
 public:
  MinMaxOptimizer(PathProvider* path_provider, double capacity_multiplier,
                  bool also_minimize_delay,
                  const nc::net::ExclusionSet& exclusion_set = {})
      : Optimizer(path_provider),
        exclusion_set_(exclusion_set),
        link_capacity_multiplier_(capacity_multiplier),
        also_minimize_delay_(also_minimize_delay) {}

  std::unique_ptr<RoutingConfiguration> Optimize(
      const TrafficMatrix& tm) override;

 private:
  // Elements to exclude.
  const nc::net::ExclusionSet exclusion_set_;

  // All links' capacity is multiplied by this number.
  double link_capacity_multiplier_;

  // If true will minmize delay as a secondary objective. If false will maximize
  // it.
  bool also_minimize_delay_;
};

// Also computes the MinMax solution of an optimization problem, but only uses
// up to the K shortest paths.
class MinMaxPathBasedOptimizer : public Optimizer {
 public:
  MinMaxPathBasedOptimizer(PathProvider* path_provider,
                           double capacity_multiplier, bool also_minimize_delay,
                           size_t k)
      : Optimizer(path_provider),
        k_(k),
        capacity_multiplier_(capacity_multiplier),
        also_minimize_delay_(also_minimize_delay) {}

  std::unique_ptr<RoutingConfiguration> Optimize(
      const TrafficMatrix& tm) override;

 private:
  // Number of K shortest path.
  size_t k_;

  // All links' capacity is multiplied by this number.
  double capacity_multiplier_;

  // If true will minmize delay as a secondary objective. If false will maximize
  // it.
  bool also_minimize_delay_;
};

// Runs a heuristic similar to that of B4. Each aggregate's "fair share" will be
// set to its flow count from the TM.
class B4Optimizer : public Optimizer {
 public:
  B4Optimizer(PathProvider* path_provider, bool flow_count_as_fair_share,
              double capacity_multiplier)
      : Optimizer(path_provider),
        flow_count_as_fair_share_(flow_count_as_fair_share),
        link_capacity_multiplier_(capacity_multiplier) {}

  std::unique_ptr<RoutingConfiguration> Optimize(
      const TrafficMatrix& tm) override;

 private:
  bool flow_count_as_fair_share_;

  // All links' capacity is multiplied by this number.
  double link_capacity_multiplier_;
};

// A link-based formulation of LDR.
class LDRLinkBased : public Optimizer {
 public:
  LDRLinkBased(PathProvider* path_provider, double link_capacity_multiplier,
               const nc::net::ExclusionSet& exclusion_set = {})
      : Optimizer(path_provider),
        exclusion_set_(exclusion_set),
        link_capacity_multiplier_(link_capacity_multiplier) {}

  std::unique_ptr<RoutingConfiguration> Optimize(
      const TrafficMatrix& tm) override;

 private:
  // Elements to exclude.
  const nc::net::ExclusionSet exclusion_set_;

  // All links' capacities will be multiplied by this number.
  const double link_capacity_multiplier_;
};

// Returns for each of the N (N-1) aggregates the amount of flow that can be
// sent on paths up to 'fraction' away from the aggregate's shortest path
// divided by the capacity of its shortest path. Fraction should be >= 1.
std::map<AggregateId, double> GetCapacityAtDelay(
    const nc::net::GraphStorage& graph, double delay_fraction);

// Return for each aggregate the fraction of links on the aggregate's shortest
// path that can be saturated and still route the aggregate on an alternate
// path.
std::map<AggregateId, double> GetLinkFractionAtDelay(
    const nc::net::GraphStorage& graph, double delay_fraction);

// Returns the fraction of pairs for which at least 'link_fraction' of the
// shortest path links can be routed around, using a path of at most
// 'delay_fraction' delay away from the shortest.
double GetFractionOfPairsAboveLinkFraction(const nc::net::GraphStorage& graph,
                                           double delay_fraction,
                                           double link_fraction);

size_t CapacityDiversity(const nc::net::GraphStorage& graph);

}  // namespace tm_gen

#endif
