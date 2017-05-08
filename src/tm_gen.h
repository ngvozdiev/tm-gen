#include <stddef.h>
#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <type_traits>
#include <utility>
#include <vector>

#include "ncode-src/src/common/common.h"
#include "ncode-src/src/net/net_common.h"

namespace fubar {
class Input;
} /* namespace fubar */

namespace e2e {

struct TrafficMatrixElement {
  TrafficMatrixElement(ncode::net::GraphNodeIndex src,
                       ncode::net::GraphNodeIndex dst,
                       ncode::net::Bandwidth load)
      : src(src), dst(dst), load(load) {}

  ncode::net::GraphNodeIndex src;
  ncode::net::GraphNodeIndex dst;
  ncode::net::Bandwidth load;
};

class TrafficMatrix {
 public:
  using NodePair =
      std::pair<ncode::net::GraphNodeIndex, ncode::net::GraphNodeIndex>;

  TrafficMatrix(const std::vector<TrafficMatrixElement>& elements,
                const ncode::net::GraphStorage* graph_storage)
      : elements_(elements), graph_storage_(graph_storage) {}

  TrafficMatrix(std::vector<TrafficMatrixElement>&& elements,
                const ncode::net::GraphStorage* graph_storage)
      : elements_(std::move(elements)), graph_storage_(graph_storage) {}

  // The elements of this traffic matrix.
  const std::vector<TrafficMatrixElement>& elements() const {
    return elements_;
  }

  // Per-link utilization, when everything is routed over the shortest path.
  ncode::net::GraphLinkMap<double> SPUtilization() const;

  // For each ingress-egress pair returns the number of hops on the SP and the
  // delay of the path.
  std::map<NodePair, std::pair<size_t, ncode::net::Delay>> SPStats() const;

  // Returns the load that the ingress sends to the egress.
  ncode::net::Bandwidth Load(const NodePair& node_pair) const;

  // Total load for all ingress-egress pairs.
  ncode::net::Bandwidth TotalLoad() const;

  // Returns the sum of the load over all links divided by the sum of all links'
  // capacity when all aggregates are routed over their shortest paths.
  double SPGlobalUtilization() const;

  // Transforms this matrix into an Input. Each aggregate will have a number of
  // flows based on its volume, the one with the maximum volume will get
  // max_count, no aggregate will get lower than min_count.
  fubar::Input ToInput(size_t max_flow_count, size_t min_flow_count,
                       size_t bin_count, double spread,
                       std::chrono::milliseconds bin_size) const;

  // Returns <max_flow, scale_factor>, where max flow is the max flow through
  // the network, and scale_factor is a number by which all aggregates can be
  // multiplied to get to max flow.
  std::pair<ncode::net::Bandwidth, double> GetMaxFlow(
      const ncode::net::GraphLinkSet& to_exclude) const;

  // True if traffic can be routed through the network.
  bool IsFeasible(const ncode::net::GraphLinkSet& to_exclude) const;

  // Returns the max commodity scale factor for this matrix.
  double MaxCommodityScaleFractor() const;

  // Returns true if the traffic matrix is resilient to any single link failure
  // (the load can still fit).
  bool ResilientToFailures() const;

  bool empty() const { return elements_.empty(); }

  // Returns a traffic matrix with the same pairs, but with load of all pairs
  // scaled by 'factor'.
  std::unique_ptr<TrafficMatrix> Scale(double factor) const;

  // Returns a traffic matrix that contains only the largest aggregate from this
  // traffic matrix.
  std::unique_ptr<TrafficMatrix> IsolateLargest() const;

  const ncode::net::GraphStorage* graph_storage() const {
    return graph_storage_;
  }

 private:
  static constexpr double kDefaultB4FairShare = 1.0;
  static constexpr double kDefaultFUBARConstant = 1.0;
  static constexpr double kDefaultCapacityMultiplier = 1.0;
  static constexpr double kDefaultMinMaxRadiusFromSP = 5;

  std::vector<TrafficMatrixElement> elements_;
  const ncode::net::GraphStorage* graph_storage_;

  DISALLOW_COPY_AND_ASSIGN(TrafficMatrix);
};

// A class that generates traffic matrices based on a set of constraints.
class TMGenerator {
 public:
  TMGenerator(uint64_t seed, ncode::net::GraphStorage* graph)
      : graph_(graph),
        max_global_utilization_(std::numeric_limits<double>::max()),
        rnd_(seed),
        min_scale_factor_(1.0) {}

  // Adds a constraint that makes 'fraction' of all links have utilization less
  // than or equal to 'utilization' when aggregates are routed over their
  // shortest paths.
  void AddUtilizationConstraint(double fraction, double utilization);

  // Adds a constraint that makes 'fraction' of all load go over ie pairs whose
  // shortest paths are hop count or longer.
  void AddHopCountLocalityConstraint(double fraction, size_t hop_count);

  // Adds a constraint that makes 'fraction' of all load go over ie pairs whose
  // shortest paths are distance delay or longer.
  void AddDistanceLocalityConstraint(double fraction,
                                     std::chrono::milliseconds delay);

  // Sets the max global utilization.
  void SetMaxGlobalUtilization(double fraction);

  void SetMinScaleFactor(double factor);

  // Adds a constraint that makes 'fraction' of all aggregates use up to
  // 'out_fraction' of their respective total outgoing capacity.
  void AddOutgoingFractionConstraint(double fraction, double out_fraction);

  // Generates a matrix. If the explore_alternatives argument is true will
  // generate kMaxTries matrices and pick the one with the highest global
  // utility. The matrix will be scaled by the scale argument after generation
  // (but this function guarantees that it will return a feasible matrix).
  std::unique_ptr<TrafficMatrix> GenerateMatrix(
      bool explore_alternatives = false, double scale = 1.0);

 private:
  static constexpr size_t kMaxTries = 100;

  // Called by GenerateMatrix to generate a single matrix, if the matrix does
  // not satisfy the global utilization constraint it is discarded and this
  // function is called again.
  std::unique_ptr<TrafficMatrix> GenerateMatrixPrivate();

  // The graph.
  ncode::net::GraphStorage* graph_;

  // Define the overall distribution of link utilizations.
  using FractionAndUtilization = std::pair<double, double>;
  std::vector<FractionAndUtilization> utilization_constraints_;

  using FractionAndDistance = std::pair<double, std::chrono::milliseconds>;
  std::vector<FractionAndDistance> locality_delay_constraints_;

  using FractionAndHopCount = std::pair<double, size_t>;
  std::vector<FractionAndHopCount> locality_hop_constraints_;

  using FractionAndOutgoingFraction = std::pair<double, double>;
  std::vector<FractionAndOutgoingFraction> outgoing_fraction_constraints_;

  // The sum of all load that crosses all links divided by the sum of all link
  // capacities will be less than this number.
  double max_global_utilization_;

  // Randomness is needed when links are shuffled before solving the problem.
  std::mt19937 rnd_;

  // All aggregates in the generated matrix should be scaleable by at least this
  // much, while keeping the matrix feasible.
  double min_scale_factor_;

  DISALLOW_COPY_AND_ASSIGN(TMGenerator);
};

// Tests a given traffic matrix with MinMax, B4, SP and FUBAR and outputs
// results to 'out'.
void TMRun(const e2e::TrafficMatrix& tm, const std::string& out,
           ncode::net::GraphStorage* storage);

}  // namespace e2e
