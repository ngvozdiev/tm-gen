#ifndef TM_GEN_COMMON_H
#define TM_GEN_COMMON_H

#include <ncode/common.h>
#include <ncode/htsim/match.h>
#include <ncode/htsim/packet.h>
#include <ncode/lp/mc_flow.h>
#include <ncode/net/net_common.h>
#include <stddef.h>
#include <random>
#include <set>
#include <string>
#include <tuple>

namespace tm_gen {
class PathProvider;
} /* namespace tm_gen */
namespace nc {
namespace lp {
class DemandMatrix;
} /* namespace lp */
} /* namespace tm_gen */

namespace tm_gen {

// Each aggregate is identified by a combination of src, dst.
class AggregateId {
 public:
  AggregateId(nc::net::GraphNodeIndex src, nc::net::GraphNodeIndex dst)
      : src_(src), dst_(dst) {}

  explicit AggregateId(const std::pair<nc::net::GraphNodeIndex,
                                       nc::net::GraphNodeIndex>& src_and_dst)
      : src_(src_and_dst.first), dst_(src_and_dst.second) {}

  nc::net::GraphNodeIndex src() const { return src_; }

  nc::net::GraphNodeIndex dst() const { return dst_; }

  std::string ToString(const nc::net::GraphStorage& graph) const;

  // Returns the delay of the shortest path through the graph that this
  // aggregate can be routed on.
  nc::net::Delay GetSPDelay(const nc::net::GraphStorage& graph) const;

  // Returns the shortest path.
  std::unique_ptr<nc::net::Walk> GetSP(
      const nc::net::GraphStorage& graph) const;

  AggregateId Reverse() const { return {dst_, src_}; }

  friend bool operator<(const AggregateId& a, const AggregateId& b);
  friend bool operator==(const AggregateId& a, const AggregateId& b);
  friend bool operator!=(const AggregateId& a, const AggregateId& b);

 private:
  nc::net::GraphNodeIndex src_;
  nc::net::GraphNodeIndex dst_;
};

// A bandwidth demand and flow count.
using DemandAndFlowCount = std::pair<nc::net::Bandwidth, uint64_t>;

// A path and a fraction of demand that should go over it.
using RouteAndFraction = std::pair<const nc::net::Walk*, double>;

// For each aggregate, a bandwidth demand and a flow count.
class TrafficMatrix {
 public:
  // Generates a TM where each aggregate's flow count is proportional to its
  // traffic volume.
  static std::unique_ptr<TrafficMatrix> ProportionalFromDemandMatrix(
      const nc::lp::DemandMatrix& demand_matrix,
      size_t top_aggregate_flow_count = 1000);

  // Distributes a total number of flows based on traffic volume.
  static std::unique_ptr<TrafficMatrix> DistributeFromDemandMatrix(
      const nc::lp::DemandMatrix& demand_matrix,
      size_t total_flow_count = 10000);

  // Sets all flow counts to be the same.
  static std::unique_ptr<TrafficMatrix> ConstantFromDemandMatrix(
      const nc::lp::DemandMatrix& demand_matrix, size_t flow_count = 1000);

  // Constructs an empty traffic matrix, or optionally let the caller
  // pre-populate it with demands and flow counts.
  explicit TrafficMatrix(
      const nc::net::GraphStorage* graph,
      const std::map<AggregateId, DemandAndFlowCount>& demand_and_counts = {})
      : graph_(graph) {
    demands_ = demand_and_counts;
  }

  // Constructs a traffic matrix from a demand matrix. The demand matrix has no
  // flow counts, so they need to be supplied explicitly per src/dst pair (the
  // second argument). If there is no entry in the map for a src/dst pair its
  // flow count is assumed to be 1.
  explicit TrafficMatrix(
      const nc::lp::DemandMatrix& demand_matrix,
      const std::map<nc::lp::SrcAndDst, size_t>& flow_counts = {});

  const std::map<AggregateId, DemandAndFlowCount>& demands() const {
    return demands_;
  }

  void AddDemand(const AggregateId& aggregate_id,
                 const DemandAndFlowCount& demand_and_flow_count);

  // Scales demands by a factor. If no aggregates are specified will scale all
  // aggregates.
  std::unique_ptr<TrafficMatrix> ScaleDemands(
      double factor, const std::set<AggregateId>& to_scale) const;

  // Adds the same value to all demands.
  std::unique_ptr<TrafficMatrix> AddToDemands(
      nc::net::Bandwidth value, const std::set<AggregateId>& to_extend) const;

  // A DemandMatrix is similar to TrafficMatrix, but has no flow counts.
  std::unique_ptr<nc::lp::DemandMatrix> ToDemandMatrix() const;

  // Returns a new traffic matrix with the same aggregates, but
  // 'aggregate_count' aggregates have their demand/flow count +- a fraction of
  // the demand/flow count in this one.
  std::unique_ptr<TrafficMatrix> Randomize(double demand_fraction,
                                           double flow_count_fraction,
                                           size_t aggregate_count,
                                           std::mt19937* rnd) const;

  const nc::net::GraphStorage* graph() const { return graph_; }

  // Dumps the entire TM to string.
  std::string ToString() const;

  // Dumps a single aggregate from the TM to string.
  std::string AggregateToString(const AggregateId& aggregate) const;

  // Prints a summary of the TM.
  std::string SummaryToString() const;

  std::pair<nc::net::Bandwidth, nc::net::Bandwidth> MinMaxAggregates() const;

  // Adds meta information to this traffic matrix.
  void UpdateProperty(const std::string& key, const std::string& value) {
    properties_[key] = value;
  }

  const std::map<std::string, std::string>& properties() const {
    return properties_;
  }

 protected:
  // The graph.
  const nc::net::GraphStorage* graph_;

 private:
  // For each aggregate its demand and its flow count.
  std::map<AggregateId, DemandAndFlowCount> demands_;

  // Same as the properties of DemandMatrix.
  std::map<std::string, std::string> properties_;

  DISALLOW_COPY_AND_ASSIGN(TrafficMatrix);
};

// Extends a TM with for each aggregate a set of paths and a fraction of demand
// to route.
class RoutingConfiguration : public TrafficMatrix {
 public:
  static constexpr char kDefaultOptimizerString[] = "UNKNOWN";

  // Loads a routing configuration from the text produced by SerializeToText.
  static std::unique_ptr<RoutingConfiguration> LoadFromSerializedText(
      const TrafficMatrix& base_matrix,
      const std::vector<std::string>& node_order, const std::string& text,
      PathProvider* path_provider);

  explicit RoutingConfiguration(const TrafficMatrix& base_matrix)
      : TrafficMatrix(base_matrix.graph(), base_matrix.demands()),
        time_to_compute_(0),
        optimizer_string_(kDefaultOptimizerString) {}

  void AddRouteAndFraction(
      const AggregateId& aggregate_id,
      const std::vector<RouteAndFraction>& routes_and_fractions);

  const std::vector<RouteAndFraction>& FindRoutesOrDie(
      const AggregateId& aggregate_id) const;

  const std::map<AggregateId, std::vector<RouteAndFraction>>& routes() const {
    return configuration_;
  }

  // Returns a new RC, with a set of aggregates excluded.
  std::unique_ptr<RoutingConfiguration> ExcludeAggregates(
      const std::set<AggregateId>& aggregates) const;

  std::string ToString() const;

  std::string AggregateToString(const AggregateId& aggregate,
                                double* aggregate_contribution = nullptr) const;

  // A simple text-based serialization of the routing configuration. Each line
  // after the first one represents one path and has the following
  // comma-separated format:
  // node_0,...,node_n,fraction_of_demand
  // where node_0 is the index of the source node, node_n is the index of the
  // destination node (indices are based on the original .graph file) and
  // fraction_of_demand is the fraction of the total demand between the source
  // and the destination that goes along this path. This representation assumes
  // that there is at most one link between any two nodes in the graph. The
  // first line has the following format:
  // optimizer,compute_time
  // where num_paths is the number of paths, optimizer is a string that
  // identifies the optimizer that was used to generate this configuration and
  // compute_time is the time (in milliseconds) it took to produce it.
  std::string SerializeToText(const std::vector<std::string>& node_order) const;

  // For each aggregate, returns the indices of the aggregate's paths in the K
  // shortest paths sequence of the aggregate.
  std::map<AggregateId, std::vector<size_t>> GetKValues() const;

  // Returns sum of all flows' delay. If the sp argument is true will only add
  // up the flows' best possible shortest path delay.
  nc::net::Delay TotalPerFlowDelay(bool sp = false) const;

  // Returns a map from a link to its utilization.
  nc::net::GraphLinkMap<double> LinkUtilizations() const;

  // Aggregates that cross links with utilization > 1. The first number for each
  // aggregate is the max overloaded link it crosses.
  std::vector<std::pair<double, AggregateId>> OverloadedAggregates() const;

  // Returns the max link utilization.
  double MaxLinkUtilization() const;

  // Returns the maximum number of paths an aggregate has.
  size_t MaxNumberOfPathsInAggregate() const {
    size_t out = 0;
    for (const auto& aggregate_and_routes : configuration_) {
      size_t count = aggregate_and_routes.second.size();
      out = std::max(out, count);
    }

    return out;
  }

  // Returns for each aggregate the number of paths in the aggregate.
  std::vector<size_t> NumberOfPathsInAggregate() const {
    std::vector<size_t> out;
    for (const auto& aggregate_and_routes : configuration_) {
      size_t count = aggregate_and_routes.second.size();
      out.emplace_back(count);
    }

    return out;
  }

  // Makes a copy.
  std::unique_ptr<RoutingConfiguration> Copy() const;

  void set_time_to_compute(std::chrono::milliseconds time) {
    time_to_compute_ = time;
  }

  void set_optimizer_string(const std::string& opt_string) {
    optimizer_string_ = opt_string;
  }

  const std::string& optimizer_string() const { return optimizer_string_; }

  const std::chrono::milliseconds& time_to_compute() const {
    return time_to_compute_;
  }

 private:
  // For each aggregate the path and fraction through the network.
  std::map<AggregateId, std::vector<RouteAndFraction>> configuration_;

  // Optional fields that identify the time it took to compute the solution and
  // the optimizer that was used. Set to 0 if unknown.
  std::chrono::milliseconds time_to_compute_;
  std::string optimizer_string_;
};

}  // namespace tm_gen
#endif
