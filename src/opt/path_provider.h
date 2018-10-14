#ifndef TM_GEN_PATH_PROVIDER_H
#define TM_GEN_PATH_PROVIDER_H

#include <map>
#include <memory>
#include <vector>
#include <ncode/net/net_common.h>
#include <ncode/net/algorithm.h>

#include "common.h"

namespace tm_gen {

// Can provide the K next shortest paths on demand for an aggregate.
class PathProvider {
 public:
  PathProvider(const nc::net::GraphStorage* graph) : graph_(graph) {}

  // Returns the K shortest paths for an aggregate.
  std::vector<const nc::net::Walk*> KShorestPaths(const AggregateId& aggregate,
                                                  size_t start_k,
                                                  size_t max_count);

  // Adds paths in K shortest order until a path is added that avoids a set of
  // links or max_count is reached. Paths are added starting at start_k.
  std::vector<const nc::net::Walk*> KShortestUntilAvoidingPath(
      const AggregateId& aggregate, const nc::net::GraphLinkSet& to_avoid,
      size_t start_k, size_t max_count);

  // Returns a path that avoids the given set of links.
  const nc::net::Walk* AvoidingPathOrNull(
      const AggregateId& aggregate, const nc::net::GraphLinkSet& to_avoid);

  // Takes ownership of a path.
  const nc::net::Walk* TakeOwnership(std::unique_ptr<nc::net::Walk> path);

  const nc::net::GraphStorage* graph() const { return graph_; };

 private:
  using Generator = nc::net::DisjunctKShortestPathsGenerator;

  Generator* FindOrCreateGenerator(const AggregateId& aggregate_id);

  // The graph.
  const nc::net::GraphStorage* graph_;

  // A path generator for each aggregate.
  std::map<AggregateId, std::unique_ptr<Generator>> generators_;

  // Paths owned by this object should be compared by their value, not by the
  // value of their pointers.
  struct OwnedPathsComparator {
    bool operator()(const std::unique_ptr<nc::net::Walk>& lhs,
                    const std::unique_ptr<nc::net::Walk>& rhs) const {
      return *lhs < *rhs;
    }
  };

  // A set of paths owned by this object.
  std::set<std::unique_ptr<nc::net::Walk>, OwnedPathsComparator> owned_paths_;

  DISALLOW_COPY_AND_ASSIGN(PathProvider);
};

}  // namespace tm_gen

#endif
