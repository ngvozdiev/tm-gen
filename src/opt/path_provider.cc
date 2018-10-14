#include "path_provider.h"

#include <algorithm>
#include <set>
#include <ncode/common.h>
#include <ncode/map_util.h>
#include <ncode/net/algorithm.h>

namespace tm_gen {

PathProvider::Generator* PathProvider::FindOrCreateGenerator(
    const AggregateId& aggregate_id) {
  PathProvider::Generator* generator_ptr =
      nc::FindSmartPtrOrNull(generators_, aggregate_id);
  if (generator_ptr != nullptr) {
    return generator_ptr;
  }

  std::set<nc::net::ConstraintSet> constraint_sets;
  nc::net::ConstraintSet empty_set;
  constraint_sets.emplace(empty_set);

  auto new_gen = nc::make_unique<PathProvider::Generator>(
      aggregate_id.src(), aggregate_id.dst(), *graph_, constraint_sets);
  PathProvider::Generator* raw_ptr = new_gen.get();

  generators_[aggregate_id] = std::move(new_gen);
  return raw_ptr;
}

std::vector<const nc::net::Walk*> PathProvider::KShortestUntilAvoidingPath(
    const AggregateId& aggregate, const nc::net::GraphLinkSet& to_avoid,
    size_t start_k, size_t max_count) {
  std::vector<const nc::net::Walk*> out;

  Generator* generator = FindOrCreateGenerator(aggregate);
  size_t i = start_k;
  while (out.size() != max_count) {
    const nc::net::Walk* next_path = generator->KthShortestPathOrNull(i++);
    if (next_path == nullptr) {
      return {};
    }

    out.emplace_back(next_path);
    if (next_path->ContainsAny(to_avoid)) {
      continue;
    }

    break;
  }

  return out;
}

std::vector<const nc::net::Walk*> PathProvider::KShorestPaths(
    const AggregateId& aggregate, size_t start_k, size_t max_count) {
  PathProvider::Generator* generator = FindOrCreateGenerator(aggregate);

  std::vector<const nc::net::Walk*> out;
  size_t i = start_k;
  while (out.size() != max_count) {
    const nc::net::Walk* next_path = generator->KthShortestPathOrNull(i++);
    if (next_path == nullptr) {
      break;
    }

    out.emplace_back(next_path);
  }

  return out;
}

const nc::net::Walk* PathProvider::AvoidingPathOrNull(
    const AggregateId& aggregate, const nc::net::GraphLinkSet& to_avoid) {
  PathProvider::Generator* generator = FindOrCreateGenerator(aggregate);
  auto path = generator->ShortestPathThatAvoids({}, to_avoid);
  if (!path) {
    return nullptr;
  }

  return TakeOwnership(std::move(path));
}

const nc::net::Walk* PathProvider::TakeOwnership(
    std::unique_ptr<nc::net::Walk> path) {
  auto it = owned_paths_.find(path);
  if (it != owned_paths_.end()) {
    return it->get();
  }

  const nc::net::Walk* raw_ptr = path.get();
  owned_paths_.insert(std::move(path));
  return raw_ptr;
}

}  // namespace tm_gen
