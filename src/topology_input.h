#ifndef TM_GEN_TOPOLOGY_INPUT_H
#define TM_GEN_TOPOLOGY_INPUT_H

#include <iostream>
#include <memory>
#include <vector>

#include "ncode/net/net_common.h"

namespace tm_gen {

// Combination of a topology and a filename.
struct TopologyAndFilename {
  TopologyAndFilename(std::unique_ptr<nc::net::GraphStorage> graph,
                      const std::vector<std::string>& node_order,
                      const std::string& file)
      : graph(std::move(graph)), node_order(node_order), file(file) {}

  std::unique_ptr<nc::net::GraphStorage> graph;
  std::vector<std::string> node_order;
  std::string file;
};

// Returns a set of topologies from input flags. The flags are in
// topology_input.cc.
std::vector<TopologyAndFilename> GetTopologyInputs();

}  // namespace tm_gen

#endif
