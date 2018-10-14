#ifndef TM_GEN_DEMAND_MATRIX_INPUT_H
#define TM_GEN_DEMAND_MATRIX_INPUT_H

#include <iostream>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "ncode/lp/demand_matrix.h"
#include "topology_input.h"

namespace tm_gen {

// Combination of a demand filename and demand matrix.
struct DemandMatrixAndFilename {
  DemandMatrixAndFilename(const std::string& topology_file,
                          const std::string& file,
                          std::unique_ptr<nc::lp::DemandMatrix> demand_matrix)
      : topology_file(topology_file),
        file(file),
        demand_matrix(std::move(demand_matrix)) {}

  DemandMatrixAndFilename() {}

  std::string topology_file;
  std::string file;
  std::unique_ptr<nc::lp::DemandMatrix> demand_matrix;
};

// Parses input for both traffic matrices and demands for those traffic
// matrices.
std::pair<std::vector<TopologyAndFilename>,
          std::vector<DemandMatrixAndFilename>>
GetDemandMatrixInputs(bool skip_trivial);

}  // namespace tm_gen

#endif
