#include <gflags/gflags.h>
#include <stddef.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "ncode/common.h"
#include "ncode/file.h"
#include "ncode/logging.h"
#include "ncode/lp/demand_matrix.h"
#include "ncode/lp/lp.h"
#include "ncode/lp/mc_flow.h"
#include "ncode/net/algorithm.h"
#include "ncode/net/net_common.h"
#include "ncode/perfect_hash.h"
#include "ncode/strutil.h"
#include "ncode/substitute.h"
#include "ncode/thread_runner.h"
#include "topology_input.h"
#include "tm_gen.h"

DEFINE_string(output_pattern,
              "demand_matrices/scale_factor_$2/locality_$1/$0_$3.demands",
              "Traffic matrices will be saved to files named after this "
              "pattern, with $0 replaced by the topology name, $1 replaced by "
              "locality, $2 replaced by scale factor and $3 replaced by a "
              "unique integer identifier.");
DEFINE_string(min_scale_factor, "1.3",
              "The generated matrix should be scaleable by at least this much "
              "without becoming unfeasible.");
DEFINE_string(locality, "0.0",
              "How much of does locality factor into the traffic matrix.");
DEFINE_uint64(seed, 1ul, "Seed for the generated TM.");
DEFINE_uint64(tm_count, 1, "Number of TMs to generate.");
DEFINE_uint64(threads, 2, "Number of threads to use.");

using namespace std::chrono;

using DemandMatrixVector = std::vector<std::unique_ptr<nc::lp::DemandMatrix>>;
using Input = std::pair<const tm_gen::TopologyAndFilename*, uint32_t>;

// Generates a single traffic matrix for a given graph.
void ProcessMatrix(const Input& input, const std::vector<double>& localities,
                   const std::vector<double>& scale_factors) {
  std::string topology_filename = input.first->file;
  const nc::net::GraphStorage* graph = input.first->graph.get();
  topology_filename = nc::File::ExtractFileName(topology_filename);
  topology_filename = nc::Split(topology_filename, ".").front();

  const std::vector<std::string>& node_order = input.first->node_order;
  uint32_t id = input.second;

  for (double locality : localities) {
    for (double scale_factor : scale_factors) {
      std::string output_location =
          nc::Substitute(FLAGS_output_pattern.c_str(), topology_filename,
                         locality, scale_factor, id);
      if (nc::File::Exists(output_location)) {
        // Will check if it has the 'trivial' property and update it if not.
        auto demand_matrix = nc::lp::DemandMatrix::LoadRepetitaFileOrDie(
            output_location, node_order, graph);
        const std::map<std::string, std::string>& props =
            demand_matrix->properties();

        if (nc::ContainsKey(props, "trivial")) {
          LOG(INFO) << "Skipping existing " << output_location;
        } else {
          bool trivial = demand_matrix->IsTriviallySatisfiable();
          demand_matrix->UpdateProperty("trivial", trivial ? "true" : "false");
          demand_matrix->ToRepetitaFileOrDie(node_order, output_location);
          LOG(INFO) << "Updated 'trivial' property of " << output_location;
        }

        continue;
      }

      std::mt19937 gen(FLAGS_seed + id);
      auto demand_matrix = tm_gen::GenerateRoughan(
          *graph, nc::net::Bandwidth::FromGBitsPerSecond(1), &gen);
      demand_matrix = tm_gen::LocalizeDemandMatrix(*demand_matrix, locality);
      demand_matrix = tm_gen::BalanceReverseDemands(*demand_matrix, 0.1);

      double csf = demand_matrix->MaxCommodityScaleFactor({}, 1.0);
      CHECK(csf != 0);
      CHECK(csf == csf);
      demand_matrix = demand_matrix->Scale(csf);

      double load = 1.0 / scale_factor;
      demand_matrix = demand_matrix->Scale(load);

      std::string directory = nc::File::ExtractDirectoryName(output_location);
      if (directory != "") {
        nc::File::RecursivelyCreateDir(directory, 0777);
      }

      LOG(INFO) << "Will write " << demand_matrix->ToString() << " to "
                << output_location;
      demand_matrix->UpdateProperty("locality",
                                    nc::ToStringMaxDecimals(locality, 2));
      demand_matrix->UpdateProperty("load", nc::ToStringMaxDecimals(load, 2));
      demand_matrix->UpdateProperty(
          "seed", nc::ToStringMaxDecimals(FLAGS_seed + id, 2));

      bool trivial = demand_matrix->IsTriviallySatisfiable();
      demand_matrix->UpdateProperty("trivial", trivial ? "true" : "false");
      demand_matrix->ToRepetitaFileOrDie(node_order, output_location);
    }
  }
}

static std::vector<double> GetDoubles(const std::string& in) {
  std::vector<std::string> split = nc::Split(in, ",");
  std::vector<double> out;
  for (const std::string& piece : split) {
    double v;
    CHECK(nc::safe_strtod(piece, &v));
    out.emplace_back(v);
  }

  return out;
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::vector<tm_gen::TopologyAndFilename> topologies =
      tm_gen::GetTopologyInputs();

  std::vector<Input> inputs;
  for (const auto& topology : topologies) {
    for (size_t i = 0; i < FLAGS_tm_count; ++i) {
      inputs.emplace_back(&topology, i);
    }
  }

  std::vector<double> localities = GetDoubles(FLAGS_locality);
  std::vector<double> scale_factors = GetDoubles(FLAGS_min_scale_factor);

  nc::RunInParallel<Input>(
      inputs, [&localities, &scale_factors](const Input& input) {
        ProcessMatrix(input, localities, scale_factors);
      }, FLAGS_threads);
}
