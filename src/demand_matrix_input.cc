#include "demand_matrix_input.h"

#include <gflags/gflags.h>
#include <stddef.h>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <string>

#include "ncode/common.h"
#include "ncode/file.h"
#include "ncode/logging.h"
#include "ncode/net/net_common.h"
#include "ncode/net/net_gen.h"
#include "ncode/strutil.h"
#include "topology_input.h"

DEFINE_string(tm_root, "", "Root for traffic matrix file(s). Required.");
DEFINE_string(
    topology_root, "/usr/local/tm-gen/topologies",
    "Root for topologies. If empty will be set to the same as tm_root.");
DEFINE_uint64(tm_per_topology, std::numeric_limits<uint64_t>::max(),
              "How many TMs to choose for each topology");
DEFINE_uint64(seed, 1,
              "Seed used when choosing TMs for each topology. Set to 0 to "
              "disable random choice of TMs.");
DEFINE_uint64(topology_size_limit, 100000,
              "Topologies with size more than this will be skipped");
DEFINE_uint64(topology_delay_limit_ms, 10,
              "Topologies with diameter less than this limit will be skipped");

static constexpr char kDemandsExtension[] = ".demands";
static constexpr char kTopologyExtension[] = ".graph";

using namespace std::chrono;

namespace tm_gen {

static std::string FindTopologyFileForDemandFileOrDie(
    const std::string& demand_file,
    const std::vector<std::string>& topology_files) {
  std::string base = nc::File::ExtractFileName(demand_file);
  std::string demand_base =
      nc::StringReplace(base, kDemandsExtension, kTopologyExtension, true);

  std::string to_return = "";
  for (const auto& file : topology_files) {
    std::string top_base = nc::File::ExtractFileName(file);
    top_base = nc::StringReplace(top_base, kTopologyExtension, "", true);
    top_base = nc::StrCat(top_base, "_");

    if (nc::HasPrefixString(demand_base, top_base) ||
        nc::HasSuffixString(demand_base, top_base)) {
      if (to_return.size() < file.size()) {
        to_return = file;
      }
    }
  }

  CHECK(to_return != "") << "Unable to find topology for file " << demand_file;
  return to_return;
}

static std::map<std::string, std::vector<std::string>> GetTopologyToDemands(
    size_t* topology_count, size_t* tm_count) {
  std::string tm_root = FLAGS_tm_root;
  CHECK(tm_root != "");
  std::string top_root =
      FLAGS_topology_root == "" ? FLAGS_tm_root : FLAGS_topology_root;

  std::vector<std::string> traffic_matrices =
      nc::File::GetFilesWithExtension(tm_root, kDemandsExtension);
  std::vector<std::string> topologies =
      nc::File::GetFilesWithExtension(top_root, kTopologyExtension);

  std::map<std::string, std::vector<std::string>> out;
  for (const std::string& tm_file : traffic_matrices) {
    std::string topology_file =
        FindTopologyFileForDemandFileOrDie(tm_file, topologies);
    out[topology_file].emplace_back(tm_file);
  }
  *tm_count = traffic_matrices.size();
  *topology_count = out.size();

  return out;
}

std::pair<std::vector<TopologyAndFilename>,
          std::vector<DemandMatrixAndFilename>>
GetDemandMatrixInputs(bool skip_trivial) {
  // Inputs.
  std::vector<TopologyAndFilename> topologies_out;
  std::vector<DemandMatrixAndFilename> demands_out;

  // Randomness to pick tms for each topology.
  std::mt19937 rnd(FLAGS_seed);

  size_t topology_count;
  size_t tm_count;
  std::map<std::string, std::vector<std::string>> topology_to_demands =
      GetTopologyToDemands(&topology_count, &tm_count);
  LOG(INFO) << "Will load " << tm_count << " TMs from " << topology_count
            << " topologies";
  for (auto& topology_and_demands : topology_to_demands) {
    const std::string& topology_file = topology_and_demands.first;
    std::vector<std::string>& demands_for_topology =
        topology_and_demands.second;

    std::vector<std::string> node_order;
    nc::net::GraphBuilder builder = nc::net::LoadRepetitaOrDie(
        nc::File::ReadFileToStringOrDie(topology_file), &node_order);
    builder.RemoveMultipleLinks();

    auto graph = nc::make_unique<nc::net::GraphStorage>(builder);
    nc::net::GraphStats stats = graph->Stats();
    if (stats.isPartitioned()) {
      LOG(INFO) << "Skipping " << topology_file << " graph partitioned ";
      continue;
    }

    size_t node_count = graph->AllNodes().Count();
    if (node_count > FLAGS_topology_size_limit) {
      LOG(INFO) << "Skipping " << topology_file << " / " << topology_file
                << " size limit " << FLAGS_topology_size_limit << " vs "
                << node_count;
      continue;
    }

    nc::net::Delay diameter = duration_cast<nc::net::Delay>(
        microseconds(stats.sp_delays_micros.Max()));
    if (diameter < milliseconds(FLAGS_topology_delay_limit_ms)) {
      LOG(INFO) << "Skipping " << topology_file << " / " << topology_file
                << " delay limit " << FLAGS_topology_delay_limit_ms
                << "ms vs diameter delay "
                << duration_cast<milliseconds>(diameter).count() << "ms";
      continue;
    }

    if (FLAGS_seed != 0) {
      std::shuffle(demands_for_topology.begin(), demands_for_topology.end(),
                   rnd);
    }

    demands_for_topology.resize(
        std::min(demands_for_topology.size(),
                 static_cast<size_t>(FLAGS_tm_per_topology)));
    for (const std::string& demand_file : demands_for_topology) {
      std::unique_ptr<nc::lp::DemandMatrix> demand_matrix =
          nc::lp::DemandMatrix::LoadRepetitaFileOrDie(demand_file, node_order,
                                                      graph.get());
      if (!demand_matrix) {
        LOG(INFO) << "Skipping " << demand_file << " inconsistent TM";
        continue;
      }

      if (skip_trivial) {
        const std::map<std::string, std::string>& props =
            demand_matrix->properties();

        bool trivial;
        if (nc::ContainsKey(props, "trivial")) {
          const std::string& trivial_str = nc::FindOrDie(props, "trivial");
          trivial = trivial_str == "true";
        } else {
          trivial = demand_matrix->IsTriviallySatisfiable();
        }

        if (trivial) {
          LOG(INFO) << "Skipping " << demand_file << " trivially satisfiable";
          continue;
        }
      }

      demands_out.emplace_back(topology_file, demand_file,
                               std::move(demand_matrix));
    }

    topologies_out.emplace_back(std::move(graph), node_order, topology_file);
  }

  return {std::move(topologies_out), std::move(demands_out)};
}

}  // namespace tm_gen
