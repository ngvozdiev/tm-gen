#include "topology_input.h"

#include <gflags/gflags.h>
#include <stddef.h>
#include <chrono>

#include "ncode/common.h"
#include "ncode/file.h"
#include "ncode/logging.h"
#include "ncode/net/net_gen.h"
#include "ncode/perfect_hash.h"
#include "ncode/strutil.h"

DEFINE_string(topology_root, "", "Root for topologies. Required.");
DEFINE_uint64(topology_size_limit, 100000,
              "Topologies with node count more than this will be skipped");
DEFINE_uint64(topology_delay_limit_ms, 10,
              "Topologies with diameter less than this limit will be skipped");
DEFINE_uint64(topology_n_top, 100000,
              "Only the top N (in node count) topologies will be included");

namespace tm_gen {

static constexpr char kTopologyExtension[] = ".graph";

static std::vector<std::string> GetTopologyFiles() {
  std::string top_root = FLAGS_topology_root;
  CHECK(top_root != "");
  return nc::File::GetFilesWithExtension(top_root, kTopologyExtension);
}

std::vector<TopologyAndFilename> GetTopologyInputs() {
  using namespace std::chrono;

  std::vector<TopologyAndFilename> out;
  std::vector<std::string> topology_files = GetTopologyFiles();
  for (const std::string& topology_file : topology_files) {
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

    out.emplace_back(std::move(graph), node_order, topology_file);
  }

  std::sort(out.begin(), out.end(),
            [](const TopologyAndFilename& lhs, const TopologyAndFilename& rhs) {
              return lhs.graph->NodeCount() > rhs.graph->NodeCount();
            });

  std::vector<TopologyAndFilename> out_truncated;
  size_t limit = static_cast<size_t>(FLAGS_topology_n_top);
  size_t count = std::min(out.size(), limit);
  for (size_t i = 0; i < count; ++i) {
    out_truncated.emplace_back(std::move(out[i].graph), out[i].node_order,
                               out[i].file);
  }

  return out_truncated;
}

}  // namespace tm_gen
