#include <gflags/gflags.h>
#include <ncode/file.h>
#include <ncode/logging.h>
#include <ncode/lp/demand_matrix.h>
#include <ncode/map_util.h>
#include <ncode/net/net_common.h>
#include <ncode/strutil.h>
#include <ncode/thread_runner.h>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "demand_matrix_input.h"
#include "opt/common.h"
#include "opt/ldr.h"
#include "opt/opt.h"
#include "opt/path_provider.h"
#include "topology_input.h"

using namespace std::chrono;

DEFINE_uint64(threads, 4, "Number of parallel threads to run");
DEFINE_bool(run_ldr_link_based, false, "Also run a link-based version of LDR");
DEFINE_bool(run_headroom_vs_delay, false, "Explore headroom vs delay");
DEFINE_bool(skip_trivial, false, "Skips trivially satisfiable TMs");
DEFINE_bool(force, false, "Will regenerate existing solutions.");
DEFINE_double(link_capacity_multiplier, 1.0, "Link capacity multiplier.");

struct Input {
  const tm_gen::DemandMatrixAndFilename* demand_matrix_and_filename;
  const std::vector<std::string>* node_order;
};

namespace tm_gen {

static constexpr double kLinkScaleStride = 0.02;

static std::vector<double> GetHeadroomVsDelay(const TrafficMatrix& tm) {
  using namespace std::chrono;
  const nc::net::GraphStorage* graph = tm.graph();
  PathProvider path_provider(graph);

  std::vector<double> out;
  for (double link_capacity_multiplier = 1.0; link_capacity_multiplier > 0;
       link_capacity_multiplier -= kLinkScaleStride) {
    if (!tm.ToDemandMatrix()->IsFeasible({}, link_capacity_multiplier)) {
      break;
    }

    LDROptimizer ldr_optimizer(&path_provider, link_capacity_multiplier, false,
                               false);
    std::unique_ptr<RoutingConfiguration> routing = ldr_optimizer.Optimize(tm);
    nc::net::Delay total_delay = routing->TotalPerFlowDelay();
    double value = duration_cast<milliseconds>(total_delay).count() / 1000.0;
    out.emplace_back(value);
  }

  return out;
}

static std::string GetFilename(const std::string& tm_file,
                               const std::string opt_string) {
  std::string tm_base = nc::StringReplace(tm_file, ".demands", "", true);
  std::string link_capacity_str =
      FLAGS_link_capacity_multiplier == 1.0
          ? ""
          : nc::StrCat("_lm_", nc::ToStringMaxDecimals(
                                   FLAGS_link_capacity_multiplier, 2));
  return nc::StrCat(tm_base, link_capacity_str, "_", opt_string, ".rc");
}

static void RecordRoutingConfig(const std::string& out,
                                const std::vector<std::string>& node_order,
                                const RoutingConfiguration& routing) {
  std::string serialized = routing.SerializeToText(node_order);
  LOG(INFO) << "Will write routing config to " << out;
  nc::File::WriteStringToFileOrDie(serialized, out);
}

static void OptAndRecord(const std::string& tm_file, const TrafficMatrix& tm,
                         const std::vector<std::string>& node_order,
                         const std::string opt_string, Optimizer* opt) {
  std::string out = GetFilename(tm_file, opt_string);
  if (!FLAGS_force && nc::File::Exists(out)) {
    LOG(INFO) << "File exists " << out << ", will skip";
    return;
  }

  auto rc = opt->OptimizeWithTimeAndOptString(tm, opt_string);
  RecordRoutingConfig(out, node_order, *rc);
}

static void RunOptimizers(const Input& input) {
  const std::string& top_file = input.demand_matrix_and_filename->topology_file;
  const std::string& tm_file = input.demand_matrix_and_filename->file;
  nc::lp::DemandMatrix* demand_matrix =
      input.demand_matrix_and_filename->demand_matrix.get();
  const nc::net::GraphStorage* graph = demand_matrix->graph();
  const std::vector<std::string>& node_order = *(input.node_order);
  LOG(ERROR) << "Running " << top_file << " " << tm_file;

  std::unique_ptr<TrafficMatrix> tm =
      TrafficMatrix::DistributeFromDemandMatrix(*demand_matrix);
  if (FLAGS_run_headroom_vs_delay &&
      !nc::ContainsKey(demand_matrix->properties(), "headroom_vs_delay")) {
    std::vector<double> headroom_vs_delay = GetHeadroomVsDelay(*tm);
    std::string to_record = nc::Join(headroom_vs_delay, ",");

    demand_matrix->UpdateProperty("headroom_vs_delay", to_record);
    demand_matrix->UpdateProperty("headroom_vs_delay_step",
                                  nc::StrCat(kLinkScaleStride));
    LOG(INFO) << "Will overwrite " << tm_file << " with headroom vs delay info";
    demand_matrix->ToRepetitaFileOrDie(node_order, tm_file);
  }

  PathProvider path_provider(graph);
  LDROptimizer ldr_optimizer(&path_provider, FLAGS_link_capacity_multiplier,
                             false, false);
  LDROptimizer ldr_optimizer_no_flow_counts(
      &path_provider, FLAGS_link_capacity_multiplier, false, true);
  MinMaxOptimizer minmax_low_delay_optimizer(
      &path_provider, FLAGS_link_capacity_multiplier, true);
  MinMaxPathBasedOptimizer minmax_ksp_optimizer(
      &path_provider, FLAGS_link_capacity_multiplier, true, 10);
  B4Optimizer b4_optimizer(&path_provider, false,
                           FLAGS_link_capacity_multiplier);
  LDRLinkBased ldr_link_based_optimizer(&path_provider,
                                        FLAGS_link_capacity_multiplier);
  ShortestPathOptimizer sp_optimizer(&path_provider);

  OptAndRecord(tm_file, *tm, node_order, "LDRNOCACHE", &ldr_optimizer);
  OptAndRecord(tm_file, *tm, node_order, "LDR", &ldr_optimizer);
  OptAndRecord(tm_file, *tm, node_order, "LDRNFC",
               &ldr_optimizer_no_flow_counts);
  OptAndRecord(tm_file, *tm, node_order, "MinMaxLD",
               &minmax_low_delay_optimizer);
  OptAndRecord(tm_file, *tm, node_order, "MinMaxK10", &minmax_ksp_optimizer);
  OptAndRecord(tm_file, *tm, node_order, "B4", &b4_optimizer);
  OptAndRecord(tm_file, *tm, node_order, "SP", &sp_optimizer);
  if (FLAGS_run_ldr_link_based) {
    OptAndRecord(tm_file, *tm, node_order, "LDRLB", &ldr_link_based_optimizer);
  }
}

}  // namespace tm_gen

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::vector<tm_gen::TopologyAndFilename> topologies;
  std::vector<tm_gen::DemandMatrixAndFilename> matrices;
  std::tie(topologies, matrices) =
      tm_gen::GetDemandMatrixInputs(FLAGS_skip_trivial);

  std::map<std::string, const tm_gen::TopologyAndFilename*> topologies_by_name;
  for (const auto& topology : topologies) {
    topologies_by_name[topology.file] = &topology;
  }

  std::vector<Input> to_process;
  for (const auto& matrix : matrices) {
    const std::string& top_file = matrix.topology_file;
    const tm_gen::TopologyAndFilename* top_ptr =
        nc::FindOrDie(topologies_by_name, top_file);
    to_process.push_back({&matrix, &top_ptr->node_order});
  }

  nc::RunInParallel<Input>(
      to_process, [](const Input& input) { tm_gen::RunOptimizers(input); },
      FLAGS_threads);
}
