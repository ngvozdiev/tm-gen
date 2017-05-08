#include <gflags/gflags.h>
#include <stddef.h>
#include <stdint.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "e2e_input.pb.h"
#include "ncode-build/net.pb.h"
#include "ncode-src/src/common/common.h"
#include "ncode-src/src/common/file.h"
#include "ncode-src/src/common/logging.h"
#include "ncode-src/src/common/perfect_hash.h"
#include "ncode-src/src/common/strutil.h"
#include "ncode-src/src/grapher/grapher.h"
#include "ncode-src/src/net/algorithm.h"
#include "ncode-src/src/net/net_common.h"
#include "ncode-src/src/net/net_gen.h"
#include "ncode-src/src/net/path_cache.h"
#include "fubar/b4.h"
#include "fubar/fubar.h"
#include "fubar/fubar_common.h"
#include "fubar/minmax.h"
#include "fubar/sp.h"
#include "pcap_data.h"
#include "tm_gen.h"

using namespace e2e;
using namespace ncode::net;

DEFINE_uint64(seed, 1, "Seed to use when running.");
DEFINE_uint64(count, 1, "Traffic matrices to generate.");
DEFINE_string(out_dir, "tm_gen_out", "Output directory");
DEFINE_double(matrix_scale, 1.2, "By how much to scale each matrix.");
DEFINE_uint64(tcp_short_flow_object_size_bytes, 50000ul,
              "Size of each object that the short flows will transmit.");
DEFINE_uint64(
    tcp_short_flow_wait_time_ms, 5000,
    "Average time a short-flow source will wait before transmission.");
DEFINE_uint64(tcp_short_flow_access_link_rate_bps, 10000000,
              "Access link rate for short flows");
DEFINE_bool(only_short_flows, false,
            "If true only short TCP flows will be added, the big .pcap traces "
            "will not be added.");
DEFINE_uint64(short_flow_source_chunk, 100000000,
              "For each aggregate max(1, total_rate / short_flow_chunk) short "
              "flow sources will be added.");
DEFINE_string(pcap_trace_store, "",
              "A file with information about .pcap traces");
DEFINE_string(pcap_trace_tag_regex, "",
              "Will only consider traces that match this regex");
DEFINE_string(topology, "NTT", "Can be one of HE,NTT,RF");
DEFINE_double(topology_default_bw_gbps, 10.0,
              "Default link speed if the topology does not have link speeds.");
DEFINE_string(
    base_config_output, "",
    "If not empty will generate and save a BaseConfig into this file.");
DEFINE_double(max_global_utilization, std::numeric_limits<double>::max(),
              "Max global utilization.");
DEFINE_bool(isolate_largest, false,
            "If true will generate a single-aggregate traffic matrix.");

static void InitShortFlows(const std::string& src, const std::string& dst,
                           size_t count, e2e::DataStream* data_stream) {
  data_stream->set_source(src);
  data_stream->set_destination(dst);

  e2e::TCPFlowGroup* tcp_flow_group = data_stream->add_flow_groups();
  tcp_flow_group->set_internal_delay_ms(50);
  tcp_flow_group->set_min_delay_ms(5);
  tcp_flow_group->set_max_delay_ms(20);
  tcp_flow_group->set_mean_object_size_bytes(
      FLAGS_tcp_short_flow_object_size_bytes);
  tcp_flow_group->set_mean_object_size_fixed(true);
  tcp_flow_group->set_mean_wait_time_ms(FLAGS_tcp_short_flow_wait_time_ms);
  tcp_flow_group->set_num_flows(count);
  tcp_flow_group->set_rate_spread(0.3);
  tcp_flow_group->set_random_access_link_queue(false);

  // All TCP flows will start 10sec into the simulation. This avoids the case
  // where a whole initial congestion window is lost due to lack of installed
  // rules on the devices, and then being recovered very slowly due to not
  // simulating a proper retx queue.
  tcp_flow_group->set_initial_time_offset_ms(10000);

  e2e::RateKeyFrame* key_frame = tcp_flow_group->add_key_frames();
  key_frame->set_time_ms(10);
  key_frame->set_rate_bps(FLAGS_tcp_short_flow_access_link_rate_bps * count);
}

// Initializes an e2e::DataStream with a pcap data trace
static void InitAggregate(const std::vector<TraceId>& all_ids,
                          const std::string& src, const std::string& dst,
                          Bandwidth rate, PcapTraceStore* trace_store,
                          e2e::DataStream* data_stream) {
  using namespace std::chrono;
  data_stream->set_source(src);
  data_stream->set_destination(dst);

  std::vector<TraceId> ids = trace_store->IdsWithMaxRate(
      all_ids, rate, milliseconds(100), milliseconds(10000));
  CHECK(!ids.empty());

  if (!FLAGS_only_short_flows) {
    uint64_t mean_total = 0;
    uint64_t max_total = 0;
    for (const TraceId& trace_id : ids) {
      PcapDataTrace& data_trace = trace_store->GetTraceOrDie(trace_id);
      data_trace.BinCache({milliseconds(100), milliseconds(10000)},
                          milliseconds(60000));

      tldr::AggregateHistory aggregate_history = data_trace.InitialHistoryOrDie(
          milliseconds(100), milliseconds(10000));
      mean_total += aggregate_history.mean_rate().bps();
      max_total += aggregate_history.max_rate().bps();

      *data_stream->add_pcap_data_streams() = trace_id.ToProtobuf();
    }
    LOG(ERROR) << src << " -> " << dst << " " << rate.bps() << " vs "
               << mean_total << " max " << max_total;
    CHECK(rate.bps() >= max_total);
  }

  size_t short_flow_count = std::max(
      static_cast<uint64_t>(1), rate.bps() / FLAGS_short_flow_source_chunk);
  InitShortFlows(src, dst, short_flow_count, data_stream);
}

static e2e::BaseConfig ToBaseConfig(const PBNet& net, const TrafficMatrix& tm,
                                    PcapTraceStore* trace_store) {
  e2e::BaseConfig base_config;
  base_config.set_fraction_update_empty_threshold(0.05);
  base_config.set_fraction_update_absolute_threshold(0.01);
  base_config.set_switch_poll_period_ms(100);

  *base_config.mutable_topology() = net;
  base_config.set_topology_queue_depth_ms(100);

  // Have to be careful when assigning addresses not to collide with any
  // real-world addresses from the trace.
  base_config.set_device_ip_address_base(
      ncode::net::StringToIPOrDie("10.1.0.1").Raw());
  base_config.set_tldr_ip_address_base(
      ncode::net::StringToIPOrDie("10.2.0.1").Raw());
  base_config.set_controller_ip_address(
      ncode::net::StringToIPOrDie("10.3.0.1").Raw());
  base_config.set_tcp_source_ip_address_base(
      ncode::net::StringToIPOrDie("10.4.0.1").Raw());

  base_config.set_default_enter_port(5000);
  base_config.set_default_tldr_input_port(6000);
  base_config.set_min_delay_controller_tldr_ms(10);
  base_config.set_max_delay_controller_tldr_ms(100);
  base_config.set_min_delay_tldr_device_ms(1);
  base_config.set_max_delay_tldr_device_ms(10);
  base_config.set_tldr_period_ms(60000);

  std::mt19937 rnd(1);
  std::vector<TraceId> ids = trace_store->AllIds(FLAGS_pcap_trace_tag_regex);
  const GraphStorage* graph_storage = tm.graph_storage();
  for (const TrafficMatrixElement& element : tm.elements()) {
    const std::string& src = graph_storage->GetNode(element.src)->id();
    const std::string& dst = graph_storage->GetNode(element.dst)->id();

    e2e::DataStream* stream = base_config.add_data_streams();
    std::shuffle(ids.begin(), ids.end(), rnd);
    InitAggregate(ids, src, dst, element.load, trace_store, stream);
  }

  return base_config;
}

static std::unique_ptr<TrafficMatrix> GetMatrix(
    size_t seed, ncode::net::GraphStorage* graph_storage) {
  TMGenerator generator(seed, graph_storage);

  generator.AddUtilizationConstraint(0.1283987915407855, 0.0);
  generator.AddUtilizationConstraint(0.1521299093655589, 0.011384075954554769);
  generator.AddUtilizationConstraint(0.16832326283987917, 0.032535689078117526);
  generator.AddUtilizationConstraint(0.18342900302114803, 0.04879214954122174);
  generator.AddUtilizationConstraint(0.2006948640483384, 0.07481614717333394);
  generator.AddUtilizationConstraint(0.21255287009063445, 0.08781676191343549);
  generator.AddUtilizationConstraint(0.22658610271903323, 0.09920083786799026);
  generator.AddUtilizationConstraint(0.24169184290030213, 0.11058491382254503);
  generator.AddUtilizationConstraint(0.25463746223564954, 0.1219689897770998);
  generator.AddUtilizationConstraint(0.2632779456193353, 0.12848068122310513);
  generator.AddUtilizationConstraint(0.2805287009063444, 0.1398647571776599);
  generator.AddUtilizationConstraint(0.2934894259818731, 0.14960952619475876);
  generator.AddUtilizationConstraint(0.30534743202416914, 0.1561212176407641);
  generator.AddUtilizationConstraint(0.32045317220543806, 0.1642494478723162);
  generator.AddUtilizationConstraint(0.33555891238670693, 0.17563352382687097);
  generator.AddUtilizationConstraint(0.3474320241691843, 0.185401060995879);
  generator.AddUtilizationConstraint(0.3636102719033233, 0.19352929122743107);
  generator.AddUtilizationConstraint(0.37441087613293056, 0.19840167573598053);
  generator.AddUtilizationConstraint(0.38741691842900305, 0.20652990596753262);
  generator.AddUtilizationConstraint(0.39706948640483386, 0.21304159741353793);
  generator.AddUtilizationConstraint(0.4100151057401813, 0.22280913458254595);
  generator.AddUtilizationConstraint(0.41864048338368576, 0.2341932105371007);
  generator.AddUtilizationConstraint(0.4326737160120846, 0.24232144076865283);
  generator.AddUtilizationConstraint(0.44669184290030206, 0.25532205550875436);
  generator.AddUtilizationConstraint(0.4574924471299094, 0.2650895926777624);
  generator.AddUtilizationConstraint(0.4639577039274924, 0.268345438400765);
  generator.AddUtilizationConstraint(0.47690332326283985, 0.2829853600783225);
  generator.AddUtilizationConstraint(0.4909365558912387, 0.29111359030987455);
  generator.AddUtilizationConstraint(0.5017220543806646, 0.2976025136039708);
  generator.AddUtilizationConstraint(0.5146676737160121, 0.3089865895585256);
  generator.AddUtilizationConstraint(0.5254682779456193, 0.31549828100453087);
  generator.AddUtilizationConstraint(0.5330211480362538, 0.32688235695908563);
  generator.AddUtilizationConstraint(0.5448791540785498, 0.33664989412809365);
  generator.AddUtilizationConstraint(0.5535196374622356, 0.3447781243596458);
  generator.AddUtilizationConstraint(0.5621450151057401, 0.35290635459119785);
  generator.AddUtilizationConstraint(0.5718580060422961, 0.3610345848227499);
  generator.AddUtilizationConstraint(0.5837311178247734, 0.37567450650030737);
  generator.AddUtilizationConstraint(0.592356495468278, 0.38705858245486213);
  generator.AddUtilizationConstraint(0.6031570996978852, 0.39844265840941695);
  generator.AddUtilizationConstraint(0.6117824773413897, 0.4082101955784249);
  generator.AddUtilizationConstraint(0.6247280966767371, 0.42282734910407327);
  generator.AddUtilizationConstraint(0.6355135951661631, 0.4407231165046333);
  generator.AddUtilizationConstraint(0.649546827794562, 0.45210719245918807);
  generator.AddUtilizationConstraint(0.6614199395770393, 0.4683636529222923);
  generator.AddUtilizationConstraint(0.6700453172205438, 0.4781311900913003);
  generator.AddUtilizationConstraint(0.6840785498489427, 0.4943876505544045);
  generator.AddUtilizationConstraint(0.6948640483383686, 0.5074110334464152);
  generator.AddUtilizationConstraint(0.7088972809667674, 0.5285398784180688);
  generator.AddUtilizationConstraint(0.7218429003021148, 0.5431798000956263);
  generator.AddUtilizationConstraint(0.7326283987915407, 0.5626921062817332);
  generator.AddUtilizationConstraint(0.7531268882175226, 0.5887161039138453);
  generator.AddUtilizationConstraint(0.7682326283987916, 0.6082284100999522);
  generator.AddUtilizationConstraint(0.7768731117824773, 0.6277407162860591);
  generator.AddUtilizationConstraint(0.786570996978852, 0.6456364836866192);
  generator.AddUtilizationConstraint(0.8027643504531722, 0.6651487898727261);
  generator.AddUtilizationConstraint(0.8146374622356495, 0.6911727875048382);
  generator.AddUtilizationConstraint(0.8286555891238672, 0.7155802463514037);
  generator.AddUtilizationConstraint(0.8426888217522659, 0.7367090913230573);
  generator.AddUtilizationConstraint(0.8513141993957705, 0.7546048587236175);
  generator.AddUtilizationConstraint(0.8642598187311178, 0.7741171649097243);
  generator.AddUtilizationConstraint(0.8739728096676738, 0.7952687780332871);
  generator.AddUtilizationConstraint(0.885845921450151, 0.8098859315589354);
  generator.AddUtilizationConstraint(0.8944712990936555, 0.8277816989594955);
  generator.AddUtilizationConstraint(0.9031117824773414, 0.8489333120830583);
  generator.AddUtilizationConstraint(0.9138972809667674, 0.8700621570547119);
  generator.AddUtilizationConstraint(0.9225226586102719, 0.8895744632408188);
  generator.AddUtilizationConstraint(0.9322356495468278, 0.9058536918558321);
  generator.AddUtilizationConstraint(0.9387160120845921, 0.925365998041939);
  generator.AddUtilizationConstraint(0.9451812688821752, 0.9464948430135925);
  generator.AddUtilizationConstraint(0.9559818731117825, 0.962774071628606);
  generator.AddUtilizationConstraint(0.9635347432024169, 0.9790305320917102);
  generator.AddUtilizationConstraint(1.0, 0.9790305320917102);
  //  generator.AddUtilizationConstraint(1.0, 0.8);

  generator.AddHopCountLocalityConstraint(0.3, 2);
  generator.AddHopCountLocalityConstraint(0.2, 3);
  generator.AddHopCountLocalityConstraint(0.1, 4);
  generator.AddOutgoingFractionConstraint(1.0, 0.1 / FLAGS_matrix_scale);
  if (FLAGS_max_global_utilization != std::numeric_limits<double>::max()) {
    generator.SetMaxGlobalUtilization(FLAGS_max_global_utilization);
  }

  generator.SetMinScaleFactor(1.3);
  std::unique_ptr<TrafficMatrix> matrix =
      generator.GenerateMatrix(true, FLAGS_matrix_scale);
  CHECK(matrix) << "Unable to find matrix";
  return matrix;
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  PBNet net;
  auto default_bw =
      Bandwidth::FromGBitsPerSecond(FLAGS_topology_default_bw_gbps);
  if (FLAGS_topology == "HE") {
    // All links will be 10Gbps
    net = GenerateHE(default_bw);
  } else if (FLAGS_topology == "NTT") {
    net = ncode::net::GenerateNTT(ncode::net::Delay(0), 1.5);
  } else if (FLAGS_topology == "RF") {
    net = GenerateSprint(default_bw);
  } else {
    LOG(FATAL) << "Bad topology " << FLAGS_topology;
  }

  ncode::File::CreateDir(FLAGS_out_dir, 0777);
  GraphStorage graph_storage(net);
  for (uint64_t i = 0; i < FLAGS_count; ++i) {
    auto matrix = GetMatrix(FLAGS_seed + i, &graph_storage);
    matrix = matrix->Scale(1.5);
    if (FLAGS_isolate_largest) {
      matrix = matrix->IsolateLargest();
    }

    std::string tag = ncode::StrCat(FLAGS_out_dir, "/", FLAGS_seed, "_", i);
    TMRun(*matrix, tag, &graph_storage);

    size_t node_count = graph_storage.AllNodes().Count();
    LOG(ERROR) << matrix->elements().size() << " pairs out of "
               << node_count * (node_count - 1);
    LOG(ERROR) << "GU " << matrix->SPGlobalUtilization() << " total "
               << matrix->TotalLoad().Gbps() << "Gbps";

    if (!FLAGS_base_config_output.empty()) {
      // By default the controller is assumed to try to leave 5% headroom on all
      // links, need to scale down the matrix accordingly.
      auto downscaled_matrix = matrix->Scale(0.95);
      PcapTraceStore trace_store(FLAGS_pcap_trace_store);
      BaseConfig base_config =
          ToBaseConfig(net, *downscaled_matrix, &trace_store);
      ncode::File::WriteStringToFileOrDie(base_config.SerializeAsString(),
                                          FLAGS_base_config_output);
    }
  }
}
