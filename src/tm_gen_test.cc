#include "tm_gen.h"

#include "gtest/gtest.h"
#include <string>
#include <utility>

#include "../build/ncode-build/net.pb.h"
#include "../build/ncode-src/src/common/logging.h"
#include "../build/ncode-src/src/common/strutil.h"
#include "../build/ncode-src/src/net/net_gen.h"

namespace e2e {
namespace {

using namespace ncode::net;

static constexpr Bandwidth kBw = Bandwidth::FromBitsPerSecond(1000000);
static constexpr Delay kDelay = Delay(10);

static bool LinkLoadEq(const std::set<double>& utilizations,
                       const TrafficMatrix& tm) {
  std::set<double> link_load;
  for (const auto& link_and_load : tm.SPUtilization()) {
    double load = *link_and_load.second;
    link_load.insert(static_cast<int>(load * 1000.0) / 1000.0);
  }
  return utilizations == link_load;
}

class TmGenTest : public ::testing::Test {
 protected:
  TmGenTest()
      : graph_storage_(GenerateFullGraph(2, kBw, kDelay)),
        generator_(1, &graph_storage_) {}

  GraphStorage graph_storage_;
  TMGenerator generator_;
};

TEST_F(TmGenTest, Empty) { ASSERT_FALSE(generator_.GenerateMatrix()); }

// 0% of the links have utilization 0.5 or more.
TEST_F(TmGenTest, ZeroPercent) {
  generator_.AddUtilizationConstraint(0.0, 0.5);

  auto matrix = generator_.GenerateMatrix();
  ASSERT_TRUE(LinkLoadEq({0.0, 0.0}, *matrix));
}

// 50% of the links have utilization 0.6 or more, all links should have
// utilization of 0.1 or more.
TEST_F(TmGenTest, Half) {
  generator_.AddUtilizationConstraint(0.5, 0.6);
  generator_.AddUtilizationConstraint(1.0, 0.1);

  auto matrix = generator_.GenerateMatrix();
  ASSERT_TRUE(LinkLoadEq({0.1, 0.6}, *matrix));
}

// 60% of the links have utilization 0.6 or more, 90% links should have
// utilization of 0.1 or more. This is not exactly achievable since we only
// have 2 links, but the result should be the same as before.
TEST_F(TmGenTest, HalfApprox) {
  generator_.AddUtilizationConstraint(0.6, 0.6);
  generator_.AddUtilizationConstraint(0.9, 0.1);

  auto matrix = generator_.GenerateMatrix();
  ASSERT_TRUE(LinkLoadEq({0.1, 0.6}, *matrix));
}

// All links have utilizaion of 0.8.
TEST_F(TmGenTest, Full) {
  generator_.AddUtilizationConstraint(1.0, 0.8);

  auto matrix = generator_.GenerateMatrix();
  ASSERT_TRUE(LinkLoadEq({0.8, 0.8}, *matrix));
}

}  // namespace
}  // naemspace e2e
