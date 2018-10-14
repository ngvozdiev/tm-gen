#include "opt.h"
#include "ldr.h"

#include <gtest/gtest.h>
#include <set>

namespace tm_gen {

constexpr nc::net::Bandwidth kDefaultLinkspeed =
    nc::net::Bandwidth::FromBitsPerSecond(1000000);

static nc::net::GraphBuilder GetGraph() {
  using namespace std::chrono;
  using namespace nc::net;
  GraphBuilder builder;
  builder.AddLink({"C", "D", kDefaultLinkspeed, milliseconds(10)});
  builder.AddLink({"D", "C", kDefaultLinkspeed, milliseconds(10)});
  builder.AddLink({"A", "B", kDefaultLinkspeed, milliseconds(10)});
  builder.AddLink({"B", "A", kDefaultLinkspeed, milliseconds(10)});
  builder.AddLink({"A", "D", kDefaultLinkspeed, milliseconds(6)});
  builder.AddLink({"D", "A", kDefaultLinkspeed, milliseconds(6)});
  builder.AddLink({"B", "C", kDefaultLinkspeed, milliseconds(5)});
  builder.AddLink({"C", "B", kDefaultLinkspeed, milliseconds(5)});

  return builder;
}

class TestBase : public ::testing::Test {
 protected:
  TestBase() : graph_(GetGraph()) {
    path_provider_ = nc::make_unique<PathProvider>(&graph_);
  }

  // Given an output and a path will check if the output contains the path and
  // sends a given fraction down that path.
  bool HasPath(const RoutingConfiguration& output, const std::string& path_str,
               double fraction) {
    std::unique_ptr<nc::net::Walk> path = graph_.WalkFromStringOrDie(path_str);
    AggregateId id(path->FirstHop(graph_), path->LastHop(graph_));

    for (const auto& aggregate_and_routes : output.routes()) {
      const AggregateId& aggregate_id = aggregate_and_routes.first;
      if (aggregate_id != id) {
        continue;
      }

      const std::vector<RouteAndFraction>& routes = aggregate_and_routes.second;
      for (const auto& route_and_fraction : routes) {
        if (*route_and_fraction.first == *path) {
          double fraction_in_result = route_and_fraction.second;
          if (std::fabs(fraction_in_result - fraction) < 0.01) {
            return true;
          }
        }
      }
    }

    return false;
  }

  void AddAggregate(const std::string& src, const std::string& dst,
                    uint32_t num_flows, nc::net::Bandwidth total_volume,
                    TrafficMatrix* tm) {
    AggregateId id(graph_.NodeFromStringOrDie(src),
                   graph_.NodeFromStringOrDie(dst));
    tm->AddDemand(id, {total_volume, num_flows});
  }

  std::unique_ptr<PathProvider> path_provider_;
  nc::net::GraphStorage graph_;
};

class ShortestPathTest : public TestBase {
 protected:
  ShortestPathTest() : sp_optimizer_(path_provider_.get()), tm_(&graph_) {}

  void AddSPAggregate(const std::string& src, const std::string& dst) {
    AddAggregate(src, dst, 1, kDefaultLinkspeed, &tm_);
  }

  ShortestPathOptimizer sp_optimizer_;
  TrafficMatrix tm_;
};

TEST_F(ShortestPathTest, SingleAggregate) {
  AddSPAggregate("A", "B");
  auto routing = sp_optimizer_.Optimize(tm_);
  ASSERT_TRUE(HasPath(*routing, "[A->B]", 1.0));
}

TEST_F(ShortestPathTest, TwoAggregates) {
  AddSPAggregate("A", "B");
  AddSPAggregate("A", "C");
  auto routing = sp_optimizer_.Optimize(tm_);
  ASSERT_TRUE(HasPath(*routing, "[A->B]", 1.0));
  ASSERT_TRUE(HasPath(*routing, "[A->B, B->C]", 1.0));
}

class B4Test : public TestBase {
 protected:
  B4Test() : tm_(&graph_) {}

  void AddAggregate(const std::string& src, const std::string& dst,
                    nc::net::Bandwidth bw) {
    TestBase::AddAggregate(src, dst, 1, bw, &tm_);
  }

  TrafficMatrix tm_;
};

// Fits on the shortest path.
TEST_F(B4Test, SinglePath) {
  B4Optimizer b4_optimizer(path_provider_.get(), false, 1.0);
  AddAggregate("A", "B", kDefaultLinkspeed);
  auto routing = b4_optimizer.Optimize(tm_);

  ASSERT_TRUE(HasPath(*routing, "[A->B]", 1.0));
}

// Does not fit on the shortest path.
TEST_F(B4Test, SinglePathNoFitLowerCapacity) {
  B4Optimizer b4_optimizer(path_provider_.get(), false, 1.0);
  AddAggregate("A", "B", kDefaultLinkspeed * 1.2);
  auto routing = b4_optimizer.Optimize(tm_);

  ASSERT_TRUE(HasPath(*routing, "[A->B]", 0.833));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C, C->B]", 0.166));
}

// A couple of aggregates that do not fit on the shortest path
TEST_F(B4Test, MultiSinglePathNoFit) {
  B4Optimizer b4_optimizer(path_provider_.get(), false, 1.0);
  AddAggregate("A", "B", kDefaultLinkspeed * 0.6);
  AddAggregate("A", "C", kDefaultLinkspeed * 0.6);
  auto routing = b4_optimizer.Optimize(tm_);

  ASSERT_TRUE(HasPath(*routing, "[A->B]", 1 / 1.2));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C, C->B]", 0.2 / 1.2));
  ASSERT_TRUE(HasPath(*routing, "[A->B, B->C]", 1 / 1.2));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C]", 0.2 / 1.2));
}

// A couple of aggregates, one does not fit.
TEST_F(B4Test, MultiSinglePathOneNoFit) {
  B4Optimizer b4_optimizer(path_provider_.get(), false, 1.0);
  AddAggregate("A", "B", kDefaultLinkspeed * 0.3);
  AddAggregate("A", "C", kDefaultLinkspeed * 0.9);
  auto routing = b4_optimizer.Optimize(tm_);

  // Should be the same as in the previous test!
  ASSERT_TRUE(HasPath(*routing, "[A->B]", 1 / 1.2));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C, C->B]", 0.2 / 1.2));
  ASSERT_TRUE(HasPath(*routing, "[A->B, B->C]", 1 / 1.2));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C]", 0.2 / 1.2));
}

// Traffic does not fit.
TEST_F(B4Test, NoFit) {
  B4Optimizer b4_optimizer(path_provider_.get(), false, 1.0);
  AddAggregate("A", "B", kDefaultLinkspeed * 3);
  auto routing = b4_optimizer.Optimize(tm_);

  ASSERT_TRUE(HasPath(*routing, "[A->B]", 0.5));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C, C->B]", 0.5));
}

class MinMaxTest : public TestBase {
 protected:
  MinMaxTest() : tm_(&graph_) {}

  void AddAggregate(const std::string& src, const std::string& dst,
                    nc::net::Bandwidth bw) {
    TestBase::AddAggregate(src, dst, 1, bw, &tm_);
  }

  TrafficMatrix tm_;
};

// A single aggregate with 2 disjoint paths through the network. The optimizer
// should split traffic evenly among the two paths (given enough capacity).
TEST_F(MinMaxTest, SingleAggregate) {
  MinMaxOptimizer minmax_optimizer(path_provider_.get(), 1.0, false);
  AddAggregate("A", "B", kDefaultLinkspeed);
  auto routing = minmax_optimizer.Optimize(tm_);

  ASSERT_TRUE(HasPath(*routing, "[A->B]", 0.5));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C, C->B]", 0.5));
}

TEST_F(MinMaxTest, TwoAggregates) {
  MinMaxOptimizer minmax_optimizer(path_provider_.get(), 1.0, false);
  AddAggregate("A", "B", kDefaultLinkspeed);
  AddAggregate("A", "D", kDefaultLinkspeed);
  auto routing = minmax_optimizer.Optimize(tm_);

  // Traffic should be split among the 2 shortest paths.
  ASSERT_TRUE(HasPath(*routing, "[A->B]", 1.0));
  ASSERT_TRUE(HasPath(*routing, "[A->D]", 1.0));
}

TEST_F(MinMaxTest, TwoAggregatesNoFit) {
  MinMaxOptimizer minmax_optimizer(path_provider_.get(), 1.0, false);
  AddAggregate("A", "B", kDefaultLinkspeed * 2);
  AddAggregate("A", "D", kDefaultLinkspeed);
  auto routing = minmax_optimizer.Optimize(tm_);

  // The optimizer should equalize overload:
  // 0.75 * 2 is 1.5 and 1 + 0.25 * 2 is also 1.5.
  ASSERT_TRUE(HasPath(*routing, "[A->B]", 0.75));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C, C->B]", 0.25));
  ASSERT_TRUE(HasPath(*routing, "[A->D]", 1.0));
}

class LDRTest : public TestBase {
 protected:
  LDRTest() : tm_(&graph_) {}

  void AddAggregate(const std::string& src, const std::string& dst,
                    nc::net::Bandwidth bw) {
    TestBase::AddAggregate(src, dst, 1, bw, &tm_);
  }

  TrafficMatrix tm_;
};

TEST_F(LDRTest, SingleAggregateFit) {
  LDROptimizer ldr_optimizer(path_provider_.get(), 1.0, true, false);
  AddAggregate("A", "C", kDefaultLinkspeed);
  auto routing = ldr_optimizer.Optimize(tm_);

  ASSERT_TRUE(HasPath(*routing, "[A->B, B->C]", 1.0));
}

TEST_F(LDRTest, SingleAggregateFitTwoPaths) {
  LDROptimizer ldr_optimizer(path_provider_.get(), 1.0, true, false);
  AddAggregate("A", "C", kDefaultLinkspeed * 2);
  auto routing = ldr_optimizer.Optimize(tm_);

  ASSERT_TRUE(HasPath(*routing, "[A->B, B->C]", 0.5));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C]", 0.5));
}

// Will set the link multiplier to 0.9. This should cause the shortest path to
// be loaded up to kDefaultLinkspeed * 0.9 and the rest of the aggregate to end
// up on the second best path. There should be no oversubscription.
TEST_F(LDRTest, SingleAggregateTwoPaths) {
  LDROptimizer ldr_optimizer(path_provider_.get(), 1.0, true, false);
  AddAggregate("A", "B", kDefaultLinkspeed * 1.2);
  auto routing = ldr_optimizer.Optimize(tm_);

  ASSERT_TRUE(HasPath(*routing, "[A->B]", 1 / 1.2));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C, C->B]", 0.2 / 1.2));
}

TEST_F(LDRTest, SingleAggregateNoFitTwoPaths) {
  LDROptimizer ldr_optimizer(path_provider_.get(), 1.0, true, false);
  AddAggregate("A", "C", kDefaultLinkspeed * 3);
  auto routing = ldr_optimizer.Optimize(tm_);

  ASSERT_TRUE(HasPath(*routing, "[A->B, B->C]", 0.5));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C]", 0.5));
}

TEST_F(LDRTest, FourAggregates) {
  LDROptimizer ldr_optimizer(path_provider_.get(), 1.0, true, false);
  AddAggregate("A", "B", nc::net::Bandwidth::FromKBitsPerSecond(1500));
  AddAggregate("B", "A", nc::net::Bandwidth::FromKBitsPerSecond(200));
  AddAggregate("C", "D", nc::net::Bandwidth::FromKBitsPerSecond(1500));
  AddAggregate("D", "C", nc::net::Bandwidth::FromKBitsPerSecond(200));
  auto routing = ldr_optimizer.Optimize(tm_);

  ASSERT_TRUE(HasPath(*routing, "[A->B]", 0.666));
  ASSERT_TRUE(HasPath(*routing, "[A->D, D->C, C->B]", 0.333));
  ASSERT_TRUE(HasPath(*routing, "[C->B, B->A, A->D]", 0.333));
  ASSERT_TRUE(HasPath(*routing, "[C->D]", 0.666));
  ASSERT_TRUE(HasPath(*routing, "[B->A]", 1.0));
  ASSERT_TRUE(HasPath(*routing, "[D->C]", 1.0));

  auto routing_two = ldr_optimizer.Optimize(tm_);
  ASSERT_TRUE(HasPath(*routing_two, "[A->B]", 0.666));
}

}  // namespace tm_gen
