#include "tm_gen.h"

#include <ncode/common.h>
#include <ncode/logging.h>
#include <ncode/lp/lp.h>
#include <ncode/lp/mc_flow.h>
#include <ncode/net/algorithm.h>
#include <ncode/perfect_hash.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace tm_gen {

static void AddSumConstraints(
    const nc::net::GraphNodeMap<nc::net::Bandwidth>& demands,
    const nc::net::GraphNodeMap<std::vector<nc::lp::VariableIndex>>& vars,
    std::vector<nc::lp::ProblemMatrixElement>* problem_matrix,
    nc::lp::Problem* lp_problem) {
  for (const auto& node_and_demand : demands) {
    nc::net::GraphNodeIndex node = node_and_demand.first;
    double demand_Mbps = node_and_demand.second->Mbps();

    nc::lp::ConstraintIndex constraint = lp_problem->AddConstraint();
    lp_problem->SetConstraintRange(constraint, demand_Mbps, demand_Mbps);
    for (nc::lp::VariableIndex var : vars.GetValueOrDie(node)) {
      problem_matrix->emplace_back(constraint, var, 1.0);
    }
  }
}

std::unique_ptr<nc::lp::DemandMatrix> LocalizeDemandMatrix(
    const nc::lp::DemandMatrix& seed_matrix, double fraction_allowance) {
  nc::net::GraphNodeMap<nc::net::Bandwidth> demand_out;
  nc::net::GraphNodeMap<nc::net::Bandwidth> demand_in;
  nc::net::GraphNodeMap<std::vector<nc::lp::VariableIndex>> vars_in;
  nc::net::GraphNodeMap<std::vector<nc::lp::VariableIndex>> vars_out;
  std::map<std::pair<nc::net::GraphNodeIndex, nc::net::GraphNodeIndex>,
           nc::lp::VariableIndex> all_vars;

  const nc::net::GraphStorage* graph = seed_matrix.graph();
  nc::net::AllPairShortestPath sp({}, graph->AdjacencyList(), nullptr, nullptr);

  nc::lp::Problem lp_problem(nc::lp::MINIMIZE);
  for (const auto& element : seed_matrix.elements()) {
    nc::net::GraphNodeIndex src = element.src;
    nc::net::GraphNodeIndex dst = element.dst;

    demand_out[src] += element.demand;
    demand_in[dst] += element.demand;

    nc::lp::VariableIndex var = lp_problem.AddVariable();
    vars_out[src].emplace_back(var);
    vars_in[dst].emplace_back(var);

    double element_demand = element.demand.Mbps();
    double min = element_demand * (1 - fraction_allowance);
    double max = element_demand * (1 + fraction_allowance);
    min = std::max(0.0, min);
    lp_problem.SetVariableRange(var, min, max);

    double obj_c = sp.GetDistance(element.src, element.dst).count();
    lp_problem.SetObjectiveCoefficient(var, obj_c * element_demand);
    all_vars[std::make_pair(src, dst)] = var;
  }

  std::vector<nc::lp::ProblemMatrixElement> problem_matrix;
  AddSumConstraints(demand_in, vars_in, &problem_matrix, &lp_problem);
  AddSumConstraints(demand_out, vars_out, &problem_matrix, &lp_problem);
  lp_problem.SetMatrix(problem_matrix);

  std::unique_ptr<nc::lp::Solution> solution = lp_problem.Solve();
  CHECK(solution->type() == nc::lp::OPTIMAL ||
        solution->type() == nc::lp::FEASIBLE);

  std::vector<nc::lp::DemandMatrixElement> new_elements;
  for (const auto& src_and_dst_and_var : all_vars) {
    nc::net::GraphNodeIndex src = src_and_dst_and_var.first.first;
    nc::net::GraphNodeIndex dst = src_and_dst_and_var.first.second;
    double demand_Mbps = solution->VariableValue(src_and_dst_and_var.second);
    new_elements.emplace_back(
        src, dst, nc::net::Bandwidth::FromMBitsPerSecond(demand_Mbps));
  }

  return nc::make_unique<nc::lp::DemandMatrix>(new_elements, graph);
}

std::unique_ptr<nc::lp::DemandMatrix> GenerateRoughan(
    const nc::net::GraphStorage& graph, nc::net::Bandwidth mean,
    std::mt19937* rnd) {
  // Will start by getting the total incoming/outgoing traffic at each node.
  // These will come from an exponential distribution with the given mean.
  std::exponential_distribution<double> dist(1.0 / mean.Mbps());
  nc::net::GraphNodeMap<double> incoming_traffic_Mbps;
  nc::net::GraphNodeMap<double> outgoing_traffic_Mbps;
  double total_Mbps = 0;
  double sum_in = 0;
  double sum_out = 0;
  for (nc::net::GraphNodeIndex node : graph.AllNodes()) {
    double in = dist(*rnd);
    double out = dist(*rnd);
    incoming_traffic_Mbps[node] = in;
    outgoing_traffic_Mbps[node] = out;
    total_Mbps += in + out;
    sum_in += in;
    sum_out += out;
  }
  double sum_product = sum_in * sum_out;

  // Time to compute the demand for each aggregate.
  double total_p = 0;
  std::vector<nc::lp::DemandMatrixElement> elements;
  for (nc::net::GraphNodeIndex src : graph.AllNodes()) {
    for (nc::net::GraphNodeIndex dst : graph.AllNodes()) {
      double gravity_p = incoming_traffic_Mbps.GetValueOrDie(src) *
                         outgoing_traffic_Mbps.GetValueOrDie(dst) / sum_product;
      if (src == dst) {
        total_p += gravity_p;
        continue;
      }

      double value_Mbps = gravity_p * total_Mbps;
      total_p += gravity_p;
      auto bw = nc::net::Bandwidth::FromMBitsPerSecond(value_Mbps);
      if (bw > nc::net::Bandwidth::Zero()) {
        elements.push_back({src, dst, bw});
      }
    }
  }
  CHECK(std::abs(total_p - 1.0) < 0.0001) << "Not a distribution " << total_p;

  return nc::make_unique<nc::lp::DemandMatrix>(elements, &graph);
}

std::unique_ptr<nc::lp::DemandMatrix> BalanceReverseDemands(
    const nc::lp::DemandMatrix& seed_matrix, double fraction) {
  std::map<nc::lp::SrcAndDst, nc::net::Bandwidth> src_and_dst_to_demand;
  for (const auto& element : seed_matrix.elements()) {
    nc::lp::SrcAndDst src_and_dst = {element.src, element.dst};
    nc::net::Bandwidth& current = src_and_dst_to_demand[src_and_dst];
    current = std::max(current, element.demand);

    nc::lp::SrcAndDst reverse = {element.dst, element.src};
    nc::net::Bandwidth& current_reverse = src_and_dst_to_demand[reverse];
    current_reverse = std::max(current_reverse, element.demand * fraction);
  }

  std::vector<nc::lp::DemandMatrixElement> new_elements;
  for (const auto& src_and_dst_and_demand : src_and_dst_to_demand) {
    nc::net::GraphNodeIndex src = src_and_dst_and_demand.first.first;
    nc::net::GraphNodeIndex dst = src_and_dst_and_demand.first.second;
    nc::net::Bandwidth demand = src_and_dst_and_demand.second;
    new_elements.emplace_back(src, dst, demand);
  }

  return nc::make_unique<nc::lp::DemandMatrix>(new_elements,
                                               seed_matrix.graph());
}

}  // namespace tm_gen
