#ifndef TM_GEN_H
#define TM_GEN_H

#include <ncode/lp/demand_matrix.h>
#include <ncode/net/net_common.h>
#include <memory>
#include <random>

namespace tm_gen {

// Produces a demand matrix using a scheme based on Roughan's '93 CCR paper. The
// matrix is not guaranteed to the satisfiable.
std::unique_ptr<nc::lp::DemandMatrix> GenerateRoughan(
    const nc::net::GraphStorage& graph, nc::net::Bandwidth mean,
    std::mt19937* rnd);

// Generates a demand matrix based on another demand matrix. The new matrix will
// have the same sum of incoming and outgoing demand at each node, but will be
// more geographically local. Each demand will be allowed to vary up to
// 'fraction' from its current level.
std::unique_ptr<nc::lp::DemandMatrix> LocalizeDemandMatrix(
    const nc::lp::DemandMatrix& seed_matrix, double fraction_allowance);

// Makes sure each aggregate's reverse carries at least 'fraction' as much
// traffic as the forward does.
std::unique_ptr<nc::lp::DemandMatrix> BalanceReverseDemands(
    const nc::lp::DemandMatrix& seed_matrix, double fraction);

}  // namespace tm_gen
#endif
