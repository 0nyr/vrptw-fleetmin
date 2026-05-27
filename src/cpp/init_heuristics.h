#pragma once

#include "vrptw_data.h"

#include <vector>

/**
 * @brief Result of a construction heuristic: a set of routes.
 *
 * Each route is a vector of customer indices (1..N), excluding the depot.
 * The depot (0) is implicit at the start and end of each route.
 */
using RouteList = std::vector<std::vector<int>>;

enum class ConstructionStatus {
    OK,
    INSTANCE_INFEASIBLE,
    VEHICLE_LIMIT_EXCEEDED,
};

/**
 * @brief Result of a constructive heuristic run.
 *
 * Routes are always returned as customer-only paths. The depot (0) is implicit
 * at the start and end of each route.
 */
struct ConstructionResult {
    RouteList routes;
    ConstructionStatus status = ConstructionStatus::OK;
};

/**
 * @brief Deterministic sequential nearest-neighbor route construction.
 *
 * Builds routes sequentially: starting from the depot, greedily extends the
 * current route with the feasible unvisited customer minimizing
 * travel_time + wait. Feasibility includes capacity, customer time windows,
 * and the ability to return to the depot.
 *
 * @param data  The (scaled, integer) VRPTW instance.
 * @return      Constructed routes and a status flag.
 */
ConstructionResult
sequential_route_construction_nearest_neighbor_travel_time_wait(
    const VRPTWData& data);

/**
 * @brief Deterministic parallel best-insertion route construction.
 *
 * Builds several routes simultaneously, always selecting the feasible
 * insertion with the minimum travel-time increase over all remaining clients
 * and current routes.
 *
 * @param data  The (scaled, integer) VRPTW instance.
 * @return      Constructed routes and a status flag.
 */
ConstructionResult parallel_route_construction_best_insertion_travel_time(
    const VRPTWData& data);

/**
 * @brief Deterministic k-regret parallel best-insertion route construction.
 *
 * Selects the customer with the highest regret value, where regret is computed
 * from the k best feasible travel-time insertion moves over the current route
 * set, and inserts it using its best move.
 *
 * @param data  The (scaled, integer) VRPTW instance.
 * @param k     Regret order (must be >= 2).
 * @return      Constructed routes and a status flag.
 */
ConstructionResult regret_k_route_construction_best_insertion_travel_time(
    const VRPTWData& data,
    int k = 2);

/**
 * @brief Compatibility alias for the explicit sequential NN heuristic.
 */
ConstructionResult nearest_neighbor(const VRPTWData& data);

/**
 * @brief k-regret parallel insertion heuristic.
 *
 * At each step, computes the "regret" of not inserting a customer at its best
 * position (difference between k-th best and best insertion cost). The customer
 * with the highest regret is inserted first. This produces solutions with
 * fewer vehicles than pure nearest-neighbor.
 *
 * @param data  The (scaled, integer) VRPTW instance.
 * @param k     Regret order (typically 2 or 3).
 * @return      Constructed routes and a status flag.
 */
ConstructionResult k_regret_insertion(const VRPTWData& data, int k = 2);
