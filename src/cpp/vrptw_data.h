#pragma once

#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief Integer VRPTW instance data (post scaling and rounding).
 *
 * Mirrors the Python VRPTWInstance from vrptw_benchmarks, but stores
 * only the fields needed for C++ heuristics and bounds, all as integers.
 *
 * Node indexing:
 *   - 0         = depot
 *   - 1 .. N    = customers  (N = num_customers)
 *
 * All time-related values (time windows, service times, arc travel times)
 * are assumed to already be scaled by the chosen precision factor P.
 */
struct VRPTWData {
    // ----- Instance metadata -----
    std::string instance_name;
    int num_customers;       ///< N (excludes depot)
    int num_vehicles;        ///< Fleet upper bound (Solomon K)
    int vehicle_capacity;    ///< Homogeneous capacity Q

    // ----- Per-node arrays, size = N+1 (depot at index 0) -----
    std::vector<std::pair<int, int>> coordinates;    ///< (x, y) integer coords
    std::vector<int> demands;                         ///< demand q_i  (depot has 0)
    std::vector<int> service_times;                   ///< scaled service time s_i
    std::vector<std::pair<int, int>> time_windows;   ///< scaled [e_i, l_i]

    // ----- Distance / travel-time matrix, size (N+1) x (N+1) -----
    /// arc_travel_times[i][j] = scaled integer cost from node i to node j.
    /// Also serves as travel time (arc cost == travel time in VRPTW).
    std::vector<std::vector<int>> arc_travel_times;

    // ----- Derived helpers -----

    /// Total number of nodes (depot + customers).
    [[nodiscard]] int num_nodes() const { return num_customers + 1; }

    /// Total demand across all customers (excludes depot).
    [[nodiscard]] int64_t total_demand() const;

    /// Validate internal consistency (sizes match num_customers).
    /// Throws std::invalid_argument on mismatch.
    void validate() const;
};
