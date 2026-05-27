#include "vrptw_data.h"

#include <numeric>
#include <stdexcept>

int64_t VRPTWData::total_demand() const {
    // Sum demands for customers only (indices 1..N).
    return std::accumulate(
        demands.begin() + 1, demands.end(), int64_t{0});
}

void VRPTWData::validate() const {
    const int n = num_nodes();

    auto check_size = [&](const auto& vec, const char* name) {
        if (static_cast<int>(vec.size()) != n) {
            throw std::invalid_argument(
                std::string(name) + " has size " +
                std::to_string(vec.size()) + ", expected " +
                std::to_string(n));
        }
    };

    check_size(coordinates, "coordinates");
    check_size(demands, "demands");
    check_size(service_times, "service_times");
    check_size(time_windows, "time_windows");

    if (static_cast<int>(arc_travel_times.size()) != n) {
        throw std::invalid_argument(
            "arc_travel_times has " +
            std::to_string(arc_travel_times.size()) + " rows, expected " +
            std::to_string(n));
    }
    for (int i = 0; i < n; ++i) {
        if (static_cast<int>(arc_travel_times[i].size()) != n) {
            throw std::invalid_argument(
                "arc_travel_times[" + std::to_string(i) + "] has " +
                std::to_string(arc_travel_times[i].size()) +
                " columns, expected " + std::to_string(n));
        }
    }

    if (num_customers < 0) {
        throw std::invalid_argument("num_customers must be non-negative");
    }
    if (num_vehicles < 0) {
        throw std::invalid_argument("num_vehicles must be non-negative");
    }
    if (vehicle_capacity <= 0) {
        throw std::invalid_argument("vehicle_capacity must be positive");
    }
}
