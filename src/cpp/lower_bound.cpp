#include "lower_bound.h"

#include <cstdint>

int capacity_lower_bound(const VRPTWData& data) {
    // LB_cap = ceil( sum_i q_i / Q )
    //        = (sum_q + Q - 1) / Q   (integer ceiling division)
    const int64_t sum_q = data.total_demand();
    const int64_t Q = data.vehicle_capacity;

    // Guard: if total demand is zero, at least 1 vehicle to visit customers
    if (sum_q == 0) {
        return (data.num_customers > 0) ? 1 : 0;
    }

    return static_cast<int>((sum_q + Q - 1) / Q);
}
