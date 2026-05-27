#pragma once

#include "vrptw_data.h"

/**
 * @brief Capacity-only bin-packing lower bound on the number of vehicles.
 *
 * Ignores time windows entirely and treats the problem as a bin-packing
 * instance where vehicle capacity Q is the bin size and customer demands
 * q_i are item sizes.
 *
 * Formula:
 *   LB_cap = ceil( sum_i q_i / Q )
 *
 * This is the simplest possible vehicle LB — very fast (O(N)), often loose,
 * but valid. It tends to be tighter on R/RC instances where demand/capacity
 * is the binding constraint, and looser on C instances where time-window
 * tightness dominates.
 *
 * @param data  The (scaled) VRPTW instance.
 * @return      A valid lower bound on the number of vehicles required.
 */
int capacity_lower_bound(const VRPTWData& data);
