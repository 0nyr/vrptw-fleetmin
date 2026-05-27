#include "init_heuristics.h"

#include "lower_bound.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

namespace {

constexpr int kDepot = 0;

struct AppendRouteState {
    int last_node;
    int current_load;
    int current_departure_time;
};

struct AppendEvaluation {
    bool feasible = false;
    int metric = 0;
    int next_load = 0;
    int next_departure_time = 0;
};

struct ClosedRoute {
    std::vector<int> path;
    int load = 0;
    int travel_cost = 0;

    [[nodiscard]] bool is_empty() const { return path.size() <= 2; }
};

struct FastInsertionState {
    std::vector<int> arrival;
    std::vector<int> begin_service;
    std::vector<int> departure;
    std::vector<int> latest_arrival;

    void init(const VRPTWData& data, const ClosedRoute& route)
    {
        const size_t node_count = route.path.size();

        arrival.resize(node_count);
        begin_service.resize(node_count);
        departure.resize(node_count);
        latest_arrival.resize(node_count);

        const int first_node = route.path.front();
        arrival[0] = data.time_windows[first_node].first;
        begin_service[0] =
            std::max(arrival[0], data.time_windows[first_node].first);
        departure[0] = begin_service[0] + data.service_times[first_node];

        for (size_t idx = 1; idx < node_count; ++idx) {
            const int pred = route.path[idx - 1];
            const int node = route.path[idx];
            arrival[idx] =
                departure[idx - 1] + data.arc_travel_times[pred][node];

            if (arrival[idx] > data.time_windows[node].second) {
                throw std::runtime_error(
                    "FastInsertionState::init() encountered an infeasible route");
            }

            begin_service[idx] =
                std::max(arrival[idx], data.time_windows[node].first);
            departure[idx] = begin_service[idx] + data.service_times[node];
        }

        const int last_node = route.path.back();
        latest_arrival[node_count - 1] = data.time_windows[last_node].second;

        for (size_t idx = node_count - 1; idx-- > 0;) {
            const int node = route.path[idx];
            const int succ = route.path[idx + 1];
            latest_arrival[idx] = std::min(
                data.time_windows[node].second,
                latest_arrival[idx + 1] - data.arc_travel_times[node][succ] -
                    data.service_times[node]);
        }
    }
};

struct InsertionEvaluation {
    bool feasible = false;
    int travel_delta = 0;
    size_t insertion_pos = 0;
};

struct InsertionMove {
    bool feasible = false;
    int travel_delta = 0;
    int candidate = 0;
    size_t candidate_index = 0;
    size_t route_index = 0;
    size_t insertion_pos = 0;
};

struct RegretCandidateState {
    bool feasible = false;
    int regret = 0;
    InsertionMove best_move;
    std::vector<InsertionMove> best_k_moves;
};

InsertionMove make_insertion_move(
    const InsertionEvaluation& eval,
    int candidate,
    size_t candidate_index,
    size_t route_index)
{
    InsertionMove move;
    move.feasible = eval.feasible;
    move.travel_delta = eval.travel_delta;
    move.candidate = candidate;
    move.candidate_index = candidate_index;
    move.route_index = route_index;
    move.insertion_pos = eval.insertion_pos;
    return move;
}

void insert_candidate_update_route_and_state(
    const VRPTWData& data,
    ClosedRoute& route,
    FastInsertionState& state,
    int candidate,
    size_t insertion_pos,
    int travel_delta)
{
    route.path.insert(
        route.path.begin() + static_cast<std::ptrdiff_t>(insertion_pos),
        candidate);
    route.load += data.demands[candidate];
    route.travel_cost += travel_delta;

    state.arrival.insert(
        state.arrival.begin() + static_cast<std::ptrdiff_t>(insertion_pos), 0);
    state.begin_service.insert(
        state.begin_service.begin() + static_cast<std::ptrdiff_t>(insertion_pos),
        0);
    state.departure.insert(
        state.departure.begin() + static_cast<std::ptrdiff_t>(insertion_pos), 0);
    state.latest_arrival.insert(
        state.latest_arrival.begin() +
            static_cast<std::ptrdiff_t>(insertion_pos),
        0);

    const size_t node_count = route.path.size();

    for (size_t idx = insertion_pos; idx < node_count; ++idx) {
        const size_t pred_index = idx - 1;
        const int pred = route.path[pred_index];
        const int node = route.path[idx];
        state.arrival[idx] =
            state.departure[pred_index] + data.arc_travel_times[pred][node];
        state.begin_service[idx] =
            std::max(state.arrival[idx], data.time_windows[node].first);
        state.departure[idx] =
            state.begin_service[idx] + data.service_times[node];
    }

    for (size_t idx = insertion_pos + 1; idx-- > 0;) {
        const int node = route.path[idx];
        const int succ = route.path[idx + 1];
        state.latest_arrival[idx] = std::min(
            data.time_windows[node].second,
            state.latest_arrival[idx + 1] -
                data.arc_travel_times[node][succ] - data.service_times[node]);
    }

#ifndef NDEBUG
    FastInsertionState check_state;
    check_state.init(data, route);
    if (state.arrival != check_state.arrival ||
        state.begin_service != check_state.begin_service ||
        state.departure != check_state.departure ||
        state.latest_arrival != check_state.latest_arrival) {
        throw std::runtime_error(
            "Incremental BI state update mismatch after insertion");
    }
#endif
}

AppendRouteState make_fresh_route_state(const VRPTWData& data)
{
    const auto& depot_tw = data.time_windows[kDepot];

    return {
        kDepot,
        0,
        depot_tw.first + data.service_times[kDepot],
    };
}

AppendEvaluation evaluate_append_candidate(
    const VRPTWData& data,
    const AppendRouteState& state,
    int candidate)
{
    if (state.current_load + data.demands[candidate] > data.vehicle_capacity) {
        return {};
    }

    const int travel_time = data.arc_travel_times[state.last_node][candidate];
    const int arrival_time = state.current_departure_time + travel_time;
    const auto& candidate_tw = data.time_windows[candidate];

    if (arrival_time > candidate_tw.second) {
        return {};
    }

    const int wait_time = (arrival_time < candidate_tw.first)
                              ? (candidate_tw.first - arrival_time)
                              : 0;
    const int service_start = arrival_time + wait_time;
    const int departure_time = service_start + data.service_times[candidate];
    const int return_arrival =
        departure_time + data.arc_travel_times[candidate][kDepot];

    if (return_arrival > data.time_windows[kDepot].second) {
        return {};
    }

    return {
        true,
        travel_time + wait_time,
        state.current_load + data.demands[candidate],
        departure_time,
    };
}

ClosedRoute make_empty_closed_route()
{
    ClosedRoute route;
    route.path.reserve(2);
    route.path.push_back(kDepot);
    route.path.push_back(kDepot);
    return route;
}

void add_empty_closed_route(
    const VRPTWData& data,
    std::vector<ClosedRoute>& routes,
    std::vector<FastInsertionState>& states)
{
    routes.push_back(make_empty_closed_route());
    states.emplace_back();
    states.back().init(data, routes.back());
}

InsertionEvaluation evaluate_best_insertion_for_candidate(
    const VRPTWData& data,
    const ClosedRoute& route,
    const FastInsertionState& state,
    int candidate)
{
    if (route.load + data.demands[candidate] > data.vehicle_capacity) {
        return {};
    }

    InsertionEvaluation best_eval;

    for (size_t pos = 1; pos < route.path.size(); ++pos) {
        const size_t pred_index = pos - 1;
        const size_t succ_index = pos;
        const int pred = route.path[pred_index];
        const int succ = route.path[succ_index];

        const int arrival_candidate =
            state.departure[pred_index] + data.arc_travel_times[pred][candidate];
        if (arrival_candidate > data.time_windows[candidate].second) {
            continue;
        }

        const int begin_service_candidate = std::max(
            arrival_candidate, data.time_windows[candidate].first);
        const int departure_candidate =
            begin_service_candidate + data.service_times[candidate];
        const int arrival_successor =
            departure_candidate + data.arc_travel_times[candidate][succ];

        if (arrival_successor > state.latest_arrival[succ_index]) {
            continue;
        }

        const int travel_delta =
            data.arc_travel_times[pred][candidate] +
            data.arc_travel_times[candidate][succ] -
            data.arc_travel_times[pred][succ];

        if (!best_eval.feasible || travel_delta < best_eval.travel_delta ||
            (travel_delta == best_eval.travel_delta &&
             pos < best_eval.insertion_pos)) {
            best_eval.feasible = true;
            best_eval.travel_delta = travel_delta;
            best_eval.insertion_pos = pos;
        }
    }

    return best_eval;
}

bool is_better_insertion_move(
    const InsertionMove& lhs,
    const InsertionMove& rhs)
{
    if (!lhs.feasible) {
        return false;
    }
    if (!rhs.feasible) {
        return true;
    }

    if (lhs.travel_delta != rhs.travel_delta) {
        return lhs.travel_delta < rhs.travel_delta;
    }
    if (lhs.candidate != rhs.candidate) {
        return lhs.candidate < rhs.candidate;
    }
    if (lhs.route_index != rhs.route_index) {
        return lhs.route_index < rhs.route_index;
    }
    return lhs.insertion_pos < rhs.insertion_pos;
}

bool same_cached_insertion_move(
    const InsertionMove& lhs,
    const InsertionMove& rhs)
{
    return lhs.feasible == rhs.feasible &&
           lhs.travel_delta == rhs.travel_delta &&
           lhs.candidate == rhs.candidate &&
           lhs.route_index == rhs.route_index &&
           lhs.insertion_pos == rhs.insertion_pos;
}

struct WorseInsertionMove {
    bool operator()(const InsertionMove& lhs, const InsertionMove& rhs) const
    {
        return is_better_insertion_move(rhs, lhs);
    }
};

bool is_better_regret_candidate_state(
    const RegretCandidateState& lhs,
    const RegretCandidateState& rhs)
{
    if (!lhs.feasible) {
        return false;
    }
    if (!rhs.feasible) {
        return true;
    }

    if (lhs.regret != rhs.regret) {
        return lhs.regret > rhs.regret;
    }

    return is_better_insertion_move(lhs.best_move, rhs.best_move);
}

bool same_cached_regret_candidate_state(
    const RegretCandidateState& lhs,
    const RegretCandidateState& rhs)
{
    return lhs.feasible == rhs.feasible &&
           lhs.regret == rhs.regret &&
           same_cached_insertion_move(lhs.best_move, rhs.best_move);
}

struct WorseRegretCandidateState {
    bool operator()(
        const RegretCandidateState& lhs,
        const RegretCandidateState& rhs) const
    {
        return is_better_regret_candidate_state(rhs, lhs);
    }
};

void recompute_route_candidate_evaluations(
    const VRPTWData& data,
    const std::vector<int>& remaining_clients,
    const ClosedRoute& route,
    const FastInsertionState& state,
    std::vector<InsertionEvaluation>& route_candidate_evaluations)
{
    for (const int candidate : remaining_clients) {
        route_candidate_evaluations[candidate] =
            evaluate_best_insertion_for_candidate(data, route, state, candidate);
    }
}

InsertionMove recompute_best_move_for_candidate(
    int candidate,
    size_t candidate_index,
    const std::vector<std::vector<InsertionEvaluation>>&
        route_candidate_evaluations)
{
    InsertionMove best_move;

    for (size_t route_index = 0; route_index < route_candidate_evaluations.size();
         ++route_index) {
        const InsertionMove move = make_insertion_move(
            route_candidate_evaluations[route_index][candidate],
            candidate,
            candidate_index,
            route_index);
        if (is_better_insertion_move(move, best_move)) {
            best_move = move;
        }
    }

    return best_move;
}

void maybe_insert_into_best_k_moves(
    const InsertionMove& move,
    size_t k,
    std::vector<InsertionMove>& best_k_moves)
{
    if (!move.feasible) {
        return;
    }

    auto it = best_k_moves.begin();
    while (it != best_k_moves.end() &&
           !is_better_insertion_move(move, *it)) {
        ++it;
    }

    best_k_moves.insert(it, move);
    if (best_k_moves.size() > k) {
        best_k_moves.pop_back();
    }
}

void recompute_regret_candidate_state(
    RegretCandidateState& state,
    int candidate,
    size_t candidate_index,
    int k,
    const std::vector<std::vector<InsertionEvaluation>>&
        route_candidate_evaluations)
{
    state.feasible = false;
    state.regret = 0;
    state.best_move = {};

    const size_t top_k_size = std::min(
        static_cast<size_t>(k),
        route_candidate_evaluations.size());
    if (state.best_k_moves.capacity() < top_k_size) {
        state.best_k_moves.reserve(top_k_size);
    }
    state.best_k_moves.clear();

    for (size_t route_index = 0; route_index < route_candidate_evaluations.size();
         ++route_index) {
        maybe_insert_into_best_k_moves(
            make_insertion_move(
                route_candidate_evaluations[route_index][candidate],
                candidate,
                candidate_index,
                route_index),
            top_k_size,
            state.best_k_moves);
    }

    if (state.best_k_moves.empty()) {
        return;
    }

    state.feasible = true;
    state.best_move = state.best_k_moves.front();

    for (size_t idx = 1; idx < state.best_k_moves.size(); ++idx) {
        state.regret +=
            state.best_k_moves[idx].travel_delta - state.best_move.travel_delta;
    }
}

void remove_empty_closed_routes(std::vector<ClosedRoute>& routes)
{
    routes.erase(
        std::remove_if(
            routes.begin(),
            routes.end(),
            [](const ClosedRoute& route) { return route.is_empty(); }),
        routes.end());
}

ConstructionResult build_result_from_closed_routes(
    const std::vector<ClosedRoute>& routes,
    ConstructionStatus status,
    int vehicle_limit)
{
    ConstructionResult result;
    result.routes.reserve(routes.size());

    for (const auto& route : routes) {
        if (route.is_empty()) {
            continue;
        }

        result.routes.emplace_back(route.path.begin() + 1, route.path.end() - 1);
    }

    result.status = status;
    if (result.status == ConstructionStatus::OK &&
        result.routes.size() > static_cast<size_t>(vehicle_limit)) {
        result.status = ConstructionStatus::VEHICLE_LIMIT_EXCEEDED;
    }

    return result;
}

}  // namespace

/**
 * @brief Constructs a solution for VRPTW using nearest neighbor heuristic.
 *
 * This constructive heuristic greedily builds routes by sequentially appending
 * unrouted clients to the current route. For each route, it selects the nearest
 * feasible client (by travel time + wait time) that fits within vehicle capacity
 * and time windows. When no client can be appended, a new route is started.
 * The algorithm terminates when all clients are routed or infeasibility is detected.
 *
 * Strategy:
 * - Start with an empty route at the depot
 * - Greedily select the nearest feasible client to append to the current route
 * - Minimize metric = travel_time + wait_time for next client selection
 * - Move to next route when current route cannot accept any remaining client
 *
 * @param data The VRPTW instance containing customer data, time windows, and capacities
 * @return ConstructionResult containing the constructed routes and status
 *         (OK on success, VEHICLE_LIMIT_EXCEEDED if too many routes needed,
 *          INSTANCE_INFEASIBLE if problem cannot be solved)
 *
 * @complexity O(n²) where n = number of customers
 *            - Outer loop: O(n) routes in worst case
 *            - Inner loop: O(n) clients per route
 *            - Per iteration: O(1) arc_travel_times lookup
 */
ConstructionResult
sequential_route_construction_nearest_neighbor_travel_time_wait(
    const VRPTWData& data)
{
    data.validate();

    ConstructionResult result;
    std::vector<int> remaining_clients;
    remaining_clients.reserve(data.num_customers);

    for (int client = 1; client <= data.num_customers; ++client) {
        remaining_clients.push_back(client);
    }

    while (!remaining_clients.empty()) {
        AppendRouteState route_state = make_fresh_route_state(data);
        RouteList::value_type route;
        route.reserve(remaining_clients.size());

        while (!remaining_clients.empty()) {
            size_t best_index = remaining_clients.size();
            int best_metric = std::numeric_limits<int>::max();
            AppendEvaluation best_eval;

            for (size_t idx = 0; idx < remaining_clients.size(); ++idx) {
                const int candidate = remaining_clients[idx];
                const AppendEvaluation eval =
                    evaluate_append_candidate(data, route_state, candidate);

                if (!eval.feasible) {
                    continue;
                }

                if (best_index == remaining_clients.size() ||
                    eval.metric < best_metric ||
                    (eval.metric == best_metric &&
                     candidate < remaining_clients[best_index])) {
                    best_index = idx;
                    best_metric = eval.metric;
                    best_eval = eval;
                }
            }

            if (best_index == remaining_clients.size()) {
                break;
            }

            const int chosen_client = remaining_clients[best_index];
            route.push_back(chosen_client);
            route_state.last_node = chosen_client;
            route_state.current_load = best_eval.next_load;
            route_state.current_departure_time = best_eval.next_departure_time;

            remaining_clients[best_index] = remaining_clients.back();
            remaining_clients.pop_back();
        }

        if (route.empty()) {
            result.status = ConstructionStatus::INSTANCE_INFEASIBLE;
            return result;
        }

        result.routes.push_back(std::move(route));
    }

    if (result.routes.size() > static_cast<size_t>(data.num_vehicles)) {
        result.status = ConstructionStatus::VEHICLE_LIMIT_EXCEEDED;
    }

    return result;
}

/**
 * @brief Constructs a solution for VRPTW using parallel best insertion heuristic.
 *
 * This constructive heuristic builds routes by iteratively inserting unrouted clients
 * at positions that minimize travel time increase. Multiple routes are initialized
 * in parallel (determined by a capacity lower bound), allowing clients to be inserted
 * into any available route. For each remaining client, the algorithm maintains a
 * priority queue of best insertion moves across all routes and executes the globally
 * best move. When no feasible insertion exists in current routes, a new empty route
 * is created. The algorithm terminates when all clients are routed or infeasibility
 * is detected (more than one fresh empty route needed).
 *
 * Route initialization strategy:
 * - Initial route count = max(1, capacity_lower_bound(data))
 * - All routes start empty with depot as the only node
 * - Early parallel initialization allows parallel competition for inserting clients
 * 
 * @param data The VRPTW instance containing customer data, time windows, and capacities
 * @return ConstructionResult containing the constructed routes and status
 *         (OK on success, INSTANCE_INFEASIBLE if problem cannot be solved)
 *
 * @complexity O(n²m log(nm)) where n = number of customers, m = number of routes
 */
ConstructionResult parallel_route_construction_best_insertion_travel_time(
    const VRPTWData& data)
{
    data.validate();

    std::vector<int> remaining_clients;
    remaining_clients.reserve(data.num_customers);
    for (int client = 1; client <= data.num_customers; ++client) {
        remaining_clients.push_back(client);
    }

    if (remaining_clients.empty()) {
        return {};
    }

    const int initial_route_count = std::max(1, capacity_lower_bound(data));
    std::vector<ClosedRoute> routes;
    std::vector<FastInsertionState> states;
    std::vector<std::vector<InsertionEvaluation>> route_candidate_evaluations;
    std::vector<InsertionMove> best_move_by_candidate(
        static_cast<size_t>(data.num_customers) + 1);
    std::priority_queue<
        InsertionMove,
        std::vector<InsertionMove>,
        WorseInsertionMove>
        candidate_heap;
    std::vector<int> candidate_positions(
        static_cast<size_t>(data.num_customers) + 1,
        -1);
    routes.reserve(static_cast<size_t>(initial_route_count));
    states.reserve(static_cast<size_t>(initial_route_count));
    route_candidate_evaluations.reserve(static_cast<size_t>(initial_route_count));

    for (size_t idx = 0; idx < remaining_clients.size(); ++idx) {
        candidate_positions[remaining_clients[idx]] = static_cast<int>(idx);
    }

    for (int route_index = 0; route_index < initial_route_count; ++route_index) {
        add_empty_closed_route(data, routes, states);
        route_candidate_evaluations.emplace_back(
            static_cast<size_t>(data.num_customers) + 1);
        recompute_route_candidate_evaluations(
            data,
            remaining_clients,
            routes.back(),
            states.back(),
            route_candidate_evaluations.back());
    }

    for (const int candidate : remaining_clients) {
        best_move_by_candidate[candidate] = recompute_best_move_for_candidate(
            candidate,
            static_cast<size_t>(candidate_positions[candidate]),
            route_candidate_evaluations);
        if (best_move_by_candidate[candidate].feasible) {
            candidate_heap.push(best_move_by_candidate[candidate]);
        }
    }

    bool added_fresh_empty_route = false;

    while (!remaining_clients.empty()) {
        InsertionMove best_move;

        while (!candidate_heap.empty()) {
            const InsertionMove heap_move = candidate_heap.top();
            const int candidate = heap_move.candidate;

            if (candidate_positions[candidate] < 0) {
                candidate_heap.pop();
                continue;
            }

            const InsertionMove& current_best = best_move_by_candidate[candidate];
            if (!same_cached_insertion_move(heap_move, current_best)) {
                candidate_heap.pop();
                continue;
            }

            best_move = current_best;
            break;
        }

        if (!best_move.feasible) {
            if (added_fresh_empty_route) {
                remove_empty_closed_routes(routes);
                return build_result_from_closed_routes(
                    routes,
                    ConstructionStatus::INSTANCE_INFEASIBLE,
                    data.num_vehicles);
            }

            add_empty_closed_route(data, routes, states);
            route_candidate_evaluations.emplace_back(
                static_cast<size_t>(data.num_customers) + 1);
            recompute_route_candidate_evaluations(
                data,
                remaining_clients,
                routes.back(),
                states.back(),
                route_candidate_evaluations.back());

            const size_t new_route_index = routes.size() - 1;
            for (const int candidate : remaining_clients) {
                const InsertionMove new_route_move = make_insertion_move(
                    route_candidate_evaluations[new_route_index][candidate],
                    candidate,
                    static_cast<size_t>(candidate_positions[candidate]),
                    new_route_index);
                if (is_better_insertion_move(
                        new_route_move, best_move_by_candidate[candidate])) {
                    best_move_by_candidate[candidate] = new_route_move;
                    candidate_heap.push(best_move_by_candidate[candidate]);
                }
            }
            added_fresh_empty_route = true;
            continue;
        }

        const size_t changed_route_index = best_move.route_index;
        ClosedRoute& target_route = routes[best_move.route_index];
        insert_candidate_update_route_and_state(
            data,
            target_route,
            states[changed_route_index],
            best_move.candidate,
            best_move.insertion_pos,
            best_move.travel_delta);

        const int removed_candidate = best_move.candidate;
        const size_t removed_index = best_move.candidate_index;
        const int swapped_candidate = remaining_clients.back();
        remaining_clients[removed_index] = swapped_candidate;
        remaining_clients.pop_back();
        candidate_positions[removed_candidate] = -1;
        best_move_by_candidate[removed_candidate] = {};

        if (removed_index < remaining_clients.size()) {
            candidate_positions[swapped_candidate] =
                static_cast<int>(removed_index);
            best_move_by_candidate[swapped_candidate].candidate_index =
                removed_index;
        }

        recompute_route_candidate_evaluations(
            data,
            remaining_clients,
            routes[changed_route_index],
            states[changed_route_index],
            route_candidate_evaluations[changed_route_index]);

        for (const int candidate : remaining_clients) {
            const size_t candidate_index =
                static_cast<size_t>(candidate_positions[candidate]);
            const InsertionMove new_move = make_insertion_move(
                route_candidate_evaluations[changed_route_index][candidate],
                candidate,
                candidate_index,
                changed_route_index);

            InsertionMove& current_best = best_move_by_candidate[candidate];
            if (current_best.route_index == changed_route_index ||
                is_better_insertion_move(new_move, current_best)) {
                current_best = recompute_best_move_for_candidate(
                    candidate, candidate_index, route_candidate_evaluations);
                if (current_best.feasible) {
                    candidate_heap.push(current_best);
                }
            } else {
                current_best.candidate_index = candidate_index;
            }
        }

        added_fresh_empty_route = false;
    }

    remove_empty_closed_routes(routes);
    return build_result_from_closed_routes(
        routes, ConstructionStatus::OK, data.num_vehicles);
}

/**
 * @brief Constructs a solution for VRPTW using k-regret insertion heuristic.
 *
 * This constructive heuristic builds routes by iteratively selecting unrouted clients
 * based on regret value—the difference between the best and k-th best insertion costs
 * across all routes. High regret clients are prioritized, as inserting them later would
 * be much more expensive. The algorithm seeds with one route, starts with an empty
 * second route, and maintains a priority queue of regret values for all remaining
 * clients. When a client is inserted, all regret values are recomputed.
 *
 * Insertion strategy (per candidate):
 * - Find k best insertion positions across all routes (by travel_delta)
 * - Compute regret = sum(cost_2nd_best - cost_best, ..., cost_k_th_best - cost_best)
 * - Select candidate with highest regret (ties broken by best insertion cost)
 * - Insert at the best position found
 *
 * Route initialization:
 * - Initial route receives the first feasible seed client via best insertion
 * - Second empty route is added and stands ready for remaining insertions
 * - New routes created dynamically only if all existing routes are full
 *
 * @param data The VRPTW instance containing customer data, time windows, and capacities
 * @param k Number of alternatives to consider for regret calculation (must be >= 2)
 * @return ConstructionResult containing the constructed routes and status
 *         (OK on success, INSTANCE_INFEASIBLE if problem cannot be solved)
 *
 * @complexity O(n²m log(nm)) where n = number of customers, m = number of routes
 *            - Outer loop: O(n) iterations (one per client insertion)
 *            - Per iteration:
 *              - Heap top operation: O(log(nm)) pop operations
 *              - Route evaluation: O(m) routes × O(n) positions per route = O(mn)
 *              - Regret recomputation: O(m) route evaluations
 *            - Overall: O(n) × O(mn + log(nm)) = O(n²m)
 *            - With heap management: O(n²m log(nm))
 * 
 * @note k-regret insertion typically produces better quality solutions than nearest
 *       neighbor at the cost of increased computation time. Regret values prioritize
 *       clients whose late insertion would incur high costs, leading to more robust
 *       route construction.
 */
ConstructionResult regret_k_route_construction_best_insertion_travel_time(
    const VRPTWData& data,
    int k)
{
    data.validate();

    if (k < 2) {
        throw std::invalid_argument(
            "regret_k_route_construction_best_insertion_travel_time(): "
            "k must be >= 2");
    }

    std::vector<int> remaining_clients;
    remaining_clients.reserve(data.num_customers);
    for (int client = 1; client <= data.num_customers; ++client) {
        remaining_clients.push_back(client);
    }

    if (remaining_clients.empty()) {
        return {};
    }

    ClosedRoute seed_route = make_empty_closed_route();
    FastInsertionState seed_state;
    seed_state.init(data, seed_route);

    size_t seed_index = remaining_clients.size();
    InsertionEvaluation seed_eval;
    for (size_t idx = 0; idx < remaining_clients.size(); ++idx) {
        const int candidate = remaining_clients[idx];
        seed_eval = evaluate_best_insertion_for_candidate(
            data, seed_route, seed_state, candidate);
        if (seed_eval.feasible) {
            seed_index = idx;
            break;
        }
    }

    if (seed_index == remaining_clients.size()) {
        ConstructionResult result;
        result.status = ConstructionStatus::INSTANCE_INFEASIBLE;
        return result;
    }

    std::vector<ClosedRoute> routes;
    std::vector<FastInsertionState> states;
    std::vector<std::vector<InsertionEvaluation>> route_candidate_evaluations;
    std::vector<RegretCandidateState> regret_state_by_candidate(
        static_cast<size_t>(data.num_customers) + 1);
    std::priority_queue<
        RegretCandidateState,
        std::vector<RegretCandidateState>,
        WorseRegretCandidateState>
        regret_heap;
    std::vector<int> candidate_positions(
        static_cast<size_t>(data.num_customers) + 1,
        -1);

    const size_t reserve_size = static_cast<size_t>(data.num_customers) + 1;
    routes.reserve(reserve_size);
    states.reserve(reserve_size);
    route_candidate_evaluations.reserve(reserve_size);

    routes.push_back(make_empty_closed_route());
    states.emplace_back();
    states.back().init(data, routes.back());

    const int seeded_candidate = remaining_clients[seed_index];
    insert_candidate_update_route_and_state(
        data,
        routes.back(),
        states.back(),
        seeded_candidate,
        seed_eval.insertion_pos,
        seed_eval.travel_delta);

    remaining_clients[seed_index] = remaining_clients.back();
    remaining_clients.pop_back();

    add_empty_closed_route(data, routes, states);

    route_candidate_evaluations.emplace_back(
        static_cast<size_t>(data.num_customers) + 1);
    route_candidate_evaluations.emplace_back(
        static_cast<size_t>(data.num_customers) + 1);

    for (size_t idx = 0; idx < remaining_clients.size(); ++idx) {
        candidate_positions[remaining_clients[idx]] = static_cast<int>(idx);
    }

    for (size_t route_index = 0; route_index < routes.size(); ++route_index) {
        recompute_route_candidate_evaluations(
            data,
            remaining_clients,
            routes[route_index],
            states[route_index],
            route_candidate_evaluations[route_index]);
    }

    for (const int candidate : remaining_clients) {
        recompute_regret_candidate_state(
            regret_state_by_candidate[candidate],
            candidate,
            static_cast<size_t>(candidate_positions[candidate]),
            k,
            route_candidate_evaluations);
        if (regret_state_by_candidate[candidate].feasible) {
            regret_heap.push(regret_state_by_candidate[candidate]);
        }
    }

    while (!remaining_clients.empty()) {
        RegretCandidateState best_state;

        while (!regret_heap.empty()) {
            const RegretCandidateState heap_state = regret_heap.top();
            const int candidate = heap_state.best_move.candidate;

            if (candidate_positions[candidate] < 0) {
                regret_heap.pop();
                continue;
            }

            const RegretCandidateState& current_state =
                regret_state_by_candidate[candidate];
            if (!same_cached_regret_candidate_state(
                    heap_state, current_state)) {
                regret_heap.pop();
                continue;
            }

            best_state = current_state;
            break;
        }

        if (!best_state.feasible) {
            remove_empty_closed_routes(routes);
            return build_result_from_closed_routes(
                routes,
                ConstructionStatus::INSTANCE_INFEASIBLE,
                data.num_vehicles);
        }

        const InsertionMove chosen_move = best_state.best_move;
        const size_t changed_route_index = chosen_move.route_index;
        const bool consumed_empty_route = routes[changed_route_index].is_empty();

        insert_candidate_update_route_and_state(
            data,
            routes[changed_route_index],
            states[changed_route_index],
            chosen_move.candidate,
            chosen_move.insertion_pos,
            chosen_move.travel_delta);

        const int removed_candidate = chosen_move.candidate;
        const size_t removed_index = chosen_move.candidate_index;
        const int swapped_candidate = remaining_clients.back();
        remaining_clients[removed_index] = swapped_candidate;
        remaining_clients.pop_back();
        candidate_positions[removed_candidate] = -1;
        regret_state_by_candidate[removed_candidate] = {};

        if (removed_index < remaining_clients.size()) {
            candidate_positions[swapped_candidate] =
                static_cast<int>(removed_index);
        }

        recompute_route_candidate_evaluations(
            data,
            remaining_clients,
            routes[changed_route_index],
            states[changed_route_index],
            route_candidate_evaluations[changed_route_index]);

        if (consumed_empty_route) {
            add_empty_closed_route(data, routes, states);
            route_candidate_evaluations.emplace_back(
                static_cast<size_t>(data.num_customers) + 1);
            recompute_route_candidate_evaluations(
                data,
                remaining_clients,
                routes.back(),
                states.back(),
                route_candidate_evaluations.back());
        }

        for (const int candidate : remaining_clients) {
            recompute_regret_candidate_state(
                regret_state_by_candidate[candidate],
                candidate,
                static_cast<size_t>(candidate_positions[candidate]),
                k,
                route_candidate_evaluations);
            if (regret_state_by_candidate[candidate].feasible) {
                regret_heap.push(regret_state_by_candidate[candidate]);
            }
        }
    }

    remove_empty_closed_routes(routes);
    return build_result_from_closed_routes(
        routes, ConstructionStatus::OK, data.num_vehicles);
}

ConstructionResult nearest_neighbor(const VRPTWData& data)
{
    return sequential_route_construction_nearest_neighbor_travel_time_wait(
        data);
}

ConstructionResult k_regret_insertion(const VRPTWData& data, int k)
{
    return regret_k_route_construction_best_insertion_travel_time(data, k);
}
