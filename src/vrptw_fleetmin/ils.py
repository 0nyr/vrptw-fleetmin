"""ILS (Iterated Local Search) for vehicle-count minimization on VRPTW.

The ILS uses the local-search and repair operators of PyVRP 0.12.2. It is a
separate algorithm from the ``IteratedLocalSearch`` solver shipped by PyVRP
0.13 and later. The method is described in chapter 7 of the PhD thesis of
Florian Rascoussier (IMT Atlantique and INSA Lyon, 2026, in preparation).

Architecture
------------
- **Phase 0 — Kickstart.** Run all 4 C++ constructive heuristics (NN, BI, RG2,
  RG3), apply PyVRP local search to each, keep the lexicographically best
  (fewest routes, then lowest cost).
- **Phase 1 — Iterative route elimination.** Repeatedly attempt to reduce from
  K to K-1 routes by ejecting a target route and reinserting its customers via
  a staged repair ladder, followed by PyVRP LS polish.

Public entry point
------------------
``ils_vehicle_min(scaled_instance, time_limit_s, seed, *, variant='v4', ...)``
dispatches to one of four variants. v4 is the default.

Variants
--------
- ``v1``: greedy_repair + basic perturbation.
- ``v2``: nearest_route_insert + multi-route ruin.
- ``v3``: dual repair (both greedy + nearest) + 10% slack creation.
- ``v4``: adaptive. Starts with v2-style repair and switches to v3-style
  slack creation after repeated failures. Default.

Design references
-----------------
- Bent & Van Hentenryck (2004) — two-stage vehicle-first method.
- Lim & Zhang (2007) — ejection-pool route elimination.
- Nagata & Bräysy (2009) — route minimization heuristic.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
import pyvrp
from pyvrp import (
    CostEvaluator,
    PenaltyManager,
    ProblemData,
    RandomNumberGenerator,
    Solution,
    SolveParams,
)
from pyvrp.repair import greedy_repair, nearest_route_insert
from pyvrp.search import LocalSearch, compute_neighbours

from mamut_routing_lib.models import BenchmarkInstance
from mamut_routing_lib.solvers.pyvrp import to_pyvrp_problem

from ._pyvrp_utils import (
    extract_routes_from_pyvrp_solution,
    reevaluate_solution_on_float_instance,
)

DEFAULT_SCALING_FACTOR = 1000
"""Default arc-cost scaling factor P."""


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class ILSTrace:
    """Timestamped trace of incumbent improvements."""
    entries: list[tuple[float, int, float]] = field(default_factory=list)
    """(elapsed_seconds, num_routes, solver_cost)"""

    def record(self, elapsed: float, num_routes: int, cost: float) -> None:
        self.entries.append((elapsed, num_routes, cost))


@dataclass
class ILSResult:
    """Result of the ILS vehicle-minimization run."""
    routes: list[list[int]]
    solver_cost: float
    num_routes: int
    trace: ILSTrace
    wall_time: float


# ---------------------------------------------------------------------------
# Local-search helper (reusable across variants)
# ---------------------------------------------------------------------------

def _build_local_search(
    problem: ProblemData,
    rng: RandomNumberGenerator,
    params: SolveParams | None = None,
    scaling_factor: int = 1,
) -> tuple[LocalSearch, PenaltyManager, CostEvaluator]:
    """
    Build a PyVRP LocalSearch with all default operators.

    Returns a scaling-aware boosted cost evaluator whose TW/load penalties
    are proportional to ``scaling_factor``.  PyVRP's default
    ``repair_booster=12`` is far too weak when arc costs are scaled by P
    (e.g. P=1000), because the initial TW penalty is ~1.0 regardless of P.
    """
    if params is None:
        params = SolveParams()
    neighbours = compute_neighbours(problem, params.neighbourhood)
    ls = LocalSearch(problem, rng, neighbours)
    for node_op in params.node_ops:
        if node_op.supports(problem):
            ls.add_node_operator(node_op(problem))
    for route_op in params.route_ops:
        if route_op.supports(problem):
            ls.add_route_operator(route_op(problem))
    pm = PenaltyManager.init_from(problem, params.penalty)

    # Scaling-aware boosted evaluator: boost must dominate route-cost savings.
    # With P=1000, route savings are ~100k; default boost (x12) gives TW
    # penalties of ~1k — 100x too small.  Using max(12, P) fixes the ratio.
    repair_boost = max(12, scaling_factor)
    load_pens, tw_pen, dist_pen = pm._penalties
    if isinstance(load_pens, (int, float)):
        load_pens = [load_pens]
    boosted_eval = CostEvaluator(
        [lp * repair_boost for lp in load_pens],
        tw_pen * repair_boost,
        dist_pen * repair_boost,
    )
    return ls, pm, boosted_eval


def _ls_improve(
    sol: Solution,
    ls: LocalSearch,
    cost_eval,
    boosted_eval=None,
) -> Solution:
    """Run search + intensify. If infeasible and boosted_eval given, try repair."""
    sol = ls.search(sol, cost_eval)
    sol = ls.intensify(sol, cost_eval)
    if not sol.is_feasible() and boosted_eval is not None:
        sol = ls.search(sol, boosted_eval)
        sol = ls.intensify(sol, boosted_eval)
    return sol


# ---------------------------------------------------------------------------
# Kickstart: multi-start from C++ constructive heuristics
# ---------------------------------------------------------------------------

def _run_kickstart(
    native_data,
    problem: ProblemData,
    ls: LocalSearch,
    cost_eval,
    boosted_eval,
) -> Solution | None:
    """Run all 4 constructive heuristics, apply LS, return best feasible."""
    from vrptw_fleetmin import _native as native

    heuristics = [
        ("nn", lambda d: native.nearest_neighbor(d)),
        ("bi", lambda d: native.parallel_route_construction_best_insertion_travel_time(d)),
        ("rg2", lambda d: native.regret_k_route_construction_best_insertion_travel_time(d, k=2)),
        ("rg3", lambda d: native.regret_k_route_construction_best_insertion_travel_time(d, k=3)),
    ]

    best_sol: Solution | None = None
    best_key = (float("inf"), float("inf"))  # (num_routes, cost)

    for _name, func in heuristics:
        try:
            result = func(native_data)
            if result.status.name != "OK":
                continue
            routes = result.routes
            if not routes:
                continue
            sol = Solution(problem, routes)
            if not sol.is_feasible():
                continue

            # Constructive heuristics can produce a feasible seed that LS later
            # corrupts under penalty-based exploration. Keep the original seed as
            # a safe fallback instead of discarding the whole kickstart attempt.
            seed_sol = sol
            improved_sol = _ls_improve(sol, ls, cost_eval, boosted_eval)
            sol = improved_sol if improved_sol.is_feasible() else seed_sol
            key = (sol.num_routes(), cost_eval.cost(sol))
            if key < best_key:
                best_key = key
                best_sol = sol
        except Exception:
            continue

    return best_sol


# ---------------------------------------------------------------------------
# Route selection strategies
# ---------------------------------------------------------------------------

def _select_target_route_idx(
    routes: list[list[int]],
    rng: RandomNumberGenerator,
    instance: BenchmarkInstance,
    strategy: str = "smallest",
) -> int:
    """Select a target route index for elimination."""
    if len(routes) <= 1:
        return 0

    if strategy == "smallest":
        return min(range(len(routes)), key=lambda i: len(routes[i]))
    elif strategy == "lowest_demand":
        return min(
            range(len(routes)),
            key=lambda i: sum(instance.demands[c] for c in routes[i]),
        )
    elif strategy == "random":
        return int(rng.rand() * len(routes)) % len(routes)
    elif strategy == "random_bottom":
        # Random among bottom-3 by size
        idxs = sorted(range(len(routes)), key=lambda i: len(routes[i]))
        bottom = idxs[: min(3, len(idxs))]
        return bottom[int(rng.rand() * len(bottom)) % len(bottom)]
    else:
        return 0


def _pick_strategy(rng: RandomNumberGenerator) -> str:
    """Biased random strategy selection: 50/20/20/10."""
    r = rng.rand()
    if r < 0.50:
        return "smallest"
    elif r < 0.70:
        return "lowest_demand"
    elif r < 0.90:
        return "random_bottom"
    else:
        return "random"


# ---------------------------------------------------------------------------
# Perturbation
# ---------------------------------------------------------------------------

def _perturb_ruin_repair(
    sol: Solution,
    problem: ProblemData,
    ls: LocalSearch,
    cost_eval,
    rng: RandomNumberGenerator,
    fraction: float = 0.10,
) -> Solution:
    """Random ruin-and-repair perturbation."""
    routes_list = [list(r) for r in sol.routes()]
    all_customers = [c for route in routes_list for c in route]
    n_remove = max(1, int(fraction * len(all_customers)))

    # Randomly select customers to remove
    to_remove: set[int] = set()
    while len(to_remove) < n_remove:
        idx = int(rng.rand() * len(all_customers)) % len(all_customers)
        to_remove.add(all_customers[idx])

    # Build remaining routes and collect removed
    new_routes: list[pyvrp.Route] = []
    removed: list[int] = []
    for route in routes_list:
        kept = [c for c in route if c not in to_remove]
        ejected = [c for c in route if c in to_remove]
        removed.extend(ejected)
        if kept:
            new_routes.append(pyvrp.Route(problem, kept, vehicle_type=0))

    if not new_routes:
        # Edge case: all routes emptied; just return original
        return sol

    # Repair using PyVRP's greedy_repair
    repaired = greedy_repair(new_routes, removed, problem, cost_eval)
    try:
        new_sol = Solution(problem, repaired)
        return _ls_improve(new_sol, ls, cost_eval)
    except Exception:
        return sol


# ---------------------------------------------------------------------------
# Core: attempt route elimination
# ---------------------------------------------------------------------------

def _attempt_elimination(
    sol: Solution,
    problem: ProblemData,
    ls: LocalSearch,
    cost_eval,
    boosted_eval,
    rng: RandomNumberGenerator,
    instance: BenchmarkInstance,
    num_tries: int = 5,
    deadline: float = float("inf"),
) -> Solution | None:
    """
    Try to reduce the solution by one route.

    Returns improved solution if successful, None otherwise.
    """
    routes_list = [list(r) for r in sol.routes()]
    K = len(routes_list)
    if K <= 1:
        return None

    for attempt in range(num_tries):
        if time.perf_counter() > deadline:
            break

        strategy = _pick_strategy(rng)
        target_idx = _select_target_route_idx(routes_list, rng, instance, strategy)

        ejected = list(routes_list[target_idx])
        remaining = [r for i, r in enumerate(routes_list) if i != target_idx]

        # Build PyVRP routes from remaining
        remaining_pyvrp = [pyvrp.Route(problem, r, vehicle_type=0) for r in remaining if r]

        if not remaining_pyvrp:
            continue

        # Stage 1: greedy repair
        repaired = greedy_repair(remaining_pyvrp, ejected, problem, cost_eval)
        try:
            candidate = Solution(problem, repaired)
        except Exception:
            continue

        # Stage 2: LS polish
        candidate = _ls_improve(candidate, ls, cost_eval)

        if candidate.is_feasible() and candidate.num_routes() < K:
            return candidate

        # Stage 3: if not feasible or still K routes, try boosted LS repair
        candidate = ls.search(candidate, boosted_eval)
        if candidate.is_feasible() and candidate.num_routes() < K:
            # Polish again with normal cost
            candidate = _ls_improve(candidate, ls, cost_eval)
            if candidate.is_feasible() and candidate.num_routes() < K:
                return candidate

    return None


# ---------------------------------------------------------------------------
# ILS variants
# ---------------------------------------------------------------------------

def _ils_vehicle_min_v1(
    scaled_instance: BenchmarkInstance,
    time_limit_s: float,
    seed: int,
    float_instance: BenchmarkInstance | None = None,
    base_instance: BenchmarkInstance | None = None,
    scaling_factor: int = DEFAULT_SCALING_FACTOR,
) -> ILSResult:
    """
    ILS v1: basic route elimination with greedy repair + PyVRP LS.

    - Kickstart from all 4 heuristics
    - Iterative K→K-1 elimination
    - Greedy repair + LS polish + boosted LS fallback
    - Ruin-and-repair perturbation on failure
    - Lexicographic acceptance (global)
    - Float-feasibility check after each accepted elimination
    """
    t0 = time.perf_counter()
    deadline = t0 + time_limit_s
    trace = ILSTrace()

    # --- Setup PyVRP problem (no big-M; route reduction handled by elimination loop) ---
    problem = to_pyvrp_problem(scaled_instance)

    rng = RandomNumberGenerator(seed=seed)
    ls, pm, boosted_eval = _build_local_search(
        problem, rng, scaling_factor=scaling_factor,
    )
    cost_eval = pm.cost_evaluator()

    # --- Build native data for kickstart ---
    from vrptw_fleetmin import _native as native
    native_data = native.VRPTWData(
        instance_name=scaled_instance.instance_name,
        num_customers=scaled_instance.num_customers,
        num_vehicles=scaled_instance.num_vehicles,
        vehicle_capacity=scaled_instance.vehicle_capacity,
        coordinates=[(c[0], c[1]) for c in scaled_instance.coordinates],
        demands=scaled_instance.demands,
        service_times=scaled_instance.service_times,
        time_windows=[(tw[0], tw[1]) for tw in scaled_instance.time_windows],
        arc_travel_times=scaled_instance.arc_costs,
    )

    # --- Phase 0: Kickstart ---
    best_sol = _run_kickstart(native_data, problem, ls, cost_eval, boosted_eval)
    if best_sol is None:
        # Fallback: random PyVRP solution + repair
        best_sol = Solution.make_random(problem, rng)
        best_sol = _ls_improve(best_sol, ls, cost_eval, boosted_eval)
        if not best_sol.is_feasible():
            best_sol = ls.search(best_sol, boosted_eval)
            best_sol = ls.intensify(best_sol, boosted_eval)

    K = best_sol.num_routes()
    best_cost = cost_eval.cost(best_sol)
    trace.record(time.perf_counter() - t0, K, best_cost)

    # --- Phase 1: Iterative elimination ---
    fail_count = 0
    THRESHOLD_LIGHT = 5
    THRESHOLD_HEAVY = 15

    working_sol = best_sol

    while time.perf_counter() < deadline:
        eliminated = _attempt_elimination(
            working_sol, problem, ls, cost_eval, boosted_eval,
            rng, scaled_instance, num_tries=5, deadline=deadline,
        )

        if (
            eliminated is not None
            and eliminated.is_feasible()
            and eliminated.num_routes() < K
            and _check_float_feasible(eliminated, float_instance, base_instance)
        ):
            working_sol = eliminated
            best_sol = eliminated
            K = eliminated.num_routes()
            best_cost = cost_eval.cost(eliminated)
            fail_count = 0
            trace.record(time.perf_counter() - t0, K, best_cost)
        else:
            fail_count += 1
            if fail_count >= THRESHOLD_HEAVY:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.25,
                )
                fail_count = THRESHOLD_LIGHT  # partial reset
            elif fail_count >= THRESHOLD_LIGHT:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.15,
                )
            else:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.08,
                )

            # Update best if working improved within same K (and is feasible)
            if working_sol.is_feasible():
                w_routes = working_sol.num_routes()
                w_cost = cost_eval.cost(working_sol)
                if (w_routes, w_cost) < (K, best_cost):
                    if _check_float_feasible(working_sol, float_instance, base_instance):
                        best_sol = working_sol
                        K = w_routes
                        best_cost = w_cost
                        trace.record(time.perf_counter() - t0, K, best_cost)

    wall_time = time.perf_counter() - t0
    routes = extract_routes_from_pyvrp_solution(best_sol)
    solver_cost = _compute_cost_from_routes(routes, scaled_instance)

    return ILSResult(
        routes=routes,
        solver_cost=solver_cost,
        num_routes=len(routes),
        trace=trace,
        wall_time=wall_time,
    )


def _ils_vehicle_min_v2(
    scaled_instance: BenchmarkInstance,
    time_limit_s: float,
    seed: int,
    float_instance: BenchmarkInstance | None = None,
    base_instance: BenchmarkInstance | None = None,
    scaling_factor: int = DEFAULT_SCALING_FACTOR,
) -> ILSResult:
    """
    ILS v2: same as v1 but with nearest_route_insert repair (faster)
    and more aggressive multi-route ruin after many failures.
    """
    t0 = time.perf_counter()
    deadline = t0 + time_limit_s
    trace = ILSTrace()

    # --- Setup PyVRP problem (no big-M; route reduction handled by elimination loop) ---
    problem = to_pyvrp_problem(scaled_instance)

    rng = RandomNumberGenerator(seed=seed)
    ls, pm, boosted_eval = _build_local_search(
        problem, rng, scaling_factor=scaling_factor,
    )
    cost_eval = pm.cost_evaluator()

    from vrptw_fleetmin import _native as native
    native_data = native.VRPTWData(
        instance_name=scaled_instance.instance_name,
        num_customers=scaled_instance.num_customers,
        num_vehicles=scaled_instance.num_vehicles,
        vehicle_capacity=scaled_instance.vehicle_capacity,
        coordinates=[(c[0], c[1]) for c in scaled_instance.coordinates],
        demands=scaled_instance.demands,
        service_times=scaled_instance.service_times,
        time_windows=[(tw[0], tw[1]) for tw in scaled_instance.time_windows],
        arc_travel_times=scaled_instance.arc_costs,
    )

    # Phase 0
    best_sol = _run_kickstart(native_data, problem, ls, cost_eval, boosted_eval)
    if best_sol is None:
        best_sol = Solution.make_random(problem, rng)
        best_sol = _ls_improve(best_sol, ls, cost_eval, boosted_eval)
        if not best_sol.is_feasible():
            best_sol = ls.search(best_sol, boosted_eval)
            best_sol = ls.intensify(best_sol, boosted_eval)

    K = best_sol.num_routes()
    best_cost = cost_eval.cost(best_sol)
    trace.record(time.perf_counter() - t0, K, best_cost)

    working_sol = best_sol
    fail_count = 0

    while time.perf_counter() < deadline:
        eliminated = _attempt_elimination_v2(
            working_sol, problem, ls, cost_eval, boosted_eval,
            rng, scaled_instance, num_tries=7, deadline=deadline,
        )

        if (
            eliminated is not None
            and eliminated.is_feasible()
            and eliminated.num_routes() < K
            and _check_float_feasible(eliminated, float_instance, base_instance)
        ):
            working_sol = eliminated
            best_sol = eliminated
            K = eliminated.num_routes()
            best_cost = cost_eval.cost(eliminated)
            fail_count = 0
            trace.record(time.perf_counter() - t0, K, best_cost)
        else:
            fail_count += 1

            if fail_count >= 20:
                working_sol = _multi_route_ruin(
                    working_sol, problem, ls, cost_eval, rng, n_routes=3,
                )
                fail_count = 5
            elif fail_count >= 8:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.25,
                )
            elif fail_count >= 3:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.12,
                )
            else:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.06,
                )

            if working_sol.is_feasible():
                w_routes = working_sol.num_routes()
                w_cost = cost_eval.cost(working_sol)
                if (w_routes, w_cost) < (K, best_cost):
                    if _check_float_feasible(working_sol, float_instance, base_instance):
                        best_sol = working_sol
                        K = w_routes
                        best_cost = w_cost
                        trace.record(time.perf_counter() - t0, K, best_cost)

    wall_time = time.perf_counter() - t0
    routes = extract_routes_from_pyvrp_solution(best_sol)
    solver_cost = _compute_cost_from_routes(routes, scaled_instance)

    return ILSResult(
        routes=routes,
        solver_cost=solver_cost,
        num_routes=len(routes),
        trace=trace,
        wall_time=wall_time,
    )


# ---------------------------------------------------------------------------
# V2-specific elimination attempt (using nearest_route_insert)
# ---------------------------------------------------------------------------

def _attempt_elimination_v2(
    sol: Solution,
    problem: ProblemData,
    ls: LocalSearch,
    cost_eval,
    boosted_eval,
    rng: RandomNumberGenerator,
    instance: BenchmarkInstance,
    num_tries: int = 7,
    deadline: float = float("inf"),
) -> Solution | None:
    """Like _attempt_elimination but uses nearest_route_insert for faster repair."""
    routes_list = [list(r) for r in sol.routes()]
    K = len(routes_list)
    if K <= 1:
        return None

    for _ in range(num_tries):
        if time.perf_counter() > deadline:
            break

        strategy = _pick_strategy(rng)
        target_idx = _select_target_route_idx(routes_list, rng, instance, strategy)

        ejected = list(routes_list[target_idx])
        remaining = [r for i, r in enumerate(routes_list) if i != target_idx]
        remaining_pyvrp = [pyvrp.Route(problem, r, vehicle_type=0) for r in remaining if r]
        if not remaining_pyvrp:
            continue

        # Use nearest_route_insert (faster than greedy_repair)
        repaired = nearest_route_insert(remaining_pyvrp, ejected, problem, cost_eval)
        try:
            candidate = Solution(problem, repaired)
        except Exception:
            continue

        candidate = _ls_improve(candidate, ls, cost_eval)
        if candidate.is_feasible() and candidate.num_routes() < K:
            return candidate

        # Boosted fallback
        candidate = ls.search(candidate, boosted_eval)
        if candidate.is_feasible() and candidate.num_routes() < K:
            candidate = _ls_improve(candidate, ls, cost_eval)
            if candidate.is_feasible() and candidate.num_routes() < K:
                return candidate

    return None


# ---------------------------------------------------------------------------
# Multi-route ruin (for v2)
# ---------------------------------------------------------------------------

def _multi_route_ruin(
    sol: Solution,
    problem: ProblemData,
    ls: LocalSearch,
    cost_eval,
    rng: RandomNumberGenerator,
    n_routes: int = 3,
) -> Solution:
    """Destroy n smallest routes and rebuild via greedy_repair + LS."""
    routes_list = [list(r) for r in sol.routes()]
    if len(routes_list) <= n_routes:
        return sol

    # Sort by size, destroy smallest n
    indexed = sorted(enumerate(routes_list), key=lambda x: len(x[1]))
    to_destroy = set(idx for idx, _ in indexed[:n_routes])

    ejected = []
    remaining = []
    for i, route in enumerate(routes_list):
        if i in to_destroy:
            ejected.extend(route)
        else:
            remaining.append(route)

    remaining_pyvrp = [pyvrp.Route(problem, r, vehicle_type=0) for r in remaining if r]
    if not remaining_pyvrp:
        return sol

    repaired = greedy_repair(remaining_pyvrp, ejected, problem, cost_eval)
    try:
        new_sol = Solution(problem, repaired)
        return _ls_improve(new_sol, ls, cost_eval)
    except Exception:
        return sol


# ---------------------------------------------------------------------------
# V3: Dual-repair elimination with slack creation
# ---------------------------------------------------------------------------

def _attempt_elimination_v3(
    sol: Solution,
    problem: ProblemData,
    ls: LocalSearch,
    cost_eval,
    boosted_eval,
    rng: RandomNumberGenerator,
    instance: BenchmarkInstance,
    num_tries: int = 10,
    deadline: float = float("inf"),
) -> Solution | None:
    """
    Elimination attempt using dual repair + slack creation.

    For each try:
      1. Select target route to eject
      2. Also ruin ~10% of customers from OTHER routes (creating insertion slack)
      3. Try BOTH greedy_repair and nearest_route_insert
      4. LS polish + boosted fallback for each
      5. Return the first feasible K-1 solution found
    """
    routes_list = [list(r) for r in sol.routes()]
    K = len(routes_list)
    if K <= 1:
        return None

    for _ in range(num_tries):
        if time.perf_counter() > deadline:
            break

        strategy = _pick_strategy(rng)
        target_idx = _select_target_route_idx(routes_list, rng, instance, strategy)

        ejected = list(routes_list[target_idx])
        remaining = [list(r) for i, r in enumerate(routes_list) if i != target_idx]

        # Slack creation: ruin ~10% of customers from remaining routes
        all_remaining = [c for r in remaining for c in r]
        n_ruin = max(1, int(0.10 * len(all_remaining)))
        ruined: set[int] = set()
        while len(ruined) < n_ruin and all_remaining:
            idx = int(rng.rand() * len(all_remaining)) % len(all_remaining)
            ruined.add(all_remaining[idx])

        # Build remaining routes with ruined customers removed
        remaining_clean = []
        for route in remaining:
            kept = [c for c in route if c not in ruined]
            if kept:
                remaining_clean.append(kept)

        # All customers to reinsert: ejected + ruined
        to_reinsert = ejected + [c for c in ruined]

        remaining_pyvrp = [
            pyvrp.Route(problem, r, vehicle_type=0) for r in remaining_clean if r
        ]
        if not remaining_pyvrp:
            continue

        # Try both repair strategies
        for repair_fn in [greedy_repair, nearest_route_insert]:
            if time.perf_counter() > deadline:
                break

            repaired = repair_fn(remaining_pyvrp, to_reinsert, problem, cost_eval)
            try:
                candidate = Solution(problem, repaired)
            except Exception:
                continue

            candidate = _ls_improve(candidate, ls, cost_eval)
            if candidate.is_feasible() and candidate.num_routes() < K:
                return candidate

            # Boosted fallback
            candidate = ls.search(candidate, boosted_eval)
            if candidate.is_feasible() and candidate.num_routes() < K:
                candidate = _ls_improve(candidate, ls, cost_eval)
                if candidate.is_feasible() and candidate.num_routes() < K:
                    return candidate

    return None


def _ils_vehicle_min_v3(
    scaled_instance: BenchmarkInstance,
    time_limit_s: float,
    seed: int,
    float_instance: BenchmarkInstance | None = None,
    base_instance: BenchmarkInstance | None = None,
    scaling_factor: int = DEFAULT_SCALING_FACTOR,
) -> ILSResult:
    """
    ILS v3: dual repair + slack creation.

    Improvements over v2:
    - Tries BOTH greedy_repair and nearest_route_insert per elimination attempt
    - Creates insertion slack by ruining ~10% of non-target customers
    - More attempts per cycle (10)
    - Earlier heavy perturbation
    """
    t0 = time.perf_counter()
    deadline = t0 + time_limit_s
    trace = ILSTrace()

    problem = to_pyvrp_problem(scaled_instance)

    rng = RandomNumberGenerator(seed=seed)
    ls, pm, boosted_eval = _build_local_search(
        problem, rng, scaling_factor=scaling_factor,
    )
    cost_eval = pm.cost_evaluator()

    from vrptw_fleetmin import _native as native
    native_data = native.VRPTWData(
        instance_name=scaled_instance.instance_name,
        num_customers=scaled_instance.num_customers,
        num_vehicles=scaled_instance.num_vehicles,
        vehicle_capacity=scaled_instance.vehicle_capacity,
        coordinates=[(c[0], c[1]) for c in scaled_instance.coordinates],
        demands=scaled_instance.demands,
        service_times=scaled_instance.service_times,
        time_windows=[(tw[0], tw[1]) for tw in scaled_instance.time_windows],
        arc_travel_times=scaled_instance.arc_costs,
    )

    # Phase 0: Kickstart
    best_sol = _run_kickstart(native_data, problem, ls, cost_eval, boosted_eval)
    if best_sol is None:
        best_sol = Solution.make_random(problem, rng)
        best_sol = _ls_improve(best_sol, ls, cost_eval, boosted_eval)
        if not best_sol.is_feasible():
            best_sol = ls.search(best_sol, boosted_eval)
            best_sol = ls.intensify(best_sol, boosted_eval)

    K = best_sol.num_routes()
    best_cost = cost_eval.cost(best_sol)
    trace.record(time.perf_counter() - t0, K, best_cost)

    working_sol = best_sol
    fail_count = 0

    while time.perf_counter() < deadline:
        eliminated = _attempt_elimination_v3(
            working_sol, problem, ls, cost_eval, boosted_eval,
            rng, scaled_instance, num_tries=10, deadline=deadline,
        )

        if (
            eliminated is not None
            and eliminated.is_feasible()
            and eliminated.num_routes() < K
            and _check_float_feasible(eliminated, float_instance, base_instance)
        ):
            working_sol = eliminated
            best_sol = eliminated
            K = eliminated.num_routes()
            best_cost = cost_eval.cost(eliminated)
            fail_count = 0
            trace.record(time.perf_counter() - t0, K, best_cost)
        else:
            fail_count += 1

            # More aggressive perturbation schedule
            if fail_count >= 15:
                working_sol = _multi_route_ruin(
                    working_sol, problem, ls, cost_eval, rng, n_routes=3,
                )
                fail_count = 3
            elif fail_count >= 6:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.25,
                )
            elif fail_count >= 2:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.15,
                )
            else:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.08,
                )

            if working_sol.is_feasible():
                w_routes = working_sol.num_routes()
                w_cost = cost_eval.cost(working_sol)
                if (w_routes, w_cost) < (K, best_cost):
                    if _check_float_feasible(working_sol, float_instance, base_instance):
                        best_sol = working_sol
                        K = w_routes
                        best_cost = w_cost
                        trace.record(time.perf_counter() - t0, K, best_cost)

    wall_time = time.perf_counter() - t0
    routes = extract_routes_from_pyvrp_solution(best_sol)
    solver_cost = _compute_cost_from_routes(routes, scaled_instance)

    return ILSResult(
        routes=routes,
        solver_cost=solver_cost,
        num_routes=len(routes),
        trace=trace,
        wall_time=wall_time,
    )


# ---------------------------------------------------------------------------
# V4: Adaptive — fast elimination first, escalate to slack creation on failure
# ---------------------------------------------------------------------------

def _attempt_elimination_v4(
    sol: Solution,
    problem: ProblemData,
    ls: LocalSearch,
    cost_eval,
    boosted_eval,
    rng: RandomNumberGenerator,
    instance: BenchmarkInstance,
    num_tries: int = 10,
    deadline: float = float("inf"),
    use_slack: bool = False,
) -> Solution | None:
    """
    Adaptive elimination: fast mode (no slack) or thorough mode (with slack).

    Fast mode: single repair (nearest_route_insert), quick LS.
    Thorough mode: dual repair + 10% slack creation.
    """
    routes_list = [list(r) for r in sol.routes()]
    K = len(routes_list)
    if K <= 1:
        return None

    for _ in range(num_tries):
        if time.perf_counter() > deadline:
            break

        strategy = _pick_strategy(rng)
        target_idx = _select_target_route_idx(routes_list, rng, instance, strategy)

        ejected = list(routes_list[target_idx])
        remaining = [list(r) for i, r in enumerate(routes_list) if i != target_idx]

        if use_slack:
            # Ruin ~10% from remaining routes for insertion slack
            all_remaining = [c for r in remaining for c in r]
            n_ruin = max(1, int(0.10 * len(all_remaining)))
            ruined: set[int] = set()
            while len(ruined) < n_ruin and all_remaining:
                idx = int(rng.rand() * len(all_remaining)) % len(all_remaining)
                ruined.add(all_remaining[idx])
            remaining = [[c for c in r if c not in ruined] for r in remaining]
            remaining = [r for r in remaining if r]
            to_reinsert = ejected + list(ruined)
        else:
            to_reinsert = ejected

        remaining_pyvrp = [
            pyvrp.Route(problem, r, vehicle_type=0) for r in remaining if r
        ]
        if not remaining_pyvrp:
            continue

        # In slack mode: try both repairs. In fast mode: nearest_route_insert only.
        repair_fns = (
            [greedy_repair, nearest_route_insert] if use_slack
            else [nearest_route_insert]
        )

        for repair_fn in repair_fns:
            if time.perf_counter() > deadline:
                break

            repaired = repair_fn(remaining_pyvrp, to_reinsert, problem, cost_eval)
            try:
                candidate = Solution(problem, repaired)
            except Exception:
                continue

            candidate = _ls_improve(candidate, ls, cost_eval)
            if candidate.is_feasible() and candidate.num_routes() < K:
                return candidate

            candidate = ls.search(candidate, boosted_eval)
            if candidate.is_feasible() and candidate.num_routes() < K:
                candidate = _ls_improve(candidate, ls, cost_eval)
                if candidate.is_feasible() and candidate.num_routes() < K:
                    return candidate

    return None


def _ils_vehicle_min_v4(
    scaled_instance: BenchmarkInstance,
    time_limit_s: float,
    seed: int,
    float_instance: BenchmarkInstance | None = None,
    base_instance: BenchmarkInstance | None = None,
    scaling_factor: int = DEFAULT_SCALING_FACTOR,
) -> ILSResult:
    """
    ILS v4: adaptive — starts fast (v2-style), escalates to slack creation (v3-style).

    Strategy:
    - First tries: fast elimination (nearest_route_insert, no slack)
    - After 5 failures: switch to thorough (dual repair + slack creation)
    - After 18 failures: multi-route ruin reset
    - Perturbation intensifies with failures
    """
    t0 = time.perf_counter()
    deadline = t0 + time_limit_s
    trace = ILSTrace()

    problem = to_pyvrp_problem(scaled_instance)

    rng = RandomNumberGenerator(seed=seed)
    ls, pm, boosted_eval = _build_local_search(
        problem, rng, scaling_factor=scaling_factor,
    )
    cost_eval = pm.cost_evaluator()

    from vrptw_fleetmin import _native as native
    native_data = native.VRPTWData(
        instance_name=scaled_instance.instance_name,
        num_customers=scaled_instance.num_customers,
        num_vehicles=scaled_instance.num_vehicles,
        vehicle_capacity=scaled_instance.vehicle_capacity,
        coordinates=[(c[0], c[1]) for c in scaled_instance.coordinates],
        demands=scaled_instance.demands,
        service_times=scaled_instance.service_times,
        time_windows=[(tw[0], tw[1]) for tw in scaled_instance.time_windows],
        arc_travel_times=scaled_instance.arc_costs,
    )

    # Phase 0: Kickstart
    best_sol = _run_kickstart(native_data, problem, ls, cost_eval, boosted_eval)
    if best_sol is None:
        best_sol = Solution.make_random(problem, rng)
        best_sol = _ls_improve(best_sol, ls, cost_eval, boosted_eval)
        if not best_sol.is_feasible():
            best_sol = ls.search(best_sol, boosted_eval)
            best_sol = ls.intensify(best_sol, boosted_eval)

    K = best_sol.num_routes()
    best_cost = cost_eval.cost(best_sol)
    trace.record(time.perf_counter() - t0, K, best_cost)

    working_sol = best_sol
    fail_count = 0
    SLACK_THRESHOLD = 5

    while time.perf_counter() < deadline:
        use_slack = fail_count >= SLACK_THRESHOLD
        eliminated = _attempt_elimination_v4(
            working_sol, problem, ls, cost_eval, boosted_eval,
            rng, scaled_instance, num_tries=8, deadline=deadline,
            use_slack=use_slack,
        )

        if (
            eliminated is not None
            and eliminated.is_feasible()
            and eliminated.num_routes() < K
            and _check_float_feasible(eliminated, float_instance, base_instance)
        ):
            working_sol = eliminated
            best_sol = eliminated
            K = eliminated.num_routes()
            best_cost = cost_eval.cost(eliminated)
            fail_count = 0
            trace.record(time.perf_counter() - t0, K, best_cost)
        else:
            fail_count += 1

            if fail_count >= 18:
                working_sol = _multi_route_ruin(
                    working_sol, problem, ls, cost_eval, rng, n_routes=3,
                )
                fail_count = SLACK_THRESHOLD  # reset into slack mode
            elif fail_count >= 8:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.25,
                )
            elif fail_count >= 3:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.15,
                )
            else:
                working_sol = _perturb_ruin_repair(
                    working_sol, problem, ls, cost_eval, rng, fraction=0.06,
                )

            if working_sol.is_feasible():
                w_routes = working_sol.num_routes()
                w_cost = cost_eval.cost(working_sol)
                if (w_routes, w_cost) < (K, best_cost):
                    if _check_float_feasible(working_sol, float_instance, base_instance):
                        best_sol = working_sol
                        K = w_routes
                        best_cost = w_cost
                        trace.record(time.perf_counter() - t0, K, best_cost)

    wall_time = time.perf_counter() - t0
    routes = extract_routes_from_pyvrp_solution(best_sol)
    solver_cost = _compute_cost_from_routes(routes, scaled_instance)

    return ILSResult(
        routes=routes,
        solver_cost=solver_cost,
        num_routes=len(routes),
        trace=trace,
        wall_time=wall_time,
    )


# ---------------------------------------------------------------------------
# Float-feasibility check
# ---------------------------------------------------------------------------

def _check_float_feasible(
    sol: Solution,
    float_instance: BenchmarkInstance | None,
    base_instance: BenchmarkInstance | None,
) -> bool:
    """Check if a PyVRP solution is feasible on the original float instance."""
    if float_instance is None or base_instance is None:
        return True  # Skip check if no float instance provided
    routes = extract_routes_from_pyvrp_solution(sol)
    if not routes:
        return False
    _, is_feasible, _ = reevaluate_solution_on_float_instance(
        routes, float_instance, base_instance,
    )
    return is_feasible


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _compute_cost_from_routes(
    routes: list[list[int]],
    instance: BenchmarkInstance,
) -> float:
    """Compute total travel cost from routes on a VRPTW instance."""
    total = 0.0
    for route in routes:
        prev = 0
        for customer in route:
            total += instance.arc_costs[prev][customer]
            prev = customer
        total += instance.arc_costs[prev][0]
    return total


# ---------------------------------------------------------------------------
# Public dispatcher
# ---------------------------------------------------------------------------

_VARIANTS = {
    "v1": _ils_vehicle_min_v1,
    "v2": _ils_vehicle_min_v2,
    "v3": _ils_vehicle_min_v3,
    "v4": _ils_vehicle_min_v4,
}


def ils_vehicle_min(
    scaled_instance: BenchmarkInstance,
    time_limit_s: float,
    seed: int,
    *,
    variant: str = "v4",
    float_instance: BenchmarkInstance | None = None,
    base_instance: BenchmarkInstance | None = None,
    scaling_factor: int = DEFAULT_SCALING_FACTOR,
) -> ILSResult:
    """Run the ILS fleet-size minimizer; dispatches to the selected variant.

    Parameters
    ----------
    scaled_instance
        VRPTW instance with integer-scaled arc costs (use scaling factor ``P``
        consistent with ``scaling_factor``). The PyVRP operators and the C++
        heuristics work on integer costs, so float instances must be scaled
        before being passed in.
    time_limit_s
        Wall-clock time budget in seconds for the entire ILS run
        (kickstart + elimination loop).
    seed
        Random seed for the PyVRP ``RandomNumberGenerator`` and any
        perturbation RNGs. Reproducible runs require the same seed *and* the
        same ``scaled_instance`` (which depends on rounding mode).
    variant
        One of ``'v1'``, ``'v2'``, ``'v3'``, ``'v4'``. ``'v4'`` is the
        default. See the module docstring for behavioral differences.
    float_instance
        Optional float-cost copy of ``scaled_instance`` used to re-evaluate
        accepted solutions for float-feasibility checks. The check runs only
        when both ``float_instance`` and ``base_instance`` are given.
    base_instance
        Optional original (un-scaled, un-rounded) instance. It must be given
        for the float-feasibility check to run, but its content is not read
        by the re-evaluation.
    scaling_factor
        The factor ``P`` used to scale arc costs in ``scaled_instance``. Drives
        the scaling-aware PyVRP penalty booster.

    Returns
    -------
    ILSResult
        Routes, solver cost (on the scaled instance), number of routes,
        improvement trace, and wall time.

    Raises
    ------
    ValueError
        If ``variant`` is not one of the four supported variants.
    """
    try:
        fn = _VARIANTS[variant]
    except KeyError as e:
        raise ValueError(
            f"Unknown ILS variant: {variant!r}. "
            f"Expected one of {sorted(_VARIANTS)}."
        ) from e
    return fn(
        scaled_instance,
        time_limit_s,
        seed,
        float_instance=float_instance,
        base_instance=base_instance,
        scaling_factor=scaling_factor,
    )
