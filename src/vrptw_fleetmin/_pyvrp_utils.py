"""PyVRP helpers used by the ILS.

Contents:
- `extract_routes_from_pyvrp_solution`: convert a PyVRP solution to lists of
  customer IDs.
- `reevaluate_solution_on_float_instance`: check and cost a solution on the
  float (unscaled) instance with `mamut_routing_lib`.

These are defined here rather than imported from `mamut-routing-lib` because the
equivalent lib helper (`mamut_routing_lib.solvers.pyvrp._extract_routes`) is
private (underscore-prefixed) and not part of its stable public API surface.
"""

from __future__ import annotations

from pyvrp import Solution

from mamut_routing_lib.checker import check_vrptw_solution, compute_solution_cost
from mamut_routing_lib.models import BenchmarkInstance, BenchmarkSolution


def extract_routes_from_pyvrp_solution(sol: Solution) -> list[list[int]]:
    """Extract routes from a PyVRP solution as list of customer IDs.

    Routes exclude depot (0) and remain 1-indexed for customers.
    """
    routes: list[list[int]] = []
    for route in sol.routes():
        route_customers = [int(c) for c in route]  # Already 1-indexed
        if route_customers:
            routes.append(route_customers)
    return routes


def reevaluate_solution_on_float_instance(
    routes: list[list[int]],
    float_instance: BenchmarkInstance,
    original_instance: BenchmarkInstance,  # noqa: ARG001  (kept for caller signature compatibility)
) -> tuple[float, bool, str]:
    """Re-evaluate a solution found on a scaled instance using float costs.

    Parameters
    ----------
    routes
        Solution routes (1-indexed customer IDs).
    float_instance
        Instance with float arc costs for accurate cost computation.
    original_instance
        Original instance (kept in signature for caller compatibility; not
        consumed — `check_vrptw_solution` validates time windows directly off
        `float_instance`).

    Returns
    -------
    tuple
        ``(float_cost, is_feasible, check_status)``.
    """
    if not routes:
        return float("inf"), False, "empty_solution"

    # Sort routes by first customer for a canonical comparison order.
    sorted_routes = sorted((r for r in routes if r), key=lambda r: r[0])

    solution = BenchmarkSolution(
        instance_name=float_instance.instance_name,
        routes=sorted_routes,
        cost=None,
        metadata={},
    )

    try:
        check_result = check_vrptw_solution(float_instance, solution)
        if check_result.is_valid():
            assert check_result.routing_cost is not None
            return float(check_result.routing_cost), True, "valid"
        float_cost = float(compute_solution_cost(float_instance, solution))
        return float_cost, False, check_result.status.value
    except Exception as e:
        float_cost = float(compute_solution_cost(float_instance, solution))
        return float_cost, False, f"error: {str(e)}"
