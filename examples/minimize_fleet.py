"""Minimal end-to-end example: load a Solomon VRPTW instance and minimize its fleet size.

Run from the repository root after installing the package:

    python examples/minimize_fleet.py [path/to/instance.vrp.json]

Without an argument, the script uses the C101 test fixture shipped in
``tests/fixtures/sintef2008_n100/``. The ILS works on integer costs, so the
float instance is first scaled by ``P = 1000`` and rounded up. The result is
then checked and costed on the original float instance.
"""

from __future__ import annotations

import sys
from math import ceil
from pathlib import Path

from mamut_routing_lib.artifacts import load_benchmark_instance
from mamut_routing_lib.checker import check_vrptw_solution
from mamut_routing_lib.models import BenchmarkSolution

from vrptw_fleetmin import ils_vehicle_min

P = 1000  # scaling factor applied to arc costs, time windows and service times

default_path = Path(__file__).resolve().parents[1] / "tests/fixtures/sintef2008_n100/C101.vrp.json"
instance = load_benchmark_instance(sys.argv[1] if len(sys.argv) > 1 else default_path)

scaled = instance.model_copy(
    update={
        "arc_costs": [[ceil(c * P) for c in row] for row in instance.arc_costs],
        "time_windows": [(ceil(a * P), ceil(b * P)) for a, b in instance.time_windows],
        "service_times": [ceil(s * P) for s in instance.service_times],
    }
)

result = ils_vehicle_min(scaled, time_limit_s=10.0, seed=42, variant="v4", scaling_factor=P)

check = check_vrptw_solution(
    instance,
    BenchmarkSolution(instance_name=instance.instance_name, routes=result.routes, cost=None, metadata={}),
)
print(f"instance: {instance.instance_name} ({instance.num_customers} customers)")
print(f"routes: {result.num_routes}")
print(f"solver cost (scaled by P): {result.solver_cost:.0f}")
print(f"valid on the float instance: {check.is_valid()} ({check.status.value})")
if check.is_valid():
    print(f"float routing cost: {check.routing_cost:.2f}")
print(f"wall time: {result.wall_time:.1f} s")
