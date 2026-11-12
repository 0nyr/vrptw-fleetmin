# vrptw-fleetmin

Constructive heuristics (nearest-neighbor, best-insertion, regret-k) and an iterated local search (ILS) for fleet-size minimization on the Vehicle Routing Problem with Time Windows (VRPTW). The constructive heuristics and a capacity lower bound are written in C++ and exposed to Python through pybind11.

The ILS uses the local-search and repair operators of [PyVRP](https://github.com/PyVRP/PyVRP) 0.12.2. It is a separate algorithm from the `IteratedLocalSearch` solver that PyVRP ships in version 0.13, and it does not call PyVRP's own solvers.

## Status

Early alpha (version 0.1.0). The public API may change before version 1.0.0. The package is not published on PyPI, so it is installed from source.

## Installation

The package builds a C++ extension during installation. You need Python 3.11 or newer, a C++17 compiler and CMake 3.20 or newer. The Python build dependencies (`scikit-build-core`, `pybind11`) are fetched automatically by `pip`.

From a clone of the repository:

```bash
git clone https://github.com/0nyr/vrptw-fleetmin
cd vrptw-fleetmin
python -m venv .venv
source .venv/bin/activate
pip install .            # or: pip install ".[test]" to also install pytest
```

The same installation also works without a manual clone:

```bash
pip install git+https://github.com/0nyr/vrptw-fleetmin
```

To run the test suite (49 tests, about 20 s):

```bash
pip install ".[test]"
python -m pytest
```

### Dependencies

| Component | Version range | Use |
|---|---|---|
| [`pyvrp`](https://pypi.org/project/pyvrp/) | `>= 0.12.2, < 0.13` | local search and repair operators (`pyvrp.search`, `pyvrp.repair`) |
| [`mamut-routing-lib`](https://pypi.org/project/mamut-routing-lib/) | `>= 0.12.0, < 0.13` | instance model, PyVRP conversion, solution checker |
| [`scikit-build-core`](https://pypi.org/project/scikit-build-core/) | `>= 0.10` | build backend |
| [`pybind11`](https://pypi.org/project/pybind11/) | `>= 2.12` | C++ bindings |
| C++ compiler | C++17 | build |
| [CMake](https://cmake.org/) | `>= 3.20` | build |
| [`pytest`](https://pypi.org/project/pytest/) (extra `[test]`) | `>= 7` | tests |

The `pyvrp` upper bound is needed because PyVRP 0.13 no longer ships the `pyvrp.repair` module that the ILS imports. The package has been tested on Linux x86_64 with Python 3.13, PyVRP 0.12.2, mamut-routing-lib 0.12.0, GCC 15.2 and CMake 4.1. Other Python versions and platforms have not been tested.

The `pyvrp` extra of mamut-routing-lib (`mamut-routing-lib[pyvrp]`) asks for PyVRP 0.13.4 or later, which this package cannot use. Install mamut-routing-lib without that extra. If a newer PyVRP is installed anyway, `import vrptw_fleetmin` raises an `ImportError` that explains the conflict and gives the command to restore a supported version.

## Quick start

The ILS works on integer costs, so a float instance (such as the Solomon instances) is first scaled by a factor `P` and rounded up. The script below is [`examples/minimize_fleet.py`](examples/minimize_fleet.py). It loads the C101 instance shipped with the tests, runs the ILS for 10 s, and checks the result on the original float instance.

```python
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
```

Run from the repository root:

```bash
python examples/minimize_fleet.py
```

Output of one run on a laptop:

```text
instance: C101 (100 customers)
routes: 10
solver cost (scaled by P): 828960
valid on the float instance: True (valid)
float routing cost: 828.94
wall time: 10.0 s
```

The ILS stops on a wall-clock budget, so the cost reached within 10 s can differ between machines and runs. The best-known solution of C101 also uses 10 routes.

## Public API

```python
from vrptw_fleetmin import ils_vehicle_min               # ILS entry point (variant='v4' by default)
from vrptw_fleetmin.ils import ILSResult, ILSTrace       # result types
from vrptw_fleetmin.construct import (                   # C++ constructive heuristics
    VRPTWData,
    ConstructionResult,
    ConstructionStatus,
    capacity_lower_bound,
    nearest_neighbor,
    regret_k_route_construction_best_insertion_travel_time,
    parallel_route_construction_best_insertion_travel_time,
    sequential_route_construction_nearest_neighbor_travel_time_wait,
    k_regret_insertion,
)
```

`ils_vehicle_min(scaled_instance, time_limit_s, seed, *, variant="v4", float_instance=None, base_instance=None, scaling_factor=1000)` returns an `ILSResult` with the fields `routes`, `solver_cost` (on the scaled instance), `num_routes`, `trace` and `wall_time`. When both `float_instance` and `base_instance` are given, each accepted solution is also checked for feasibility on `float_instance`.

## ILS variants

All variants share the same structure.

1. Kickstart. The ILS runs 4 constructive heuristics (nearest-neighbor, parallel best-insertion, regret-2 and regret-3), polishes each solution with PyVRP local search, and keeps the one with the fewest routes, then the lowest cost.
2. Route elimination. The ILS removes one route and reinserts its customers into the others, then polishes with local search and, if needed, with a local search whose penalties are boosted by the scaling factor. A candidate is accepted only if it is feasible and has fewer routes. The route to remove is the smallest one (probability 0.5), the one with the lowest demand (0.2), a random one among the 3 smallest (0.2) or a random one (0.1).
3. Perturbation. After a failed elimination, the ILS removes a random fraction of the customers and reinserts them with `greedy_repair`. The fraction grows with the number of consecutive failures.

The variants differ in the reinsertion operator and the perturbation schedule.

- `v1` reinserts with `greedy_repair`, makes 5 tries per elimination attempt and uses perturbation fractions of 8 %, 15 % and 25 %.
- `v2` reinserts with `nearest_route_insert`, makes 7 tries and uses perturbation fractions of 6 %, 12 % and 25 %. After 20 consecutive failures, it destroys the 3 smallest routes and rebuilds them.
- `v3` also removes 10 % of the customers of the other routes before reinsertion and tries both `greedy_repair` and `nearest_route_insert`. It makes 10 tries, uses perturbation fractions of 8 %, 15 % and 25 %, and rebuilds the 3 smallest routes after 15 consecutive failures.
- `v4` (default) uses the `v2` reinsertion until 5 consecutive failures, then switches to the `v3` reinsertion. It makes 8 tries, uses perturbation fractions of 6 %, 15 % and 25 %, and rebuilds the 3 smallest routes after 18 consecutive failures.

The exact thresholds are in [`src/vrptw_fleetmin/ils.py`](src/vrptw_fleetmin/ils.py). The variants `v1` to `v3` are kept for reference and reproducibility.

## Test data

The directory [`tests/fixtures/sintef2008_n100/`](tests/fixtures/sintef2008_n100/) holds 6 Solomon (1987) VRPTW instances with 100 customers (C101, C201, R101, R201, RC101, RC201) and one best-known solution for each. They are copies of files from the Sintef2008 family of the [MAMUT-routing](https://github.com/ANR-MAMUT/MAMUT-routing) benchmark repository (commit `8e3aa713032a`), and they are used only by the tests and the example. These files are not covered by the MIT license of this repository. They keep the terms stated in the `LICENSE`, `README.md` and `NOTICE.mamut-routing` files of that directory, which must be preserved when the files are redistributed.

## Citation

The method is described in chapter 7 of the PhD thesis of Florian Rascoussier (IMT Atlantique and INSA Lyon, 2026, in preparation). An article with Anton Kuznets is in preparation.

In the meantime, you can cite this repository with the metadata in [`CITATION.cff`](CITATION.cff), which GitHub also exposes through the "Cite this repository" button.

## Related projects

- [MAMUT-routing](https://github.com/ANR-MAMUT/MAMUT-routing) provides VRPTW benchmark instances, best-known solutions and tooling. It is the source of the test fixtures.
- [mamut-routing-lib](https://pypi.org/project/mamut-routing-lib/) is a Python library for loading and validating routing benchmark instances. This package depends on it.
- [vrptw-powerful-route-minimization-heuristic](https://github.com/Astronomax/vrptw-powerful-route-minimization-heuristic) is an open-source implementation of the route minimization heuristic of Nagata and Bräysy (2009) by [Anton Kuznets](https://github.com/Astronomax).

## Acknowledgements

This work was developed during the [PhD of Florian Rascoussier](https://theses.fr/s386454), funded by [IMT Atlantique](https://www.imt-atlantique.fr/en) through the ANR-MAMUT project ([ANR-22-CE22-0016](https://anr.fr/Projet-ANR-22-CE22-0016)) and hosted at [INSA Lyon](https://www.insa-lyon.fr/en), within the [CITI](https://www.citi-lab.fr/) and [Lab-STICC](https://labsticc.fr/en) research laboratories.

## License

The code is released under the MIT license (see [LICENSE](LICENSE)). The test fixtures keep their own terms (see [Test data](#test-data)).
