# vrptw-fleetmin

Fast VRPTW constructive heuristics (nearest-neighbor, best-insertion, regret-k) and an ILS-based fleet-size minimizer for the Vehicle Routing Problem with Time Windows.

> **Local-search backend: PyVRP 0.12.2 HGS** (Hybrid Genetic Search).
> This package pins `pyvrp==0.12.2` (and possible future `0.12.x` versions). It does **not** use the ILS introduced in
> PyVRP 0.13+. The "ILS" exposed here is a separate algorithm developed in the
> context of paper6 (see Citation below) that uses PyVRP's HGS solely as a
> local-search polishing step.

## Status

Early alpha (`v0.0.1`). Public API may change before `v1.0.0`.

## Installation

```bash
pip install vrptw-fleetmin
```

or, using the modern [`uv`](https://github.com/astral-sh/uv) Python package manager:

```bash
uv add vrptw-fleetmin
```

Binary wheels are provided for Linux x86_64. Other platforms must build from
source (requires a C++17 compiler and CMake).

## Quick start

```python
from mamut_routing_lib.models import BenchmarkInstance
from vrptw_fleetmin import ils_vehicle_min

instance = BenchmarkInstance.from_file("C101.txt")     # Solomon-style VRPTW
result = ils_vehicle_min(instance, time_limit=10.0)    # variant='v4' by default
print(f"Routes: {len(result.routes)}, cost: {result.cost}")
```

## Public API

```python
from vrptw_fleetmin import ils_vehicle_min               # convenience (variant='v4')
from vrptw_fleetmin.ils import ils_vehicle_min           # same, explicit module
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

## ILS variants

The `variant` parameter selects between four ILS strategies documented in the
paper:

- `v1` — greedy_repair + basic perturbation
- `v2` — nearest_route_insert + multi-route ruin
- `v3` — dual repair + 10% slack creation
- `v4` — adaptive (default, recommended): starts fast (v2-style), escalates to
  slack creation (v3-style) when stagnating.

All four variants are kept for reference and reproducibility, but `v4` is recommended for general use.

## Citation

If you use this package in academic work, please cite:

```
TODO: paper6 citation block, to be filled in once accepted at C&OR.
```

See `CITATION.cff` for a structured citation entry.

## Related projects

- [MAMUT-routing](https://github.com/0nyr/MAMUT-routing) — VRPTW benchmark dataset and tooling.
- [mamut-routing-lib](https://pypi.org/project/mamut-routing-lib/) — Python
  library for loading and validating VRPTW benchmark instances; used by this package in tests and examples.
- [vrptw-powerful-route-minimization-heuristic](https://github.com/Astronomax/vrptw-powerful-route-minimization-heuristic)
  — [Anton Kuznets](https://github.com/Astronomax)'s open-source NBRMH implementation, used as a baseline in the paper.
- [vrptw-fleet-lb-cg](https://github.com/0nyr/vrptw-fleet-lb-cg) — companion
  source-only release of the column-generation lower bound on minimum fleet size used in the same paper.

## Acknowledgements

This work was developed during the [PhD of Florian Rascoussier](https://theses.fr/s386454), funded by [IMT
Atlantique](https://www.imt-atlantique.fr/en) through the ANR-MAMUT project ([ANR-22-CE22-0016](https://anr.fr/Projet-ANR-22-CE22-0016)) and hosted at [INSA Lyon](https://www.insa-lyon.fr/en), within the context of the [CITI](https://www.citi-lab.fr/) and [Lab-STICC](https://labsticc.fr/en)
research laboratories.

## License

MIT — see [LICENSE](LICENSE).
