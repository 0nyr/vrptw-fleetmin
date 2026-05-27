"""ILS-based fleet-size minimization for VRPTW.

Single public entry point: `ils_vehicle_min(..., variant='v4')`. The four variants
(v1, v2, v3, v4) correspond to the ablation table of paper6 and differ in their
ruin / repair / acceptance strategies. v4 is the recommended default.

Local-search backend: PyVRP 0.12.2 HGS (NOT the PyVRP 0.13+ ILS).

TODO: extraction step 2 — copy the body from
`papers/paper6/src/python/ub/ils_vehicle_min.py`, rewrite the 5 paper6-internal
imports per task_open_sourcing.md §6, and merge the four `ils_vehicle_min_v{1..4}`
entry points into a single dispatcher.
"""

from __future__ import annotations

DEFAULT_SCALING_FACTOR = 1000


def ils_vehicle_min(*args, variant: str = "v4", **kwargs):  # noqa: ANN001
    """Run the ILS fleet-size minimizer; dispatches to the selected variant."""
    raise NotImplementedError(
        "Extraction pending — see task_open_sourcing.md §7 step 2."
    )
