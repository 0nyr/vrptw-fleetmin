"""PyVRP helpers vendored from paper6.

Contents (to be filled during extraction step 2):
- `extract_routes_from_pyvrp_solution` — from `papers/paper6/src/python/methods/pyvrp_common.py`
- `reevaluate_solution_on_float_instance` — from `papers/paper6/src/python/experiment.py`

These are vendored rather than imported from paper6 because (a) paper6 is not a
public package, and (b) `mamut-routing-lib`'s equivalent helpers are private
(underscore-prefixed) and not part of its stable API surface.
"""

from __future__ import annotations
