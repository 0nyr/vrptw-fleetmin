"""Minimal end-to-end example: load a Solomon VRPTW instance, minimize fleet size.

TODO: fill in once the ILS extraction is complete.

    from mamut_routing_lib.models import BenchmarkInstance
    from vrptw_fleetmin import ils_vehicle_min

    instance = BenchmarkInstance.from_file("C101.txt")
    result = ils_vehicle_min(instance, time_limit=10.0)
    print(f"Routes: {len(result.routes)}, cost: {result.cost}")
"""
