find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_native
    vrptw_data.cpp
    lower_bound.cpp
    init_heuristics.cpp
    bindings.cpp
)

target_compile_features(_native PRIVATE cxx_std_17)

# Install the compiled module inside the vrptw_fleetmin package
# so it's importable as `from vrptw_fleetmin import _native`.
install(TARGETS _native LIBRARY DESTINATION vrptw_fleetmin)
