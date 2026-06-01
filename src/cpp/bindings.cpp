/**
 * @file bindings.cpp
 * @brief pybind11 module definition for the vrptw_fleetmin._native extension.
 *
 * Exposes the C++ VRPTW data structures, lower bounds, and
 * initialization heuristics to Python.
 *
 * Usage from Python (preferred via the public facade):
 *   from vrptw_fleetmin.construct import VRPTWData, capacity_lower_bound
 *   from vrptw_fleetmin.construct import (
 *       ConstructionResult,
 *       ConstructionStatus,
 *       nearest_neighbor,
 *       regret_k_route_construction_best_insertion_travel_time,
 *       parallel_route_construction_best_insertion_travel_time,
 *       sequential_route_construction_nearest_neighbor_travel_time_wait,
 *       k_regret_insertion,
 *   )
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>        // automatic std::vector / std::pair conversion

#include "vrptw_data.h"
#include "lower_bound.h"
#include "init_heuristics.h"

namespace py = pybind11;

PYBIND11_MODULE(_native, m) {
    m.doc() = "vrptw-fleetmin native extension: fast VRPTW heuristics and bounds.";

    // ---- VRPTWData ----
    py::class_<VRPTWData>(m, "VRPTWData",
        "Integer VRPTW instance data (post scaling/rounding).\n\n"
        "Node indexing: 0 = depot, 1..N = customers.\n"
        "All time-related values are assumed already scaled by factor P.")
        .def(py::init<>())
        .def(py::init([](
                const std::string& instance_name,
                int num_customers,
                int num_vehicles,
                int vehicle_capacity,
                std::vector<std::pair<int, int>> coordinates,
                std::vector<int> demands,
                std::vector<int> service_times,
                std::vector<std::pair<int, int>> time_windows,
                std::vector<std::vector<int>> arc_travel_times
            ) {
                VRPTWData data;
                data.instance_name = instance_name;
                data.num_customers = num_customers;
                data.num_vehicles = num_vehicles;
                data.vehicle_capacity = vehicle_capacity;
                data.coordinates = std::move(coordinates);
                data.demands = std::move(demands);
                data.service_times = std::move(service_times);
                data.time_windows = std::move(time_windows);
                data.arc_travel_times = std::move(arc_travel_times);
                data.validate();
                return data;
            }),
            py::arg("instance_name"),
            py::arg("num_customers"),
            py::arg("num_vehicles"),
            py::arg("vehicle_capacity"),
            py::arg("coordinates"),
            py::arg("demands"),
            py::arg("service_times"),
            py::arg("time_windows"),
            py::arg("arc_travel_times"),
            "Construct a VRPTWData from all fields. Validates on creation."
        )
        .def_readwrite("instance_name", &VRPTWData::instance_name)
        .def_readwrite("num_customers", &VRPTWData::num_customers)
        .def_readwrite("num_vehicles", &VRPTWData::num_vehicles)
        .def_readwrite("vehicle_capacity", &VRPTWData::vehicle_capacity)
        .def_readwrite("coordinates", &VRPTWData::coordinates)
        .def_readwrite("demands", &VRPTWData::demands)
        .def_readwrite("service_times", &VRPTWData::service_times)
        .def_readwrite("time_windows", &VRPTWData::time_windows)
        .def_readwrite("arc_travel_times", &VRPTWData::arc_travel_times)
        .def("num_nodes", &VRPTWData::num_nodes,
             "Total number of nodes (depot + customers).")
        .def("total_demand", &VRPTWData::total_demand,
             "Sum of customer demands (excludes depot).")
        .def("validate", &VRPTWData::validate,
             "Check internal consistency. Raises ValueError on mismatch.")
        .def("__repr__", [](const VRPTWData& d) {
            return "<VRPTWData '" + d.instance_name +
                   "' n=" + std::to_string(d.num_customers) +
                   " K=" + std::to_string(d.num_vehicles) +
                   " Q=" + std::to_string(d.vehicle_capacity) + ">";
        });

    // ---- Construction result types ----
    py::enum_<ConstructionStatus>(m, "ConstructionStatus",
        "Status returned by constructive heuristics.")
        .value("OK", ConstructionStatus::OK)
        .value("INSTANCE_INFEASIBLE", ConstructionStatus::INSTANCE_INFEASIBLE)
        .value("VEHICLE_LIMIT_EXCEEDED",
               ConstructionStatus::VEHICLE_LIMIT_EXCEEDED);

    py::class_<ConstructionResult>(m, "ConstructionResult",
        "Routes and status returned by a constructive heuristic.")
        .def(py::init<>())
        .def_readwrite("routes", &ConstructionResult::routes)
        .def_readwrite("status", &ConstructionResult::status)
        .def("__repr__", [](const ConstructionResult& result) {
            return "<ConstructionResult routes=" +
                   std::to_string(result.routes.size()) + ">";
        });

    // ---- Lower bounds ----
    m.def("capacity_lower_bound", &capacity_lower_bound,
          py::arg("data"),
          "Capacity-only bin-packing LB on vehicles: ceil(sum(q_i) / Q).\n\n"
          "Ignores time windows. O(N) time.");

    // ---- Initialization heuristics ----
    m.def("sequential_route_construction_nearest_neighbor_travel_time_wait",
          &sequential_route_construction_nearest_neighbor_travel_time_wait,
          py::arg("data"),
          "Deterministic sequential nearest-neighbor route construction.\n\n"
          "Appends the feasible customer minimizing travel_time + wait and "
          "returns routes plus a status flag.");

    m.def("parallel_route_construction_best_insertion_travel_time",
          &parallel_route_construction_best_insertion_travel_time,
          py::arg("data"),
          "Deterministic parallel best-insertion route construction.\n\n"
          "Selects the feasible insertion with the minimum travel-time delta "
          "over all current routes and returns routes plus a status flag.");

    m.def("regret_k_route_construction_best_insertion_travel_time",
          &regret_k_route_construction_best_insertion_travel_time,
          py::arg("data"),
          py::arg("k") = 2,
          "Deterministic k-regret best-insertion route construction.\n\n"
          "Selects the client with the highest regret over the k best feasible "
          "travel-time insertion moves and returns routes plus a status flag.");

    m.def("nearest_neighbor", &nearest_neighbor,
          py::arg("data"),
          "Compatibility alias for "
          "sequential_route_construction_nearest_neighbor_travel_time_wait.\n\n"
          "Returns a ConstructionResult with routes and status.");

    m.def("k_regret_insertion", &k_regret_insertion,
          py::arg("data"),
          py::arg("k") = 2,
          "Compatibility alias for "
          "regret_k_route_construction_best_insertion_travel_time.\n\n"
          "Returns a ConstructionResult with routes and status.");
}
