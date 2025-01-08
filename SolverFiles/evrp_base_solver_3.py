"""
"""

run_instances = ["rc105C5", "rc108C5"]
for instance in run_instances:
    import os
    import numpy as np
    from docplex.mp.model import Model
    from docplex.mp.conflict_refiner import ConflictRefiner
    from evrp_instance_parser import parse_instance_file
    import pandas as pd
    from pathlib import Path

    rnd = np.random
    # rnd.seed(0)
    LARGE_VALUE = 1e6

    file_name = Path(f"./Data/EVRP-TW-SPD-main/data/evrptw_instances/small_instances/Cplex5er/{instance}.txt")
    solution_path = Path("./Solution/case_1/small_instances/Cplex5er")
    number_of_dummy_charging_station_nodes = 3
    number_of_vehicles = 2
    # Lowest and highest allowed battery levels are normalised to 0 and 1. Please use values within this range.
    lowest_allowed_battery_level = 0
    highest_allowed_battery_level = 1
    # Penalty parameters
    penalty_for_overcharging = 1 * LARGE_VALUE
    penalty_for_overdischarging = 1 * LARGE_VALUE
    # When to penalise for overcharging or overdischarging
    overdischarge_level_for_penalty = 0.2
    overcharge_level_for_penalty = 0.8


    vrp_data = parse_instance_file(file_name)

    # @title Setting up the physical nodes available
    set_of_vehicles = {f"V|{i}" for i in range(number_of_vehicles)}
    set_of_customers = vrp_data.set_of_customers
    set_of_charging_stations = vrp_data.set_of_charging_stations
    set_of_dummy_charging_stations = {f"{charging_station}|{i}" for charging_station in vrp_data.set_of_charging_stations for i in range(number_of_dummy_charging_station_nodes)}
    set_of_depots = vrp_data.set_of_depots
    set_of_start_depots = {f"{depot}|S" for depot in set_of_depots}
    set_of_terminal_depots = {f"{depot}|T" for depot in set_of_depots}

    set_of_customers_charging_stations = set_of_customers.union(set_of_dummy_charging_stations)
    set_of_customers_charging_stations_start_depots = set_of_customers.union(set_of_dummy_charging_stations, set_of_start_depots)
    set_of_customers_charging_stations_terminal_depots = set_of_customers.union(
        set_of_dummy_charging_stations, set_of_terminal_depots
    )

    set_of_all_nodes = set_of_customers.union(set_of_dummy_charging_stations, set_of_start_depots, set_of_terminal_depots)

    # { "Node X": (X Coordinate, Y Coordinate) }
    coordinates_of_nodes: dict[str, tuple[float, float]] = {}
    for node, coordinate in vrp_data.coordinates_of_nodes.items():
        if node in set_of_depots:
            coordinates_of_nodes[f"{node}|S"] = coordinate
            coordinates_of_nodes[f"{node}|T"] = coordinate
        elif node in set_of_charging_stations:
            for i in range(number_of_dummy_charging_station_nodes):
                coordinates_of_nodes[f"{node}|{i}"] = coordinate
        else:
            coordinates_of_nodes[node] = coordinate

    def valid_node_pair(node_i, node_j) -> bool:
        # If the same node
        if node_i == node_j:
            return False
        # If between two depots
        if node_i.startswith("D") and node_j.startswith("D"):
            return False
        # if the same charging station
        if node_i.startswith("S") and node_j.startswith("S") and node_i.split("|")[1] == node_j.split("|")[1]:
            return False
        # if the same location
        if coordinates_of_nodes[node_i] == coordinates_of_nodes[node_j]:
            return False
        
        return True


    # { (Node i, Node j): distance }
    distance_between_nodes = {}  # d_i_j
    for node_i, (xi, yi) in coordinates_of_nodes.items():
        for node_j, (xj, yj) in coordinates_of_nodes.items():
            if valid_node_pair(node_i, node_j):
                distance_between_nodes[(node_i, node_j)] = np.hypot(xi - xj, yi - yj)


    time_taken_between_nodes = {  # t_i_j
        (node_i, node_j): distance_between_nodes[(node_i, node_j)] / vrp_data.average_velocity
        for node_i, node_j in distance_between_nodes.keys()
    }


    # @title Supply and Demand Set Up
    # { Node i: demand }
    load_capacity_of_vehicle = vrp_data.vehicle_load_capacity
    demand_at_all_nodes: dict[str, float] = {}
    for node, demand in vrp_data.demand_of_nodes.items():
        if node in set_of_depots:
            demand_at_all_nodes[f"{node}|S"] = 0
            demand_at_all_nodes[f"{node}|T"] = 0
        elif node in set_of_charging_stations:
            for i in range(number_of_dummy_charging_station_nodes):
                demand_at_all_nodes[f"{node}|{i}"] = 0
        else:
            demand_at_all_nodes[node] = demand

    # @title Time Window Set Up
    latest_arrival_time_at_depot = vrp_data.latest_arrival_time_at_depot
    time_window_of_nodes = {}
    for node, (earliest_arrival_time, latest_arrival_time) in vrp_data.time_window_of_nodes.items():
        if node in set_of_depots:
            time_window_of_nodes[f"{node}|S"] = (earliest_arrival_time, latest_arrival_time)
            time_window_of_nodes[f"{node}|T"] = (earliest_arrival_time, latest_arrival_time)
        elif node in set_of_charging_stations:
            for i in range(number_of_dummy_charging_station_nodes):
                time_window_of_nodes[f"{node}|{i}"] = (earliest_arrival_time, latest_arrival_time)
        else:
            time_window_of_nodes[node] = (earliest_arrival_time, latest_arrival_time)

    service_time_at_node: dict[str, float] = {}
    for node, service_time in vrp_data.service_time.items():
        if node in set_of_depots:
            service_time_at_node[f"{node}|S"] = 0
            service_time_at_node[f"{node}|T"] = 0
        elif node in set_of_charging_stations:
            for i in range(number_of_dummy_charging_station_nodes):
                service_time_at_node[f"{node}|{i}"] = 0
        else:
            service_time_at_node[node] = service_time

    # @title Battery Set Up
    battery_capacity_of_vehicle = vrp_data.vehicle_fuel_capacity
    battery_discharge_rate = vrp_data.fuel_consumption_rate
    battery_recharge_rate = 1 / vrp_data.inverse_refueling_rate


    # @title Model Definition
    mdl = Model("Electric Vehicle Routing Problem With Time Windows and Partial Charging")
    mdl.set_time_limit(60 * 1)
    mdl.set_log_output(True)

    is_arc_traversed = mdl.binary_var_dict( # x
        {
            (node_i, node_j, vehicle) 
            for (node_i, node_j) in distance_between_nodes.keys() 
            for vehicle in set_of_vehicles
        }, 
        name="route"
    )  

    cargo_level_at_node = mdl.continuous_var_dict( # u
        {
            (node_i, vehicle) 
            for node_i in set_of_all_nodes 
            for vehicle in set_of_vehicles
        }, 
        lb=0, 
        ub=load_capacity_of_vehicle, 
        name="cargo_level@"
    )  

    service_start_time = mdl.continuous_var_dict( # τ
        {
            (node_i, vehicle) 
            for node_i in set_of_all_nodes 
            for vehicle in set_of_vehicles
        },
        lb=0, 
        ub=latest_arrival_time_at_depot, 
        name="arrival_time@"
    )  

    battery_soc_at_arrival = mdl.continuous_var_dict( # y
        {
            (node_i, vehicle) 
            for node_i in set_of_all_nodes 
            for vehicle in set_of_vehicles
        },
        lb=lowest_allowed_battery_level * battery_capacity_of_vehicle, 
        ub=highest_allowed_battery_level * battery_capacity_of_vehicle, 
        name="soc_at_arrival@"
    )

    battery_soc_at_departure = mdl.continuous_var_dict( # Y
        {
            (node_i, vehicle) 
            for node_i in set_of_all_nodes 
            for vehicle in set_of_vehicles
        }, 
        lb=lowest_allowed_battery_level * battery_capacity_of_vehicle, 
        ub=highest_allowed_battery_level * battery_capacity_of_vehicle, 
        name="soc_at_departure@"
    )  

    battery_overcharged_level = mdl.continuous_var_dict(
        {
            (node_i, vehicle)
            for node_i in set_of_all_nodes
            for vehicle in set_of_vehicles
        },
        lb=0,
        ub=(1-overcharge_level_for_penalty) * battery_capacity_of_vehicle,
        name="soc_overcharge_level@" 
    )

    is_battery_overcharged = mdl.binary_var_dict(
        {
            (node_i, vehicle)
            for node_i in set_of_all_nodes
            for vehicle in set_of_vehicles
        },
        name="is_overcharged@"
    )

    battery_overdischarged_level = mdl.continuous_var_dict(
        {
            (node_i, vehicle)
            for node_i in set_of_all_nodes
            for vehicle in set_of_vehicles
        },
        lb=0,
        ub=(overdischarge_level_for_penalty * battery_capacity_of_vehicle),
        name="soc_overdischarge_level@" 
    )

    is_battery_overdischarged = mdl.binary_var_dict(
        {
            (node_i, vehicle)
            for node_i in set_of_all_nodes
            for vehicle in set_of_vehicles
        },
        name="is_overdischarged@"
    )

    time_spent_at_node = mdl.continuous_var_dict( # θ
        {
            (node_i, vehicle) 
            for node_i in set_of_all_nodes 
            for vehicle in set_of_vehicles
        },
        lb=0,
        ub=latest_arrival_time_at_depot,
        name="time_spent@"
    )


    # @title Minimize cost (equation 1)
    mdl.minimize(
        1e-3 * mdl.sum(
            time_taken_between_nodes[(node_i, node_j)] * is_arc_traversed[(node_i, node_j, vehicle)]
            for node_i, node_j, vehicle in is_arc_traversed.keys()
            if valid_node_pair(node_i, node_j)
        )
        # + mdl.sum(
        #     time_spent_at_node[(node_i, vehicle)] for node_i in set_of_all_nodes for vehicle in set_of_vehicles
        # )
        penalty_for_overcharging * mdl.sum(
            battery_overcharged_level[(node_i, vehicle)] for node_i in set_of_all_nodes for vehicle in set_of_vehicles
        ) / battery_capacity_of_vehicle
        + penalty_for_overdischarging * mdl.sum(
            battery_overdischarged_level[(node_i, vehicle)] for node_i in set_of_all_nodes for vehicle in set_of_vehicles
        ) / battery_capacity_of_vehicle
    )

    # @title Vehicle Travelling Constraints
    # Each customer must be departed exactly once regardless of which vehicle
    mdl.add_constraints(
        mdl.sum(is_arc_traversed[(node_i, node_j, vehicle)] for node_j in set_of_all_nodes for vehicle in set_of_vehicles if valid_node_pair(node_i, node_j)) == 1
        for node_i in set_of_customers
    )

    # Each dummy charging station is arrived at most once per vehicle
    mdl.add_constraints(
        mdl.sum(is_arc_traversed[(node_i, node_j, vehicle)] for node_i in set_of_all_nodes if valid_node_pair(node_i, node_j)) <= 1
        for node_j in set_of_dummy_charging_stations
        for vehicle in set_of_vehicles
    )

    # Each dummy charging station is departed at most once per vehicle
    mdl.add_constraints(
        mdl.sum(is_arc_traversed[(node_i, node_j, vehicle)] for node_j in set_of_all_nodes if valid_node_pair(node_i, node_j)) <= 1
        for node_i in set_of_dummy_charging_stations
        for vehicle in set_of_vehicles
    )

    # Each customer must be arrived exactly once regardless of vehicle
    mdl.add_constraints(
        mdl.sum(is_arc_traversed[(node_i, node_j, vehicle)] for node_i in set_of_all_nodes for vehicle in set_of_vehicles if valid_node_pair(node_i, node_j)) == 1
        for node_j in set_of_customers
    )

    # Each of customer or charging_station that has been arrived must have been departed by the same vehicle
    mdl.add_constraints(
        mdl.sum(is_arc_traversed[(node_i, node_j, vehicle)] for node_j in set_of_all_nodes if valid_node_pair(node_i, node_j))
        == mdl.sum(is_arc_traversed[(node_j, node_i, vehicle)] for node_j in set_of_all_nodes if valid_node_pair(node_i, node_j))
        for node_i in set_of_customers_charging_stations
        for vehicle in set_of_vehicles
    )

    # @title Demand Constraints at all non-origin nodes should be satisfied per vehicle
    for vehicle in set_of_vehicles:
        for node_i in set_of_customers_charging_stations_start_depots:
            for node_j in set_of_customers_charging_stations_terminal_depots:
                if valid_node_pair(node_i, node_j):
                    mdl.add_indicator(
                        is_arc_traversed[(node_i, node_j, vehicle)],
                        cargo_level_at_node[(node_i, vehicle)]
                        == cargo_level_at_node[(node_j, vehicle)] + demand_at_all_nodes[node_j],
                        active_value=1,
                        name=f"Demand Constraint: {node_i} -> {node_j}",
                    )

    # @title Make sure times are accounted for per vehicle
    for vehicle in set_of_vehicles:
        for node_i in set_of_customers_charging_stations_start_depots:
            for node_j in set_of_customers_charging_stations_terminal_depots:
                if valid_node_pair(node_i, node_j):
                    mdl.add_indicator(
                        is_arc_traversed[(node_i, node_j, vehicle)],
                        service_start_time[(node_i, vehicle)] + time_spent_at_node[(node_i, vehicle)] + time_taken_between_nodes[(node_i, node_j)]
                        == service_start_time[(node_j, vehicle)],
                        active_value=1,
                    )

    # @title Ensure that service is done in the window per vehicle
    mdl.add_constraints(
        time_window_of_nodes[node_i][0] <= service_start_time[(node_i, vehicle)] 
        for node_i in set_of_all_nodes 
        for vehicle in set_of_vehicles
    )
    mdl.add_constraints(
        service_start_time[(node_i, vehicle)] <= time_window_of_nodes[node_i][1] 
        for node_i in set_of_all_nodes 
        for vehicle in set_of_vehicles
    )

    # @ title Spend only the required service time at customer and depot per vehicle
    mdl.add_constraints(
        time_spent_at_node[(node_i, vehicle)] == service_time_at_node[node_i] 
        for node_i in set_of_customers.union(set_of_start_depots, set_of_terminal_depots) 
        for vehicle in set_of_vehicles 
    )

    # Extra conditions to ensure start and terminal depot conditions are enforced per vehicle
    # (i.e. vehicle leaves start depot, and returns to terminal depot)
    mdl.add_constraints(
        mdl.sum(
            is_arc_traversed[(start_depot, node_i, vehicle)] for node_i in set_of_customers_charging_stations if valid_node_pair(start_depot, node_i)
        ) <= 1
        for start_depot in set_of_start_depots
        for vehicle in set_of_vehicles
    )
    mdl.add_constraints(
        mdl.sum(
            is_arc_traversed[(node_i, start_depot, vehicle)] for node_i in set_of_customers_charging_stations if valid_node_pair(node_i, start_depot)
        ) == 0
        for start_depot in set_of_start_depots
        for vehicle in set_of_vehicles
    )
    mdl.add_constraints(
        mdl.sum(
            is_arc_traversed[(node_i, terminal_depot, vehicle)] for node_i in set_of_customers_charging_stations if valid_node_pair(node_i, terminal_depot)
        ) <= 1
        for terminal_depot in set_of_terminal_depots
        for vehicle in set_of_vehicles
    )
    mdl.add_constraints(
        mdl.sum(
            is_arc_traversed[(terminal_depot, node_i, vehicle)] for node_i in set_of_customers_charging_stations if valid_node_pair(terminal_depot, node_i)
        ) == 0
        for terminal_depot in set_of_terminal_depots
        for vehicle in set_of_vehicles
    )


    # @title SoC constraint at all non-origin nodes per vehicle
    for vehicle in set_of_vehicles:
        for node_i in set_of_customers_charging_stations_start_depots:
            for node_j in set_of_customers_charging_stations_terminal_depots:
                if valid_node_pair(node_i, node_j):
                    mdl.add_indicator(
                        is_arc_traversed[(node_i, node_j, vehicle)],
                        battery_soc_at_departure[(node_i, vehicle)] - battery_discharge_rate * distance_between_nodes[(node_i, node_j)] == battery_soc_at_arrival[(node_j, vehicle)]
                    )

    # @title SoC constraint at customers (what you come with is what you leave with) per vehicle
    mdl.add_constraints(
        battery_soc_at_arrival[(node_i, vehicle)] == battery_soc_at_departure[(node_i, vehicle)]
        for node_i in set_of_customers 
        for vehicle in set_of_vehicles
    )

    # @title SoC constraint at charging stations
    for vehicle in set_of_vehicles:
        for node_i in set_of_dummy_charging_stations:
            for node_j in set_of_customers_charging_stations_start_depots:
                if valid_node_pair(node_i, node_j):
                    mdl.add_indicator(
                        is_arc_traversed[(node_j, node_i, vehicle)],
                        battery_soc_at_departure[(node_i, vehicle)] == battery_soc_at_arrival[(node_i, vehicle)] + battery_recharge_rate * time_spent_at_node[(node_i, vehicle)]
                )

    for vehicle in set_of_vehicles:
        for node_i in set_of_dummy_charging_stations:
            for node_j in set_of_customers_charging_stations_terminal_depots:
                if valid_node_pair(node_i, node_j):
                    mdl.add_indicator(
                        is_arc_traversed[(node_i, node_j, vehicle)],
                        battery_soc_at_departure[(node_i, vehicle)] == battery_soc_at_arrival[(node_i, vehicle)] + battery_recharge_rate * time_spent_at_node[(node_i, vehicle)]
                    )

    #@title SoC overcharged check
    for vehicle in set_of_vehicles:
        for node_i in set_of_all_nodes:
            mdl.add_constraint(
                battery_overcharged_level[(node_i, vehicle)] >= battery_soc_at_departure[(node_i, vehicle)] - (overcharge_level_for_penalty * battery_capacity_of_vehicle) - LARGE_VALUE * (1 - is_battery_overcharged[(node_i, vehicle)])
            )
            mdl.add_constraint(
                battery_overcharged_level[(node_i, vehicle)] <= battery_soc_at_departure[(node_i, vehicle)] - (overcharge_level_for_penalty * battery_capacity_of_vehicle) + LARGE_VALUE * (1 - is_battery_overcharged[(node_i, vehicle)])
            )
            mdl.add_constraint(
                battery_overcharged_level[(node_i, vehicle)] <= LARGE_VALUE * is_battery_overcharged[(node_i, vehicle)]
            )
            mdl.add_constraint(
                battery_soc_at_departure[(node_i, vehicle)] - (overcharge_level_for_penalty * battery_capacity_of_vehicle) >= -LARGE_VALUE * (1 - is_battery_overcharged[(node_i, vehicle)])
            )
            mdl.add_constraint(
                battery_soc_at_departure[(node_i, vehicle)] - (overcharge_level_for_penalty * battery_capacity_of_vehicle) <= LARGE_VALUE * is_battery_overcharged[(node_i, vehicle)]
            )

    #@title SoC overdischarge check
    for vehicle in set_of_vehicles:
        for node_i in set_of_all_nodes:
            mdl.add_constraint(
                battery_overdischarged_level[(node_i, vehicle)] >= (overdischarge_level_for_penalty * battery_capacity_of_vehicle) - battery_soc_at_arrival[(node_i, vehicle)] - LARGE_VALUE * (1 - is_battery_overdischarged[(node_i, vehicle)])
            )
            mdl.add_constraint(
                battery_overdischarged_level[(node_i, vehicle)] <= (overdischarge_level_for_penalty * battery_capacity_of_vehicle) - battery_soc_at_arrival[(node_i, vehicle)] + LARGE_VALUE * (1 - is_battery_overdischarged[(node_i, vehicle)])
            )
            mdl.add_constraint(
                battery_overdischarged_level[(node_i, vehicle)] <= LARGE_VALUE * is_battery_overdischarged[(node_i, vehicle)]
            )
            mdl.add_constraint(
                battery_soc_at_arrival[(node_i, vehicle)] <= (overdischarge_level_for_penalty * battery_capacity_of_vehicle) + LARGE_VALUE * (1 - is_battery_overdischarged[(node_i, vehicle)])
            )
            mdl.add_constraint(
                battery_soc_at_arrival[(node_i, vehicle)] >= (overdischarge_level_for_penalty * battery_capacity_of_vehicle) - LARGE_VALUE * is_battery_overdischarged[(node_i, vehicle)]
            )

    #@title Ensure SoC at arrival and at departure for terminal nodes is the same
    for vehicle in set_of_vehicles:
        for node_i in set_of_start_depots.union(set_of_terminal_depots):
            mdl.add_constraint(battery_soc_at_arrival[(node_i, vehicle)] == battery_soc_at_departure[(node_i, vehicle)])


    # Let's solve
    mdl.solve(log_output=True)

    print(f"Solve Status: {mdl.solve_status}")
    print(f"Solve Details: {mdl.solve_details}")
    print(f"Solution: {mdl.solution}")

    def determine_starting_route(active_routes: list[tuple[str, str, str]]) -> str:
        start_routes = list(filter(lambda node: node[0].startswith("D"), active_routes))
        first_route_from_depot = min(start_routes, key=lambda node: service_start_time[node[1:]].solution_value)
        return first_route_from_depot

    if mdl.solution:
        all_active_routes = [node for node, variable in is_arc_traversed.items() if variable.solution_value == 1]
        all_active_vehicles = set(route[2] for route in all_active_routes)
        print(f"All Routes: {all_active_routes}")
        routes = {}
        for vehicle in all_active_vehicles:
            all_active_routes_for_vehicle = list(filter(lambda node: node[2] == vehicle, all_active_routes))
            all_possible_starting_routes = [route for route in all_active_routes_for_vehicle if route[0].startswith("D")]
            if len(all_possible_starting_routes) == 0:
                print(f"Could not determine starting route for vehicle: {vehicle}")
                print(f"All routes: {all_active_routes}")
                print("For now, skipping")
                continue
            starting_route = determine_starting_route(all_possible_starting_routes)
            all_possible_starting_routes.pop(all_possible_starting_routes.index(starting_route))
            route = [starting_route[0]]
            while all_active_routes_for_vehicle:
                last_node = route[-1]
                if last_node.startswith("D") and last_node.endswith("T"):
                    starting_route = determine_starting_route(all_possible_starting_routes)
                    all_possible_starting_routes.pop(all_possible_starting_routes.index(starting_route))
                    route.append(starting_route[0])
                    continue

                next_path = next(filter(lambda node: node[0] == last_node, all_active_routes_for_vehicle))
                del all_active_routes_for_vehicle[all_active_routes_for_vehicle.index(next_path)]
                next_node = next_path[1]
                route.append(next_node)
            routes[vehicle] = route

            print(f"\n\n{vehicle}:: " + " -> ".join(route))


            print(f"\nFor vehicle {vehicle}, the following result is available")
            answers = [
                ["Time Spent", *[time_spent_at_node[(node, vehicle)].solution_value for node in route]],
                ["Service Time", *[service_time_at_node[node] for node in route]],
                ["Arrival Time", *[service_start_time[(node, vehicle)].solution_value for node in route]],
                ["SoC @ Arrival", *[battery_soc_at_arrival[(node, vehicle)].solution_value for node in route]],
                ["SoC @ Departure", *[battery_soc_at_departure[(node, vehicle)].solution_value for node in route]],
                ["Cargo Level", *[cargo_level_at_node[(node, vehicle)].solution_value for node in route]],
            ]

            results = pd.DataFrame(answers, columns = ["", *[node for node in route]])
            base_path = solution_path / file_name.with_suffix("").name
            if not os.path.exists(base_path):
                os.makedirs(base_path)
            save_path = base_path / f"{vehicle.replace('|', '')}.csv"
            results.to_csv(save_path, index=False)
            print(results)
        
        with open(base_path / "obj.txt", "w") as f:
            f.write("Solve Status\n")
            f.write(str(mdl.solve_status))
            f.write("\nSolve Details\n")
            f.write(str(mdl.solve_details))
            f.write("\nSolution\n")
            f.write(str(mdl.solution))

    else:
        conflict_refinement = ConflictRefiner()
        conflicts = conflict_refinement.refine_conflict(mdl, display=True)
        conflict_refinement.display_conflicts(conflicts)