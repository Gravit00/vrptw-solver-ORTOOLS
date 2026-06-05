import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


# =============================================================================
# STEP 1: Parse Dataset
# =============================================================================

def parse_solomon(filepath):
    """
    Parse a Solomon VRPTW benchmark file (e.g. c101.txt).

    Returns:
        num_vehicles: number of available vehicles
        capacity:     vehicle capacity
        depot:        depot node as a dict
        customers:    list of customer nodes as dicts
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Find vehicle count and capacity (line after "NUMBER")
    for i, line in enumerate(lines):
        if 'NUMBER' in line:
            parts = lines[i + 1].split()
            num_vehicles = int(parts[0])
            capacity = int(parts[1])
            break

    # Find start of customer data (line after "CUST NO.")
    customer_start = None
    for i, line in enumerate(lines):
        if 'CUST' in line and 'NO' in line:
            customer_start = i + 1
            break

    depot = None
    customers = []

    for line in lines[customer_start:]:
        parts = line.split()
        if len(parts) == 0:
            continue

        node = {
            'id':           int(parts[0]),
            'x':            float(parts[1]),
            'y':            float(parts[2]),
            'demand':       float(parts[3]),
            'ready_time':   float(parts[4]),
            'due_date':     float(parts[5]),
            'service_time': float(parts[6]),
        }

        if node['id'] == 0:
            depot = node
        else:
            customers.append(node)

    return num_vehicles, capacity, depot, customers


# =============================================================================
# STEP 2: Build Model
# =============================================================================

def compute_distance_matrix(depot, customers):
    """Compute Euclidean distance matrix for all nodes (depot + customers)."""
    nodes = [depot] + customers
    n = len(nodes)
    dist_matrix = [[0.0] * n for _ in range(n)]

    for i in range(n):
        for j in range(n):
            if i != j:
                dx = nodes[i]['x'] - nodes[j]['x']
                dy = nodes[i]['y'] - nodes[j]['y']
                dist_matrix[i][j] = math.sqrt(dx**2 + dy**2)

    return dist_matrix


def create_model(num_vehicles, capacity, depot, customers, dist_matrix):
    """Create the OR-Tools routing model and register the distance callback."""
    manager = pywrapcp.RoutingIndexManager(
        len(dist_matrix),   # total nodes (depot + customers)
        num_vehicles,
        0                   # depot index
    )
    routing = pywrapcp.RoutingModel(manager)

    # OR-Tools requires integer costs; round floats to nearest int
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return round(dist_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    return manager, routing


def add_capacity_constraint(routing, manager, customers, capacity, num_vehicles):
    """Add vehicle capacity constraint."""
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        if from_node == 0:
            return 0  # depot has no demand
        return int(customers[from_node - 1]['demand'])

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,                          # no slack
        [capacity] * num_vehicles,  # capacity per vehicle
        True,                       # start cumul from zero
        'Capacity'
    )


def add_time_window_constraint(routing, manager, depot, customers, dist_matrix):
    """Add time window constraints for all nodes."""
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        # Travel time + service time at current node
        service_time = 0 if from_node == 0 else customers[from_node - 1]['service_time']
        return round(dist_matrix[from_node][to_node] + service_time)

    time_callback_index = routing.RegisterTransitCallback(time_callback)

    routing.AddDimension(
        time_callback_index,
        1236,   # max waiting time (slack)
        1236,   # max time horizon (depot due date)
        False,  # do not force start cumul to zero (vehicles may wait)
        'Time'
    )

    time_dimension = routing.GetDimensionOrDie('Time')

    nodes = [depot] + customers
    for i, node in enumerate(nodes):
        index = manager.NodeToIndex(i)
        time_dimension.CumulVar(index).SetRange(
            int(node['ready_time']),
            int(node['due_date'])
        )


# =============================================================================
# STEP 3: Solve with OR-Tools
# =============================================================================

def solve(routing, manager, customers, num_vehicles, dist_matrix):
    """
    Solve the VRPTW using OR-Tools with Guided Local Search.

    Returns:
        routes:        list of routes, each as a list of node indices [0, ..., 0]
        total_distance: total travel distance (float)
    """
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.seconds = 30

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        routes = []
        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)
            route = []

            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                route.append(node)
                index = solution.Value(routing.NextVar(index))

            route.append(0)  # return to depot

            if len(route) > 2:  # skip empty routes
                routes.append(route)

        # Recompute distance directly from routes for consistency with heuristic
        total_distance = sum(
            dist_matrix[route[i]][route[i + 1]]
            for route in routes
            for i in range(len(route) - 1)
        )

        print(f"[OR-Tools] Vehicles used: {len(routes)}")
        print(f"[OR-Tools] Total distance: {total_distance:.2f}")
        return routes, total_distance
    else:
        print("No solution found.")
        return None, None


# =============================================================================
# STEP 4: Nearest Neighbor Heuristic (Baseline)
# =============================================================================

def nearest_neighbor_vrptw(depot, customers, dist_matrix, capacity):
    """
    Greedy Nearest Neighbor heuristic for VRPTW.

    At each step, the vehicle moves to the nearest unvisited customer
    that satisfies both the time window and capacity constraints.
    When no feasible customer is reachable, the vehicle returns to the
    depot and a new vehicle is dispatched.

    Returns:
        routes:         list of routes, each as [0, ..., 0]
        total_distance: total travel distance (float)
    """
    nodes = [depot] + customers
    unvisited = set(range(1, len(nodes)))  # customer indices 1..n
    routes = []
    total_distance = 0.0

    while unvisited:
        # Dispatch a new vehicle from the depot
        route = [0]
        current = 0
        current_time = 0.0
        current_load = 0.0
        route_distance = 0.0

        while True:
            best = None
            best_dist = float('inf')

            for j in unvisited:
                node = nodes[j]
                d = dist_matrix[current][j]
                arrival = max(current_time + d, node['ready_time'])

                # Check time window and capacity feasibility
                if arrival <= node['due_date'] and current_load + node['demand'] <= capacity:
                    if d < best_dist:
                        best_dist = d
                        best = j

            if best is None:
                break  # no feasible customer; return to depot

            # Visit the chosen customer
            node = nodes[best]
            route_distance += dist_matrix[current][best]
            current_time = max(current_time + dist_matrix[current][best], node['ready_time'])
            current_time += node['service_time']
            current_load += node['demand']
            route.append(best)
            unvisited.remove(best)
            current = best

        # Return to depot
        route_distance += dist_matrix[current][0]
        route.append(0)
        routes.append(route)
        total_distance += route_distance

    print(f"[Nearest Neighbor] Vehicles used: {len(routes)}")
    print(f"[Nearest Neighbor] Total distance: {total_distance:.2f}")
    return routes, total_distance


# =============================================================================
# STEP 5: Visualisation
# =============================================================================

def visualize_routes(depot, customers, routes, title='VRPTW Routes - C101'):
    """Plot vehicle routes on a 2D map."""
    nodes = [depot] + customers
    fig, ax = plt.subplots(figsize=(12, 10))
    colors = cm.tab20(np.linspace(0, 1, len(routes)))

    for route, color in zip(routes, colors):
        xs = [nodes[n]['x'] for n in route]
        ys = [nodes[n]['y'] for n in route]
        ax.plot(xs, ys, color=color, linewidth=1.5)

    for c in customers:
        ax.scatter(c['x'], c['y'], color='steelblue', s=30, zorder=3)

    ax.scatter(depot['x'], depot['y'], color='red', s=100, marker='*', zorder=4, label='Depot')
    ax.set_title(title)
    ax.legend()
    plt.tight_layout()
    plt.show()


def plot_comparison(nn_routes, nn_distance, or_routes, or_distance):
    """Bar chart comparing Nearest Neighbor and OR-Tools on vehicles and distance."""
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    methods = ['Nearest Neighbor', 'OR-Tools']
    vehicles = [len(nn_routes), len(or_routes)]
    distances = [nn_distance, or_distance]
    colors = ['#e07b54', '#5b8db8']

    # Left: vehicles used
    axes[0].bar(methods, vehicles, color=colors)
    axes[0].set_title('Vehicles Used')
    axes[0].set_ylabel('Number of Vehicles')
    for i, v in enumerate(vehicles):
        axes[0].text(i, v + 0.3, str(v), ha='center', fontweight='bold')

    # Right: total distance
    axes[1].bar(methods, distances, color=colors)
    axes[1].set_title('Total Distance')
    axes[1].set_ylabel('Distance')
    for i, v in enumerate(distances):
        axes[1].text(i, v + 10, f'{v:.0f}', ha='center', fontweight='bold')

    plt.suptitle('VRPTW C101: Nearest Neighbor vs OR-Tools', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig('comparison.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("Comparison chart saved as comparison.png")


# =============================================================================
# Main
# =============================================================================

if __name__ == '__main__':
    # --- Parse data ---
    num_vehicles, capacity, depot, customers = parse_solomon('c101.txt')

    # --- Build distance matrix ---
    dist_matrix = compute_distance_matrix(depot, customers)

    # --- Build and solve OR-Tools model ---
    manager, routing = create_model(num_vehicles, capacity, depot, customers, dist_matrix)
    add_capacity_constraint(routing, manager, customers, capacity, num_vehicles)
    add_time_window_constraint(routing, manager, depot, customers, dist_matrix)
    or_routes, or_distance = solve(routing, manager, customers, num_vehicles, dist_matrix)

    # --- Visualise OR-Tools routes ---
    visualize_routes(depot, customers, or_routes)

    # --- Run Nearest Neighbor heuristic ---
    nn_routes, nn_distance = nearest_neighbor_vrptw(depot, customers, dist_matrix, capacity)

    # --- Compare ---
    plot_comparison(nn_routes, nn_distance, or_routes, or_distance)

