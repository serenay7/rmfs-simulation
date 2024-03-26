import numpy as np
import networkx as nx
import koridor_deneme
import pandas as pd
import ast
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def distanceMatrixCreate(G):
    """Takes a network as input, returns a Distance Matrix and the list of nodes."""

    shortest_paths = dict(nx.all_pairs_shortest_path_length(G))
    
    nodes = list(G.nodes)
    num_nodes = len(nodes)
    distance_matrix = np.zeros((num_nodes, num_nodes))

    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                if nodes[j] in shortest_paths[nodes[i]]:
                    distance_matrix[i][j] = shortest_paths[nodes[i]][nodes[j]]
                else:
                    distance_matrix[i][j] = float('inf')
    
    return distance_matrix, nodes

def fixedLocationVRP(distanceMatrix, taskList):
    pass

def convert_string_tuples_to_int_tuples(array_of_string_tuples):
    return [tuple(ast.literal_eval(item)) for item in array_of_string_tuples]

def get_node_index(nodes, target_node):
    """Returns the index of target_node in the nodes list.
    Parameters:
    - nodes (list): A list of nodes (tuples).
    - target_node (tuple): The node for which to find the index.
    
    Returns:
    - int: The index of target_node in nodes
    """
    try:
        return nodes.index(target_node)
    except ValueError:
        return print("error")


def taskDistanceMatrix(tasks, nodes, distanceMatrix, start_node, end_node):
    """Inputs
    tasks (ndarray): array of task nodes
    nodes (list): nodes list directly from networkx
    distanceMatrix (ndarray): created with distanceMatrixCreate
    start_node (tuple): initial node to be added to tasks if not present, default is (0,0)
    
    Ouput: Distance matrix consisting distances only from task nodes to other task nodes."""

    formatted_tasks = convert_string_tuples_to_int_tuples(tasks.tolist())

    if start_node not in formatted_tasks:
        formatted_tasks.append(start_node)

    if end_node not in formatted_tasks:
        formatted_tasks.append(end_node)
    
    matching_indexes = [i for i, item in enumerate(nodes) if item in formatted_tasks]

    taskDistanceMatrix = distanceMatrix[np.ix_(matching_indexes, matching_indexes)]
    filteredNodes = [nodes[i] for i in matching_indexes]  # Filter nodes based on matching_indexes

    taskDistanceMatrix = np.rint(taskDistanceMatrix).astype(int)

    return taskDistanceMatrix, filteredNodes

## VRP MODEL
def create_data_model(distanceMatrix, numVehicles, start_index, end_index):
    """Stores the data for the problem."""
    data = {}
    data["distance_matrix"] = distanceMatrix
    data["num_vehicles"] = numVehicles
    data["starts"] = start_index
    data["ends"] = end_index
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    max_route_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print(f"Maximum of the route distances: {max_route_distance}m")


def main(distanceMatrix, numVehicles, start_index, end_index):
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model(distanceMatrix, numVehicles, start_index, end_index)
    number_of_nodes = len(data["distance_matrix"])
    number_of_vehicles = data["num_vehicles"]
    starts = start_index  # Ensure this is a list, even for a single vehicle
    ends = end_index  # Ensure this is a list, even for a single vehicle

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(number_of_nodes, number_of_vehicles, starts, ends)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        2000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)



if __name__ == "__main__":

    rows = 10  # 3x3
    columns = 16

    rectangular_network, pos = koridor_deneme.create_rectangular_network_with_attributes(columns, rows)
    koridor_deneme.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
    koridor_deneme.draw_network_with_shelves(rectangular_network, pos)
    network_corridors = koridor_deneme.create_corridor_subgraph(rectangular_network)

    distMatrix, nodes = distanceMatrixCreate(rectangular_network)

    taskdf1 = pd.read_excel("2pickstation-2robot.xlsx", sheet_name="Sim3-East")
    taskdf1 = taskdf1["SimPy Location"]
    task1arr = np.unique(taskdf1.to_numpy())

    taskdf2 = pd.read_excel("2pickstation-2robot.xlsx", sheet_name="Sim3-North")
    taskdf2 = taskdf2["SimPy Location"]
    task2arr = taskdf2.to_numpy()
    stacked_arr = np.concatenate((task1arr, task2arr))
    print(distMatrix)
    print(nodes)
    ab=10
    
    # VRP
    start_node = (0,0)
    end_node = (0,0)
    numVehicles = 2

    distMatrix_stacked, nodes_stacked = taskDistanceMatrix(stacked_arr, nodes, distMatrix, start_node, end_node)

    start_index = get_node_index(nodes_stacked, start_node)
    end_index = get_node_index(nodes_stacked, end_node)

    start_idx = [start_index, start_index]
    end_idx = [start_index, start_index]

    main(distMatrix_stacked, numVehicles, start_idx, end_idx)