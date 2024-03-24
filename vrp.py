import numpy as np
import networkx as nx
import ortools
import koridor_deneme
import pandas as pd
import ast

def distanceMatrixCreate(G):

    shortest_paths = dict(nx.all_pairs_shortest_path_length(G))

    nodes = list(G.nodes)
    num_nodes = len(nodes)
    distance_matrix = np.zeros((num_nodes, num_nodes))

    # Fill distance matrix with shortest path lengths
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                if nodes[j] in shortest_paths[nodes[i]]:
                    distance_matrix[i][j] = shortest_paths[nodes[i]][nodes[j]]
                else:
                    distance_matrix[i][j] = float('inf')

    #print("Distance Matrix:")
    #print(distance_matrix)
    return distance_matrix, nodes

def fixedLocationVRP(distanceMatrix, taskList):
    pass

def convert_string_tuples_to_int_tuples(array_of_string_tuples):
    # Use ast.literal_eval to safely evaluate the string as a Python expression
    # Then convert each element to a tuple of integers
    return [tuple(ast.literal_eval(item)) for item in array_of_string_tuples]

def taskDistanceMatrix(tasks, nodes, distanceMatrix):
    """Inputs
    tasks (ndarray): array of tasks from
    nodes (list): nodes list directly from networkx
    distanceMatrix (ndarray): created with distanceMatrixCreate
    
    Ouput: Distance matrix consisting distances only from task nodes to other task nodes."""

    formatted_tasks = convert_string_tuples_to_int_tuples(tasks.tolist())

    same_count = sum(1 for item in formatted_tasks if item in nodes)
    print(f"Number of matching items: {same_count}")
    
    matching_indexes = [i for i, item in enumerate(nodes) if item in formatted_tasks]
    print(f"Indexes of matching items in nodes_df1: {matching_indexes}")

    taskDistanceMatrix = distanceMatrix[np.ix_(matching_indexes, matching_indexes)]
    return taskDistanceMatrix


if __name__ == "__main__":

    rows = 10  # 3x3
    columns = 16

    rectangular_network, pos = koridor_deneme.create_rectangular_network_with_attributes(columns, rows)
    koridor_deneme.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
    koridor_deneme.draw_network_with_shelves(rectangular_network, pos)
    network_corridors = koridor_deneme.create_corridor_subgraph(rectangular_network)

    distMatrix, nodes = distanceMatrixCreate(rectangular_network)

    taskdf1 = pd.read_excel("2pickstation-2robot.xlsx", sheet_name="Sim2-East")
    taskdf1 = taskdf1["SimPy Location"]
    task1arr = np.unique(taskdf1.to_numpy())


    taskdf2 = pd.read_excel("2pickstation-2robot.xlsx", sheet_name="Sim2-West")
    taskdf2 = taskdf2["SimPy Location"]
    task2arr = taskdf2.to_numpy()
    stacked_arr = np.concatenate((task1arr, task2arr))
    print(distMatrix)
    print(nodes)
    ab=10


    bot1DistMatrix = taskDistanceMatrix(task1arr, nodes, distMatrix)
    print(bot1DistMatrix)