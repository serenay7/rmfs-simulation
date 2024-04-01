import numpy as np
import pandas as pd
import simpy
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask
import koridor_deneme
import random
import ast
from scipy.optimize import linear_sum_assignment
from vrp import distanceMatrixCreate
import rawsimo
import generators
import podstorage


def podAndStation_combination_recursive(pods, stations, current_distribution=[], all_distributions=[]):
    # Base case: if we have allocated all baskets
    if stations == 1:
        # Add the remaining apples to the last basket
        all_distributions.append(current_distribution + [pods])
    else:
        # Distribute apples among remaining baskets
        for i in range(pods + 1):
            podAndStation_combination_recursive(pods - i, stations - 1, current_distribution + [i], all_distributions)
    return all_distributions


def podAndStation_combination(pods, stations):
    # Get all distributions
    distributions = podAndStation_combination_recursive(pods, stations)
    # Convert the list of distributions to a NumPy array
    distribution_matrix = np.array(distributions)
    return distribution_matrix


def convert_to_tuple(node):
    """Convert a string representation of a tuple to an actual tuple of integers."""
    return tuple(map(int, node.strip('()').split(',')))


def min_max_diff(combination, no_of_pods):
    min_values = np.min(combination, axis=1)
    max_values = np.max(combination, axis=1)
    min_max_diff = (max_values - min_values) / no_of_pods
    return min_max_diff


def check_feasbility(arr):
    if np.all(arr == np.inf):
        print("No feasible solution.")
    else:
        min_index = np.argmin(arr)
        return min_index


def columnMultiplication(distanceMatrix, requirements):
    """distanceMatrix: nparray rows as pods, columns as stations
    requirements: nparray length is no of stations"""

    new_columns = [np.repeat(distanceMatrix[:, i:i + 1], repeats=req, axis=1) for i, req in enumerate(requirements)]
    expanded_matrix = np.hstack(new_columns)

    return expanded_matrix


def column_to_station_mapping(requirements):
    """
    Create a mapping array from expanded matrix columns back to original station indices.
    requirements: np.array, where each element is the number of workers required for the corresponding station.
    Returns:
    - np.array, where each element indicates the original station index for each column in the expanded matrix.
    """
    return np.concatenate([np.full(req, i) for i, req in enumerate(requirements)])


def assign_pods_to_stations(distanceMatrix, requirements):
    """
    Assign pods to stations based on distanceMatrix and requirements using linear sum assignment,
    with an expanded matrix to handle multiple worker requirements per station.

    Parameters:
    - distanceMatrix: np.array, rows as pods and columns as stations, containing distances or costs.
    - requirements: np.array, length equal to the number of stations, indicating workers required per station.

    Returns:
    - A tuple of (assigned_pods, assigned_stations, total_distance), where:
        - assigned_pods: np.array, indices of pods assigned to each task.
        - assigned_stations: np.array, corresponding station indices for each assignment.
        - total_distance: float, the sum of distances for the optimal assignment.
    """
    from scipy.optimize import linear_sum_assignment

    # Expand the distance matrix based on requirements
    expanded_matrix = columnMultiplication(distanceMatrix, requirements)

    # Use linear sum assignment on the expanded matrix
    row_ind, col_ind = linear_sum_assignment(expanded_matrix)
    total_distance = expanded_matrix[row_ind, col_ind].sum()

    # Map the column indices in the expanded matrix back to original station indices
    column_station_mapping = column_to_station_mapping(requirements)
    assigned_stations = column_station_mapping[col_ind]

    return (row_ind, assigned_stations, total_distance)


def calculate_total_distances_for_all_requirements(distanceMatrix, PS_combination):
    """
    Calculate total distances for each set of requirements in PS_combination.

    Parameters:
    - distanceMatrix: np.array, rows as pods and columns as stations, containing distances or costs.
    - PS_combination: np.array, each row represents a set of requirements for the stations.

    Returns:
    - total_distances: np.array, containing the total distance for each set of requirements.
    """
    total_distances = np.zeros(len(PS_combination))  # Initialize an array to store total distances

    # Iterate through each set of requirements in PS_combination
    for i, requirements in enumerate(PS_combination):
        # Use assign_pods_to_stations to calculate the total distance for the current set of requirements
        _, _, total_distance = assign_pods_to_stations(distanceMatrix, requirements)

        # Store the calculated total distance in the array
        total_distances[i] = total_distance

    return total_distances


def podSelectionLP(itemList, podDict, podMatrix):
    selectedPodNodes = []
    while len(itemList) > 0:
        selected, satisfiedSKU = rawsimo.rawsimoPodSelectionExperiment(itemList=itemList, podMatrix=podMatrix)
        itemList = [sublist for sublist in itemList if sublist[0] not in satisfiedSKU]
        selectedNode = podDict[selected]
        selectedPodNodes.append(selectedNode)

    return selectedPodNodes


def mainPodAssignment(pod_nodes, station_nodes, max_percentage):
    no_of_pods = len(pod_nodes)
    no_of_stations = len(station_nodes)

    podAndStation_distance = np.zeros(shape=(no_of_pods, no_of_stations))  # empty matrix

    combination = podAndStation_combination(no_of_pods, no_of_stations)

    # Convert the string representations to tuples
    station_nodes_tuples = np.array([convert_to_tuple(node) for node in station_nodes])
    pod_nodes_tuples = np.array([convert_to_tuple(node) for node in pod_nodes])

    for i, pod in enumerate(pod_nodes_tuples):
        for j, station in enumerate(station_nodes_tuples):
            distance = abs(pod[0] - station[0]) + abs(pod[1] - station[1])
            podAndStation_distance[i, j] = distance

    print("pod and station distance")
    print(podAndStation_distance)

    # total distances of combinations
    combinationTotalDistance = calculate_total_distances_for_all_requirements(podAndStation_distance, combination)

    # print("total distances for each combination")
    # print(combinationTotalDistance)

    percentages = min_max_diff(combination, no_of_pods)
    # print("Percentages for each combination")
    # print(percentages)

    # Find indexes where percentages exceed max_percentage
    exceed_indexes = np.where(percentages > max_percentage)[0]

    # Set the values at these indexes in combinationTotalDistance to infinity
    combinationTotalDistance[exceed_indexes] = np.inf

    print("Adjusted total distances for each combination")
    print(combinationTotalDistance)

    result_idx = check_feasbility(combinationTotalDistance)
    requirement = combination[result_idx]
    testMatrix = columnMultiplication(podAndStation_distance, requirement)
    assigned_pods, assigned_stations, total_distance = assign_pods_to_stations(podAndStation_distance, requirement)

    if result_idx is not None:
        print("Index of the minimum value:", result_idx)
        print("Combination at index:", requirement)
        print("Total Distance at index:", total_distance)
        print("assgned pods:", assigned_pods)
        print("assgned stations:", assigned_stations)

    # PS_distance = her podun her istasyona uzaklığı
    # PS_combination = all possible combinations
    # requirement = en iyileyen kombinasyon
    # testMatrix = hungarian için duplike edilen distance matrix
    # assigned_pods = direkt olarak saf hungarian çıktısı, istasyon bilgisi yok
    # assigned_stations = pod istasyon eşleşmeleri, from assigned_pods

    return podAndStation_distance, combination, requirement, testMatrix, assigned_pods, assigned_stations, total_distance


def PhaseIExperiment(orderList, podMatrix, network, stationNodes, max_percentage=0.5):
    # bizim assignment problem kısmı
    itemList = np.sum(orderList, axis=0)
    itemList = [[sku, int(amount)] for sku, amount in enumerate(itemList)]
    nodesDict = network._node  # kendi node değişkeninden farklı, pod olup olmadığının bilgisini de dict olarak veriyor
    shelvesNetworkNodes = {k: v for k, v in nodesDict.items() if v == {'shelf': True}}.keys()
    # podların indexi ve nodelar arasındaki bağlantıyı gösteren dictionary, pod listesindeki sıraya göre podları labellıyor
    podDict = {k: v for k, v in enumerate(shelvesNetworkNodes)}
    selectedPodNodes = podSelectionLP(itemList, podDict, podMatrix)

    # bu kısım mainPodAssignment ile inputlar uyumlu olsun diye yapıldı
    selectedPodNodes = [str(i) for i in selectedPodNodes]
    selectedPodNodes = np.array(selectedPodNodes).T
    PS_distance, PS_combination, requirement, testMatrix, assigned_pods, assigned_stations, total_distance = mainPodAssignment(
        selectedPodNodes, stationNodes, max_percentage)
    # assigned_stations hangi pod nereye atandı

    # rawsimo kısmı
    def manhattan_distance(tuple1, tuple2):
        return sum(abs(a - b) for a, b in zip(tuple1, tuple2))

    def sum_manhattan_distance(target_tuple, list_of_tuples):
        return sum(manhattan_distance(target_tuple, t) for t in list_of_tuples)

    orderListDivided = np.reshape(orderList, newshape=(len(stationNodes), orderList.shape[0] // len(stationNodes), orderList.shape[1]))
    numSelectedPods = 0
    totalDist = 0
    for stationIdx, stationLocation in enumerate(stationNodes):
        itemListDivided = np.sum(orderListDivided[stationIdx], axis=0)
        itemListDivided = [[sku, int(amount)] for sku, amount in enumerate(itemListDivided)]
        selectedPodNodes = podSelectionLP(itemListDivided, podDict, podMatrix)
        numSelectedPods += len(selectedPodNodes)
        totalDist += sum_manhattan_distance(eval(stationLocation), selectedPodNodes)



if __name__ == "__main__":
    """
    rows = 10  # 3x3
    columns = 16
    rectangular_network, pos = koridor_deneme.create_rectangular_network_with_attributes(columns, rows)
    koridor_deneme.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
    koridor_deneme.draw_network_with_shelves(rectangular_network, pos)
    network_corridors = koridor_deneme.create_corridor_subgraph(rectangular_network)

    distMatrix, nodes = distanceMatrixCreate(rectangular_network)
    """
    ###

    station_nodes = np.array(['(0,0)', '(15,0)'])  # sol alt
    pod_nodes = np.array(['(1,1)', '(1,4)', '(1,7)', '(6,1)', '(6,4)', '(6,7)', '(11,1)', '(11,4)', '(11,7)', ])
    max_percentage = 0.5

    # PS_distance, PS_combination, requirement, testMatrix, assigned_pods, assigned_stations, total_distance = mainPodAssignment(pod_nodes, station_nodes, max_percentage)

    # requirements = np.array([7, 2])
    # testMatrix = columnMultiplication(PS_distance, requirements)
    # assigned_pods, assigned_stations, total_distance = assign_pods_to_stations(PS_distance, requirements)

    # all_total_distances = calculate_total_distances_for_all_requirements(PS_distance, PS_combination)
    # print("all total distances", all_total_distances)

    s = 5  # Number of SKUs
    r = 72  # Number of storage pods
    n = 500  # Number of each SKU stored in the warehouse
    k = 20  # Maximum number of pods for each SKU
    lower_bound = 100  # Lower bound of the amount interval
    upper_bound = 200  # Upper bound of the amount interval

    rectangular_network, network_corridors = generators.create_network(3, 3)
    podMatrix = podstorage.generate_distribution_matrix(s, r, k, lower_bound, upper_bound).T
    orderList = generators.orderGenerator(5, 2, s, 5)
    PhaseIExperiment(orderList, podMatrix, rectangular_network, station_nodes)