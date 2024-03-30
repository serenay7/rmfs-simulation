import numpy as np
import pandas as pd
import simpy
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask
import koridor_deneme
import random
import ast
from vrp import distanceMatrixCreate

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

rows = 10  # 3x3
columns = 16
rectangular_network, pos = koridor_deneme.create_rectangular_network_with_attributes(columns, rows)
koridor_deneme.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
koridor_deneme.draw_network_with_shelves(rectangular_network, pos)
network_corridors = koridor_deneme.create_corridor_subgraph(rectangular_network)

distMatrix, nodes = distanceMatrixCreate(rectangular_network)

###

station_nodes = np.array(['(0,0)', '(15,0)']) # sol alt
pod_nodes = np.array(['(1,1)', '(1,4)', '(1,7)', '(6,1)', '(6,4)', '(6,7)', '(11,1)', '(11,4)', '(11,7)',])
max_percentage = 0.5

def min_max_diff(combination, no_of_pods):
    min_values = np.min(combination, axis=1)
    max_values = np.max(combination, axis=1)
    min_max_diff = (max_values - min_values)/no_of_pods
    return min_max_diff

def check_feasbility(arr):
    if np.all(arr == np.inf):
        print("No feasible solution.")
    else:
        min_index = np.argmin(arr)
        return min_index

def mainPodSelection(pod_nodes, station_nodes, max_percentage):
    
    no_of_pods = len(pod_nodes)
    no_of_stations = len(station_nodes)

    podAndStation_distance = np.zeros(shape=(no_of_pods, no_of_stations)) # empty matrix

    combination = podAndStation_combination(no_of_pods,no_of_stations)
    no_of_combinations = len(combination)
    
    combinationPodDistance = np.zeros(shape=(no_of_pods, no_of_combinations)) # empty matrix

    # Convert the string representations to tuples
    station_nodes_tuples = np.array([convert_to_tuple(node) for node in station_nodes])
    pod_nodes_tuples = np.array([convert_to_tuple(node) for node in pod_nodes])

    for i, pod in enumerate(pod_nodes_tuples):
        for j, station in enumerate(station_nodes_tuples):
            distance = abs(pod[0] - station[0]) + abs(pod[1] - station[1])
            podAndStation_distance[i, j] = distance

    print("pod and station distance")
    print(podAndStation_distance)

    # Iterate through each combination
    for combination_index, comb in enumerate(combination):
        # Iterate through each pod
        for pod_index in range(no_of_pods):
            # Initialize the total distance for this pod in this combination
            total_distance = 0
            # Iterate through each station in the combination
            for station_index, multiplier in enumerate(comb):
                # Calculate the distance for this pod to this station, multiplied by the combination's multiplier
                distance = podAndStation_distance[pod_index, station_index] * multiplier
                # Add to the total distance for this pod in this combination
                total_distance += distance
            # Store the total distance in the matrix
            combinationPodDistance[pod_index, combination_index] = total_distance

    print("pod-combination distances")
    print(combinationPodDistance)

    # total distances of combinations
    combinationTotalDistance = np.zeros(shape=(1, no_of_combinations)) # empty matrix
    combinationTotalDistance = np.sum(combinationPodDistance, axis=0)

    #print("total distances for each combination")
    #print(combinationTotalDistance)

    percentages = min_max_diff(combination, no_of_pods)
    #print("Percentages for each combination")
    #print(percentages)

     # Find indexes where percentages exceed max_percentage
    exceed_indexes = np.where(percentages > max_percentage)[0]

    # Set the values at these indexes in combinationTotalDistance to infinity
    combinationTotalDistance[exceed_indexes] = np.inf

    print("Adjusted total distances for each combination")
    print(combinationTotalDistance)

    result_idx = check_feasbility(combinationTotalDistance)
    result_combination = combination[result_idx]
    result_totalDistance = combinationTotalDistance[result_idx]

    if result_idx is not None:
        print("Index of the minimum value in arr1:", result_idx)
        print("Combination at index:", result_combination)
        print("Total Distance at index:", result_totalDistance)

    return print("over")

t = mainPodSelection(pod_nodes, station_nodes, max_percentage)
t