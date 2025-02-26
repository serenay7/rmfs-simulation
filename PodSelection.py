import numpy as np


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
    distributions = [lst for lst in distributions if len(lst) == stations]
    distributions = [lst for lst in distributions if sum(lst) == pods]
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

def check_feasibility(arr):
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

    # print("pod and station distance")
    # print(podAndStation_distance)

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

    #print("Adjusted total distances for each combination")
    #print(combinationTotalDistance)

    result_idx = check_feasibility(combinationTotalDistance)
    requirement = combination[result_idx]
    testMatrix = columnMultiplication(podAndStation_distance, requirement)
    assigned_pods, assigned_stations, total_distance = assign_pods_to_stations(podAndStation_distance, requirement)

    return podAndStation_distance, combination, requirement, testMatrix, assigned_pods, assigned_stations, total_distance



def stationLocationFinder(network, numStation):
    nodes = list(network.nodes)
    station_nodes = []

    if numStation == 2:
        firstStation = (nodes[0][0], (nodes[0][1] + nodes[-1][1])//2)
        station_nodes.append(str(firstStation))
        secondStation = (nodes[-1][0], (nodes[0][1] + nodes[-1][1])//2)
        station_nodes.append(str(secondStation))

    elif numStation == 4:
        firstStation = ((nodes[0][0] + nodes[-1][0]) // 2, nodes[0][1])
        station_nodes.append(str(firstStation))
        secondStation = ((nodes[0][0] + nodes[-1][0]) // 2, nodes[-1][1])
        station_nodes.append(str(secondStation))
        thirdStation = (nodes[0][0], (nodes[0][1] + nodes[-1][1]) // 2)
        station_nodes.append(str(thirdStation))
        fourthStation = (nodes[-1][0], (nodes[0][1] + nodes[-1][1]) // 2)
        station_nodes.append(str(fourthStation))
    else:
        print("Warning wrong numStation in stationLocationFinder")

    return np.array(station_nodes)

