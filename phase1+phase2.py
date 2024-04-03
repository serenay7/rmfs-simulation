import lp_podselection
import vrp
import generators
import podstorage
import pandas as pd
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np

def main(distanceMatrix, numVehicles, start_index, end_index):
    """Entry point of the program."""
    # Instantiate the data problem.
    data = vrp.create_data_model(distanceMatrix, numVehicles, start_index, end_index)
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
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # initial solution için buradaki yöntemi kullanıyor
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        dflist = vrp.print_solution(data, manager, routing, solution)
        return data, manager, routing, solution, dflist


def solve_vrp(numVehicles, rectangular_network, stacked_arr):
    #Gerekli inputları oluşturup main'i çağırıyor
    distMatrix, nodes = vrp.distanceMatrixCreate(rectangular_network)

        # VRP
    start_node = (0,0)
    nodes = list(rectangular_network.nodes)
    end_node = nodes[-1]

    distMatrix_stacked, nodes_stacked = vrp.taskDistanceMatrix(stacked_arr, nodes, distMatrix, start_node, end_node)

    n = len(distMatrix_stacked)
    new_n = n + 3
    larger_array = np.full((new_n, new_n),1000)
    larger_array[3:n + 3, 3:n + 3] = distMatrix_stacked
    larger_array[0] = np.zeros(shape=(1,len(larger_array[0])))
    larger_array[1] = np.zeros(shape=(1, len(larger_array[1])))
    larger_array[:, 2] = 0

    start_index = vrp.get_node_index(nodes_stacked, start_node)
    end_index = vrp.get_node_index(nodes_stacked, end_node)
    #Stacked array 2 columnlı verilebilir, columnlardan biri drop edilip kalan column 1-d array yapılacak
    start_idx = [0, 1]
    #if numVehicles == 4: start_idx = [0,0,20,20]
    end_idx = [2, 2]
    #if numVehicles == 4: end_idx = [0, 0, 20, 20]
    data, manager, routing, solution, dflist = main(larger_array, numVehicles, start_idx, end_idx)
    df = vrp.list_to_df(numVehicles, dflist)

    return df

def PhaseIandIIComplete(orderList, podMatrix, network, stationNodes, numRobot, max_percentage=0.5):

    selectedPodNodes, numSelectedPodsP1, total_distance, selectedPodNodesRawsimo, numSelectedPodsRawsimo, totalDistRawsimo = lp_podselection.PhaseIExperiment(orderList, podMatrix, network, stationNodes, max_percentage, returnSelected=True)

    vrpdf = solve_vrp(numRobot, network, selectedPodNodes)
    vrpdf['CarriedPods'] = vrpdf['Route'].apply(lambda x: len(x)-2)

    rawsimodf = vrp.filter_and_calculate_distance(numRobot, selectedPodNodesRawsimo)
    tempDF = pd.DataFrame(selectedPodNodesRawsimo, columns=["Nodes","Robot"])
    grouped_counts = tempDF.groupby('Robot').size().reset_index()
    grouped_counts.columns = ["Robot", "Count"]
    rawsimodf["CarriedPods"] = grouped_counts["Count"]

    return vrpdf, rawsimodf

def PhaseIandIIOuter(networkList, numRepeatForInstance, orderPerStation=20):

    excel_file = 'complete_phase1+2_output.xlsx'
    excel_rawsimo_file = 'complete_rawsimo_output.xlsx'

    writer = pd.ExcelWriter(excel_file, engine='xlsxwriter')
    writer_rawsimo = pd.ExcelWriter(excel_rawsimo_file, engine='xlsxwriter')

    idx = 0
    for networkSTR in networkList:
        print(networkSTR)

        dimensions = networkSTR.split("x")
        row = int(dimensions[0])
        column = int(dimensions[1])
        network, network_corridors = generators.create_network(vertical=row, horizontal=column)

        r = row * column * 8  # Number of storage pods
        s = r*2  # Number of SKUs
        k = r*1//20  # Maximum number of pods for each SKU
        lower_bound = 100  # Lower bound of the amount interval
        upper_bound = 200  # Upper bound of the amount interval

        for numStation in [2]:# 4]:

            for run in range(numRepeatForInstance):
                sheet_name = f'Sheet_{idx}'
                idx += 1
                orderList = generators.orderGenerator(stationCapacity=orderPerStation, numStation=numStation, numSKU=s, skuExistencethreshold=0.9)
                podMatrix = podstorage.generate_distribution_matrix(s, r, k, lower_bound, upper_bound).T
                station_nodes = lp_podselection.stationLocationFinder(network, numStation)
                vrpdf, rawsimodf = PhaseIandIIComplete(orderList=orderList, podMatrix=podMatrix, network=network, stationNodes=station_nodes, numRobot=numStation, max_percentage=0.5)
                vrpdf.to_excel(writer, sheet_name=sheet_name, index=False)
                rawsimodf.to_excel(writer_rawsimo, sheet_name=sheet_name, index=False)
    writer._save()
    writer_rawsimo._save()

if __name__ == "__main__":
    #networkList = ["4x8", "5x5", "6x12", "8x8", "10x20"]
    #PhaseIandIIOuter(networkList, 1, 20)
    network = ["5x5"]
    PhaseIandIIOuter(network, 1, 5)
