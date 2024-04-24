import numpy as np
import pandas as pd
import simpy

import generators
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask, SKU, ChargingStation
import layout
import random
import ast
from lp_podselection import podAndStation_combination, calculate_total_distances_for_all_requirements, min_max_diff, check_feasibility, columnMultiplication, assign_pods_to_stations
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import networkx as nx
import vrp



class RMFS_Model():
    def __init__(self, env, network):
        self.env = env
        self.network = network
        self.corridorSubgraph = layout.create_corridor_subgraph(network)

        pod_nodes = [node for node, data in network.nodes(data=True) if data.get('shelf', False)]
        self.podGraph = network.subgraph(pod_nodes) # Did not write .copy() to reference network itself

    def createPods(self):
        podNodes = list(self.podGraph.nodes)

        self.Pods = []
        for i in podNodes:
            tempPod = Pod(self.env, i)
            self.Pods.append(tempPod)

    def createSKUs(self):
        s = len(self.podGraph.nodes) * 4  # Number of SKUs
        self.SKUs = {}
        for id in range(s):
            tempSKU = SKU(self.env, id, 0)
            self.SKUs[id] = tempSKU


    def fillPods(self):

        pod_mean = 3 # Mean number of pods in which an SKU is stored
        pod_std = 1 # Standard deviation of pods in which an SKU is stored

        sku_mean = 7 # Mean number of sku which are stored in a pod
        sku_std = 3 # Standard deviation of sku which are stored in a pod

        lower_bound_amount = 50  # Lower bound of the amount interval
        upper_bound_amount = 100  # Upper bound of the amount interval

        for s_id, s in self.SKUs.items():
            random_float = np.random.normal(pod_mean, pod_std)
            random_integer = np.round(random_float).astype(int)
            while random_integer <= 0:
                random_float = np.random.normal(pod_mean, pod_std)
                random_integer = np.round(random_float).astype(int)
            randomPodsList = random.sample(self.Pods, random_integer)

            for pod in randomPodsList:
                amount = random.randint(lower_bound_amount, upper_bound_amount)
                pod.skuDict[s_id] = amount
                s.totalAmount += amount

        #check for empty pods
        for pod in self.Pods:
            if pod.skuDict == {}:
                random_float = np.random.normal(sku_mean, sku_std)
                random_integer = np.round(random_float).astype(int)
                while random_integer <= 0:
                    random_float = np.random.normal(sku_mean, sku_std)
                    random_integer = np.round(random_float).astype(int)
                randomSKUList = random.sample(list(self.SKUs.values()), random_integer)
                for sku in randomSKUList:
                    amount = random.randint(lower_bound_amount, upper_bound_amount)
                    pod.skuDict[sku.id] = amount
                    sku.totalAmount += amount




    def createOutputStations(self, locations):
        """
        Creates output stations and adds to a list which is a feature of RMFS_Model class
        :param locations: List of tuples
        """
        self.OutputStations = []
        for loc in locations:
            tempStation = OutputStation(env=self.env, location=loc)
            self.OutputStations.append(tempStation)

    def createChargingStations(self, locations):
        self.ChargingStations = []
        for loc in locations:
            tempStation = ChargingStation(env=self.env, capacity=1, location=loc)
            self.ChargingStations.append(tempStation)

    def createRobots(self, startLocations):
        """
        Creates robots and adds to a list which is a feature of RMFS_Model class
        :param startLocations: List of tuples [(0,0), (5,0)]
        """
        self.Robots = []
        self.ChargeQueue = []
        for idx, loc in enumerate(startLocations):
            tempRobot = Robot(self.env, network_corridors=self.corridorSubgraph, network=self.network, robotID=idx, currentNode=loc, taskList=[], batteryLevel=10, Model=self, chargingStationList=self.ChargingStations)
            self.Robots.append(tempRobot)

    def insertChargeQueue(self, robot):
        self.ChargeQueue.append(robot)

    def removeChargeQueue(self, robot=None):
        if robot == None:
            if self.ChargeQueue:
                return self.ChargeQueue.pop(0)
            else:
                return None

    def podSelectionHitRateCalculation(self, itemList):
        """
        For a given itemList, finds the pod who has maximum hit rate
        :param itemList: 2d np array
        :return: max_hit_pod: pod object, satisfiedSKU: dictionary, rtrItemList: modified itemList
        """
        max_hit = 0
        max_hit_pod = None
        satisfiedSKU = {} # {item1: amount1, item2: amount2}
        rtrItemList = itemList.copy()
        for pod_idx, pod in enumerate(self.Pods):
            hit = 0
            satisfiedSKU_temp = {}
            itemListTemp = itemList.copy()
            for idx, item in enumerate(itemListTemp): # iterates through items in the itemList (merged orders)
                if item[0] in pod.skuDict.keys():
                #if pod.skuDict[item[0]] > 0:
                    # for each pod check existence of a specific SKU, increases hitRate and decreases amount from temporary itemList
                    # done for partial fulfillment (same SKU from different pods)
                    amount = min(itemListTemp[idx][1], pod.skuDict[item[0]])
                    hit += amount
                    itemListTemp[idx][1] -= amount
                    satisfiedSKU_temp[item[0]] = amount
            if hit > max_hit:
                max_hit = hit
                max_hit_pod = pod
                satisfiedSKU = satisfiedSKU_temp.copy()
                rtrItemList = itemListTemp
        max_hit_pod.takeItemList = satisfiedSKU # updates pod's takeItemList so that OutputStation can retrieve items from this list
        return max_hit_pod, satisfiedSKU, rtrItemList

    #DAHA ÇOK TEST YAZILIP DENENEBİLİR
    def podSelectionMaxHitRate(self, itemList, satisfiedReturn = False):
        """

        :param itemList:  2d np array
        :param satisfiedReturn:
        :return: list of pod objects
        """
        def itemListSum(array_2d):
            """
            Aggregates itemList SKU-wise
            :param array_2d:
            :return: sums: 2d np array
            """
            # Extract unique values from the first column
            unique_first_column = np.unique(array_2d[:, 0])

            # Initialize an array to store the sums
            sums = np.zeros(shape=(len(unique_first_column),2), dtype=int)

            # Iterate over the unique values in the first column
            for i, value in enumerate(unique_first_column):
                # Sum the second column where the first column matches the current unique value
                sums[i,0] = value
                sums[i,1] = np.sum(array_2d[array_2d[:, 0] == value, 1])
            return sums

        itemList = itemListSum(itemList)
        selectedPodsList = []
        satisfiedList = []

        while len(itemList) > 0:
            selectedPod, satisfiedSKU, itemList = self.podSelectionHitRateCalculation(itemList=itemList)
            itemList = np.array([sublist for sublist in itemList if sublist[1] > 0])
            selectedPodsList.append(selectedPod)
            satisfiedList.append(satisfiedSKU)
        if satisfiedReturn:
            return selectedPodsList, satisfiedList

        return selectedPodsList

    def podSelectionHungarian(self, selectedPodsList, max_percentage=0.5, outputTask=False):
        no_of_pods = len(selectedPodsList)
        no_of_stations = len(self.OutputStations)

        podAndStation_distance = np.zeros(shape=(no_of_pods, no_of_stations))   # empty matrix
        combination = podAndStation_combination(no_of_pods, no_of_stations)

        for i, pod in enumerate(selectedPodsList):
            for j, station in enumerate(self.OutputStations):
                distance = abs(pod.location[0] - station.location[0]) + abs(pod.location[1] - station.location[1])
                podAndStation_distance[i, j] = distance

        combinationTotalDistance = calculate_total_distances_for_all_requirements(podAndStation_distance, combination)
        percentages = min_max_diff(combination, no_of_pods)
        # Find indexes where percentages exceed max_percentage
        exceed_indexes = np.where(percentages > max_percentage)[0]
        # Set the values at these indexes in combinationTotalDistance to infinity
        combinationTotalDistance[exceed_indexes] = np.inf

        result_idx = check_feasibility(combinationTotalDistance)
        requirement = combination[result_idx]
        testMatrix = columnMultiplication(podAndStation_distance, requirement)
        assigned_pods, assigned_stations, total_distance = assign_pods_to_stations(podAndStation_distance, requirement)

        # PS_distance = her podun her istasyona uzaklığı
        # PS_combination = all possible combinations
        # requirement = en iyileyen kombinasyon
        # testMatrix = hungarian için duplike edilen distance matrix
        # assigned_pods = direkt olarak saf hungarian çıktısı, istasyon bilgisi yok
        # assigned_stations = pod istasyon eşleşmeleri, from assigned_pods

        if outputTask:
            taskList = []
            for pod_idx, station_idx in enumerate(assigned_stations):
                tempTask = ExtractTask(env=self.env, robot=None, outputstation=self.OutputStations[station_idx], pod=selectedPodsList[pod_idx])
                taskList.append(tempTask)
            return taskList
        return podAndStation_distance, combination, requirement, testMatrix, assigned_pods, assigned_stations, total_distance



    def PhaseIExperiment(self, orderList, max_percentage=0.5, returnSelected=False):

        # Assignment by Hungarian Method Part
        selectedPodsList, satisfiedList = self.podSelectionMaxHitRate(itemlist,satisfiedReturn=True)
        PS_distance, PS_combination, requirement, testMatrix, assigned_pods, assigned_stations, total_distance = self.podSelectionHungarian(selectedPodsList, max_percentage)
        numSelectedPodsP1 = len(selectedPodsList)

        # Rawsimo default assignment
        def manhattan_distance(tuple1, tuple2):
            return sum(abs(a - b) for a, b in zip(tuple1, tuple2))

        def sum_manhattan_distance(target_tuple, listPods):
            return sum(manhattan_distance(target_tuple, t.location) for t in listPods)

        orderListDivided = np.reshape(orderList, newshape=(len(self.OutputStations), orderList.shape[0] // len(self.OutputStations), orderList.shape[1]))
        numSelectedPodsRawsimo = 0
        totalDistRawsimo = 0
        selectedPodsListRawsimo = []
        for stationIdx, station in enumerate(self.OutputStations):
            stationLocation = station.location
            itemListDivided = np.sum(orderListDivided[stationIdx], axis=0)
            itemListDivided = [[sku, int(amount)] for sku, amount in enumerate(itemListDivided)]
            tempList = self.podSelectionMaxHitRate(itemListDivided)

            selectedPodsListRawsimo.extend(tempList)
            numSelectedPodsRawsimo += len(selectedPodsListRawsimo)
            totalDistRawsimo += sum_manhattan_distance(stationLocation, selectedPodsListRawsimo)

        if returnSelected:
            return selectedPodsList, numSelectedPodsP1, int(total_distance), selectedPodsListRawsimo, numSelectedPodsRawsimo, totalDistRawsimo

        return numSelectedPodsP1, int(total_distance), numSelectedPodsRawsimo, totalDistRawsimo




    def distanceMatrixCalculate(self):
        """Takes a network as input, returns a Distance Matrix and the list of nodes."""

        shortest_paths = dict(nx.all_pairs_shortest_path_length(self.network))

        nodes = list(self.network.nodes)
        num_nodes = len(nodes)
        distance_matrix = np.zeros((num_nodes, num_nodes))

        for i in range(num_nodes):
            for j in range(num_nodes):
                if i != j:
                    if nodes[j] in shortest_paths[nodes[i]]:
                        distance_matrix[i][j] = shortest_paths[nodes[i]][nodes[j]]
                    else:
                        distance_matrix[i][j] = float('inf')

        self.distanceMatrix = distance_matrix
        return distance_matrix, nodes

    def fixedLocationVRP(self, taskList, start_nodes=None, end_nodes=None, assign=True):
        """

        :param taskList: List that contains ExtractTask objects
        :param start_nodes: List of tuples
        :param end_nodes: List of tuples
        :param assign: Boolean variable, if True adds tasks to robots' task lists,
        :return: data, manager, routing, solution, dflist
        """
        def distanceMatrixModify(taskList, start_nodes=None, end_nodes=None):
            """
            Using self.distanceMatrix, returns a distance matrix only contains necessary nodes for OR-Tools VRP
            :param taskList: List that contains ExtractTask objects
            :param start_nodes: List of tuples
            :param end_nodes: List of tuples
            :return: 2d np array
            """
            node_idx = []
            start_idx = []
            end_idx = []
            task_dict = {} #pod-column index matching for later parts
            for i, task in enumerate(taskList): #adding index of pods to index list
                idx = list(self.network.nodes).index(task.pod.location) # To speed up, search only in podNodes
                task_dict[i] = task
                node_idx.append(idx)

            # ŞARJA GÖRE GÜNCELLE HER ROBOTU DÖNMEMESİ LAZIM
            if start_nodes == None:
                for i, robot in enumerate(self.Robots):
                    idx = list(self.network.nodes).index(robot.currentNode)
                    node_idx.append(idx)
                    start_idx.append(len(node_idx)-1)
            else:
                for i, node in enumerate(start_nodes):
                    idx = list(self.network.nodes).index(node)
                    node_idx.append(idx)
                    start_idx.append(len(node_idx)-1)

            if end_nodes != None:
                for i, node in enumerate(end_nodes):
                    idx = list(self.network.nodes).index(node)
                    node_idx.append(idx)
                    end_idx.append(len(node_idx)-1)
                vrp_matrix = self.distanceMatrix[node_idx, :][:, node_idx]
                return vrp_matrix, start_idx, end_idx
            else:
                #adding dummy node
                vrp_matrix = self.distanceMatrix[node_idx, :][:, node_idx]
                zero_column = np.zeros((vrp_matrix.shape[0], 1), dtype=vrp_matrix.dtype)
                #vrp_matrix = np.insert(vrp_matrix, vrp_matrix.shape[1], zero_column, axis=1)
                vrp_matrix = np.append(vrp_matrix, zero_column, axis=1)
                infinity_row = np.full((1, vrp_matrix.shape[1]), 10000)
                vrp_matrix = np.insert(vrp_matrix, vrp_matrix.shape[0],infinity_row, axis=0)
                vrp_matrix[-1, -1] = 0

                end_idx = [len(node_idx) for i in range(len(self.Robots))] #ŞARJA GÖRE GÜNCELLE
                return vrp_matrix.astype(int), start_idx, end_idx, task_dict

        def create_data_model(distanceMatrix, start_index, end_index):
            """Stores the data for the problem."""
            data = {}
            data["distance_matrix"] = distanceMatrix
            data["num_vehicles"] = len(self.Robots) #ŞARJA GÖRE GÜNCELLE
            data["starts"] = start_index
            data["ends"] = end_index
            return data

        def createRoutes(data, manager, routing):
            allRoutes = []
            for vehicle_id in range(data["num_vehicles"]):

                index = routing.Start(vehicle_id)
                route_array = np.array([])

                while not routing.IsEnd(index):
                    plan = manager.IndexToNode(index)
                    index = solution.Value(routing.NextVar(index))
                    route_array = np.append(route_array, plan)

                last = manager.IndexToNode(index)
                route_array = np.append(route_array, last)
                allRoutes.append(route_array.astype(int))
            return allRoutes

        def assignTasks(routeList, task_dict):

            for idx, robot in enumerate(self.Robots):
                #if charge mı değil mi bak, veya rotalamada bu robot görev istedi mi direkt sıraya göre atama yapıyor
                #sadece extract create ediyor
                for node in routeList[idx][1:-1]:
                    tempTask = task_dict[node]
                    tempTask.robot = robot
                    robot.taskList.append(tempTask)


        distMatrixModified, start_index, end_index, task_dict = distanceMatrixModify(taskList,start_nodes,end_nodes)
        data = create_data_model(distMatrixModified, start_index, end_index)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(len(data["distance_matrix"]), data["num_vehicles"], data["starts"], data["ends"])
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
        # initial solution için buradaki yöntemi kullanıyor
        )

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            allRoutes = createRoutes(data=data, manager=manager, routing=routing)
            if assign:
                assignTasks(allRoutes, task_dict)
            dflist = vrp.print_solution(data, manager, routing, solution)
            return data, manager, routing, solution, dflist

    def startCycleVRP(self):
        #generatordan itemList createle
        newItemlist = np.array(([1, 10],
                         [2, 10],
                         [3, 10],
                         [4, 10],
                         [5, 10],
                         [6, 10],
                         [7, 10],))

        selectedPodsList = self.podSelectionMaxHitRate(itemlist)
        extractTaskList = self.podSelectionHungarian(selectedPodsList, outputTask=True)
        self.fixedLocationVRP(extractTaskList, assign=True)

        for robot in self.Robots:
            self.env.process(robot.DoExtractTask(robot.taskList[0]))
        env.run()

    def taskGeneratorV2(self, numTask, forVRP=True, forRawSIMO=True):
        """
        Creates a list contains extract task object from random selected pods
        :param numTask: int, number of tasks
        :param forVRP: if True creates tasks without assigning robots
        :param forRawSIMO: if True creates tasks with assigning robots
        :return:
        """
        randomPodsList = random.sample(self.Pods, numTask)
        vrpTaskList = []
        rawsimoTaskList = []
        for idx, pod in enumerate(randomPodsList):
            if forVRP:
                vrpTask = ExtractTask(env=self.env, robot=None, outputstation=None, pod=pod)
                vrpTaskList.append(vrpTask)

            if forRawSIMO:
                robot_idx = idx % len(self.Robots)
                rawsimoTask = ExtractTask(env=self.env, robot=self.Robots[robot_idx], outputstation=self.OutputStations[robot_idx], pod=pod)
                rawsimoTaskList.append(rawsimoTask)

        return vrpTaskList, rawsimoTaskList


    def fixedLocationRawSIMO(self, taskList, start_nodes=None, end_nodes=None, assign=True):

        for task in taskList:
            robot = task.robot
            robot.taskList.append(task)

        for robot in self.Robots:
            self.env.process(robot.DoExtractTask(robot.taskList[0]))
        env.run()



    def PhaseIIExperimentOneCycle(self, numTask):
        # burayı yap
        taskListRaw = generators.taskGenerator(network=simulation.network, numTask=numTask, numRobot=len(self.Robots))



if __name__ == "__main__":
    env = simpy.Environment()


    #rows = 13  4x8
    #columns = 41

    #rows = 16  #5x5
    #columns = 26

    #rows = 10 #3x6
    #columns = 31

    rows = 10 #3x3
    columns = 16

    rectangular_network, pos = layout.create_rectangular_network_with_attributes(columns, rows)
    layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))

    nodes = list(rectangular_network.nodes)
    simulation = RMFS_Model(env=env, network=rectangular_network)
    simulation.createPods()
    simulation.createSKUs()
    """
    startLocations = [(0,0), (0,9)]
    simulation.createRobots(startLocations)

    firstStation = (nodes[0][0], (nodes[0][1] + nodes[-1][1]) // 2)
    secondStation = (nodes[-1][0], (nodes[0][1] + nodes[-1][1]) // 2)
    locations = [firstStation, secondStation]

    simulation.createOutputStations(locations)
    simulation.fillPods()
    simulation.distanceMatrixCalculate()

    taskListVRP, taskListRawSIMO = simulation.taskGeneratorV2(4)
    simulation.fixedLocationRawSIMO(taskListRawSIMO)


    itemlist = np.array(([1, 10],
                         [2, 10],
                         [3, 10],
                         [4, 10],
                         [5, 10],
                         [6, 10],
                         [7, 10],))

    selectedPodsList = simulation.podSelectionMaxHitRate(itemlist)
    extractTaskList = simulation.podSelectionHungarian(selectedPodsList, outputTask=True)
    simulation.fixedLocationVRP(extractTaskList, assign=True)

    for robot in simulation.Robots:
        simulation.env.process(robot.DoExtractTask(robot.taskList[0]))
        #simulation.env.process(robot.DoStorageTask())

    #simulation.env.run(until=10)
    #simulation.env.run(until=15)
    simulation.env.run()
    """


    simulation.createChargingStations([(0, 9)])
    startLocations = [(0, 0), (5, 0)]
    simulation.createRobots(startLocations)

    firstStation = (0,4)
    locations = [firstStation]

    simulation.createOutputStations(locations)

    simulation.fillPods()
    simulation.distanceMatrixCalculate()

    itemlist = np.array(([1, 10],
                         [2, 10],
                         [3, 10],
                         [4, 10],
                         [5, 10],
                         [6, 10],
                         [7, 10],
                         [8, 10],
                         [9, 10],))

    selectedPodsList = simulation.podSelectionMaxHitRate(itemlist)
    extractTaskList = simulation.podSelectionHungarian(selectedPodsList, outputTask=True)
    simulation.fixedLocationVRP(extractTaskList, assign=True)

    def deneme(env, t):

        yield env.timeout(t)
        for robot in simulation.Robots:
            if robot.status == "charging":
                robot.batteryLevel += 1 #bunu 1 saniyede kaç şarj ediyorsa onunla değiştir
            print(robot.batteryLevel)

    for i in range(10):
        simulation.env.process(deneme(env, i))

    for robot in simulation.Robots:
        simulation.env.process(robot.DoExtractTask(robot.taskList[0]))

    simulation.env.run()
    a = 10

