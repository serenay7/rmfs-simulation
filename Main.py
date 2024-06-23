import time
import numpy as np
import pandas as pd
import simpy
from simpy.events import AllOf
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask, SKU, ChargingStation
import Layout
import random
from PodSelection import podAndStation_combination, calculate_total_distances_for_all_requirements, min_max_diff, check_feasibility, columnMultiplication, assign_pods_to_stations
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import networkx as nx
import copy
from RL_test import VRPDatasetNew
from torch.utils.data import DataLoader, Dataset
from RL.utils import load_model
import torch
from scipy.optimize import linear_sum_assignment



class RMFS_Model():
    def __init__(self, env, network, TaskAssignmentPolicy="vrp", ChargePolicy="pearl", DropPodPolicy="fixed"):
        self.env = env
        self.network = network
        self.corridorSubgraph = layout.create_corridor_subgraph(network)

        pod_nodes = [node for node, data in network.nodes(data=True) if data.get('shelf', False)]
        self.podGraph = network.subgraph(pod_nodes) # Did not write .copy() to reference network itself

        self.TaskAssignmentPolicy = TaskAssignmentPolicy #"rawsimo" or "vrp"
        self.ChargePolicy = ChargePolicy #"rawsimo" or "pearl"
        self.DropPodPolicy = DropPodPolicy #"fixed" or "closestTask"
        self.timeStatDF = pd.DataFrame(columns=['time', 'robotID', 'robotStatus', 'stepsTaken', 'batteryLevel', 'remainingTasks', 'completedTasks'])
        self.selectedPodsList = []
        self.satisfiedList = []
        self.totalPodNumber = 0
        self.totalPodStationDist = 0
    def createPods(self):
        podNodes = list(self.podGraph.nodes)

        self.Pods = []
        for i in podNodes:
            tempPod = Pod(self.env, i)
            self.Pods.append(tempPod)

    def createSKUs(self):
        s = len(self.podGraph.nodes) * 4  # Number of SKUs 12 for real
        self.SKUs = {}
        for id in range(s):
            tempSKU = SKU(self.env, id, 0)
            self.SKUs[id] = tempSKU


    def fillPods(self):

        pod_mean = 3 #8 Mean number of pods in which an SKU is stored #BUNU SKU SAYISINA BAĞLA
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
        for idx, loc in enumerate(locations):
            tempStation = OutputStation(env=self.env, location=loc, outputStationID=idx)
            self.OutputStations.append(tempStation)

    def createChargingStations(self, locations):
        self.ChargingStations = []
        for loc in locations:
            tempStation = ChargingStation(env=self.env, capacity=1, location=loc)
            self.ChargingStations.append(tempStation)

    """
    def createRobots(self, startLocations):
        
        Creates robots and adds to a list which is a feature of RMFS_Model class
        :param startLocations: List of tuples [(0,0), (5,0)]
        
        self.Robots = []
        self.ChargeQueue = []
        for idx, loc in enumerate(startLocations):
            tempRobot = Robot(self.env, network_corridors=self.corridorSubgraph, network=self.network, robotID=idx, currentNode=loc, taskList=[], batteryLevel=41.6, chargingRate=41.6, Model=self, ChargeFlagRate=0.85, MaxChargeRate = 0.95, PearlRate = 0.4, RestRate = 0.1, chargingStationList=self.ChargingStations)
            self.Robots.append(tempRobot)
    """

    def createRobots(self, startLocations, charging_rate=41.6, max_battery=41.6, pearl_rate=0.4, rest_rate=0.1, charge_flag_rate=0.85,
                     max_charge_rate=0.95):
        """
        Creates robots and adds to a list which is a feature of RMFS_Model class
        :param startLocations: List of tuples [(0,0), (5,0)]
        chargingRate=41.6
        ChargeFlagRate=0.85
        MaxChargeRate = 0.95
        PearlRate = 0.4
        RestRate = 0.1,
        robotlar max bataryayla başlıyor
        """
        self.Robots = []
        self.ChargeQueue = []
        for idx, loc in enumerate(startLocations):
            tempRobot = Robot(self.env, network_corridors=self.corridorSubgraph, network=self.network, robotID=idx,
                              currentNode=loc, taskList=[], batteryLevel=max_battery, chargingRate=charging_rate,
                              Model=self, ChargeFlagRate=charge_flag_rate, MaxChargeRate=max_charge_rate,
                              PearlRate=pearl_rate, RestRate=rest_rate, chargingStationList=self.ChargingStations
                              )
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
    def podSelectionMaxHitRate(self, itemList, satisfiedReturn = False, stationID=0):
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
            if self.TaskAssignmentPolicy == "vrp" or self.TaskAssignmentPolicy == "rl":
                self.selectedPodsList = selectedPodsList
                self.satisfiedList = satisfiedList
            elif self.TaskAssignmentPolicy == "rawsimo":
                self.selectedPodsList.append(selectedPodsList)
                self.satisfiedList.append(satisfiedList)
            else:
                raise Exception("Unknown TaskAssignmentPolicy")

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

        taskList = []
        for pod_idx, station_idx in enumerate(assigned_stations):
            tempTask = ExtractTask(env=self.env, robot=None, outputstation=self.OutputStations[station_idx],
                                   pod=selectedPodsList[pod_idx])
            taskList.append(tempTask)

        return podAndStation_distance, combination, requirement, testMatrix, assigned_pods, assigned_stations, total_distance, taskList



    def PhaseIExperiment(self, orderList, max_percentage=0.5, returnSelected=False):
        def manhattan_distance(tuple1, tuple2):
            return sum(abs(a - b) for a, b in zip(tuple1, tuple2))

        # Assignment by Hungarian Method Part
        selectedPodsList, satisfiedList = self.podSelectionMaxHitRate(orderList,satisfiedReturn=True)
        PS_distance, PS_combination, requirement, testMatrix, assigned_pods, assigned_stations, total_distance, taskList = self.podSelectionHungarian(selectedPodsList, max_percentage)
        numSelectedPodsP1 = len(selectedPodsList)

        totalDistHungarian = 0
        for task in taskList:
            totalDistHungarian += manhattan_distance(task.pod.fixedLocation, task.outputstation.location)

        # Rawsimo default assignment


        def sum_manhattan_distance(target_tuple, listPods):
            return sum(manhattan_distance(target_tuple, t.location) for t in listPods)

        #orderListDivided = np.reshape(orderList, newshape=(len(self.OutputStations), orderList.shape[0] // len(self.OutputStations), orderList.shape[1]))

        rows_per_station = orderList.shape[0] // len(self.OutputStations)

        # Initialize a list to store the divided arrays
        orderListDivided = []

        # Iterate over the output stations
        for i in range(len(self.OutputStations)):
            # Calculate the start and end indices for the current output station
            start_index = i * rows_per_station
            end_index = (i + 1) * rows_per_station if i < len(self.OutputStations) - 1 else None

            # Extract the rows for the current output station and append to the list
            orderListDivided.append(orderList[start_index:end_index])


        numSelectedPodsRawsimo = 0
        totalDistRawsimo = 0
        selectedPodsListRawsimo = []
        for stationIdx, station in enumerate(self.OutputStations):
            stationLocation = station.location
            #itemListDivided = np.sum(orderListDivided[stationIdx], axis=0)
            itemListDivided = orderListDivided[stationIdx]
            #itemListDivided = [[sku, int(amount)] for sku, amount in enumerate(itemListDivided)]
            tempList = self.podSelectionMaxHitRate(itemListDivided)

            selectedPodsListRawsimo.extend(tempList)
            numSelectedPodsRawsimo += len(tempList)
            totalDistRawsimo += sum_manhattan_distance(stationLocation, tempList)

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
        print("TIME: ", self.env.now)
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
                    if robot.status != "charging" and robot.batteryLevel > robot.MaxBattery * robot.RestRate:
                        #if robot.pod != None: idx = list(self.network.nodes).index(robot.currentTask.pod.fixedLocation)
                        #else: idx = list(self.network.nodes).index(robot.currentNode)
                        if robot.currentTask:
                            idx = list(self.network.nodes).index(robot.currentTask.pod.fixedLocation)
                        else:
                            idx = list(self.network.nodes).index(robot.currentNode)
                        node_idx.append(idx)
                        start_idx.append(len(node_idx)-1)
            else:
                for i, node in enumerate(start_nodes):
                    #şarjdaki robotu katıyor
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

                end_idx = [len(node_idx) for i in range(len(start_idx))] #ŞARJA GÖRE GÜNCELLE

                return vrp_matrix.astype(int), start_idx, end_idx, task_dict

        def create_data_model(distanceMatrix, start_index, end_index):
            """Stores the data for the problem."""
            data = {}
            data["distance_matrix"] = distanceMatrix
            data["num_vehicles"] = len(start_index) #ŞARJA GÖRE GÜNCELLE
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

            idx = 0
            for robot in self.Robots:
                #if charge mı değil mi bak, veya rotalamada bu robot görev istedi mi direkt sıraya göre atama yapıyor
                #sadece extract create ediyor
                robot.taskList = []
                #if robot.pod == None: robot.currentTask = None

                if robot.status != "charging" and robot.batteryLevel > robot.MaxBattery * robot.RestRate:
                    for node in routeList[idx][1:-1]:
                        tempTask = task_dict[node]
                        tempTask.robot = robot
                        robot.taskList.append(tempTask)
                    idx += 1

        def print_solution(data, manager, routing, solution):
            """Prints solution on console."""
            output_list = [solution.ObjectiveValue()]
            print(f"Objective: {solution.ObjectiveValue()}")
            max_route_distance = 0

            for vehicle_id in range(data["num_vehicles"]):
                output_list.append(vehicle_id)

                index = routing.Start(vehicle_id)
                plan_output = f"Route for vehicle {vehicle_id}:\n"
                route_distance = 0
                # route_array = np.array([index])
                route_array = np.array([])

                while not routing.IsEnd(index):
                    plan = manager.IndexToNode(index)
                    plan_output += f" {manager.IndexToNode(index)} -> "
                    previous_index = index
                    index = solution.Value(routing.NextVar(index))
                    route_distance += routing.GetArcCostForVehicle(
                        previous_index, index, vehicle_id
                    )
                    route_array = np.append(route_array, plan)

                last = manager.IndexToNode(index)
                plan_output += f"{manager.IndexToNode(index)}\n"
                route_array = np.append(route_array, last)
                output_list.append(route_array)
                plan_output += f"Distance of the route: {route_distance}m\n"
                output_list.append(route_distance)
                print(plan_output)
                max_route_distance = max(route_distance, max_route_distance)
            print(f"Maximum of the route distances: {max_route_distance}m")

            return output_list


        distMatrixModified, start_index, end_index, task_dict = distanceMatrixModify(taskList,start_nodes,end_nodes)
        data = create_data_model(distMatrixModified, start_index, end_index)

        if data["num_vehicles"] == 0:
            return

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


        count_dimension_name = 'count'
        # assume some variable num_nodes holds the total number of nodes
        routing.AddConstantDimension(
            1,  # increment by one every time
            len(self.extractTaskList) // len(start_index) + len(start_index),  # max value forces equivalent # of jobs
            True,  # set count to zero
            count_dimension_name)
        count_dimension = routing.GetDimensionOrDie(count_dimension_name)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
        # initial solution için buradaki yöntemi kullanıyor
        )

        search_parameters.time_limit.seconds = 30 #timelimit

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            allRoutes = createRoutes(data=data, manager=manager, routing=routing)
            if assign:
                assignTasks(allRoutes, task_dict)
            dflist = print_solution(data, manager, routing, solution)
            return data, manager, routing, solution, dflist
        else:
            raise Exception("VRP solution not found.")

    def combineItemListsVRP(self, itemlist):

        def itemListSum(array_2d):
            """
            Aggregates itemList SKU-wise
            :param array_2d:
            :return: sums: 2d np array
            """
            # Extract unique values from the first column
            unique_first_column = np.unique(array_2d[:, 0])

            # Initialize an array to store the sums
            sums = np.zeros(shape=(len(unique_first_column), 2), dtype=int)

            # Iterate over the unique values in the first column
            for i, value in enumerate(unique_first_column):
                # Sum the second column where the first column matches the current unique value
                sums[i, 0] = value
                sums[i, 1] = np.sum(array_2d[array_2d[:, 0] == value, 1])
            return sums

        notDeliveredPods = []

        for robot in self.Robots:
            #if robot.pod == None and type(robot.currentTask) == ExtractTask: # vrp anında pod almaya giden robotların görevlerini silip onları da dahil etmek için
            #    notDeliveredPods.append(robot.currentTask.pod)
            if robot.taskList:
                for task in robot.taskList:
                    notDeliveredPods.append(task.pod)

        self.podStationDistCalculate(notDeliveredPods=notDeliveredPods)

        tempList = []
        for p in notDeliveredPods:
            try:
                idx = self.selectedPodsList.index(p)
            except:
                a = 10
            for sku, value in self.satisfiedList[idx].items():
                tempList.append([sku, value])

        tempArr = np.array(tempList)
        if len(tempArr)>0:
            itemListRaw = np.vstack((itemlist, tempArr))
            finalItemList = itemListSum(itemListRaw)
            return finalItemList
        else:
            return itemlist



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


    def fixedLocationRawSIMO(self, assign=True):

        def divide_list(lst, num_groups):
            # Calculate the size of each group
            group_size = len(lst) // num_groups
            remainder = len(lst) % num_groups

            # Divide the list into groups
            groups = []
            start = 0
            for i in range(num_groups):
                group_end = start + group_size + (1 if i < remainder else 0)
                groups.append(lst[start:group_end])
                start = group_end

            return groups

        allocatedRobotsList = divide_list(self.Robots, len(self.OutputStations))

        for robot in self.Robots:
            robot.taskList = []

        for idx, stationTaskList in enumerate(self.extractTaskList):
            stationRobots = allocatedRobotsList[idx]
            numRobot = len(stationRobots)
            for taskNum, task in enumerate(stationTaskList):
                task.robot = stationRobots[taskNum % numRobot]
                stationRobots[taskNum % numRobot].taskList.append(task)

    """
    def orderGenerator(self, numOrder, skuExistencethreshold=0.5, maxAmount=10):

        skus = random.sample(range(1, len(self.Pods)*4), numOrder)
        tempList = []

        for sku in skus:
            amount = random.randint(5, 10)
            tempList.append([sku, amount])

        orders = np.array(tempList)
        return orders
    """

    def orderGenerator(self, numOrder=23, skuExistenceThreshold=0.5, mean=6, std=2):
        orderCount = np.random.poisson(lam=numOrder, size=1)[0]
        skus = random.sample(range(1, len(self.Pods) * 4), orderCount)
        tempList = []

        for sku in skus:
            amount = max(1, int(random.normalvariate(mean, std)))
            tempList.append([sku, amount])

        orders = np.array(tempList)
        return orders



    def updateCharge(self, t, updateTime, addition=0):

        yield self.env.timeout(t*updateTime+addition)
        if self.ChargePolicy == "rawsimo":
            for chargingStation in self.ChargingStations:
                if chargingStation.currentRobot != None:
                    robot = chargingStation.currentRobot
                    if robot.currentNode == chargingStation.location:
                        robot.batteryLevel += robot.chargingRate/3600*updateTime
                        if robot.batteryLevel >= robot.MaxBattery * robot.MaxChargeRate:
                            newRobot = self.removeChargeQueue()
                            all_events = []
                            chargingStation.currentRobot = None
                            robot.status = "extract"

                            if newRobot:
                                newRobot.status = "charging"
                                chargingStation.currentRobot = newRobot
                                #all_events.append(self.env.process(newRobot.moveToChargingStation(chargingStation)))
                                yield self.env.process(newRobot.moveToChargingStation(chargingStation))
                            else:
                                for newRobot in self.Robots:
                                    if newRobot.status == "rest" and newRobot.batteryLevel < newRobot.MaxBattery * newRobot.ChargeFlagRate:
                                        newRobot.status = "charging"
                                        chargingStation.currentRobot = newRobot
                                        yield self.env.process(newRobot.moveToChargingStation(chargingStation))
                                        break

                            if robot.taskList:
                                #all_events.append(self.env.process(robot.DoExtractTask(robot.taskList[0])))
                                yield self.env.process(robot.DoExtractTask(robot.taskList[0]))
                            else:
                                #all_events.append(self.env.process(robot.goRest()))
                                yield self.env.process(robot.goRest())

                            #yield AllOf(self.env, all_events)

        if self.ChargePolicy == "pearl":
            for chargingStation in self.ChargingStations:
                if chargingStation.currentRobot != None:
                    robot = chargingStation.currentRobot
                    if robot.currentNode == chargingStation.location:
                        robot.batteryLevel += robot.chargingRate / 3600 * updateTime
                        if robot.batteryLevel >= robot.MaxBattery * robot.MaxChargeRate:
                            newRobot = self.removeChargeQueue()
                            chargingStation.currentRobot = None
                            robot.status = "extract"
                            if newRobot:
                                newRobot.status = "charging"
                                yield self.env.process(self.pearlVRP(simultaneousEvent=newRobot.moveToChargingStation(chargingStation)))
                                yield self.env.process(newRobot.moveToChargingStation(chargingStation))
                            else:
                                yield self.env.process(self.pearlVRP())
                            if robot.taskList:
                                yield self.env.process(robot.DoExtractTask(robot.taskList[0]))
                            else:
                                yield self.env.process(robot.goRest())



    def pearlVRP(self, simultaneousEvent=None):

        tempList = []
        for robot in self.Robots:
            tempList.append(robot.currentTask)

        remainingTasks = []
        #uncompletedTasks = list(filter(lambda x: x not in tempList, self.extractTaskList))
        for robot in self.Robots:
            if robot.taskList:
                remainingTasks.extend(robot.taskList)
        if self.TaskAssignmentPolicy == "vrp":
            self.fixedLocationVRP(remainingTasks, assign=True)
        elif self.TaskAssignmentPolicy == "rl":
            self.fixedLocationRL(remainingTasks, assign=True)
        """
        self.env._queue = []

        # env saati mod(cycleSeconds) ve bir sonraki cyclea olan uzaklık?
        time = self.env.now
        start = (self.cycleSeconds-time)%1

        remaining = self.cycleSeconds-math.ceil(time)

        for i in range(0, remaining + 1):
            self.env.process(self.updateCharge(i, updateTime=1, addition=start))

        if simultaneousEvent:
            self.env.process(simultaneousEvent)

        for robot in self.Robots:
            if robot.status == "charging" or robot.batteryLevel < robot.MaxBattery * robot.RestRate:
                continue
            elif robot.currentTask != None:
                self.env.process(robot.DoExtractTask(robot.currentTask))
            else:
                self.env.process(robot.DoExtractTask(robot.taskList[0]))

        until = self.cycleSeconds * (self.currentCycle + 1)
        #self.env.run(until=self.cycleSeconds * (self.currentCycle + 1))
        """
        yield self.env.timeout(0)

    def addCollectedSKUCount(self):
        if self.TaskAssignmentPolicy == "vrp" or self.TaskAssignmentPolicy == "rl":
            uncompletedTasks = []
            for robot in self.Robots:
                uncompletedTasks.extend(robot.taskList)
            completedTasks = [task for task in self.extractTaskList if task not in uncompletedTasks]
            self.totalPodNumber += len(completedTasks)

            for task in completedTasks:
                idx = self.selectedPodsList.index(task.pod)
                takenItems = self.satisfiedList[idx]
                task.outputstation.totalPickedCount += sum(takenItems.values())

        elif self.TaskAssignmentPolicy == "rawsimo":
            uncompletedTasks = []
            for robot in self.Robots:
                uncompletedTasks.extend(robot.taskList)

            fullTaskList = []
            for lst in self.extractTaskList:
                fullTaskList.extend(lst)

            completedTasks = [task for task in fullTaskList if task not in uncompletedTasks]
            self.totalPodNumber += len(completedTasks)

            allSelectedPods = []
            allSatisfiedSKUs = []
            for idx, station in enumerate(self.OutputStations):
                allSelectedPods.extend(self.selectedPodsList[idx])
                allSatisfiedSKUs.extend(self.satisfiedList[idx])


            for task in completedTasks:
                idx = allSelectedPods.index(task.pod)
                takenItems = allSatisfiedSKUs[idx]
                task.outputstation.totalPickedCount += sum(takenItems.values())

    def calculateObservationStat(self):
        df = pd.DataFrame(columns=["Statistics", "Value"])
        for outputStation in self.OutputStations:
            feature = "Station" + str(outputStation.outputStationID) + "TotalCollect"
            new_row = {'Statistics': feature, 'Value': outputStation.totalPickedCount}
            df.loc[len(df)] = new_row

        for robot in self.Robots:
            feature = "Robot" + str(robot.robotID) + "NumberOfCharge"
            new_row = {'Statistics': feature, 'Value': robot.chargeCount}
            df.loc[len(df)] = new_row

        for robot in self.Robots:
            feature = "Robot" + str(robot.robotID) + "NumberOfReplace"
            new_row = {'Statistics': feature, 'Value': robot.replaceCount}
            df.loc[len(df)] = new_row

        new_row = {'Statistics': "SelectedPodNum", 'Value': self.totalPodNumber}
        df.loc[len(df)] = new_row

        new_row = {'Statistics': "TotalPodStationDistance", 'Value': self.totalPodStationDist}
        df.loc[len(df)] = new_row

        return df

    def plotTimeStat(self):
        pass
    def collectTimeStat(self, t, cycleSeconds):

        yield self.env.timeout(t * 60)
        for robot in self.Robots:
            #new_row = {'time': self.env.now, 'robotID': robot.robotID, 'robotStatus': robot.status, 'stepsTaken':robot.stepsTaken, 'batteryLevel':robot.batteryLevel}
            new_row = [self.env.now, robot.robotID, robot.status, robot.stepsTaken, robot.batteryLevel, None, None]
            #self.timeStatDF = self.timeStatDF.append(new_row, ignore_index=True)
            #if t*60 % cycleSeconds == 0 and t != 0:
            if True:
                if self.TaskAssignmentPolicy == "vrp" or self.TaskAssignmentPolicy == "rl":
                    uncompletedTasks = []
                    for robot1 in self.Robots:
                        uncompletedTasks.extend(robot1.taskList)
                        #if robot.currentTask: bunu yoruma alınca şu an devam eden taskın da bittiğini assume ettik
                        #    uncompletedTasks.append(robot.currentTask)
                    completedTasks = [task for task in self.extractTaskList if task not in uncompletedTasks]

                elif self.TaskAssignmentPolicy == "rawsimo":
                    uncompletedTasks = []
                    for robot1 in self.Robots:
                        uncompletedTasks.extend(robot1.taskList)
                        #if robot.currentTask:
                        #    uncompletedTasks.append(robot.currentTask)
                    fullTaskList = []
                    for lst in self.extractTaskList:
                        fullTaskList.extend(lst)
                    completedTasks = [task for task in fullTaskList if task not in uncompletedTasks]

                new_row = [self.env.now, robot.robotID, robot.status, robot.stepsTaken, robot.batteryLevel, len(uncompletedTasks), len(completedTasks)]

            self.timeStatDF.loc[len(self.timeStatDF.index)] = new_row



    def startCycleVRP(self, itemlist, cycleSeconds, cycleIdx):

        if cycleIdx != 0:
            itemlist = self.combineItemListsVRP(itemlist=itemlist)

        start = time.time()
        selectedPodsList, satisfiedList = self.podSelectionMaxHitRate(itemlist, satisfiedReturn=True)
        extractTaskList = self.podSelectionHungarian(selectedPodsList, outputTask=True)
        #self.totalPodNumber += len(selectedPodsList)
        end = time.time()
        print("POD SELECTION TIME: ", end-start)
        start = time.time()
        self.extractTaskList = extractTaskList


        self.fixedLocationVRP(extractTaskList, assign=True)
        end = time.time()
        print("VRP TIME: ", end - start)
        #self.env._queue = []

        for i in range(1, cycleSeconds+1):
            self.env.process(self.updateCharge(t=i, updateTime=1))

        for i in range(0, self.cycleSeconds//60 + 1):
            self.env.process(self.collectTimeStat(t=i, cycleSeconds=cycleSeconds))

        if cycleIdx == 0:
            for robot in self.Robots:
                if robot.status == "charging" and robot.targetNode != robot.currentNode:
                    robot.createPath(robot.targetNode)
                    self.env.process(robot.move())
                elif robot.currentTask != None:
                    self.env.process(robot.DoExtractTask(robot.currentTask))
                elif robot.taskList:
                    self.env.process(robot.DoExtractTask(robot.taskList[0]))
                else:
                    self.env.process(robot.goRest())
        else:
            for robot in self.Robots:
                if robot.status == "rest" and robot.batteryLevel > robot.MaxBattery * robot.RestRate:
                    if robot.taskList:
                        self.env.process(robot.DoExtractTask(robot.taskList[0]))


        #self.env.run(until=cycleSeconds*(cycleIdx+1))

    def MultiCycleVRP(self, numCycle, cycleSeconds, printOutput=False, allItemList = None, numOrderPerCycle=30):

        self.numCycle = numCycle
        self.cycleSeconds = cycleSeconds
        for cycle_idx in range(numCycle):
            self.currentCycle = cycle_idx
            print("Cycle: ", cycle_idx)
            if allItemList:
                itemlist = allItemList[cycle_idx]
            else:
                itemlist = (self.orderGenerator(numOrder=numOrderPerCycle))
            for robot in self.Robots:
                if robot.taskList:
                    a = 10
            self.startCycleVRP(itemlist=itemlist, cycleSeconds=cycleSeconds, cycleIdx=cycle_idx)
            self.env.run(until=self.env.now + cycleSeconds)
            self.addCollectedSKUCount()
        if printOutput:
            #self.timeStatDF.to_excel('outputVRP.xlsx', index=False)
            #cycle başında ve sonu üst üste gelince duplicate var
            writer = pd.ExcelWriter('experiment/outputVRP.xlsx', engine='xlsxwriter')
            self.timeStatDF.to_excel(writer, sheet_name='Sheet1', index=False)
            df = self.calculateObservationStat()
            df.to_excel(writer, sheet_name='Sheet2', index=False)
            writer._save()

    def podStationDistCalculate(self, notDeliveredPods):
        def manhattan_distance(tuple1, tuple2):
            return sum(abs(a - b) for a, b in zip(tuple1, tuple2))

        if self.TaskAssignmentPolicy == "vrp" or self.TaskAssignmentPolicy == "rl":
            for task in self.extractTaskList:
                if task.pod not in notDeliveredPods:
                    self.totalPodStationDist += 2 * manhattan_distance(task.pod.fixedLocation, task.outputstation.location)
        elif self.TaskAssignmentPolicy == "rawsimo":
            tempTaskList = []
            for lst in self.extractTaskList:
                tempTaskList.extend(lst)

            for task in tempTaskList:
                if task.pod not in notDeliveredPods:
                    self.totalPodStationDist += 2 * manhattan_distance(task.pod.fixedLocation, task.outputstation.location)
        else:
            raise Exception("Unknown TaskAssignmentPolicy")
    def TaguchiVRP(self, numCycle, cycleSeconds, printOutput=False, allItemList = None, numOrderPerCycle=30):

        self.numCycle = numCycle
        self.cycleSeconds = cycleSeconds
        for cycle_idx in range(numCycle):
            self.currentCycle = cycle_idx
            print("Cycle: ", cycle_idx)
            if allItemList:
                itemlist = allItemList[cycle_idx]
            else:
                itemlist = (self.orderGenerator(numOrder=numOrderPerCycle))
            for robot in self.Robots:
                if robot.taskList:
                    a = 10
            self.startCycleVRP(itemlist=itemlist, cycleSeconds=cycleSeconds, cycleIdx=cycle_idx)
            self.env.run(until=self.env.now + cycleSeconds)
            self.addCollectedSKUCount()
        if printOutput:
            #self.timeStatDF.to_excel('outputVRP.xlsx', index=False)
            #cycle başında ve sonu üst üste gelince duplicate var
            #writer = pd.ExcelWriter('outputVRP.xlsx', engine='xlsxwriter')
            #self.timeStatDF.to_excel(writer, sheet_name='Sheet1', index=False)
            df = self.calculateObservationStat()
            #df.to_excel(writer, sheet_name='Sheet2', index=False)
            #writer._save()

        return self.timeStatDF, df

    def podSelectionRawSIMO(self, selectedPodsList, station):
        taskList = []
        for pod in selectedPodsList:
            task = ExtractTask(env=env, robot=None, outputstation=station, pod=pod)
            taskList.append(task)
        return taskList

    def combineItemListsRawSIMO(self, itemlist):

        def itemListSum(array_2d):
            """
            Aggregates itemList SKU-wise
            :param array_2d:
            :return: sums: 2d np array
            """
            # Extract unique values from the first column
            unique_first_column = np.unique(array_2d[:, 0])

            # Initialize an array to store the sums
            sums = np.zeros(shape=(len(unique_first_column), 2), dtype=int)

            # Iterate over the unique values in the first column
            for i, value in enumerate(unique_first_column):
                # Sum the second column where the first column matches the current unique value
                sums[i, 0] = value
                sums[i, 1] = np.sum(array_2d[array_2d[:, 0] == value, 1])
            return sums

        notDeliveredPods = []

        for robot in self.Robots:
            #if robot.pod == None and type(robot.currentTask) == ExtractTask: # vrp anında pod almaya giden robotların görevlerini silip onları da dahil etmek için
            #    notDeliveredPods.append(robot.currentTask.pod)
            if robot.taskList:
                for task in robot.taskList:
                    notDeliveredPods.append(task.pod)

        self.podStationDistCalculate(notDeliveredPods=notDeliveredPods)

        tempList = []
        for p in notDeliveredPods:
            for idx_list, podListForStation in enumerate(self.selectedPodsList):
                try:
                    idx = podListForStation.index(p)
                    for sku, value in self.satisfiedList[idx_list][idx].items():
                        tempList.append([sku, value])
                except:
                    continue

        tempArr = np.array(tempList)
        if len(tempArr)>0:
            itemListRaw = np.vstack((itemlist, tempArr))
            finalItemList = itemListSum(itemListRaw)
            return finalItemList
        else:
            return itemlist


    def startCycleRawSIMO(self, itemlist, cycleSeconds, cycleIdx):

        def divide_list_into_equal_sublists(lst, num_sublists):
            # Calculate the size of each sublist
            sublist_size = len(lst) // num_sublists
            # Calculate the number of remaining elements
            remaining = len(lst) % num_sublists
            # Initialize the start index for slicing
            start = 0
            # Initialize the list of sublists
            sublists = []

            # Divide the list into sublists
            for _ in range(num_sublists):
                # Calculate the end index for slicing
                end = start + sublist_size + (1 if remaining > 0 else 0)
                # Append the sublist to the list of sublists
                sublists.append(lst[start:end])
                # Update the start index for the next sublist
                start = end
                # Decrement the remaining elements
                remaining -= 1
            return sublists

        if cycleIdx != 0:
            itemlist = self.combineItemListsRawSIMO(itemlist=itemlist)

        #itemListDivided = np.reshape(itemlist, newshape=(len(self.OutputStations), itemlist.shape[0] // len(self.OutputStations), itemlist.shape[1]))
        n = len(self.OutputStations)
        itemListDivided = divide_list_into_equal_sublists(lst=itemlist, num_sublists=n)
        self.extractTaskList = []
        self.selectedPodsList = []
        self.satisfiedList = []

        for stationIdx, station in enumerate(self.OutputStations):
            itemListStation = itemListDivided[stationIdx]
            selectedPodsList, satisfiedList = self.podSelectionMaxHitRate(itemList=itemListStation, satisfiedReturn=True)
            taskList = self.podSelectionRawSIMO(selectedPodsList=selectedPodsList, station=station)
            self.extractTaskList.append(taskList)

        allTasks = []
        for lst in self.extractTaskList:
            allTasks.extend(lst)
        #self.totalPodNumber += len(allTasks)



        self.fixedLocationRawSIMO(assign=True)

        #self.env._queue = []

        for i in range(1, cycleSeconds+1):
            self.env.process(self.updateCharge(t=i, updateTime=1))

        for i in range(0, self.cycleSeconds//60 + 1):
            self.env.process(self.collectTimeStat(t=i, cycleSeconds=cycleSeconds))

        if cycleIdx == 0:
            for robot in self.Robots:
                if robot.status == "charging" and robot.targetNode != robot.currentNode:
                    robot.createPath(robot.targetNode)
                    self.env.process(robot.move())
                elif robot.currentTask != None:
                    self.env.process(robot.DoExtractTask(robot.currentTask))
                elif robot.taskList:
                    self.env.process(robot.DoExtractTask(robot.taskList[0]))
                else:
                    self.env.process(robot.goRest())
        else:
            for robot in self.Robots:
                if robot.status == "rest" and robot.batteryLevel > robot.MaxBattery * robot.RestRate:
                    if robot.taskList:
                        self.env.process(robot.DoExtractTask(robot.taskList[0]))

    def MultiCycleRawSIMO(self, numCycle, cycleSeconds, printOutput=False, allItemList = None, numOrderPerCycle = 30):
        #random.seed(42)
        self.numCycle = numCycle
        self.cycleSeconds = cycleSeconds
        for cycle_idx in range(numCycle):
            self.currentCycle = cycle_idx
            print(cycle_idx)
            if allItemList:
                itemlist = allItemList[cycle_idx]
            else:
                itemlist = (self.orderGenerator(numOrder=numOrderPerCycle))

            for robot in self.Robots:
                if robot.taskList:
                    a = 10
            self.startCycleRawSIMO(itemlist=itemlist, cycleSeconds=cycleSeconds, cycleIdx=cycle_idx)
            self.env.run(until=self.env.now + cycleSeconds)
            self.addCollectedSKUCount()
        if printOutput:
            writer = pd.ExcelWriter('experiment/outputRAWSIMO.xlsx', engine='xlsxwriter')
            #self.timeStatDF.to_excel('outputRAWSIMO.xlsx', index=False)
            self.timeStatDF.to_excel(writer, sheet_name='Sheet1', index=False)
            df = self.calculateObservationStat()
            df.to_excel(writer, sheet_name='Sheet2', index=False)
            writer._save()

    def assignRLroutes(self, toursListSplit, properRobots):
        def manhattan_distance(tuple1, tuple2):
            return sum(abs(a - b) for a, b in zip(tuple1, tuple2))

        robotCount = 0
        numTours = len(toursListSplit)
        # properRobots = []
        # for i, robot in enumerate(self.Robots):
        #     if robot.status != "charging" and robot.batteryLevel > robot.MaxBattery * robot.RestRate:
        #         robotCount += 1
        #         properRobots.append(robot)
        dimension = max(len(properRobots), numTours)

        robotAndTask_distance = np.zeros(shape=(dimension, dimension))
        orderOfTasks = np.zeros(shape=(dimension, dimension))

        if numTours > len(properRobots):
            raise Exception("Task sayısı robottan daha fazla")

        for robot_idx, robot in enumerate(properRobots):
            for route_idx, route in enumerate(toursListSplit):
                if len(route) == 1:
                    task = self.extractTaskList[route[0]]
                    dist = manhattan_distance(robot.currentNode, task.pod.fixedLocation)
                    robotAndTask_distance[robot_idx][route_idx] = dist
                else:
                    startTask = self.extractTaskList[route[0]]
                    endTask = self.extractTaskList[route[-1]]

                    distStart = manhattan_distance(startTask.pod.fixedLocation, robot.currentNode)
                    distEnd = manhattan_distance(endTask.pod.fixedLocation, robot.currentNode)

                    if distStart <= distEnd:
                        robotAndTask_distance[robot_idx][route_idx] = distStart
                        orderOfTasks[robot_idx][route_idx] = 1
                    else:
                        robotAndTask_distance[robot_idx][route_idx] = distEnd
                        orderOfTasks[robot_idx][route_idx] = -1

        row_ind, col_ind = linear_sum_assignment(robotAndTask_distance)

        for idx, robot in enumerate(properRobots):
            if orderOfTasks[idx][col_ind[idx]] == 0:
                continue
            else:
                tour = toursListSplit[col_ind[idx]]
                if orderOfTasks[idx][col_ind[idx]] == -1:
                    tour.reverse()
                for idx1 in tour:
                    robot.taskList.append(self.extractTaskList[idx1])




    def fixedLocationRL(self, taskList, assign=True):
        max_y = self.network.graph["cols"]-1
        max_x = self.network.graph["rows"]-1

        loc = []
        for task in self.extractTaskList:
            x = task.pod.fixedLocation[0]/max_x
            y = task.pod.fixedLocation[1]/max_y
            loc.append([x,y])
        locArr = np.array(loc)

        properRobots = []
        robotCount = 0
        for i, robot in enumerate(self.Robots):
            if robot.status != "charging" and robot.batteryLevel > robot.MaxBattery * robot.RestRate:
                robotCount += 1
                properRobots.append(robot)

        #demand = np.zeros(shape=len(loc))
        demand = np.full(shape=len(loc),fill_value=len(properRobots)/(1.5*len(loc)+2*len(properRobots)))
        depot = np.array([0.5, 0.5])

        locArr = torch.Tensor(locArr)
        demand = torch.Tensor(demand)
        depot = torch.Tensor(depot)

        data = VRPDatasetNew(size=len(loc), num_samples=1, loc=locArr, demand=demand, depot=depot)
        dataloader = DataLoader(data, batch_size=len(loc))
        batch = next(iter(dataloader))

        model, _ = load_model('RL/pretrained/cvrp_20/')
        model.eval()
        model.set_decode_type('greedy')
        with torch.no_grad():
            length, log_p, pi = model(batch, return_pi=True)
        tours = pi
        print(tours)

        toursList = tours.tolist()[0]
        toursList = [x - 1 for x in toursList]
        toursListSplit = []
        temp_list = []
        for i in toursList:
            if i == -1:
                toursListSplit.append(temp_list)
                temp_list = []
            else:
                temp_list.append(i)
        if temp_list != []: toursListSplit.append(temp_list)

        if assign == True:
            self.assignRLroutes(toursListSplit=toursListSplit, properRobots=properRobots)
        b = 10
    def startCycleRL(self, itemlist, cycleSeconds, cycleIdx):

        if cycleIdx != 0:
            itemlist = self.combineItemListsVRP(itemlist=itemlist)

        selectedPodsList, satisfiedList = self.podSelectionMaxHitRate(itemlist, satisfiedReturn=True)
        extractTaskList = self.podSelectionHungarian(selectedPodsList, outputTask=True)
        #self.totalPodNumber += len(selectedPodsList)
        start = time.time()
        self.extractTaskList = extractTaskList
        self.fixedLocationRL(assign=True, taskList=extractTaskList)
        end = time.time()
        print("VRP TIME: ", end - start)

        for i in range(1, cycleSeconds + 1):
            self.env.process(self.updateCharge(t=i, updateTime=1))

        for i in range(0, self.cycleSeconds // 60 + 1):
            self.env.process(self.collectTimeStat(t=i, cycleSeconds=cycleSeconds))

        if cycleIdx == 0:
            for robot in self.Robots:
                if robot.status == "charging" and robot.targetNode != robot.currentNode:
                    robot.createPath(robot.targetNode)
                    self.env.process(robot.move())
                elif robot.currentTask != None:
                    self.env.process(robot.DoExtractTask(robot.currentTask))
                elif robot.taskList:
                    self.env.process(robot.DoExtractTask(robot.taskList[0]))
                else:
                    self.env.process(robot.goRest())
        else:
            for robot in self.Robots:
                if robot.status == "rest" and robot.batteryLevel > robot.MaxBattery * robot.RestRate:
                    if robot.taskList:
                        self.env.process(robot.DoExtractTask(robot.taskList[0]))

    def MultiCycleRL(self, numCycle, cycleSeconds, printOutput=False, allItemList = None, numOrderPerCycle=30):

        self.numCycle = numCycle
        self.cycleSeconds = cycleSeconds
        for cycle_idx in range(numCycle):
            self.currentCycle = cycle_idx
            print("Cycle: ", cycle_idx)
            if allItemList:
                itemlist = allItemList[cycle_idx]
            else:
                itemlist = (self.orderGenerator(numOrder=numOrderPerCycle))
            for robot in self.Robots:
                if robot.taskList:
                    a = 10
            self.startCycleRL(itemlist=itemlist, cycleSeconds=cycleSeconds, cycleIdx=cycle_idx)
            self.env.run(until=self.env.now + cycleSeconds)
            self.addCollectedSKUCount()
        if printOutput:
            #self.timeStatDF.to_excel('outputVRP.xlsx', index=False)
            #cycle başında ve sonu üst üste gelince duplicate var
            writer = pd.ExcelWriter('experiment/outputRL.xlsx', engine='xlsxwriter')
            self.timeStatDF.to_excel(writer, sheet_name='Sheet1', index=False)
            df = self.calculateObservationStat()
            df.to_excel(writer, sheet_name='Sheet2', index=False)
            writer._save()





def PhaseITaskAssignmentExperiment(numTask, network, OutputLocations, ChargeLocations, RobotLocations):
    def divide_list_into_n_sublists(lst, n):
        # Calculate the length of each sublist
        sublist_length = len(lst) // n
        # Initialize the list of sublists
        sublists = [lst[i:i + sublist_length] for i in range(0, len(lst), sublist_length)]
        return sublists
    def divide_list(lst, num_groups):
        # Calculate the size of each group
        group_size = len(lst) // num_groups
        remainder = len(lst) % num_groups

        # Divide the list into groups
        groups = []
        start = 0
        for i in range(num_groups):
            group_end = start + group_size + (1 if i < remainder else 0)
            groups.append(lst[start:group_end])
            start = group_end

        return groups


    env1 = simpy.Environment()
    rawsimoModel = RMFS_Model(env=env1, network=network, TaskAssignmentPolicy="rawsimo", ChargePolicy="rawsimo")
    rawsimoModel.createPods()
    rawsimoModel.createSKUs()
    rawsimoModel.fillPods()
    rawsimoModel.createChargingStations(ChargeLocations)
    rawsimoModel.createRobots(RobotLocations)
    rawsimoModel.createOutputStations(OutputLocations)
    rawsimoModel.distanceMatrixCalculate()


    env2 = simpy.Environment()
    anomalyModel = RMFS_Model(env=env2, network=network, TaskAssignmentPolicy="vrp", ChargePolicy="pearl")
    anomalyModel.Pods = copy.deepcopy(rawsimoModel.Pods)
    anomalyModel.SKUs = copy.deepcopy(rawsimoModel.SKUs)
    anomalyModel.createChargingStations(ChargeLocations)
    anomalyModel.createRobots(RobotLocations)
    anomalyModel.createOutputStations(OutputLocations)
    anomalyModel.distanceMatrixCalculate()

    randomPodIndexList = random.sample(range(len(rawsimoModel.Pods)), numTask)

    rawsimoPodList = divide_list_into_n_sublists(randomPodIndexList, len(rawsimoModel.OutputStations))
    rawsimoModel.extractTaskList = []

    for stationIdx, station in enumerate(rawsimoModel.OutputStations):
        taskList = []
        for pod_idx in rawsimoPodList[stationIdx]:
            pod = rawsimoModel.Pods[pod_idx]
            sampleTask = ExtractTask(env=env1,robot=None, pod=pod, outputstation=station)
            taskList.append(sampleTask)
        rawsimoModel.extractTaskList.append(taskList)
    #satisfied eklenmedi, gerek yok

    allocatedRobotsList = divide_list(rawsimoModel.Robots, len(rawsimoModel.OutputStations))

    for idx, stationTaskList in enumerate(rawsimoModel.extractTaskList):
        stationRobots = allocatedRobotsList[idx]
        numRobot = len(stationRobots)
        for taskNum, task in enumerate(stationTaskList):
            task.robot = stationRobots[taskNum % numRobot]
            stationRobots[taskNum % numRobot].taskList.append(task)


    selectedPodsList = [anomalyModel.Pods[i] for i in randomPodIndexList]
    extractTaskListVRP = anomalyModel.podSelectionHungarian(selectedPodsList, outputTask=True)
    anomalyModel.extractTaskList = extractTaskListVRP
    anomalyModel.fixedLocationVRP(extractTaskListVRP, assign=True)


    for robot in rawsimoModel.Robots:
        if robot.taskList:
            rawsimoModel.env.process(robot.DoExtractTask(robot.taskList[0]))
        else:
            rawsimoModel.env.process(robot.goRest())



    for robot in anomalyModel.Robots:
        if robot.taskList:
            anomalyModel.env.process(robot.DoExtractTask(robot.taskList[0]))
        else:
            anomalyModel.env.process(robot.goRest())

    env1.run()
    env2.run()

    RawsimoDist = 0
    AnomalyDist = 0


    for robot in rawsimoModel.Robots:
        RawsimoDist += robot.stepsTaken

    for robot in anomalyModel.Robots:
        AnomalyDist += robot.stepsTaken

    print("Rawsimo task assignment steps taken: ", RawsimoDist)
    print("VRP task assignment steps taken: ", AnomalyDist)



def PhaseIandIICompleteExperiment(numOrderPerCycle, network, OutputLocations, ChargeLocations, RobotLocations, numCycle, cycleSeconds):
    #robot sayısı outputstation katı olmalı
    env1 = simpy.Environment()
    rawsimoModel = RMFS_Model(env=env1, network=network, TaskAssignmentPolicy="rawsimo", ChargePolicy="rawsimo")
    rawsimoModel.createPods()
    rawsimoModel.createSKUs()
    rawsimoModel.fillPods()
    rawsimoModel.createChargingStations(ChargeLocations)
    rawsimoModel.createRobots(RobotLocations)
    rawsimoModel.createOutputStations(OutputLocations)
    rawsimoModel.distanceMatrixCalculate()


    env2 = simpy.Environment()
    anomalyModel = RMFS_Model(env=env2, network=network, TaskAssignmentPolicy="vrp", ChargePolicy="pearl")
    anomalyModel.Pods = copy.deepcopy(rawsimoModel.Pods)
    anomalyModel.SKUs = copy.deepcopy(rawsimoModel.SKUs)
    anomalyModel.createChargingStations(ChargeLocations)
    anomalyModel.createRobots(RobotLocations)
    anomalyModel.createOutputStations(OutputLocations)
    anomalyModel.distanceMatrixCalculate()

    allItemList = []
    for cycle_idx in range(numCycle):
        itemList = anomalyModel.orderGenerator(numOrder=numOrderPerCycle)
        allItemList.append(itemList)

    rawsimoModel.MultiCycleRawSIMO(numCycle=numCycle, cycleSeconds=cycleSeconds, printOutput=True, allItemList = allItemList)
    anomalyModel.MultiCycleVRP(numCycle=numCycle, cycleSeconds=cycleSeconds, printOutput=True, allItemList=allItemList)

def RawsimovsVRPvsRLexp(numOrderPerCycle, network, OutputLocations, ChargeLocations, RobotLocations, numCycle, cycleSeconds):
    env1 = simpy.Environment()

    rawsimoModel = RMFS_Model(env=env1, network=network, TaskAssignmentPolicy="rawsimo", ChargePolicy="rawsimo")
    rawsimoModel.createPods()
    rawsimoModel.createSKUs()
    rawsimoModel.fillPods()
    rawsimoModel.createChargingStations(ChargeLocations)
    rawsimoModel.createRobots(RobotLocations)
    rawsimoModel.createOutputStations(OutputLocations)
    rawsimoModel.distanceMatrixCalculate()


    env2 = simpy.Environment()
    anomalyModel = RMFS_Model(env=env2, network=network, TaskAssignmentPolicy="vrp", ChargePolicy="pearl")
    anomalyModel.Pods = copy.deepcopy(rawsimoModel.Pods)
    anomalyModel.SKUs = copy.deepcopy(rawsimoModel.SKUs)
    anomalyModel.createChargingStations(ChargeLocations)
    anomalyModel.createRobots(RobotLocations)
    anomalyModel.createOutputStations(OutputLocations)
    anomalyModel.distanceMatrixCalculate()

    env3 = simpy.Environment()
    RlModel = RMFS_Model(env=env3, network=network, TaskAssignmentPolicy="rl", ChargePolicy="pearl")
    RlModel.Pods = copy.deepcopy(rawsimoModel.Pods)
    RlModel.SKUs = copy.deepcopy(rawsimoModel.SKUs)
    RlModel.createChargingStations(ChargeLocations)
    RlModel.createRobots(RobotLocations)
    RlModel.createOutputStations(OutputLocations)
    RlModel.distanceMatrixCalculate()

    allItemList = []
    for cycle_idx in range(numCycle):
        itemList = anomalyModel.orderGenerator(numOrder=numOrderPerCycle)
        allItemList.append(itemList)

    rawsimoModel.MultiCycleRawSIMO(numCycle=numCycle, cycleSeconds=cycleSeconds, printOutput=True, allItemList = allItemList)
    anomalyModel.MultiCycleVRP(numCycle=numCycle, cycleSeconds=cycleSeconds, printOutput=True, allItemList=allItemList)
    RlModel.MultiCycleRL(numCycle=numCycle, cycleSeconds=cycleSeconds, printOutput=True, allItemList=allItemList)

def oneCycleVRPvsRL(numTask, network, OutputLocations, ChargeLocations, RobotLocations, returnStat=False):
    env1 = simpy.Environment()

    RlModel = RMFS_Model(env=env1, network=network, TaskAssignmentPolicy="rl", ChargePolicy="pearl")
    RlModel.createPods()
    RlModel.createSKUs()
    RlModel.fillPods()
    RlModel.createChargingStations(ChargeLocations)
    RlModel.createRobots(RobotLocations)
    RlModel.createOutputStations(OutputLocations)
    RlModel.distanceMatrixCalculate()


    env2 = simpy.Environment()
    anomalyModel = RMFS_Model(env=env2, network=network, TaskAssignmentPolicy="vrp", ChargePolicy="pearl")
    anomalyModel.Pods = copy.deepcopy(RlModel.Pods)
    anomalyModel.SKUs = copy.deepcopy(RlModel.SKUs)
    anomalyModel.createChargingStations(ChargeLocations)
    anomalyModel.createRobots(RobotLocations)
    anomalyModel.createOutputStations(OutputLocations)
    anomalyModel.distanceMatrixCalculate()

    randomPodIndexList = random.sample(range(len(RlModel.Pods)), numTask)

    selectedPodsList = [anomalyModel.Pods[i] for i in randomPodIndexList]
    extractTaskListVRP = anomalyModel.podSelectionHungarian(selectedPodsList, outputTask=True)
    anomalyModel.extractTaskList = extractTaskListVRP

    start = time.time()
    anomalyModel.fixedLocationVRP(extractTaskListVRP, assign=True)
    end = time.time()
    vrpTime = end - start
    print("VRP TIME: ", vrpTime)

    RlModel.extractTaskList = extractTaskListVRP
    start = time.time()
    RlModel.fixedLocationRL(taskList=extractTaskListVRP, assign=True)
    end = time.time()
    rlTime = end - start
    print("RL TIME: ", rlTime)

    for robot in RlModel.Robots:
        if robot.taskList:
            RlModel.env.process(robot.DoExtractTask(robot.taskList[0]))
        else:
            RlModel.env.process(robot.goRest())



    for robot in anomalyModel.Robots:
        if robot.taskList:
            anomalyModel.env.process(robot.DoExtractTask(robot.taskList[0]))
        else:
            anomalyModel.env.process(robot.goRest())

    env1.run()
    env2.run()

    RlDist = 0
    AnomalyDist = 0


    for robot in RlModel.Robots:
        RlDist += robot.stepsTaken

    for robot in anomalyModel.Robots:
        AnomalyDist += robot.stepsTaken

    print("RL task assignment steps taken: ", RlDist)
    print("VRP task assignment steps taken: ", AnomalyDist)

    if returnStat:
        return AnomalyDist, RlDist, vrpTime, rlTime

if __name__ == "__main__":
    env = simpy.Environment()


    #rows = 13  #4x8
    #columns = 41

    #rows = 16  #5x5
    #columns = 26

    rows = 19 #6x12
    columns = 61

    #rows = 25 #8x8
    #columns = 41

    #rows = 31 #10x20
    #columns = 101

    #rows = 10 #3x6
    #columns = 31

    #rows = 10 #3x3
    #columns = 16

    #rows=16 #5x15
    #columns=76

    #rows=31 #10x10
    #columns = 51

    rectangular_network, pos = layout.create_rectangular_network_with_attributes(columns, rows)
    layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
    output = [(20, 12), (40, 6)]#, (20,0), (0,6)]
    charging = [(0, 12)]
    robots = [(0, 0), (40, 0)]#, (0,12), (40,12)]#, (0,30), (100,30)]#, (0, 7), (1, 8)]# (1, 9), (1, 0), (1, 7), (0, 4), (15,0)]
    #output = [(12, 15), (25, 7)]
    #layout.draw_network_with_shelves(rectangular_network, pos)




    # Rawsimo task assignment vs VRP aynı podları ikisine de veriyor, bir cycledaki toplam alınan mesafeyi veriyor
    # PhaseITaskAssignmentExperiment(numTask=30, network=rectangular_network, OutputLocations=output, ChargeLocations=charging, RobotLocations=robots)

    for numTask in [20, 40]:
        totalVRPDist = 0
        totalRlDist = 0
        totalVRPtime = 0
        totalRLtime = 0
        for i in range(10):
            vrpDist, RlDist, vrpTime, rlTime = oneCycleVRPvsRL(numTask=numTask, network=rectangular_network, OutputLocations=output, ChargeLocations=charging, RobotLocations=robots, returnStat=True)
            totalVRPDist += vrpDist
            totalRlDist += RlDist
            totalVRPtime += vrpTime
            totalRLtime += rlTime
        print("NUMTASK: ",numTask)
        avgVRPDist = totalVRPDist/10
        avgRlDist = totalRlDist/10
        avgVRPtime = totalVRPtime/10
        avgRLtime = totalRLtime/10
        print("avgVRPDist: ",avgVRPDist)
        print("avgRlDist: ", avgRlDist)
        print("avgVRPtime: ", avgVRPtime)
        print("avgRLtime: ", avgRLtime)


    # Aynı orderları her cycleda veriyor ve her şeyi karşılaştırıyor; pod seçimi, task assignment ve şarj politikası
    #PhaseIandIICompleteExperiment(numOrderPerCycle=20, network=rectangular_network, OutputLocations=output, ChargeLocations=charging, RobotLocations=robots, numCycle=32, cycleSeconds=900)
    #RawsimovsVRPvsRLexp(numOrderPerCycle=23, network=rectangular_network, OutputLocations=output, ChargeLocations=charging, RobotLocations=robots, numCycle=5, cycleSeconds=900)
    a = 15




    nodes = list(rectangular_network.nodes)
    #simulation = RMFS_Model(env=env, network=rectangular_network, TaskAssignmentPolicy="vrp", ChargePolicy="pearl")
    #simulation = RMFS_Model(env=env, network=rectangular_network, TaskAssignmentPolicy="rawsimo", ChargePolicy="rawsimo")
    simulation = RMFS_Model(env=env, network=rectangular_network, TaskAssignmentPolicy="rl",ChargePolicy="pearl")

    simulation.createPods()
    simulation.createSKUs()




    simulation.createChargingStations([(0, 9)])
    #startLocations = [(0, 8), (5, 0), (10, 9), (15, 0), (5, 6), (1, 8), (5, 2), (10, 8), (15, 1)]
    startLocations = [(0, 0), (50, 30)]
    simulation.createRobots(startLocations)

    firstStation = (25, 30)
    secondStation = (50, 15)
    thirdStation = (20, 0)
    fourthStation = (0, 6)
    locations = [firstStation, secondStation]

    simulation.createOutputStations(locations)

    simulation.fillPods()
    simulation.distanceMatrixCalculate()

    orderlist = simulation.orderGenerator(20)
    # Phase I pod selection karşılaştırması
    selectedPodsList, numSelectedPodsP1, total_distance, selectedPodsListRawsimo, numSelectedPodsRawsimo, totalDistRawsimo = simulation.PhaseIExperiment(orderList=orderlist, returnSelected=True)
    a = 10


    #simulation.Robots[0].batteryLevel = 41.6
    #simulation.Robots[1].batteryLevel = 35.36

    #simulation.MultiCycleVRP(5,900, printOutput=True, numOrderPerCycle=80)
    #simulation.MultiCycleRawSIMO(32,900, printOutput=True, numOrderPerCycle=100)
    #simulation.MultiCycleRL(32, 900, printOutput=True, numOrderPerCycle=23)

