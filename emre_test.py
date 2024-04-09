import numpy as np
import pandas as pd
import simpy
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask, SKU
import layout
import random
import ast
from lp_podselection import podAndStation_combination, calculate_total_distances_for_all_requirements, min_max_diff, check_feasibility, columnMultiplication, assign_pods_to_stations




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
            tempStation = OutputStation(self.env, location=loc)
            self.OutputStations.append(tempStation)

    def createRobots(self, startLocations):
        """
        Creates robots and adds to a list which is a feature of RMFS_Model class
        :param startLocations: List of tuples
        """
        self.Robots = []
        for idx, loc in enumerate(startLocations):
            tempRobot = Robot(self.env, network_corridors=self.corridorSubgraph, network=self.network, robotID=idx, currentNode=loc)
            self.Robots.append(tempRobot)


    def podSelectionHitRateCalculation(self, itemList):
        """
        For a given itemList, finds the pod who has maximum hit rate
        :param itemList:
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
            itemList = [sublist for sublist in itemList if sublist[1] > 0]
            selectedPodsList.append(selectedPod)
            satisfiedList.append(satisfiedSKU)
        if satisfiedReturn:
            return selectedPodsList, satisfiedList

        return selectedPodsList

    def podSelectionHungarian(self, selectedPodsList, max_percentage=0.5):
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

    simulation = RMFS_Model(env=env, network=rectangular_network)
    simulation.createPods()
    simulation.createSKUs()
    simulation.fillPods()

    itemlist = np.array(([1, 10],
                         [2, 10],
                         [2, 10]))

    simulation.podSelectionMaxHitRate(itemlist)

    a = 10


    layout.draw_network_with_shelves(rectangular_network, pos)

    network_corridors = layout.create_corridor_subgraph(rectangular_network)

    #node = list(rectangular_network.nodes)
    #start_node1 = node[12]
    start_node1 = (3,3)
    #start_node2 = node[1]
    #target_node1 = node[15]
    #target_node1 = (3,1)
    #target_node2 = node[10]

    itemlist = np.array(([1, 10],
                         [2, 10]))

    podSKUList = np.array(([1, 50],
                           [2, 50]))
    #samplePod1 = Pod(env, podSKUList, target_node1, None, "idle")
    #samplePod2 = Pod(env, podSKUList, target_node2, None, "idle")


    #robot2 = Robot(env, rectangular_network, network_corridors,2, None, start_node2)
    #robot1.createPath(target_node1)
    #env.process(robot1.move())
    #env.process(robot1.takePod(samplePod))

    #sampleStorageTask1 = StorageTask(env, robot1, samplePod1, (3,1))
    #env.process(robot1.DoStorageTask(sampleStorageTask1))

    OStation1 = OutputStation(env, (0, 3), itemlist)
    OStation2 = OutputStation(env, (5, 9), itemlist)

    exp_df = pd.read_excel("./experiment/3x6-1Robot-2İstasyon.xlsx",sheet_name="OUTPUT FILTERED")
    #exp_df["Bot"] = exp_df["Bot"].astype(int)
    robot_count  = np.int_(exp_df["Bot"].max()) + 1
    total_dist = 0
    robot1 = Robot(env, network_corridors, rectangular_network, 1, None, (0, 0))
    test = 0
    for r in range(robot_count):
        for index, row in exp_df.iterrows():
            robot_id = int(row["Bot"])
            if robot_id == r:
                pod_loc = row["Coordinates"]
                station = row["OutputStation"]
                OStation = OStation1 if int(station) == 0 else OStation2
                pod_loc_tuple = ast.literal_eval(pod_loc)
                samplePod1 = Pod(env, podSKUList, pod_loc_tuple, None, "idle")
                sampleExtractTask1 = ExtractTask(env, robot1, OStation, samplePod1)
                env.process(robot1.DoExtractTask(sampleExtractTask1))

                sampleStorageTask1 = StorageTask(env, robot1, samplePod1, pod_loc_tuple)
                env.process(robot1.DoStorageTask(sampleStorageTask1))
                env.run()
                test += 1
        #total_dist += robot1.stepsTaken
        #print(robot1.stepsTaken)
        print(test)







    a = 10



    #robot1.pod = samplePod1
    #sampleExtractTask1 = ExtractTask(env, robot1, OStation1, samplePod1)
    #sampleExtractTask2 = ExtractTask(env, robot2, OStation1, samplePod2)

    #env.process(robot1.DoExtractTask(sampleExtractTask1))
    #env.process(robot2.DoExtractTask(sampleExtractTask2))

    #env.run(until=20)


### UPH İÇİN
# output_station = OutputStation(env, location, pickItemList)
# After some simulation steps where PickItems might be called
# print(f"PickItems was called {output_station.getPickItemsCount()} times.")

'''import numpy as np
import random

def generate_distribution_matrix(s, r, k, lower_bound, upper_bound):
    # Ensure k does not exceed the number of pods
    k = min(k, r)
    
    # Initialize the distribution matrix with zeros
    matrix = np.zeros((s, r), dtype=int)
    
    # Ensure every pod gets at least one SKU, if possible
    for sku in range(s):
        # Randomly select k unique pods for this SKU
        pods = random.sample(range(r), k)
        for pod in pods:
            # Assign a random amount within the bounds to this SKU in these pods
            matrix[sku, pod] = random.randint(lower_bound, upper_bound)
    
    # Check if there are any empty pods and try to redistribute if any
    for pod in range(r):
        if not matrix[:, pod].any():
            # Find a SKU and pod to redistribute
            for sku in range(s):
                if sum(matrix[sku, :]) > 1:  # SKU must be in more than one pod to redistribute
                    redistribute_pod = random.choice([p for p in range(r) if matrix[sku, p] > 0])
                    # Move a portion of the SKU from one pod to the empty pod
                    amount_to_move = matrix[sku, redistribute_pod] // 2
                    matrix[sku, redistribute_pod] -= amount_to_move
                    matrix[sku, pod] += amount_to_move
                    break
    
    return matrix.tolist()

# Example usage
s = 5  # SKUs
r = 3  # Pods
k = 2  # Max pods per SKU
lower_bound = 1
upper_bound = 100

# Generate and print the distribution matrix
distribution_matrix = generate_distribution_matrix(s, r, k, lower_bound, upper_bound)
distribution_matrix
'''