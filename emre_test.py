import numpy as np
import pandas as pd
import simpy
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask
import koridor_deneme
import random
import ast




class RMFS_Model():
    def __init__(self, env, network):
        self.env = env
        self.network = network



if __name__ == "__main__":
    env = simpy.Environment()


    #rows = 13  4x8
    #columns = 41

    #rows = 16  #5x5
    #columns = 26

    rows = 10 #3x6
    columns = 31

    #rows = 10 #3x3
    #columns = 16

    rectangular_network, pos = koridor_deneme.create_rectangular_network_with_attributes(columns, rows)
    koridor_deneme.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
    koridor_deneme.draw_network_with_shelves(rectangular_network, pos)

    network_corridors = koridor_deneme.create_corridor_subgraph(rectangular_network)

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