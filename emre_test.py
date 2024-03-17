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

    rows = 16  #5x5
    columns = 26

    #rows = 10 3x6
    #columns = 31

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
    OStation2 = OutputStation(env, (15, 5), itemlist)

    exp_df = pd.read_excel("./experiment/5x5 4robot 2istasyon.xlsx",sheet_name="OUTPUT FILTERED")
    #exp_df["Bot"] = exp_df["Bot"].astype(int)
    robot_count  = np.int_(exp_df["Bot"].max()) + 1
    total_dist = 0
    robot1 = Robot(env, network_corridors, rectangular_network, 1, None, (0, 0))
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
        #total_dist += robot1.stepsTaken
        #print(robot1.stepsTaken)







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