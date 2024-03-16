import numpy as np
import simpy
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask
import koridor_deneme




class RMFS_Model():
    def __init__(self, env, network):
        self.env = env
        self.network = network



if __name__ == "__main__":
    env = simpy.Environment()


    rows = 4
    columns = 4

    rectangular_network, pos = koridor_deneme.create_rectangular_network_with_attributes(rows, columns)
    koridor_deneme.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
    koridor_deneme.draw_network_with_shelves(rectangular_network, pos)

    network_corridors = koridor_deneme.create_corridor_subgraph(rectangular_network)

    node = list(rectangular_network.nodes)
    #start_node1 = node[12]
    start_node1 = (3,3)
    start_node2 = node[1]
    #target_node1 = node[15]
    target_node1 = (3,1)
    target_node2 = node[10]

    itemlist = np.array(([1, 10],
                         [2, 10]))

    podSKUList = np.array(([1, 50],
                           [2, 50]))
    samplePod1 = Pod(env, podSKUList, target_node1, None, "idle")
    samplePod2 = Pod(env, podSKUList, target_node2, None, "idle")

    robot1 = Robot(env, network_corridors, rectangular_network,1, samplePod1, start_node1)
    #robot2 = Robot(env, rectangular_network, network_corridors,2, None, start_node2)
    #robot1.createPath(target_node1)
    #env.process(robot1.move())
    #env.process(robot1.takePod(samplePod))

    sampleStorageTask1 = StorageTask(env, robot1, samplePod1, (3,1))
    env.process(robot1.DoStorageTask(sampleStorageTask1))

    env.run(until=20)


    OStation1 = OutputStation(env,(0,3),itemlist)

    robot1.pod = samplePod1
    #sampleExtractTask1 = ExtractTask(env, robot1, OStation1, samplePod1)
    #sampleExtractTask2 = ExtractTask(env, robot2, OStation1, samplePod2)

    #env.process(robot1.DoExtractTask(sampleExtractTask1))
    #env.process(robot2.DoExtractTask(sampleExtractTask2))

    #env.run(until=20)


### UPH İÇİN
# output_station = OutputStation(env, location, pickItemList)
# After some simulation steps where PickItems might be called
# print(f"PickItems was called {output_station.getPickItemsCount()} times.")