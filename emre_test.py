import numpy as np
import simpy
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask
import network




class RMFS_Model():
    def __init__(self, env, network):
        self.env = env
        self.network = network



if __name__ == "__main__":
    env = simpy.Environment()


    rows = 10
    columns = 20

    rectangular_network = network.create_rectangular_network(rows, columns)
    node = list(rectangular_network.nodes)
    start_node = node[0]
    target_node = node[2]

    robot1 = Robot(env, rectangular_network, 1, None, start_node)

    itemlist = np.array(([1,10],
                         [2,10]))


    podSKUList = np.array(([1,50],
                           [2,50]))
    samplePod = Pod(env,podSKUList, None, robot1)
    OStation1 = OutputStation(env,(0,3),itemlist)
    robot1.pod = samplePod
    sampleExtractTask = ExtractTask(env, robot1, OStation1, samplePod)

    env.process(robot1.DoExtractTask(sampleExtractTask))
    env.run(until=10)

### UPH İÇİN
# output_station = OutputStation(env, location, pickItemList)
# After some simulation steps where PickItems might be called
# print(f"PickItems was called {output_station.getPickItemsCount()} times.")