import numpy as np
import pandas as pd
import simpy
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask, SKU
import layout
import random
import ast
from lp_podselection import podAndStation_combination, calculate_total_distances_for_all_requirements, min_max_diff, check_feasibility, columnMultiplication, assign_pods_to_stations
#from vrp import create_data_model

if __name__ == "__main__":
    env = simpy.Environment()

    rows = 10 #3x3
    columns = 16

    rectangular_network, pos = layout.create_rectangular_network_with_attributes(columns, rows)
    #layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))

    nodes = list(rectangular_network.nodes)

    network_corridors = layout.create_corridor_subgraph(rectangular_network)

    #layout.draw_network_with_shelves(rectangular_network, pos)

    station1 = OutputStation(env, (15,1), pickItemList=None)

    pod1 = Pod(env, (1,1))

    robot1 = Robot(env, network_corridors, rectangular_network, 1, pod=None, currentNode=(0,0),
                 targetNode=None, currentTask=None, taskList=None, loadedSpeed=1, emptySpeed=2, takeTime=3, dropTime=3, batteryLevel = 9, moveLoaded = 14.53265, moveEmpty = 11.9566, chargingRate = 41.6, chargeThreshold=35, chargingStationLocation=(0, 1))
    
    sampleTask = ExtractTask(env, robot=robot1, outputstation=station1, pod=pod1)

    
    #robot1.DoExtractTask(sampleTask)
    env.process(robot1.DoExtractTask(sampleTask))
    env.run()

a = 10