import numpy as np
import pandas as pd
import simpy
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask
import koridor_deneme
import random
import ast
from vrp import distanceMatrixCreate

rows = 10  # 3x3
columns = 16

rectangular_network, pos = koridor_deneme.create_rectangular_network_with_attributes(columns, rows)
koridor_deneme.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
koridor_deneme.draw_network_with_shelves(rectangular_network, pos)
network_corridors = koridor_deneme.create_corridor_subgraph(rectangular_network)

distMatrix, nodes = distanceMatrixCreate(rectangular_network)

station1_node = np.array(['(0,0)']) # sol alt
station2_node = np.array(['(15,0)']) # sağ alt

pod_node = np.array(['(1,1)', '(1,4)', '(1,7)', '(6,1)', '(6,4)', '(6,7)', '(11,1)', '(11,4)', '(11,7)',])




