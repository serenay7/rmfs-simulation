import time

import numpy as np
import pandas as pd
import simpy
from simpy.events import AllOf
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
import math
import copy

rows = 10 #3x3
columns = 16


rectangular_network, pos = layout.create_rectangular_network_with_attributes(columns, rows)
layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
output = [(0, 5)]
charging = [(0, 9)]
robots = [(0, 8), (5, 0), (10, 9)]

layout.draw_network_with_shelves(rectangular_network, pos)