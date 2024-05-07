import tkinter as tk
from tkinter import ttk
import simpy
from emre_test import RMFS_Model
from interface_initializer import station_location, robot_location
from pyDOE3 import *

import numpy as np
import pandas as pd
from simpy.events import AllOf

from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask, SKU, ChargingStation
import layout

from lp_podselection import podAndStation_combination, calculate_total_distances_for_all_requirements, min_max_diff, check_feasibility, columnMultiplication, assign_pods_to_stations
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import networkx as nx


def run_simulation():
    print("started")

    # taguchi booleans
    pick_station_amount_tg = int(pick_station_amount_taguchi.get())
    pick_station_location_tg = int(pick_station_location_taguchi.get())
    charge_station_amount_tg = int(charge_station_amount_taguchi.get())
    charge_station_location_tg = int(charge_station_location_taguchi.get())
    robot_amount_tg = int(robot_amount_taguchi.get())
    charge_flag_rate_tg = int(charge_flag_rate_taguchi.get())
    max_charge_rate_tg = int(max_charge_rate_taguchi.get())

    # taguchi levels
    pick_station_amount_tg_lvl = 2 #tbu
    pick_station_location_tg_lvl = 4
    charge_station_amount_tg_lvl = 2 #tbu
    charge_station_location_tg_lvl = 4
    robot_amount_tg_lvl = 2 #tbu
    charge_flag_rate_tg_lvl = 2 #tbu
    max_charge_rate_tg_lvl = 2 #tbu

    tg_levels = [pick_station_amount_tg_lvl, pick_station_location_tg_lvl, charge_station_amount_tg_lvl, charge_station_location_tg_lvl, robot_amount_tg_lvl, charge_flag_rate_tg_lvl, max_charge_rate_tg_lvl]

    tg_experiment = fullfact(tg_levels)
    print(tg_experiment)

    # inputs
    horizontal_ailes = int(warehouse_horizontal_entry.get())
    vertical_ailes = int(warehouse_vertical_entry.get())
    pick_station_amount = int(pick_station_amount_entry.get()) 
    charge_station_amount = int(charge_station_amount_entry.get()) 
    pick_station_location = pick_station_location_combo.get()
    charge_station_location = charge_station_location_combo.get()
    cycle_amount = int(cycle_amount_entry.get())
    cycle_runtime = int(cycle_runtime_entry.get())
    robot_amount = int(robot_amount_entry.get())
    charging_rate = float(charging_rate_entry.get())
    max_battery = float(maximum_battery_entry.get())
    pearl_rate = float(pearl_rate_entry.get())
    rest_rate = float(rest_rate_entry.get())
    charge_flag_rate = float(charge_flag_rate_entry.get())
    max_charge_rate = float(max_charge_rate_entry.get())

    # is taguchi enabled?
    if pick_station_amount_tg==1 or pick_station_location_tg==1 or charge_station_amount_tg==1 or charge_station_location_tg==1 or robot_amount_tg==1 or charge_flag_rate_tg==1 or max_charge_rate_tg==1:
        enable_taguchi = 1
    else:
        enable_taguchi = 0

    # no taguchi, single run
    if enable_taguchi==0: 
        env = simpy.Environment()

        rows = (3*int(horizontal_ailes))+4 # 10
        columns = (5*int(vertical_ailes))+6 # 16

        rectangular_network, pos = layout.create_rectangular_network_with_attributes(columns, rows)
        layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))

        nodes = list(rectangular_network.nodes)
        simulation = RMFS_Model(env=env, network=rectangular_network)
        simulation.createPods()
        simulation.createSKUs()

        startLocations = robot_location(pick_station_location, charge_station_location, columns, rows, robot_amount)
        pick_locations, charge_locations = station_location(pick_station_amount, pick_station_location, charge_station_amount, charge_station_location, horizontal_ailes, vertical_ailes, columns, rows)
        
        simulation.createChargingStations(charge_locations)
        simulation.createOutputStations(pick_locations)

        simulation.fillPods()
        simulation.distanceMatrixCalculate()

        simulation.createRobots(startLocations, charging_rate, max_battery, pearl_rate, rest_rate, charge_flag_rate, max_charge_rate)

        simulation.MultiCycleVRP(cycle_amount,cycle_runtime, printOutput=True)
    
    elif enable_taguchi==1:
        print("taguchi is working")

    result_label.config(text="Simulation started...")  # Update this based on your simulation output

app = tk.Tk()
app.title("RMFS Simulation Interface")

# Create a frame for better organization
frame = ttk.Frame(app, padding="10")
frame.pack(fill=tk.BOTH, expand=True)

# Variables to hold the state of checkbuttons
do_taguchi = tk.BooleanVar()
pick_station_amount_taguchi = tk.BooleanVar()
pick_station_location_taguchi = tk.BooleanVar()
charge_station_amount_taguchi = tk.BooleanVar()
charge_station_location_taguchi = tk.BooleanVar()
robot_amount_taguchi = tk.BooleanVar()
charge_flag_rate_taguchi = tk.BooleanVar()
max_charge_rate_taguchi = tk.BooleanVar()

# SETTINGS START

ttk.Label(frame, text="Warehouse & Simulation Settings", font=('Helvetica', 12, 'bold')).grid(row=0, column=0, columnspan=2, sticky=tk.W)

ttk.Label(frame, text="Vertical Aisles:").grid(row=1, column=0, sticky=tk.W)
warehouse_vertical_entry = ttk.Entry(frame)
warehouse_vertical_entry.grid(row=1, column=1)
warehouse_vertical_entry.insert(0, "2")  # Default value

ttk.Label(frame, text="Horizontal Aisles:").grid(row=2, column=0, sticky=tk.W)
warehouse_horizontal_entry = ttk.Entry(frame)
warehouse_horizontal_entry.grid(row=2, column=1)
warehouse_horizontal_entry.insert(0, "2")  # Default value

ttk.Label(frame, text="Pick Station Amount:").grid(row=3, column=0, sticky=tk.W)
pick_station_amount_entry = ttk.Entry(frame)
pick_station_amount_entry.grid(row=3, column=1)
pick_station_amount_entry.insert(0, "1")  # Default value

ttk.Label(frame, text="Pick Station Location:").grid(row=4, column=0, sticky=tk.W)
pick_station_location_combo = ttk.Combobox(frame, values=["TOP", "BOTTOM", "LEFT", "RIGHT"])
pick_station_location_combo.grid(row=4, column=1)
pick_station_location_combo.set("TOP")  # Default value

ttk.Label(frame, text="Charge Station Amount:").grid(row=5, column=0, sticky=tk.W)
charge_station_amount_entry = ttk.Entry(frame)
charge_station_amount_entry.grid(row=5, column=1)
charge_station_amount_entry.insert(0, "1")  # Default value

ttk.Label(frame, text="Charge Station Location:").grid(row=6, column=0, sticky=tk.W)
charge_station_location_combo = ttk.Combobox(frame, values=["TOP", "BOTTOM", "LEFT", "RIGHT"])
charge_station_location_combo.grid(row=6, column=1)
charge_station_location_combo.set("BOTTOM")

ttk.Label(frame, text="Cycle Runtime (sec):").grid(row=7, column=0, sticky=tk.W)
cycle_runtime_entry = ttk.Entry(frame)
cycle_runtime_entry.grid(row=7, column=1)
cycle_runtime_entry.insert(0, "900")

ttk.Label(frame, text="Cycle Amount:").grid(row=8, column=0, sticky=tk.W)
cycle_amount_entry = ttk.Entry(frame)
cycle_amount_entry.grid(row=8, column=1)
cycle_amount_entry.insert(0, "1")

ttk.Label(frame, text="").grid(row=9, column=0, columnspan=2)
ttk.Label(frame, text="Robot Settings", font=('Helvetica', 12, 'bold')).grid(row=10, column=0, columnspan=2, sticky=tk.W)

ttk.Label(frame, text="Robot Amount:").grid(row=11, column=0, sticky=tk.W)
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=11, column=1)
robot_amount_entry.insert(0, "2")

'''
ttk.Label(frame, text="Loaded Speed (m/s):").grid(row=12, column=0, sticky=tk.W)
loaded_speed_entry = ttk.Entry(frame)
loaded_speed_entry.grid(row=12, column=1)
loaded_speed_entry.insert(0, "1")

ttk.Label(frame, text="Empty Speed (m/s):").grid(row=13, column=0, sticky=tk.W)
empty_speed_entry = ttk.Entry(frame)
empty_speed_entry.grid(row=13, column=1)
empty_speed_entry.insert(0, "2")

ttk.Label(frame, text="Take Time (sec):").grid(row=14, column=0, sticky=tk.W) #TO BE UPDATED
take_time_entry = ttk.Entry(frame)
take_time_entry.grid(row=14, column=1)
take_time_entry.insert(0, "3")

ttk.Label(frame, text="Drop Time (sec):").grid(row=15, column=0, sticky=tk.W) #TO BE UPDATED
drop_time_entry = ttk.Entry(frame)
drop_time_entry.grid(row=15, column=1)
drop_time_entry.insert(0, "3")

ttk.Label(frame, text="Loaded Battery Consumption (Ah):").grid(row=16, column=0, sticky=tk.W)
loaded_battery_consumption_entry = ttk.Entry(frame)
loaded_battery_consumption_entry.grid(row=16, column=1)
loaded_battery_consumption_entry.insert(0, "14.53265")

ttk.Label(frame, text="Empty Battery Consumption (Ah):").grid(row=17, column=0, sticky=tk.W)
empty_battery_consumption_entry = ttk.Entry(frame)
empty_battery_consumption_entry.grid(row=17, column=1)
empty_battery_consumption_entry.insert(0, "11.9566")'''

ttk.Label(frame, text="Charging Rate (Ah):").grid(row=18, column=0, sticky=tk.W)
charging_rate_entry = ttk.Entry(frame)
charging_rate_entry.grid(row=18, column=1)
charging_rate_entry.insert(0, "41.6")

ttk.Label(frame, text="Maximum Battery (Ah):").grid(row=19, column=0, sticky=tk.W)
maximum_battery_entry = ttk.Entry(frame)
maximum_battery_entry.grid(row=19, column=1)
maximum_battery_entry.insert(0, "41.6")

ttk.Label(frame, text="").grid(row=20, column=0, columnspan=2)
ttk.Label(frame, text="Charge Policy Settings", font=('Helvetica', 12, 'bold')).grid(row=21, column=0, columnspan=2, sticky=tk.W)

ttk.Label(frame, text="Pearl Rate:").grid(row=22, column=0, sticky=tk.W)
pearl_rate_entry = ttk.Entry(frame)
pearl_rate_entry.grid(row=22, column=1)
pearl_rate_entry.insert(0, "0.4")

ttk.Label(frame, text="Rest Rate:").grid(row=23, column=0, sticky=tk.W) #TO BE UPDATED
rest_rate_entry = ttk.Entry(frame)
rest_rate_entry.grid(row=23, column=1)
rest_rate_entry.insert(0, "0.1")

ttk.Label(frame, text="Charge Flag Rate:").grid(row=24, column=0, sticky=tk.W) #TO BE UPDATED
charge_flag_rate_entry = ttk.Entry(frame)
charge_flag_rate_entry.grid(row=24, column=1)
charge_flag_rate_entry.insert(0, "0.8")

ttk.Label(frame, text="Max Charge Rate:").grid(row=25, column=0, sticky=tk.W) #TO BE UPDATED
max_charge_rate_entry = ttk.Entry(frame)
max_charge_rate_entry.grid(row=25, column=1)
max_charge_rate_entry.insert(0, "0.85")

ttk.Label(frame, text="").grid(row=26, column=0, columnspan=2)
ttk.Label(frame, text="Taguchi Experiment Frame", font=('Helvetica', 12, 'bold')).grid(row=27, column=0, columnspan=2, sticky=tk.W)

# ttk.Checkbutton(frame, text="Enable Taguchi Experiment", variable=do_taguchi).grid(row=28, column=0, sticky=tk.W)

ttk.Checkbutton(frame, text="Pick Station Amount", variable=pick_station_amount_taguchi).grid(row=29, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Pick Station Location", variable=pick_station_location_taguchi).grid(row=30, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Charge Station Amount", variable=charge_station_amount_taguchi).grid(row=31, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Charge Station Location", variable=charge_station_location_taguchi).grid(row=32, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Robot Amount", variable=robot_amount_taguchi).grid(row=33, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Charge Flag Rate", variable=charge_flag_rate_taguchi).grid(row=34, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Max Charge Rate", variable=max_charge_rate_taguchi).grid(row=35, column=0, sticky=tk.W)

# Simulation button
run_button = ttk.Button(frame, text="Run Simulation", command=run_simulation)
run_button.grid(row=36, column=0, columnspan=2)

# Result label
result_label = ttk.Label(frame, text="")
result_label.grid(row=37, column=0, columnspan=2)

app.mainloop()