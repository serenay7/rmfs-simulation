import tkinter as tk
from tkinter import ttk
import simpy
from emre_test import RMFS_Model
from interface_initializer import station_location, robot_location
from pyDOE3 import *
import sys

import numpy as np
import pandas as pd
from simpy.events import AllOf

from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask, SKU, ChargingStation
import layout

from lp_podselection import podAndStation_combination, calculate_total_distances_for_all_requirements, min_max_diff, check_feasibility, columnMultiplication, assign_pods_to_stations
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import networkx as nx

sys.setrecursionlimit(1500) 

def run_simulation():
    print("started")

    do_tg= int(do_taguchi.get())

    do_pick_station_amount = int(pick_station_amount_taguchi.get())
    do_charge_station_amount = int(charge_station_amount_taguchi.get())
    do_robot_amount = int(robot_amount_taguchi.get())
    do_charge_flag_rate = int(charge_flag_rate_taguchi.get())
    do_max_charge_rate = int(max_charge_rate_taguchi.get())
    do_pearl_rate = int(pearl_rate_taguchi.get())

    parameter_count = do_pick_station_amount + do_charge_station_amount + do_robot_amount + do_charge_flag_rate + do_max_charge_rate + do_pearl_rate

    # inputs
    horizontal_ailes = int(warehouse_horizontal_entry.get())
    vertical_ailes = int(warehouse_vertical_entry.get())
    pick_station_location = pick_station_location_combo.get()
    charge_station_location = charge_station_location_combo.get()
    cycle_amount = int(cycle_amount_entry.get())
    cycle_runtime = int(cycle_runtime_entry.get())
    charging_rate = float(charging_rate_entry.get())
    max_battery = float(maximum_battery_entry.get())
    pearl_rate = float(pearl_rate_entry.get())
    rest_rate = float(rest_rate_entry.get())
    pick_station_amount = int(pick_station_amount_entry.get()) #
    charge_station_amount = int(charge_station_amount_entry.get()) #
    robot_amount = int(robot_amount_entry.get()) #
    charge_flag_rate = float(charge_flag_rate_entry.get()) #
    max_charge_rate = float(max_charge_rate_entry.get()) #
    
    
    # no taguchi, single run
    if do_tg==0: 

        env = simpy.Environment()

        rows = (3*int(horizontal_ailes))+4 # 10
        columns = (5*int(vertical_ailes))+6 # 16

        rectangular_network, pos = layout.create_rectangular_network_with_attributes(columns, rows)
        layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
        # nodes = list(rectangular_network.nodes)
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
    
    elif do_tg==1:
        print("taguchi is working")

        experiment = ff2n(parameter_count)
        print(experiment)
        
        for i in range(len(experiment)):
            for j in range(len(experiment[i])):
                if experiment[i][j] == -1:
                    experiment[i][j] = 0

        no_of_experiments = experiment.shape[0]
        exp_array = np.zeros((no_of_experiments,6))

        print(exp_array)

        testdict = {'do_pick_station_amount':0, 'do_charge_station_amount':1, 'do_robot_amount':2, 'do_charge_flag_rate':3, 'do_max_charge_rate':4, 'do_pearl_rate':5}
        cols = []

        if do_pick_station_amount:
            cols.append(testdict["do_pick_station_amount"])
            pick_station_amount2 = int(pick_station_amount2_entry.get())
            pick_station_amounts = [pick_station_amount, pick_station_amount2]
        else:
            pick_station_amounts = [pick_station_amount]
            
        if do_charge_station_amount:
            cols.append(testdict["do_charge_station_amount"])
            charge_station_amount2 = int(charge_station_amount2_entry.get())
            charge_station_amounts = [charge_station_amount, charge_station_amount2]
        else: 
            charge_station_amounts = [charge_station_amount]

        if do_robot_amount:
            cols.append(testdict["do_robot_amount"])
            robot_amount2 = int(robot_amount2_entry.get())
            robot_amounts = [robot_amount, robot_amount2]
        else:
            robot_amounts = [robot_amount]

        if do_charge_flag_rate:
            cols.append(testdict["do_charge_flag_rate"])
            charge_flag_rate2 = float(charge_flag_rate2_entry.get())
            charge_flag_rates = [charge_flag_rate, charge_flag_rate2]
        else:
            charge_flag_rates = [charge_flag_rate]

        if do_max_charge_rate:
            cols.append(testdict["do_max_charge_rate"])
            max_charge_rate2 = float(max_charge_rate2_entry.get())
            max_charge_rates = [max_charge_rate, max_charge_rate2]
        else:
            max_charge_rates = [max_charge_rate]

        if do_pearl_rate:
            cols.append(testdict["do_pearl_rate"])
            pearl_rate2 = float(pearl_rate2_entry.get())
            pearl_rates = [pearl_rate, pearl_rate2]
        else:
            pearl_rates = [pearl_rate]
        
        exp_array[:, cols] = experiment
        print(exp_array)

        for i in range(len(experiment)):
            a = int(exp_array[i,0]) #pick station
            exp_array[i,0] = pick_station_amounts[a]
            
            b = int(exp_array[i,1]) #charge station
            exp_array[i,1] = charge_station_amounts[b]

            c = int(exp_array[i,2]) #robot amount
            exp_array[i,2] = robot_amounts[c]

            d = int(exp_array[i,3]) #charge flag rate
            exp_array[i,3] = charge_flag_rates[d]

            e = int(exp_array[i,4]) #max charge rate
            exp_array[i,4] = max_charge_rates[e]

            f = int(exp_array[i,5]) #max charge rate
            exp_array[i,5] = max_charge_rates[f]
            
            print(exp_array)

        exp_df = pd.DataFrame(data = exp_array,   
                  columns = ['Pick Station Amount', 'Charge Station Amount', 'Robot Amount', 'Charge Flag Rate', 'Max Charge Rate', 'Pearl Rate']) 
        
        writer = pd.ExcelWriter('TaguchiVRP.xlsx', engine='xlsxwriter')

        exp_df.to_excel(writer, sheet_name='Experiment', index=False)

        total_steps_list = []

        for i in range(len(experiment)):

            pick_station_amount = int(exp_array[i,0])
            charge_station_amount = int(exp_array[i,1])
            robot_amount = int(exp_array[i,2])
            charge_flag_rate = exp_array[i,3]
            max_charge_rate = exp_array[i,4]
            pearl_rate = exp_array[i,5]

            env = simpy.Environment()

            rows = (3*int(horizontal_ailes))+4 # 10
            columns = (5*int(vertical_ailes))+6 # 16

            rectangular_network, pos = layout.create_rectangular_network_with_attributes(columns, rows)
            layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
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

            sheet1, sheet2 = simulation.TaguchiVRP(cycle_amount,cycle_runtime, printOutput=True)

            print(sheet1)

            max_steps_per_robot = sheet1.groupby('robotID')['stepsTaken'].max()
            total_steps = max_steps_per_robot.sum()
            total_steps_list.append(total_steps)
            print(total_steps)

            sheet1.to_excel(writer, sheet_name=f'TimeSeries{i+1}', index=False)
            sheet2.to_excel(writer, sheet_name=f'Observed{i+1}', index=False)

        total_steps_df = pd.DataFrame({'TotalSteps': total_steps_list})
        total_steps_df.to_excel(writer, sheet_name='TotalSteps', index=False)

        min_index = total_steps_df['TotalSteps'].idxmin()
        print('Best Experiment:', min_index+1)

        writer._save()
        print("run is over")


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
pearl_rate_taguchi = tk.BooleanVar()

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

ttk.Checkbutton(frame, text="Enable Taguchi Experiment", variable=do_taguchi).grid(row=28, column=0, sticky=tk.W)

ttk.Checkbutton(frame, text="Pick Station Amount", variable=pick_station_amount_taguchi).grid(row=29, column=0, sticky=tk.W)
ttk.Label(frame, text="Pick Station Amount 2:").grid(row=30, column=0, sticky=tk.W)
pick_station_amount2_entry = ttk.Entry(frame)
pick_station_amount2_entry.grid(row=30, column=1)
pick_station_amount2_entry.insert(0, "2")  # Default value

# ttk.Checkbutton(frame, text="Pick Station Location", variable=pick_station_location_taguchi).grid(row=30, column=0, sticky=tk.W)

ttk.Checkbutton(frame, text="Charge Station Amount", variable=charge_station_amount_taguchi).grid(row=31, column=0, sticky=tk.W)
ttk.Label(frame, text="Charge Station Amount 2:").grid(row=32, column=0, sticky=tk.W)
charge_station_amount2_entry = ttk.Entry(frame)
charge_station_amount2_entry.grid(row=32, column=1)
charge_station_amount2_entry.insert(0, "2")  # Default value

# ttk.Checkbutton(frame, text="Charge Station Location", variable=charge_station_location_taguchi).grid(row=32, column=0, sticky=tk.W)

ttk.Checkbutton(frame, text="Robot Amount", variable=robot_amount_taguchi).grid(row=33, column=0, sticky=tk.W)
ttk.Label(frame, text="Robot Amount 2:").grid(row=34, column=0, sticky=tk.W)
robot_amount2_entry = ttk.Entry(frame)
robot_amount2_entry.grid(row=34, column=1)
robot_amount2_entry.insert(0, "3")

ttk.Checkbutton(frame, text="Charge Flag Rate", variable=charge_flag_rate_taguchi).grid(row=35, column=0, sticky=tk.W)
ttk.Label(frame, text="Charge Flag Rate 2:").grid(row=36, column=0, sticky=tk.W) #TO BE UPDATED
charge_flag_rate2_entry = ttk.Entry(frame)
charge_flag_rate2_entry.grid(row=36, column=1)
charge_flag_rate2_entry.insert(0, "0.9")

ttk.Checkbutton(frame, text="Max Charge Rate", variable=max_charge_rate_taguchi).grid(row=37, column=0, sticky=tk.W)
ttk.Label(frame, text="Max Charge Rate 2:").grid(row=38, column=0, sticky=tk.W) #TO BE UPDATED
max_charge_rate2_entry = ttk.Entry(frame)
max_charge_rate2_entry.grid(row=38, column=1)
max_charge_rate2_entry.insert(0, "0.95")

ttk.Checkbutton(frame, text="Pearl Rate", variable=pearl_rate_taguchi).grid(row=39, column=0, sticky=tk.W)
ttk.Label(frame, text="Pearl Rate 2:").grid(row=40, column=0, sticky=tk.W) #TO BE UPDATED
pearl_rate2_entry = ttk.Entry(frame)
pearl_rate2_entry.grid(row=40, column=1)
pearl_rate2_entry.insert(0, "0.95")

# Simulation button
run_button = ttk.Button(frame, text="Run Simulation", command=run_simulation)
run_button.grid(row=50, column=0, columnspan=2)

# Result label
result_label = ttk.Label(frame, text="")
result_label.grid(row=51, column=0, columnspan=2)

app.mainloop()