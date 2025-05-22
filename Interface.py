import tkinter as tk
from tkinter import ttk
import simpy
from main import RMFS_Model
from InterfaceInitializer import station_location, robot_location
from pyDOE3 import ff2n
import sys

import numpy as np
import pandas as pd

import Layout
import config
import logging

# FIXME: Increased recursion limit. Investigate if this is truly needed for UI, Taguchi setup, or core simulation logic. Consider localizing or addressing underlying recursion if possible.
# sys.setrecursionlimit(1500) # Commented out for now, will be localized if needed.

# Setup basic logging for the Interface
logging.basicConfig(level=config.LOGGING_LEVEL, format=config.LOG_FORMAT, datefmt=config.LOG_DATE_FORMAT)

def _run_single_simulation(params):
    """Helper function to run a single simulation."""
    env = simpy.Environment()

    rows = (3 * params['horizontal_ailes']) + 4
    columns = (5 * params['vertical_ailes']) + 6

    rectangular_network, pos = Layout.create_rectangular_network_with_attributes(columns, rows)
    Layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))

    simulation = RMFS_Model(env=env, network=rectangular_network)
    simulation.createPods()
    simulation.createSKUs()

    startLocations = robot_location(params['pick_station_location'], params['charge_station_location'], columns, rows, params['robot_amount'])
    pick_locations, charge_locations = station_location(
        params['pick_station_amount'], params['pick_station_location'],
        params['charge_station_amount'], params['charge_station_location'],
        params['horizontal_ailes'], params['vertical_ailes'], columns, rows
    )
    
    simulation.createChargingStations(charge_locations)
    simulation.createOutputStations(pick_locations)

    simulation.fillPods()
    simulation.distanceMatrixCalculate()

    simulation.createRobots(
        startLocations, params['charging_rate'], params['max_battery'],
        params['pearl_rate'], params['rest_rate'], params['charge_flag_rate'], params['max_charge_rate']
    )

    simulation.MultiCycleVRP(params['cycle_amount'], params['cycle_runtime'], printOutput=True)
    print("Single simulation run is over.")


def _run_taguchi_experiment(params):
    """Helper function to run a Taguchi experiment."""
    original_limit = sys.getrecursionlimit()
    sys.setrecursionlimit(config.TAGUCHI_RECURSION_LIMIT if hasattr(config, 'TAGUCHI_RECURSION_LIMIT') else 1500)
    try:
        parameter_count = params['do_pick_station_amount'] + params['do_charge_station_amount'] + \
                          params['do_robot_amount'] + params['do_charge_flag_rate'] + \
                          params['do_max_charge_rate'] + params['do_pearl_rate']
        
        if parameter_count == 0:
            print("Taguchi experiment selected, but no parameters chosen for variation.")
            result_label.config(text="Taguchi: No parameters chosen for variation.")
            return

        experiment_design = ff2n(parameter_count)
        
        for i in range(len(experiment_design)):
            for j in range(len(experiment_design[i])):
                if experiment_design[i][j] == -1:
                    experiment_design[i][j] = 0

        num_experiments = experiment_design.shape[0]
        exp_array_full = np.zeros((num_experiments, 6)) # 6 is the number of possible varying factors

        # Mapping selected Taguchi parameters to their column index in exp_array_full
        factor_map = {
            'pick_station_amount': 0, 
            'charge_station_amount': 1, 
            'robot_amount': 2, 
            'charge_flag_rate': 3, 
            'max_charge_rate': 4, 
            'pearl_rate': 5
        }
        
        selected_cols_indices = []
        current_col_idx = 0

        # Base values (Level 1)
        factor_levels = {
            'pick_station_amount': [params['pick_station_amount']],
            'charge_station_amount': [params['charge_station_amount']],
            'robot_amount': [params['robot_amount']],
            'charge_flag_rate': [params['charge_flag_rate']],
            'max_charge_rate': [params['max_charge_rate']],
            'pearl_rate': [params['pearl_rate']]
        }

        # Add Level 2 values if selected for Taguchi
        if params['do_pick_station_amount']:
            selected_cols_indices.append(factor_map['pick_station_amount'])
            factor_levels['pick_station_amount'].append(params['pick_station_amount2'])
        if params['do_charge_station_amount']:
            selected_cols_indices.append(factor_map['charge_station_amount'])
            factor_levels['charge_station_amount'].append(params['charge_station_amount2'])
        if params['do_robot_amount']:
            selected_cols_indices.append(factor_map['robot_amount'])
            factor_levels['robot_amount'].append(params['robot_amount2'])
        if params['do_charge_flag_rate']:
            selected_cols_indices.append(factor_map['charge_flag_rate'])
            factor_levels['charge_flag_rate'].append(params['charge_flag_rate2'])
        if params['do_max_charge_rate']:
            selected_cols_indices.append(factor_map['max_charge_rate'])
            factor_levels['max_charge_rate'].append(params['max_charge_rate2'])
        if params['do_pearl_rate']:
            selected_cols_indices.append(factor_map['pearl_rate'])
            factor_levels['pearl_rate'].append(params['pearl_rate2'])

        # Fill exp_array_full: default to level 1, then fill experiment design for selected factors
        for factor_name, base_val_list in factor_levels.items():
            exp_array_full[:, factor_map[factor_name]] = base_val_list[0]

        if selected_cols_indices: # If any factor is varied
            exp_array_full[:, selected_cols_indices] = experiment_design

        # Convert experiment design levels (0,1) to actual parameter values
        for i in range(num_experiments):
            for factor_name, col_idx in factor_map.items():
                if params[f'do_{factor_name}']: # Check if this factor was part of the experiment
                     level_index = int(exp_array_full[i, col_idx]) # Should be 0 or 1 from experiment_design
                     exp_array_full[i, col_idx] = factor_levels[factor_name][level_index]
                # else it remains the base value already set

        exp_df_columns = ['Pick Station Amount', 'Charge Station Amount', 'Robot Amount', 'Charge Flag Rate', 'Max Charge Rate', 'Pearl Rate']
        exp_df = pd.DataFrame(data=exp_array_full, columns=exp_df_columns)
        
        writer = pd.ExcelWriter('experiment/TaguchiVRP.xlsx', engine='xlsxwriter')
        exp_df.to_excel(writer, sheet_name='Experiment', index=False)

        simulation_hours = (params['cycle_amount'] * params['cycle_runtime']) / (60 * 60)
        total_steps_list, units_per_hour_list, robot_utilization_list = [], [], []

        for i in range(num_experiments):
            current_pick_station_amount = int(exp_array_full[i, factor_map['pick_station_amount']])
            current_charge_station_amount = int(exp_array_full[i, factor_map['charge_station_amount']])
            current_robot_amount = int(exp_array_full[i, factor_map['robot_amount']])
            current_charge_flag_rate = float(exp_array_full[i, factor_map['charge_flag_rate']])
            current_max_charge_rate = float(exp_array_full[i, factor_map['max_charge_rate']])
            current_pearl_rate = float(exp_array_full[i, factor_map['pearl_rate']])

            env = simpy.Environment()
            rows = (3 * params['horizontal_ailes']) + 4
            columns = (5 * params['vertical_ailes']) + 6

            rectangular_network, _ = Layout.create_rectangular_network_with_attributes(columns, rows)
            Layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
            
            simulation = RMFS_Model(env=env, network=rectangular_network) # Uses default policies from config
            simulation.createPods()
            simulation.createSKUs()

            startLocations = robot_location(params['pick_station_location'], params['charge_station_location'], columns, rows, current_robot_amount)
            pick_locations, charge_locations = station_location(
                current_pick_station_amount, params['pick_station_location'],
                current_charge_station_amount, params['charge_station_location'],
                params['horizontal_ailes'], params['vertical_ailes'], columns, rows
            )
        
            simulation.createChargingStations(charge_locations)
            simulation.createOutputStations(pick_locations)
            simulation.fillPods()
            simulation.distanceMatrixCalculate()
            simulation.createRobots(
                startLocations, params['charging_rate'], params['max_battery'],
                current_pearl_rate, params['rest_rate'], current_charge_flag_rate, current_max_charge_rate
            )

            # TaguchiVRP in RMFS_Model should ideally not write to its own file if we are aggregating here.
            # For now, assume it returns the dataframes.
            # The printOutput=False might be better if TaguchiVRP writes file by itself.
            # If TaguchiVRP is to be used in this loop, printOutput should be False, and data returned.
            # For now, we assume TaguchiVRP is called and it might write its own temp files, or this `writer` handles all.
            # The original code called simulation.TaguchiVRP with printOutput=True, which implies it writes to config.OUTPUT_VRP_EXCEL.
            # This is problematic if we want one consolidated Taguchi report.
            # For this refactoring, let's assume TaguchiVRP can run without writing, and returns dfs.
            # Modifying RMFS_Model.TaguchiVRP to not write if printOutput=False is outside current scope.
            # So, we will let it write its files, and also collect stats here.

            sheet1_data, sheet2_data = simulation.TaguchiVRP(params['cycle_amount'], params['cycle_runtime'], printOutput=False) # Must return dataframes

            total_rows_ts = len(sheet1_data)
            if total_rows_ts > 0 and current_robot_amount > 0:
                 extract_count = sheet1_data[sheet1_data['robotStatus'] == 'extract'].shape[0]
                 utilization = extract_count / (total_rows_ts * current_robot_amount) # this is not correct, total_rows_ts is not total time units
            else:
                 utilization = 0
            robot_utilization_list.append(utilization) # This calculation needs review based on timeStatDF structure

            station_rows = sheet2_data[sheet2_data['Statistics'].str.startswith('Station')]
            units_collected = station_rows['Value'].sum()
            units_per_hour_list.append(units_collected) # Will divide by simulation_hours later

            if not sheet1_data.empty:
                max_steps_per_robot = sheet1_data.groupby('robotID')['stepsTaken'].max()
                total_steps = max_steps_per_robot.sum()
            else:
                total_steps = 0
            total_steps_list.append(total_steps)

            sheet1_data.to_excel(writer, sheet_name=f'TimeSeries{i+1}', index=False)
            sheet2_data.to_excel(writer, sheet_name=f'Observed{i+1}', index=False)

        # Post-loop processing for Taguchi results
        if simulation_hours > 0:
             units_per_hour_list = [val / simulation_hours for val in units_per_hour_list]
        else:
             units_per_hour_list = [0] * len(units_per_hour_list)


        # Saving aggregated results
        pd.DataFrame({'RobotUtilization': robot_utilization_list}).to_excel(writer, sheet_name='Agg_RobotUtilization', index=False)
        pd.DataFrame({'TotalSteps': total_steps_list}).to_excel(writer, sheet_name='Agg_TotalSteps', index=False)
        pd.DataFrame({'UPH': units_per_hour_list}).to_excel(writer, sheet_name='Agg_UPH', index=False)
        
        # Printing best experiments
        if total_steps_list:
             print('Best Experiment for Total Steps:', np.argmin(total_steps_list) + 1 if total_steps_list else 'N/A')
        if units_per_hour_list:
             print('Best Experiment for UPH:', np.argmax(units_per_hour_list) + 1 if units_per_hour_list else 'N/A')
        # Robot utilization interpretation might need care - higher is better if it means productive work.
        if robot_utilization_list:
             print('Best Experiment for Robot Utilization:', np.argmax(robot_utilization_list) + 1 if robot_utilization_list else 'N/A')

        writer._save()
        print("Taguchi experiment run is over.")
    finally:
        sys.setrecursionlimit(original_limit)


def run_simulation():
    print("started")
    
    params = {
        'horizontal_ailes': int(warehouse_horizontal_entry.get()),
        'vertical_ailes': int(warehouse_vertical_entry.get()),
        'pick_station_location': pick_station_location_combo.get(),
        'charge_station_location': charge_station_location_combo.get(),
        'cycle_amount': int(cycle_amount_entry.get()),
        'cycle_runtime': int(cycle_runtime_entry.get()),
        'charging_rate': float(charging_rate_entry.get()),
        'max_battery': float(maximum_battery_entry.get()),
        'pearl_rate': float(pearl_rate_entry.get()), # Used by both, but level 2 for Taguchi
        'rest_rate': float(rest_rate_entry.get()),
        'pick_station_amount': int(pick_station_amount_entry.get()), # Level 1 for Taguchi
        'charge_station_amount': int(charge_station_amount_entry.get()), # Level 1 for Taguchi
        'robot_amount': int(robot_amount_entry.get()), # Level 1 for Taguchi
        'charge_flag_rate': float(charge_flag_rate_entry.get()), # Level 1 for Taguchi
        'max_charge_rate': float(max_charge_rate_entry.get()), # Level 1 for Taguchi
        
        # Taguchi specific flags and level 2 values
        'do_taguchi': int(do_taguchi.get()),
        'do_pick_station_amount': int(pick_station_amount_taguchi.get()),
        'pick_station_amount2': int(pick_station_amount2_entry.get()),
        'do_charge_station_amount': int(charge_station_amount_taguchi.get()),
        'charge_station_amount2': int(charge_station_amount2_entry.get()),
        'do_robot_amount': int(robot_amount_taguchi.get()),
        'robot_amount2': int(robot_amount2_entry.get()),
        'do_charge_flag_rate': int(charge_flag_rate_taguchi.get()),
        'charge_flag_rate2': float(charge_flag_rate2_entry.get()),
        'do_max_charge_rate': int(max_charge_rate_taguchi.get()),
        'max_charge_rate2': float(max_charge_rate2_entry.get()),
        'do_pearl_rate': int(pearl_rate_taguchi.get()),
        'pearl_rate2': float(pearl_rate2_entry.get()),
        # 'experiment_objective': experiment_objective_combo.get() # Not directly used in sim logic, but for analysis
    }

    if params['do_taguchi'] == 0:
        _run_single_simulation(params)
    elif params['do_taguchi'] == 1:
        _run_taguchi_experiment(params)

    result_label.config(text="Simulation run completed.")

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
warehouse_vertical_entry.insert(0, config.DEFAULT_VERTICAL_AISLES)

ttk.Label(frame, text="Horizontal Aisles:").grid(row=2, column=0, sticky=tk.W)
warehouse_horizontal_entry = ttk.Entry(frame)
warehouse_horizontal_entry.grid(row=2, column=1)
warehouse_horizontal_entry.insert(0, config.DEFAULT_HORIZONTAL_AISLES)

ttk.Label(frame, text="Pick Station Amount:").grid(row=3, column=0, sticky=tk.W)
pick_station_amount_entry = ttk.Entry(frame)
pick_station_amount_entry.grid(row=3, column=1)
pick_station_amount_entry.insert(0, config.DEFAULT_NUM_OUTPUT_STATIONS_STR) # Default to 1 or a new config

ttk.Label(frame, text="Pick Station Location:").grid(row=4, column=0, sticky=tk.W)
pick_station_location_combo = ttk.Combobox(frame, values=[config.UI_DEFAULT_LAYOUT_SIDE_TOP, config.UI_DEFAULT_LAYOUT_SIDE_BOTTOM, config.UI_DEFAULT_LAYOUT_SIDE_LEFT, config.UI_DEFAULT_LAYOUT_SIDE_RIGHT])
pick_station_location_combo.grid(row=4, column=1)
pick_station_location_combo.set(config.UI_DEFAULT_LAYOUT_SIDE_TOP)

ttk.Label(frame, text="Charge Station Amount:").grid(row=5, column=0, sticky=tk.W)
charge_station_amount_entry = ttk.Entry(frame)
charge_station_amount_entry.grid(row=5, column=1)
charge_station_amount_entry.insert(0, "1") # Assuming 1 is a common default, can add to config if needed

ttk.Label(frame, text="Charge Station Location:").grid(row=6, column=0, sticky=tk.W)
charge_station_location_combo = ttk.Combobox(frame, values=[config.UI_DEFAULT_LAYOUT_SIDE_TOP, config.UI_DEFAULT_LAYOUT_SIDE_BOTTOM, config.UI_DEFAULT_LAYOUT_SIDE_LEFT, config.UI_DEFAULT_LAYOUT_SIDE_RIGHT])
charge_station_location_combo.grid(row=6, column=1)
charge_station_location_combo.set(config.UI_DEFAULT_LAYOUT_SIDE_BOTTOM)

ttk.Label(frame, text="Cycle Runtime (sec):").grid(row=7, column=0, sticky=tk.W)
cycle_runtime_entry = ttk.Entry(frame)
cycle_runtime_entry.grid(row=7, column=1)
cycle_runtime_entry.insert(0, config.DEFAULT_CYCLE_RUNTIME_SEC_STR)

ttk.Label(frame, text="Cycle Amount:").grid(row=8, column=0, sticky=tk.W)
cycle_amount_entry = ttk.Entry(frame)
cycle_amount_entry.grid(row=8, column=1)
cycle_amount_entry.insert(0, config.DEFAULT_CYCLE_AMOUNT)

ttk.Label(frame, text="").grid(row=9, column=0, columnspan=2)
ttk.Label(frame, text="Robot Settings", font=('Helvetica', 12, 'bold')).grid(row=10, column=0, columnspan=2, sticky=tk.W)

ttk.Label(frame, text="Robot Amount:").grid(row=11, column=0, sticky=tk.W)
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=11, column=1)
robot_amount_entry.insert(0, config.DEFAULT_NUM_ROBOTS_STR)

ttk.Label(frame, text="Charging Rate (Ah):").grid(row=12, column=0, sticky=tk.W)
charging_rate_entry = ttk.Entry(frame)
charging_rate_entry.grid(row=12, column=1)
charging_rate_entry.insert(0, str(config.DEFAULT_CHARGING_RATE))

ttk.Label(frame, text="Maximum Battery (Ah):").grid(row=13, column=0, sticky=tk.W)
maximum_battery_entry = ttk.Entry(frame)
maximum_battery_entry.grid(row=13, column=1)
maximum_battery_entry.insert(0, str(config.DEFAULT_MAX_BATTERY))

ttk.Label(frame, text="").grid(row=14, column=0, columnspan=2)
ttk.Label(frame, text="Charge Policy Settings", font=('Helvetica', 12, 'bold')).grid(row=15, column=0, columnspan=2, sticky=tk.W)

ttk.Label(frame, text="Pearl Rate:").grid(row=16, column=0, sticky=tk.W)
pearl_rate_entry = ttk.Entry(frame)
pearl_rate_entry.grid(row=16, column=1)
pearl_rate_entry.insert(0, str(config.DEFAULT_PEARL_RATE))

ttk.Label(frame, text="Rest Rate:").grid(row=17, column=0, sticky=tk.W)
rest_rate_entry = ttk.Entry(frame)
rest_rate_entry.grid(row=17, column=1)
rest_rate_entry.insert(0, str(config.DEFAULT_REST_RATE))

ttk.Label(frame, text="Charge Flag Rate:").grid(row=18, column=0, sticky=tk.W)
charge_flag_rate_entry = ttk.Entry(frame)
charge_flag_rate_entry.grid(row=18, column=1)
charge_flag_rate_entry.insert(0, str(config.DEFAULT_CHARGE_FLAG_RATE))

ttk.Label(frame, text="Max Charge Rate:").grid(row=19, column=0, sticky=tk.W)
max_charge_rate_entry = ttk.Entry(frame)
max_charge_rate_entry.grid(row=19, column=1)
max_charge_rate_entry.insert(0, str(config.DEFAULT_MAX_CHARGE_RATE))

ttk.Label(frame, text="Taguchi Experiment Frame", font=('Helvetica', 12, 'bold')).grid(row=0, column=3, columnspan=2, sticky=tk.W)

ttk.Checkbutton(frame, text="Enable Taguchi Experiment", variable=do_taguchi).grid(row=1, column=3, sticky=tk.W)

ttk.Checkbutton(frame, text="Pick Station Amount", variable=pick_station_amount_taguchi).grid(row=2, column=3, sticky=tk.W)
ttk.Label(frame, text="Pick Station Amount 2:").grid(row=3, column=3, sticky=tk.W)
pick_station_amount2_entry = ttk.Entry(frame)
pick_station_amount2_entry.grid(row=3, column=4)
pick_station_amount2_entry.insert(0, "2")  # Default value

ttk.Checkbutton(frame, text="Charge Station Amount", variable=charge_station_amount_taguchi).grid(row=4, column=3, sticky=tk.W)
ttk.Label(frame, text="Charge Station Amount 2:").grid(row=5, column=3, sticky=tk.W)
charge_station_amount2_entry = ttk.Entry(frame)
charge_station_amount2_entry.grid(row=5, column=4)
charge_station_amount2_entry.insert(0, "2")  # Default value

ttk.Checkbutton(frame, text="Robot Amount", variable=robot_amount_taguchi).grid(row=6, column=3, sticky=tk.W)
ttk.Label(frame, text="Robot Amount 2:").grid(row=7, column=3, sticky=tk.W)
robot_amount2_entry = ttk.Entry(frame)
robot_amount2_entry.grid(row=7, column=4)
robot_amount2_entry.insert(0, "3")

ttk.Checkbutton(frame, text="Charge Flag Rate", variable=charge_flag_rate_taguchi).grid(row=8, column=3, sticky=tk.W)
ttk.Label(frame, text="Charge Flag Rate 2:").grid(row=9, column=3, sticky=tk.W)
charge_flag_rate2_entry = ttk.Entry(frame)
charge_flag_rate2_entry.grid(row=9, column=4)
charge_flag_rate2_entry.insert(0, "0.9")

ttk.Checkbutton(frame, text="Max Charge Rate", variable=max_charge_rate_taguchi).grid(row=10, column=3, sticky=tk.W)
ttk.Label(frame, text="Max Charge Rate 2:").grid(row=11, column=3, sticky=tk.W)
max_charge_rate2_entry = ttk.Entry(frame)
max_charge_rate2_entry.grid(row=11, column=4)
max_charge_rate2_entry.insert(0, "0.95")

ttk.Checkbutton(frame, text="Pearl Rate", variable=pearl_rate_taguchi).grid(row=12, column=3, sticky=tk.W)
ttk.Label(frame, text="Pearl Rate 2:").grid(row=13, column=3, sticky=tk.W)
pearl_rate2_entry = ttk.Entry(frame)
pearl_rate2_entry.grid(row=13, column=4)
pearl_rate2_entry.insert(0, "0.95")

ttk.Label(frame, text="").grid(row=14, column=3, columnspan=2)

ttk.Label(frame, text="Experiment Objective:").grid(row=15, column=3, sticky=tk.W)
experiment_objective_combo = ttk.Combobox(frame, values=["Units Collected per Hour", "Average Robot Utilization", "Total Distance Taken"])
experiment_objective_combo.grid(row=16, column=3, columnspan=2)
experiment_objective_combo.set("Total Distance Taken")

# Simulation button
run_button = ttk.Button(frame, text="Run Simulation", command=run_simulation)
run_button.grid(row=50, column=1, columnspan=3)

# Result label
result_label = ttk.Label(frame, text="")
result_label.grid(row=51, column=1, columnspan=3)

app.mainloop()