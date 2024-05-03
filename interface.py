import tkinter as tk
from tkinter import ttk

def run_simulation():
    # Collect all input values
    # Example: num_robots = int(robot_number_entry.get())
    # Insert your simulation logic here
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

ttk.Label(frame, text="Vertical Aisles:").grid(row=1, column=0, sticky=tk.W) #TO BE UPDATED
warehouse_vertical_entry = ttk.Entry(frame)
warehouse_vertical_entry.grid(row=1, column=1)
warehouse_vertical_entry.insert(0, "10")  # Default value

ttk.Label(frame, text="Horizontal Aisles:").grid(row=2, column=0, sticky=tk.W) #TO BE UPDATED
warehouse_horizontal_entry = ttk.Entry(frame)
warehouse_horizontal_entry.grid(row=2, column=1)
warehouse_horizontal_entry.insert(0, "16")  # Default value

ttk.Label(frame, text="Pick Station Amount:").grid(row=3, column=0, sticky=tk.W)
pick_station_amount_entry = ttk.Entry(frame)
pick_station_amount_entry.grid(row=3, column=1)
pick_station_amount_entry.insert(0, "2")  # Default value

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
charge_station_location_combo.set("DOWN")

ttk.Label(frame, text="Simulation Runtime (sec):").grid(row=7, column=0, sticky=tk.W)
runtime_entry = ttk.Entry(frame)
runtime_entry.grid(row=7, column=1)
runtime_entry.insert(0, "1000")

ttk.Label(frame, text="Robot Settings", font=('Helvetica', 12, 'bold')).grid(row=8, column=0, columnspan=2, sticky=tk.W)

ttk.Label(frame, text="Robot Amount:").grid(row=9, column=0, sticky=tk.W)
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=9, column=1)
robot_amount_entry.insert(0, "2")

ttk.Label(frame, text="Loaded Speed (m/s):").grid(row=10, column=0, sticky=tk.W)
loaded_speed_entry = ttk.Entry(frame)
loaded_speed_entry.grid(row=10, column=1)
loaded_speed_entry.insert(0, "1")

ttk.Label(frame, text="Empty Speed (m/s):").grid(row=11, column=0, sticky=tk.W)
empty_speed_entry = ttk.Entry(frame)
empty_speed_entry.grid(row=11, column=1)
empty_speed_entry.insert(0, "2")

ttk.Label(frame, text="Take Time (sec):").grid(row=12, column=0, sticky=tk.W) #TO BE UPDATED
take_time_entry = ttk.Entry(frame)
take_time_entry.grid(row=12, column=1)
take_time_entry.insert(0, "3")

ttk.Label(frame, text="Drop Time (sec):").grid(row=13, column=0, sticky=tk.W) #TO BE UPDATED
drop_time_entry = ttk.Entry(frame)
drop_time_entry.grid(row=13, column=1)
drop_time_entry.insert(0, "3")

ttk.Label(frame, text="Loaded Battery Consumption (Ah):").grid(row=14, column=0, sticky=tk.W)
loaded_battery_consumption_entry = ttk.Entry(frame)
loaded_battery_consumption_entry.grid(row=14, column=1)
loaded_battery_consumption_entry.insert(0, "14.53265")

ttk.Label(frame, text="Empty Battery Consumption (Ah):").grid(row=15, column=0, sticky=tk.W)
empty_battery_consumption_entry = ttk.Entry(frame)
empty_battery_consumption_entry.grid(row=15, column=1)
empty_battery_consumption_entry.insert(0, "11.9566")

ttk.Label(frame, text="Charging Rate (Ah):").grid(row=16, column=0, sticky=tk.W)
charging_rate_entry = ttk.Entry(frame)
charging_rate_entry.grid(row=16, column=1)
charging_rate_entry.insert(0, "41.6")

ttk.Label(frame, text="Maximum Battery (Ah):").grid(row=17, column=0, sticky=tk.W)
maximum_battery_entry = ttk.Entry(frame)
maximum_battery_entry.grid(row=17, column=1)
maximum_battery_entry.insert(0, "41.6")

ttk.Label(frame, text="Charge Policy Settings", font=('Helvetica', 12, 'bold')).grid(row=18, column=0, columnspan=2, sticky=tk.W)

ttk.Label(frame, text="Pearl Rate:").grid(row=19, column=0, sticky=tk.W)
pearl_rate_entry = ttk.Entry(frame)
pearl_rate_entry.grid(row=19, column=1)
pearl_rate_entry.insert(0, "0.4")

ttk.Label(frame, text="Rest Rate:").grid(row=20, column=0, sticky=tk.W) #TO BE UPDATED
rest_rate_entry = ttk.Entry(frame)
rest_rate_entry.grid(row=20, column=1)
rest_rate_entry.insert(0, "0.1")

ttk.Label(frame, text="Charge Flag Rate:").grid(row=21, column=0, sticky=tk.W) #TO BE UPDATED
charge_flag_rate_entry = ttk.Entry(frame)
charge_flag_rate_entry.grid(row=21, column=1)
charge_flag_rate_entry.insert(0, "0.8")

ttk.Label(frame, text="Max Charge Rate:").grid(row=22, column=0, sticky=tk.W) #TO BE UPDATED
max_charge_rate_entry = ttk.Entry(frame)
max_charge_rate_entry.grid(row=22, column=1)
max_charge_rate_entry.insert(0, "0.85")

ttk.Label(frame, text="Taguchi Experiment Frame", font=('Helvetica', 12, 'bold')).grid(row=23, column=0, columnspan=2, sticky=tk.W)

ttk.Checkbutton(frame, text="Enable Taguchi Experiment", variable=do_taguchi).grid(row=24, column=0, sticky=tk.W)

ttk.Checkbutton(frame, text="Pick Station Amount", variable=pick_station_amount_taguchi).grid(row=25, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Pick Station Location", variable=pick_station_location_taguchi).grid(row=26, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Charge Station Amount", variable=charge_station_amount_taguchi).grid(row=27, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Charge Station Location", variable=charge_station_location_taguchi).grid(row=28, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Robot Amount", variable=robot_amount_taguchi).grid(row=29, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Charge Flag Rate", variable=charge_flag_rate_taguchi).grid(row=30, column=0, sticky=tk.W)
ttk.Checkbutton(frame, text="Max Charge Rate", variable=max_charge_rate_taguchi).grid(row=31, column=0, sticky=tk.W)

# Simulation button
run_button = ttk.Button(frame, text="Run Simulation", command=run_simulation)
run_button.grid(row=35, column=0, columnspan=2)

# Result label
result_label = ttk.Label(frame, text="")
result_label.grid(row=36, column=0, columnspan=2)

app.mainloop()