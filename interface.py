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

# Adding widgets for each input
ttk.Label(frame, text="Pick Station Amount:").grid(row=0, column=0, sticky=tk.W)
pick_station_amount_entry = ttk.Entry(frame)
pick_station_amount_entry.grid(row=0, column=1)

ttk.Label(frame, text="Pick Station Location:").grid(row=1, column=0, sticky=tk.W)
pick_station_location_entry = ttk.Entry(frame)
pick_station_location_entry.grid(row=1, column=1)

'''
ttk.Label(frame, text="Replenish Station Amount:").grid(row=2, column=0, sticky=tk.W)
replenish_station_amount_entry = ttk.Entry(frame)
replenish_station_amount_entry.grid(row=2, column=1)

ttk.Label(frame, text="Replenish Station Location:").grid(row=3, column=0, sticky=tk.W)
replenish_station_location_entry = ttk.Entry(frame)
replenish_station_location_entry.grid(row=3, column=1)'''

ttk.Label(frame, text="Charge Station Amount:").grid(row=4, column=0, sticky=tk.W)
charge_station_amount_entry = ttk.Entry(frame)
charge_station_amount_entry.grid(row=4, column=1)

ttk.Label(frame, text="Charge Station Location:").grid(row=5, column=0, sticky=tk.W)
charge_station_location_entry = ttk.Entry(frame)
charge_station_location_entry.grid(row=5, column=1)

ttk.Label(frame, text="Simulation Runtime:").grid(row=6, column=0, sticky=tk.W)
runtime_entry = ttk.Entry(frame)
runtime_entry.grid(row=6, column=1)

ttk.Label(frame, text="Warehouse Size:").grid(row=7, column=0, sticky=tk.W) #TO BE UPDATED
warehouse_entry = ttk.Entry(frame)
warehouse_entry.grid(row=7, column=1)

ttk.Label(frame, text="Robot Amount:").grid(row=8, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=8, column=1)

ttk.Label(frame, text="Loaded Speed:").grid(row=9, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=9, column=1)

ttk.Label(frame, text="Empty Speed:").grid(row=10, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=10, column=1)

ttk.Label(frame, text="Take Time:").grid(row=11, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=11, column=1)

ttk.Label(frame, text="Drop Time:").grid(row=12, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=12, column=1)

ttk.Label(frame, text="Loaded Battery Consumption:").grid(row=13, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=13, column=1)

ttk.Label(frame, text="Empty Battery Consumption:").grid(row=14, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=14, column=1)

ttk.Label(frame, text="Charging Rate:").grid(row=15, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=15, column=1)

ttk.Label(frame, text="Maximum Battery:").grid(row=16, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=16, column=1)

ttk.Label(frame, text="Pearl Rate:").grid(row=17, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=17, column=1)

ttk.Label(frame, text="Rest Rate:").grid(row=18, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=18, column=1)

ttk.Label(frame, text="Charge Flag Rate:").grid(row=19, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=19, column=1)

ttk.Label(frame, text="Max Charge Rate:").grid(row=20, column=0, sticky=tk.W) #TO BE UPDATED
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=20, column=1)
# Continue adding more widgets for each input similarly...

# Simulation button
run_button = ttk.Button(frame, text="Run Simulation", command=run_simulation)
run_button.grid(row=20, column=0, columnspan=2)

# Result label
result_label = ttk.Label(frame, text="")
result_label.grid(row=21, column=0, columnspan=2)

app.mainloop()