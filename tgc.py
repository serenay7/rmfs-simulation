import tkinter as tk
from tkinter import ttk

app = tk.Tk()
frame = ttk.Frame(app)
frame.pack()

# Variables to hold the state of checkbuttons
do_taguchi = tk.BooleanVar()
pick_station_amount_taguchi = tk.BooleanVar()
pick_station_location_taguchi = tk.BooleanVar()
charge_station_amount_taguchi = tk.BooleanVar()
charge_station_location_taguchi = tk.BooleanVar()
robot_amount_taguchi = tk.BooleanVar()
charge_flag_rate_taguchi = tk.BooleanVar()
max_charge_rate_taguchi = tk.BooleanVar()

# Function to enable or disable other checkbuttons based on do_taguchi's state
def toggle_taguchi_options():
    if do_taguchi.get():
        state = 'normal'
    else:
        state = 'disabled'
    # Set the state of other checkbuttons
    pick_station_amount_checkbutton.config(state=state)
    pick_station_location_checkbutton.config(state=state)
    charge_station_amount_checkbutton.config(state=state)
    charge_station_location_checkbutton.config(state=state)
    robot_amount_checkbutton.config(state=state)
    charge_flag_rate_checkbutton.config(state=state)
    max_charge_rate_checkbutton.config(state=state)

# Checkbutton to control the experiment
taguchi_checkbutton = ttk.Checkbutton(frame, text="Enable Taguchi Experiment", variable=do_taguchi, command=toggle_taguchi_options)
taguchi_checkbutton.grid(row=0, column=0, sticky=tk.W)

# Other checkbuttons
pick_station_amount_checkbutton = ttk.Checkbutton(frame, text="Pick Station Amount", variable=pick_station_amount_taguchi)
pick_station_amount_checkbutton.grid(row=1, column=0, sticky=tk.W)

pick_station_location_checkbutton = ttk.Checkbutton(frame, text="Pick Station Location", variable=pick_station_location_taguchi)
pick_station_location_checkbutton.grid(row=2, column=0, sticky=tk.W)

charge_station_amount_checkbutton = ttk.Checkbutton(frame, text="Charge Station Amount", variable=charge_station_amount_taguchi)
charge_station_amount_checkbutton.grid(row=3, column=0, sticky=tk.W)

charge_station_location_checkbutton = ttk.Checkbutton(frame, text="Charge Station Location", variable=charge_station_location_taguchi)
charge_station_location_checkbutton.grid(row=4, column=0, sticky=tk.W)

robot_amount_checkbutton = ttk.Checkbutton(frame, text="Robot Amount", variable=robot_amount_taguchi)
robot_amount_checkbutton.grid(row=5, column=0, sticky=tk.W)

charge_flag_rate_checkbutton = ttk.Checkbutton(frame, text="Charge Flag Rate", variable=charge_flag_rate_taguchi)
charge_flag_rate_checkbutton.grid(row=6, column=0, sticky=tk.W)

max_charge_rate_checkbutton = ttk.Checkbutton(frame, text="Max Charge Rate", variable=max_charge_rate_taguchi)
max_charge_rate_checkbutton.grid(row=7, column=0, sticky=tk.W)

app.mainloop()