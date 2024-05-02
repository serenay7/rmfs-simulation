import tkinter as tk
from tkinter import ttk

def run_simulation():
    # Example: num_robots = int(robot_number_entry.get())
    # Insert your simulation logic here
    result_label.config(text="Simulation started...")  # Update this based on your simulation output

def open_settings():
    # Placeholder function for settings
    print("Settings opened")

def open_taguchi():
    # Placeholder function for Taguchi method settings
    print("Taguchi settings opened")

app = tk.Tk()
app.title("RMFS Simulation Interface")

# Create a frame for better organization
frame = ttk.Frame(app, padding="10")
frame.pack(fill=tk.BOTH, expand=True)

# Menu Bar
menu_bar = tk.Menu(app)
app.config(menu=menu_bar)

# Settings Menu
settings_menu = tk.Menu(menu_bar, tearoff=0)
menu_bar.add_cascade(label="Settings", menu=settings_menu)
settings_menu.add_command(label="Open Settings", command=open_settings)

# Taguchi Menu
taguchi_menu = tk.Menu(menu_bar, tearoff=0)
menu_bar.add_cascade(label="Taguchi", menu=taguchi_menu)
taguchi_menu.add_command(label="Open Taguchi Settings", command=open_taguchi)

# Adding widgets for integer input
ttk.Label(frame, text="Robot Amount:").grid(row=0, column=0, sticky=tk.W)
robot_amount_entry = ttk.Entry(frame)
robot_amount_entry.grid(row=0, column=1)
robot_amount_entry.insert(0, "5")  # Default value

# Simulation button
run_button = ttk.Button(frame, text="Run Simulation", command=run_simulation)
run_button.grid(row=1, column=0, columnspan=2)

# Result label
result_label = ttk.Label(frame, text="")
result_label.grid(row=2, column=0, columnspan=2)

app.mainloop()