# RMFS Simulation

This is a Robotic Mobile Fulfillment System (RMFS) simulation written to try different strategies for assignment and routing problems in a robotic warehouse. The project is a part of Systems Design course of METU Industrial Engineering department.

The simulation is a **work in progress**.

**Language:** Python

**Libraries:** pandas, numpy, simpy, networkx, gurobi, google or tools, pydoe3, tkinter

**Inputs:**


1. Horizontal & Vertical Ailes: Number of inner ailes.

Shelves are created as a group of 2x4=8 shelves. When there are 2 horizontal and 2 vertical ailes, there are 4 corridors in each direction and a layout of 10x16 is created.

2. Station Amounts and Locations: TBD

Station amount of one type cannot exceed the number of ailes in that direction.
Total station to be placed on a side cannot exceed the number of ailes intersecting with that side.

3. Cycle Amount and Runtime: The tasks and routes are determined in cycles.

Cycle runtime is in seconds, so if a cycle runtime is determined as 900 seconds and the number of cycles is 4, total simulation time is an hour.
