import numpy
import math
import pandas

def station_location(pick_station_amount, pick_station_location, charge_station_amount, charge_station_location, horizontal_ailes, vertical_ailes, columns, rows):
    pick_locations = []
    charge_locations = []

    if pick_station_location == charge_station_location: # All stations the same row

        total_stations = pick_station_amount + charge_station_amount
        topdown_interval = math.floor(columns/(total_stations+1))
        leftright_interval = math.floor(rows/(total_stations+1))
        

        if pick_station_location == "TOP" or pick_station_location == "BOTTOM":
            
            if total_stations > vertical_ailes:
                print("Number of stations to be placed on TOP/DOWN cannot exceed the number of vertical ailes.")

            else:

                if pick_station_location == "TOP":
                    for i in range(1, pick_station_amount + 1):
                        pick_locations.append((topdown_interval * i, rows - 1))

                    for i in range(pick_station_amount + 1, pick_station_amount + 1 + charge_station_amount):
                        charge_locations.append((topdown_interval * i, rows - 1))

                elif pick_station_location == "BOTTOM":
                    for i in range(1, pick_station_amount + 1):
                        pick_locations.append((topdown_interval * i, 0))

                    for i in range(pick_station_amount + 1, pick_station_amount + 1 + charge_station_amount):
                        charge_locations.append((topdown_interval * i, 0))
        
        elif pick_station_location == "LEFT" or pick_station_location == "RIGHT":

            if total_stations > horizontal_ailes:
                print("Number of stations to be placed on LEFT/RIGHT cannot exceed the number of horizontal ailes.")
            
            else:
                if pick_station_location == "LEFT":
                    for i in range(1, pick_station_amount + 1):
                        pick_locations.append((0, leftright_interval * i))

                    for i in range(pick_station_amount + 1, pick_station_amount + 1 + charge_station_amount):
                        charge_locations.append((0, leftright_interval * i))

                elif pick_station_location == "RIGHT":
                    for i in range(1, pick_station_amount + 1):
                        pick_locations.append((columns-1, leftright_interval * i))

                    for i in range(pick_station_amount + 1, pick_station_amount + 1 + charge_station_amount):
                        charge_locations.append((columns-1, leftright_interval * i))

    else: # Different stations, different sides

        if pick_station_location == "TOP":
            pick_interval = math.floor(columns/(pick_station_amount+1))

            if pick_station_amount > vertical_ailes:
                print("Number of pick stations to be placed on TOP/DOWN cannot exceed the number of vertical ailes.")
            else:
                for i in range(1, pick_station_amount + 1):
                    pick_locations.append((pick_interval * i, rows - 1))


        elif pick_station_location == "BOTTOM":
            pick_interval = math.floor(columns/(pick_station_amount+1))

            if pick_station_amount > vertical_ailes:
                print("Number of pick stations to be placed on TOP/DOWN cannot exceed the number of vertical ailes.")
            else:
                for i in range(1, pick_station_amount + 1):
                    pick_locations.append((pick_interval * i, 0))
        
        elif pick_station_location == "LEFT":
            pick_interval = math.floor(rows/(pick_station_amount+1))

            if pick_station_amount > horizontal_ailes:
                print("Number of pick stations to be placed on RIGHT/LEFT cannot exceed the number of horizontal ailes.")
            else:
                for i in range(1, pick_station_amount + 1):
                    pick_locations.append((0, pick_interval * i))

        elif pick_station_location == "RIGHT":
            pick_interval = math.floor(rows/(pick_station_amount+1))

            if pick_station_amount > horizontal_ailes:
                print("Number of pick stations to be placed on RIGHT/LEFT cannot exceed the number of horizontal ailes.")
            else:
                for i in range(1, pick_station_amount + 1):
                    pick_locations.append((columns-1, pick_interval * i))
        

        if charge_station_location == "TOP":
            charge_interval = math.floor(columns/(charge_station_amount+1))

            if charge_station_amount > vertical_ailes:
                print("Number of charge stations to be placed on TOP/DOWN cannot exceed the number of vertical ailes.")
            else:
                for i in range(1, charge_station_amount + 1):
                    charge_locations.append((charge_interval * i, rows - 1))
        
        elif charge_station_location == "BOTTOM":
            charge_interval = math.floor(columns/(charge_station_amount+1))

            if charge_station_amount > vertical_ailes:
                print("Number of charge stations to be placed on TOP/DOWN cannot exceed the number of vertical ailes.")
            else:
                for i in range(1, charge_station_amount + 1):
                    charge_locations.append((charge_interval * i, 0))
        
        elif charge_station_location == "LEFT":
            charge_interval = math.floor(rows/(charge_station_amount+1))

            if charge_station_amount > horizontal_ailes:
                print("Number of charge stations to be placed on RIGHT/LEFT cannot exceed the number of horizontal ailes.")
            else:
                for i in range(1, charge_station_amount + 1):
                    charge_locations.append((0, charge_interval * i))

        elif charge_station_location == "RIGHT":
            charge_interval = math.floor(rows/(charge_station_amount+1))

            if charge_station_amount > horizontal_ailes:
                print("Number of charge stations to be placed on RIGHT/LEFT cannot exceed the number of horizontal ailes.")
            else:
                for i in range(1, charge_station_amount + 1):
                    charge_locations.append((columns-1, charge_interval * i))

    return pick_locations, charge_locations

def robot_location(pick_station_location, charge_station_location, columns, rows, robot_amount):
    arr_name = ["TOP", "BOTTOM", "LEFT", "RIGHT"]
    robot_locations = []

    if columns > rows:
        arr = [4, 3, 2, 1]
    elif columns == rows:
        arr = [4, 3, 2, 1]
    elif columns < rows:
        arr = [2, 1, 4, 3]


    if pick_station_location == "TOP":
        arr[0] = 0
    
    elif pick_station_location == "BOTTOM":
        arr[1] = 0
    
    elif pick_station_location == "LEFT":
        arr[2] = 0
    
    elif pick_station_location == "RIGHT":
        arr[3] = 0


    if charge_station_location == "TOP":
        arr[0] = 0
    
    elif charge_station_location == "BOTTOM":
        arr[1] = 0
    
    elif charge_station_location == "LEFT":
        arr[2] = 0
    
    elif charge_station_location == "RIGHT":
        arr[3] = 0

    max_value = max(arr)
    max_index = arr.index(max_value)
    robot_initial_location = arr_name[max_index]


    if robot_initial_location == "TOP":
        for i in range(1, robot_amount + 1):
            robot_locations.append((i, rows - 1))
    
    elif robot_initial_location == "BOTTOM":
        for i in range(1, robot_amount + 1):
            robot_locations.append((i, 0))
    
    elif robot_initial_location == "LEFT":
        for i in range(1, robot_amount + 1):
            robot_locations.append((0, i))

    elif robot_initial_location == "RIGHT":
        for i in range(1, robot_amount + 1):
            robot_locations.append((columns-1, i))

    return robot_locations