import numpy as np
import pandas as pd
import simpy
from Entities import Robot, Pod, InputStation, OutputStation, ExtractTask, StorageTask, SKU, ChargingStation
import layout
import random
import ast
from lp_podselection import podAndStation_combination, calculate_total_distances_for_all_requirements, min_max_diff, \
    check_feasibility, columnMultiplication, assign_pods_to_stations

# from vrp import create_data_model

if __name__ == "__main__":
    """
    env = simpy.Environment()

    rows = 10  # 3x3
    columns = 16

    rectangular_network, pos = layout.create_rectangular_network_with_attributes(columns, rows)
    layout.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))

    nodes = list(rectangular_network.nodes)

    network_corridors = layout.create_corridor_subgraph(rectangular_network)

    layout.draw_network_with_shelves(rectangular_network, pos)

    station1 = OutputStation(env, (15, 1), pickItemList=None)

    pod1 = Pod(env, (1, 1))

    chargingStation = ChargingStation(env=env, capacity=1, location=(0,9))



    robot1 = Robot(env, network_corridors, rectangular_network, 1, pod=None, currentNode=(0, 0),
                   targetNode=None, currentTask=None, taskList=None, loadedSpeed=1, emptySpeed=2, takeTime=3,
                   dropTime=3, batteryLevel=9, moveLoaded=14.53265, moveEmpty=11.9566, chargingRate=41.6,
                   chargeThreshold=35)

    sampleTask = ExtractTask(env, robot=robot1, outputstation=station1, pod=pod1)

    # robot1.DoExtractTask(sampleTask)
    env.process(robot1.DoExtractTask(sampleTask))
    env.run()
    """
    import simpy
    import random


    # Define the function for the customer generator
    def customer(env, idx, servers, service_time):
        arrival_time = env.now
        print(f"Customer {idx} arrived at time {arrival_time}")

        with servers.request() as request:
            yield request
            server_id = server_dict[request]

            service_start_time = env.now
            print(f"Customer {idx} started service at time {service_start_time} using Server {server_id}")
            yield env.timeout(service_time())
            service_end_time = env.now
            print(
                f"Customer {idx} finished service at time {service_end_time} using Server {server_id}. Total time in system: {service_end_time - arrival_time}")


    # Define the function for generating service time
    def service_time():
        return random.expovariate(1.0 / SERVICE_RATE)


    # Define the simulation environment
    SIMULATION_TIME = 100
    SERVICE_RATE = 0.2
    INTERARRIVAL_TIME = 0.1
    NUM_SERVERS = 2

    env = simpy.Environment()

    # Start the servers
    servers = simpy.Resource(env, capacity=NUM_SERVERS)
    server_dict = {}

    # Start generating customers
    for i in range(NUM_SERVERS):
        server_dict[servers.request()] = i + 1


    # Define the function for the customer generator
    def customer(env, idx, servers, service_time):
        arrival_time = env.now
        print(f"Customer {idx} arrived at time {arrival_time}")

        with servers.request() as request:
            yield request
            server_id = server_dict[request]

            service_start_time = env.now
            print(f"Customer {idx} started service at time {service_start_time} using Server {server_id}")
            yield env.timeout(service_time())
            service_end_time = env.now
            print(
                f"Customer {idx} finished service at time {service_end_time} using Server {server_id}. Total time in system: {service_end_time - arrival_time}")


    # Start generating customers
    for i in range(5):
        env.process(customer(env, i, servers, service_time))

    # Start the simulation
    env.run()
