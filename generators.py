import numpy as np
import pandas as pd
import random
import koridor_deneme

def create_network(vertical, horizontal):
#Vertical ve Horizontal  dimensionları verdiğimizde rows ve column hesaplayarak rectangular_network ve network_corridors'u output olarak verir
    rows = (vertical*3) + 1
    columns = (horizontal*5) + 1

    rectangular_network, pos = koridor_deneme.create_rectangular_network_with_attributes(columns, rows)
    koridor_deneme.place_shelves_automatically(rectangular_network, shelf_dimensions=(4, 2), spacing=(1, 1))
    #koridor_deneme.draw_network_with_shelves(rectangular_network, pos)
    network_corridors = koridor_deneme.create_corridor_subgraph(rectangular_network)
    return rectangular_network, network_corridors

def taskGenerator(network, numTask, numRobot):
    """Generates 2d np array for tasks. First column location of the pod (tuple), second column assigned robot (int)"""
    nodesDict = network._node  # kendi node değişkeninden farklı, pod olup olmadığının bilgisini de dict olarak veriyor
    shelvesNetworkNodes = {k: v for k, v in nodesDict.items() if v == {'shelf': True}}
    randomPodsList = random.sample(list(shelvesNetworkNodes.keys()), numTask)
    tasks = []
    for idx, task in enumerate(randomPodsList):
        tasks.append([task, idx % numRobot])
    tasksArr = np.array(tasks, dtype="object")
    return tasksArr


def orderGenerator(stationCapacity, numStation, numSKU, maxAmount=1, skuExistencethreshold=0.7):
    orders = np.zeros(shape=(stationCapacity*numStation, numSKU))
    # boş order yollayabilir DİKKAT
    for i in range(stationCapacity * numStation):
        for j in range(numSKU):
            if random.random() > skuExistencethreshold:
                orders[i, j] = np.random.randint(1, maxAmount + 1)

    return orders

if __name__ == "__main__":
    rectangular_network, network_corridors = create_network(3, 3)
    tasks = taskGenerator(rectangular_network, 9, 3)
    orders = orderGenerator(10,2,10,5)
    sums = np.sum(orders, axis=1)
    a = 10