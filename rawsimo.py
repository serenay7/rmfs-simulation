import numpy as np
import pandas as pd


def manhattan_distance_between_consecutive_rows(arr):
    # Convert array of tuples to a NumPy array
    tuples_list = []

    # Iterate over the elements of the 1D array and extract the components of each tuple
    for element in arr:
        # Remove parentheses and split by comma to get individual components
        components = element.strip('()').split(',')
        # Convert components to integers and append to tuples_list
        tuples_list.append([int(component) for component in components])

    # Convert the list of tuples to a NumPy array
    arr = np.array(tuples_list)

    # Calculate Manhattan distances
    x_diff = np.abs(arr[:-1, 0] - arr[1:, 0])
    y_diff = np.abs(arr[:-1, 1] - arr[1:, 1])

    # Sum of Manhattan distances
    total_distance = np.sum(x_diff + y_diff)

    return total_distance

def rawsimoPodSelection(itemList, itemIndexDict, podMatrix):
    #bütün skular yeniden indexleniyor
    newItemList = []
    for itemInfo in itemList:
        newItem = [itemIndexDict[itemInfo[0]], itemInfo[1]]
        newItemList.append(newItem)

    max_hit = 0
    max_hit_pod = None
    for pod_idx, pod in enumerate(podMatrix):
        hit = 0
        for item in newItemList:
            if pod[item[0]] >= item[1]:
                hit += item[1]
        if hit > max_hit:
            max_hit = hit
            max_hit_pod = pod_idx
    return max_hit_pod


def rawsimoPodSelectionExperiment(itemList, podMatrix):
    #bizim generate ettiğimiz samplelar için kullanılacak
    max_hit = 0
    max_hit_pod = None
    satisfiedSKU = []
    for pod_idx, pod in enumerate(podMatrix):
        hit = 0
        satisfiedSKU_temp = []
        for item in itemList:
            if pod[item[0]] >= item[1]:
                hit += item[1]
                satisfiedSKU_temp.append(item[0])
        if hit > max_hit:
            max_hit = hit
            max_hit_pod = pod_idx
            satisfiedSKU = satisfiedSKU_temp.copy()
    return max_hit_pod, satisfiedSKU

def rawsimoTaskAssignment(taskList, numRobots):
    #buradaki taskList vrp'ye verilenden farklı olabilir istasyon bilgisi de lazım
    totalDistance = 0
    for robot_idx in range(numRobots):
        filtered_arr = taskList[taskList[:, 1] == robot_idx]
        # Drop the second column
        filtered_arr = np.delete(filtered_arr, 1, axis=1)
        totalDistance += manhattan_distance_between_consecutive_rows(filtered_arr)
    return totalDistance


def rawsimoOrderPodSelection(orders, numStation, podMatrix):
    selectedPods = []

    orderPerStation = len(orders)//numStation

    ordersDivided = orders.reshape((numStation,orderPerStation,orders.shape[1]))

    for order in ordersDivided:
        temp_pod_matrix = podMatrix.copy()

        #while order.shape[1] > 0:
        while np.sum(order) > 0:
            selectedPod, fulfilledSKU = rawsimoPodSelectionExperiment(orderMatrix=order, podMatrix=temp_pod_matrix)
            order = np.delete(order, fulfilledSKU, axis=1)
            temp_pod_matrix = np.delete(temp_pod_matrix, fulfilledSKU, axis=1)
            selectedPods.append(selectedPod)
    return selectedPods



if __name__ == "__main__":
    """
    df = pd.read_excel("pod seçme.xlsx", sheet_name="POD ICERIKLERI")
    df_shape = df.shape

    # Input string
    #excel_row = "146(6),206(4),252(12),279(6),310(4),619(5),706(12),712(9)"
    excel_row = df['SKU'].str.cat(sep=',')

    # Step 1: Split the string into individual records
    records = excel_row.split(',')

    # Step 2: Extract numbers and their indices
    data = [[int(record.split('(')[0]), int(record.split('(')[1][:-1])] for record in records]

    # Step 3: Sort unique numbers
    unique_numbers = sorted(set(number for number, _ in data))

    # Step 4: Create a nested list with each number and its index
    result = {number: index for index, number in enumerate(unique_numbers)}

    a_rs = np.zeros(shape=(df_shape[0],len(result)))

    df = df.sort_values(by='Pod_id').reset_index()
    for _, row in df.iterrows():
        index = row["Pod_id"]
        elements = row['SKU'].split(',')  # Split the second element by comma
        for element in elements:
            sku = int(element.split('(')[0])
            amount = int(element.split('(')[1][:-1])
            test = result[sku]
            a_rs[index, result[sku]] = amount

    itemList = [[435,1],[841,1]]
    selected = rawsimoPodSelection(itemList, result, a_rs)
    """

    itemList = [[435, 1], [841, 1]]
    a = 10