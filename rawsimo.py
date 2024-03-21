import numpy as np
import pandas as pd

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





if __name__ == "__main__":

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
        print(np.sum(a_rs))

    itemList = [[435,1],[841,1]]
    selected = rawsimoPodSelection(itemList, result, a_rs)
    a = 10
