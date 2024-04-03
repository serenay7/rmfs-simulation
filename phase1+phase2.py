import lp_podselection
import vrp
import generators
import podstorage
import pandas as pd



def PhaseIandIIComplete(orderList, podMatrix, network, stationNodes, numRobot, max_percentage=0.5):

    selectedPodNodes, numSelectedPodsP1, total_distance, selectedPodNodesRawsimo, numSelectedPodsRawsimo, totalDistRawsimo = lp_podselection.PhaseIExperiment(orderList, podMatrix, network, stationNodes, max_percentage, returnSelected=True)

    vrpdf = vrp.solve_vrp(numRobot, network, selectedPodNodes)
    vrpdf['CarriedPods'] = vrpdf['Route'].apply(lambda x: len(x)-2)

    rawsimodf = vrp.filter_and_calculate_distance(numRobot, selectedPodNodesRawsimo)
    tempDF = pd.DataFrame(selectedPodNodesRawsimo, columns=["Nodes","Robot"])
    grouped_counts = tempDF.groupby('Robot').size().reset_index()
    grouped_counts.columns = ["Robot", "Count"]
    rawsimodf["CarriedPods"] = grouped_counts["Count"]

    return vrpdf, rawsimodf

def PhaseIandIIOuter(networkList, numRepeatForInstance, orderPerStation=20):
    excel_file = 'complete_phase1+2_output.xlsx'
    excel_rawsimo_file = 'complete_rawsimo_output.xlsx'

    writer = pd.ExcelWriter(excel_file, engine='xlsxwriter')
    writer_rawsimo = pd.ExcelWriter(excel_rawsimo_file, engine='xlsxwriter')

    idx = 0
    for networkSTR in networkList:
        print(networkSTR)

        dimensions = networkSTR.split("x")
        row = int(dimensions[0])
        column = int(dimensions[1])
        network, network_corridors = generators.create_network(vertical=row, horizontal=column)

        r = row * column * 8  # Number of storage pods
        s = r*2  # Number of SKUs
        k = r*1//20  # Maximum number of pods for each SKU
        lower_bound = 100  # Lower bound of the amount interval
        upper_bound = 200  # Upper bound of the amount interval

        for numStation in [2, 4]:

            for run in range(numRepeatForInstance):
                sheet_name = f'Sheet_{idx}'
                idx += 1
                orderList = generators.orderGenerator(stationCapacity=orderPerStation, numStation=numStation, numSKU=s, skuExistencethreshold=0.9)
                podMatrix = podstorage.generate_distribution_matrix(s, r, k, lower_bound, upper_bound).T
                station_nodes = lp_podselection.stationLocationFinder(network, numStation)
                vrpdf, rawsimodf = PhaseIandIIComplete(orderList=orderList, podMatrix=podMatrix, network=network, stationNodes=station_nodes, numRobot=numStation, max_percentage=0.5)
                vrpdf.to_excel(writer, sheet_name=sheet_name, index=False)
                rawsimodf.to_excel(writer_rawsimo, sheet_name=sheet_name, index=False)
    writer._save()
    writer_rawsimo._save()

if __name__ == "__main__":
    networkList = ["4x8", "5x5", "6x12", "8x8", "10x20"]
    PhaseIandIIOuter(networkList, 1, 20)
