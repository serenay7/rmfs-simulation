import lp_podselection
import vrp
import generators

def PhaseIandIIComplete(orderList, podMatrix, network, stationNodes, numRobot, max_percentage=0.5):

    selectedPodNodes, numSelectedPodsP1, total_distance, selectedPodNodesRawsimo, numSelectedPodsRawsimo, totalDistRawsimo = lp_podselection.PhaseIExperiment(orderList, podMatrix, network, stationNodes, max_percentage, returnSelected=True)
    vrpdf = vrp.solve_vrp(numRobot, network, selectedPodNodes)
    rawsimodf = vrp.filter_and_calculate_distance(numRobot, )


if __name__ == "__main__":
    networkList = ["5x5", "6x12", "8x8", "10x20"]