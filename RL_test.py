import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset
from RL.utils import load_model
from RL.problems import CVRP
from RL.problems.vrp.problem_vrp import VRPDataset
import os
import pickle
from RL.problems.vrp.state_cvrp import StateCVRP
from RL.problems.vrp.state_sdvrp import StateSDVRP
from RL.utils.beam_search import beam_search
import config

class VRPDatasetNew(Dataset):

    def __init__(self, size=50, num_samples=1, loc=None, demand=None, depot=None):
        super(VRPDatasetNew, self).__init__()

        self.data_set = []
        # From VRP with RL paper https://arxiv.org/abs/1802.04240
        CAPACITIES = {
            10: 20.,
            20: 30.,
            50: 40.,
            100: 50.
        }

        self.data = [
            {
                'loc': loc,
                # Uniform 1 - 9, scaled by capacities
                #'demand': (torch.FloatTensor(size).uniform_(0, 9).int() + 1).float() / CAPACITIES[size],
                'demand': demand,
                'depot': depot
            }
            for i in range(num_samples)
        ]

        self.size = len(self.data)

    def __len__(self):
        return self.size

    def __getitem__(self, idx):
        return self.data[idx]

if __name__ == "__main__":
    model, _ = load_model(config.DEFAULT_RL_MODEL_PATH)
    torch.manual_seed(1234)
    # dataset = CVRP.make_dataset(size=100, num_samples=1)
    # dataloader = DataLoader(dataset, batch_size=100)
    # batch = next(iter(dataloader))
    #
    # # Run the model
    # model.eval()
    # model.set_decode_type('greedy')
    # with torch.no_grad():
    #     length, log_p, pi = model(batch, return_pi=True)
    # tours = pi



    values_list = [(1/25, 2/30), (3/25, 4/30), (5/25, 6/30), (7/25, 8/30), (9/25, 10/30)]
    #values_list = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10)]
    locArr = np.array(values_list)
    loc = torch.Tensor(locArr)
    demandArr = np.array([1/100, 2/100, 3/100, 1/100, 2/100])
    demandArr = np.array([1/4, 1/4, 1/4, 1/4, 1/4])
    demand = torch.Tensor(demandArr)
    depotArr = np.array([0, 0])
    depot = torch.Tensor(depotArr)
    test_dataset = VRPDatasetNew(size=20, num_samples=1, loc=loc, demand=demand, depot=depot)
    dataloader = DataLoader(test_dataset, batch_size=20)
    batch = next(iter(dataloader))

    # Run the model
    model.eval()
    model.set_decode_type('greedy')
    with torch.no_grad():
        length, log_p, pi = model(batch, return_pi=True)
    tours = pi
    a = 10