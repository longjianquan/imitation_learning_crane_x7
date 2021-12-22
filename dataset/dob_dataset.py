import torch
from torch.utils.data import Dataset
import glob
from tqdm import tqdm
import numpy as np
import pandas as pd
import os
import re


class DOBDataset(Dataset):
    def __init__(self,
        datafolder: str,
        train: bool = True,
        split_ratio: float = 0.8,
    ):
        self.train = train

        folder = datafolder
        paths = glob.glob(os.path.join(datafolder, '*.csv'))
        print('loading {} data from {}'.format(len(paths), folder))

        x_list, y_list = [], []
        for path in tqdm(paths):
            # load state
            df = pd.read_csv(path, dtype=np.float32)
            df = df.set_index('time')

            # data shaping
            df = df[df.index > 0]
            col_names = []
            col_names += [f's_presentposition[{i}]' for i in range(8)]
            col_names += [f's_presentvelocity[{i}]' for i in range(8)]
            col_names += [f's_tau_res[{i}]' for i in range(8)]
            x = torch.tensor(np.array(df.loc[:, col_names]))
            x = x[1:]
            x_list.append(x)

            y_col_names = [f's_goal_torque[{i}]' for i in range(8)]
            y = torch.tensor(np.array(df.loc[:, y_col_names]))
            y = y[:-1]
            y_list.append(y)

        self.x = torch.cat(x_list)
        self.y = torch.cat(y_list)

        print('x shape:', self.x.shape)
        print('y shape:', self.y.shape)
        print('x data size: {} [MiB]'.format(
            self.x.detach().numpy().copy().__sizeof__()/1.049e+6))
        print('y data size: {} [MiB]'.format(
            self.y.detach().numpy().copy().__sizeof__()/1.049e+6))

    def __len__(self):
        return len(self.x)

    def __getitem__(self, idx):
        return self.x[idx], self.y[idx]
