import torch
from torch.utils.data import Dataset
import glob
from tqdm import tqdm
import numpy as np
import pandas as pd
import os
import re


class MotionDataset(Dataset):
    def __init__(
        self,
        datafolder: str,
        data_num: int = None,
        train: bool = True,
        split_ratio: float = 0.8,
        max_length: int = None,
    ):
        self.train = train

        state_list = []
        label_list = []

        folders = glob.glob('{}/*'.format(datafolder))

        # temporary
        # folders = folders[:1]

        for i, folder in enumerate(folders):
            paths = glob.glob('{}/motion/*.csv'.format(folder))
            filenames = [os.path.splitext(os.path.basename(path))[0]
                         for path in paths]
            filenum = [int(re.sub(r'\D', '', filename))
                       for filename in filenames]

            # reverse
            filenum.reverse()

            train_data_num = int(split_ratio * len(filenum))
            if train:
                filenum = filenum[:train_data_num]
            else:
                filenum = filenum[train_data_num:]

            if data_num != None and data_num < len(filenum):
                filenum = filenum[:data_num]

            print('loading {} data from {}'.format(len(filenum), folder))
            for filenum in tqdm(filenum):
                # load state
                motion_path = os.path.join(
                    folder, 'motion', f'slave{filenum}.csv')
                df = pd.read_csv(motion_path, dtype=np.float32)
                df = df.set_index('time')

                col_names = []
                col_names += [f'theta_res[{i}]' for i in range(8)]
                col_names += [f'omega_res[{i}]' for i in range(8)]
                col_names += [f'tau_res[{i}]' for i in range(8)]
                col_names += [f'theta_ref[{i}]' for i in range(8)]
                col_names += [f'omega_ref[{i}]' for i in range(8)]
                col_names += [f'tau_ref[{i}]' for i in range(8)]

                # data shaping
                df = df.loc[:, col_names]
                df = df[df.index > 0]
                state = torch.tensor(np.array(df))

                skip_num = 20

                # decimation
                for start in range(skip_num):
                    state_list.append(state[start::skip_num])
                    label_list.append(i)

        # padding
        if max_length is None:
            length = [len(data_part) for data_part in state_list]
            self.max_length = int(np.mean(length) + 2 * np.std(length))
        else:
            self.max_length = max_length
        self.state = torch.stack([self._padding(data_part, self.max_length)
                                  for data_part in state_list])

        # normalization
        # batch_size, steps, _ = self.state_m.shape
        # state_m = self.state_m.reshape(batch_size * steps, -1)
        # self.mean = torch.mean(state_m, axis=0, keepdims=True)
        # self.std = torch.std(state_m, axis=0, keepdims=True)
        # self.std = torch.max(self.std, torch.ones_like(self.std))
        # if normalization:
        #     state_m = self._normalization(state_m)
        #     self.state_m = state_m.reshape(batch_size, steps, -1)

        #     state_s = self.state_s.reshape(batch_size * steps, -1)
        #     state_s = self._normalization(state_s)
        #     self.state_s = state_s.reshape(batch_size, steps, -1)

        print('state shape:', self.state.shape)
        print('state data size: {} [MiB]'.format(
            self.state.detach().numpy().copy().__sizeof__() / 1.049e+6))

    def __len__(self):
        return len(self.state)

    def __getitem__(self, idx):
        state = self.state[idx]

        # for pytorch dataloader
        if type(idx) == int:
            x_state = state[:-1]
            y_state = state[1:]
        else:
            x_state = state[:, :-1]
            y_state = state[:, 1:]

        # add noise to input data
        if self.train:
            x_state += 0.1 * torch.randn_like(x_state)

        x = {
            'state': x_state,
        }
        y = {
            'state': y_state,
        }
        return x, y

    def _padding(self, x, max_length):
        input_length = len(x)
        if input_length < max_length:
            pad = max_length - input_length
            zeros = [x[-1]] * pad
            zeros = torch.stack(zeros)
            x = torch.cat([x, zeros], dim=0)
        elif input_length > max_length:
            x = x[:max_length]
        return x

    def _split_list(self, list, l):
        return [list[l * idx: l * (idx + 1)] for idx in range(len(list) // l)]

    def _normalization(self, x):
        return (x - self.mean) / self.std

    def denormalization(self, x):
        return x * self.std + self.mean
