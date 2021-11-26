import torch
from torch.utils.data import Dataset
from torchvision import transforms
import glob
from tqdm import tqdm
import numpy as np
import pandas as pd
import os
from concurrent import futures
from PIL import Image
import re


class MotionDataset(Dataset):
    def __init__(self,
        datafolder: str,
        data_num: int = None,
        train: bool = True,
        split_ratio: float = 0.8,
        start_step: int = 50,
        max_length: int = 1000,
        normalization: bool = True,
        split_seq: bool = False,
        # image_size: int = 64,
        # image_encoder: torch.nn.Module = None,
    ):
        self.train = train
        # self.image_size = image_size
        # self.image_encoder = image_encoder

        state_list, image_list = [], []
        image_idx_list = []

        folders = glob.glob('{}/*'.format(datafolder))

        # temporary
        folders = folders[:1]

        image_idx = 0
        for folder in folders:
            paths = glob.glob('{}/motion/*.csv'.format(folder))
            filenames = [os.path.splitext(os.path.basename(path))[0]
                for path in paths]
            filenum = [int(re.sub(r'\D', '', filename))
                for filename in filenames]

            train_data_num = int(split_ratio * len(filenum))
            if train:
                filenum = filenum[:train_data_num]
            else:
                filenum = filenum[train_data_num:]

            if data_num != None and data_num < len(filenum):
                filenum = filenum[:data_num]

            print('loading {} data from {}'.format(len(filenum), folder))
            for filenum in tqdm(filenum):
                # image paths
                # image_folder_path = os.path.join(
                #     folder, 'color', f'data{filenum}')
                # image_paths = glob.glob(os.path.join(
                #     image_folder_path, '*.png'))

                # image_names = [os.path.splitext(os.path.basename(path))[0]
                #     for path in image_paths]
                # image_nums = sorted([float(re.sub(r'\D.\D', '', name))
                #     for name in image_names])

                # load state
                motion_path = os.path.join(
                    folder, 'motion', f'slave{filenum}.csv')
                df = pd.read_csv(motion_path, dtype=np.float32)
                df = df.set_index('time')

                col_names = []
                col_names += [f's_presentposition[{i}]' for i in range(8)]
                col_names += [f's_presentvelocity[{i}]' for i in range(8)]
                col_names += [f's_tau_res[{i}]' for i in range(8)]
                col_names += [f'm_presentposition[{i}]' for i in range(8)]
                col_names += [f'm_presentvelocity[{i}]' for i in range(8)]
                col_names += [f'm_tau_res[{i}]' for i in range(8)]

                # fit time
                # start = max(df.index[0], image_nums[0])
                # end = min(df.index[-1], image_nums[-1])
                df = df.loc[:, col_names]
                state = torch.tensor(np.array(df))

                # # missing image complement
                # i = 0
                # time = [num for num in image_nums if start <= num <= end]
                # while i < len(time) - 1:
                #     diff = round((time[i + 1] - time[i]) * 1000)
                #     if diff > 40:
                #         num = -(-diff//40)
                #         for j in range(num-1):
                #             time.insert(i, time[i])
                #         i += num
                #     elif diff < 40:
                #         time.pop(i+1)
                #     else:
                #         i += 1

                # # load image
                # image_paths = [
                #     os.path.join(image_folder_path, '{:.3f}.png'.format(t))
                #         for t in time
                # ]
                # image = self._load_images(image_paths)

                # skip_num = -(-len(state) // len(image))
                skip_num = 40
                l = max_length + 1 + start_step
                if len(state) > max_length and split_seq:
                    # split sequence
                    # image = self._split_list(image, l)
                    state = self._split_list(state, l * skip_num)
                else:
                    # padding
                    # image = [self._padding(image, l)]
                    state = [self._padding(state, l * skip_num)]
                # image_list.extend(image)

                # decimation
                for state_part in state:
                    for start in range(skip_num):
                        # image_idx_list.append(image_idx)
                        state_list.append(state_part[start::skip_num])
                    # image_idx += 1

        state = torch.stack(state_list)
        # self.image = torch.stack(image_list).squeeze()
        self.image_idx = torch.tensor(image_idx_list)

        # skip head data
        state = state[:, start_step:]
        # self.image = self.image[:, start_step:]

        self.state_s = state[:, :, :24]
        self.state_m = state[:, :, 24:]

        # normalization
        batch_size, steps, _ = self.state_m.shape
        state_m = self.state_m.reshape(batch_size * steps, -1)
        self.mean = torch.mean(state_m, axis=0, keepdims=True)
        self.std = torch.std(state_m, axis=0, keepdims=True)
        self.std = torch.max(self.std, torch.ones_like(self.std))
        if normalization:
            state_m = self._normalization(state_m)
            self.state_m = state_m.reshape(batch_size, steps, -1)

            state_s = self.state_s.reshape(batch_size * steps, -1)
            state_s = self._normalization(state_s)
            self.state_s = state_s.reshape(batch_size, steps, -1)

        print('state shape:', state.shape)
        # print('image shape:', self.image.shape)
        print('state data size: {} [MiB]'.format(
            state.detach().numpy().copy().__sizeof__()/1.049e+6))
        # print('image data size: {} [MiB]'.format(
        #     self.image.detach().numpy().copy().__sizeof__()/1.049e+6))

    def __len__(self):
        return len(self.state_s)

    def __getitem__(self, idx):
        state_s = self.state_s[idx]
        state_m = self.state_m[idx]
        # image = self.image[self.image_idx[idx]]

        # for pytorch dataloader
        if type(idx) == int:
            # state_s = state_s.unsqueeze(0)
            # state_m = state_m.unsqueeze(0)
            # image = image.unsqueeze(0)
            x_state = state_s[:-1].clone()
            # x_image = image[:-1].clone()
            y_state = state_m[1:]
            # y_image = image[1:]
        else:
            x_state = state_s[:, :-1].clone()
            # x_image = image[:, :-1].clone()
            y_state = state_m[:, 1:]
            # y_image = image[:, 1:]

        # add noise to input data
        if self.train:
            x_state += 0.1 * torch.randn_like(x_state)
            # x_image += 0.1 * torch.randn_like(x_image)

        x = {
            'state': x_state,
            # 'image': x_image,
        }
        y = {
            'state': y_state,
            # 'image': y_image,
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
        return [list[l * idx: l * (idx + 1)] for idx in range(len(list)//l)]

    # def _load_images(self, image_paths):
    #     transform = transforms.Compose([
    #         transforms.ToTensor(),
    #         transforms.Resize(self.image_size),
    #         transforms.CenterCrop(self.image_size),
    #     ])

    #     def load_one_frame(idx):
    #         if not os.path.exists(image_paths[idx]):
    #             return
    #         image = Image.open(image_paths[idx])
    #         image = transform(image)
    #         if self.image_encoder != None:
    #             with torch.no_grad():
    #                 image = self.image_encoder(image.unsqueeze(0))
    #         return idx, image

    #     image_list = []
    #     length = len(image_paths)
    #     image_list = [0] * length
    #     with futures.ThreadPoolExecutor(max_workers=8) as executor:
    #         future_images = [
    #             executor.submit(
    #                 load_one_frame,
    #                 idx) for idx in range(length)]
    #         for future in futures.as_completed(future_images):
    #             idx, image = future.result()
    #             image_list[idx] = image
    #     return torch.stack(image_list)

    def _normalization(self, x):
        return (x - self.mean) / self.std

    def denormalization(self, x):
        return x * self.std + self.mean
