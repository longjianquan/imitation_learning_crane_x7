import torch
from torch.utils.data import Dataset
from torchvision import transforms
import glob
from tqdm import tqdm
import os
from concurrent import futures
from PIL import Image
import re
import pandas as pd
import numpy as np


class ImageDataset(Dataset):
    def __init__(self, datafolder, data_num=None, train=True, split_ratio=0.8,
                 image_size=64, target_type='task'):
        self.image_size = image_size
        self.transform = transforms.Compose([
            transforms.ColorJitter(
                brightness=(0.2, 3.8),
                contrast=(0.2, 3.8),
                saturation=0.5,
                hue=0.5,
            ),
        ])

        image_list = []
        label_list = []

        folders = glob.glob('{}/*'.format(datafolder))
        for i, folder in enumerate(folders):
            paths = glob.glob('{}/color/*'.format(folder))

            train_data_num = int(split_ratio * len(paths))
            if train:
                paths = paths[:train_data_num]
            else:
                paths = paths[train_data_num:]

            if data_num != None and data_num < len(paths):
                paths = paths[:data_num]

            print('loading {} data from {}'.format(len(paths), folder))

            filenames = [os.path.splitext(os.path.basename(path))[0] for path in paths]
            filenums = [int(re.sub(r'\D', '', filename)) for filename in filenames]

            for filenum in tqdm(filenums):
                image_folder_path = '{}/color/data{}'.format(folder, filenum)
                image_paths = glob.glob(os.path.join(image_folder_path, '*.png'))

                image_names = [os.path.splitext(os.path.basename(path))[0] for path in image_paths]
                image_times = sorted([float(re.sub(r'\D.\D', '', name)) for name in image_names])
                image = self._load_images(image_paths)
                image_list.extend(image)

                # fit time
                if target_type == 'task':
                    label_list.extend(torch.tensor([i] * len(image)))
                elif target_type == 'joint_angle':
                    # load state
                    df = pd.read_csv(
                        os.path.join(folder, 'motion', f'slave{filenum}.csv'),
                        dtype=np.float32,
                    )
                    df = df.set_index('time')
                    index = [
                        'S_Angle[0]','S_Angle[1]','S_Angle[2]',
                    ]
                    df.index = np.round(df.index, decimals=3)
                    df = df.loc[image_times, index]
                    state = torch.tensor(np.array(df))
                    label_list.extend(state)

        self.image = torch.stack(image_list).squeeze()
        self.label = torch.stack(label_list)

        if target_type == 'task':
            self.label_dim = i + 1
        elif target_type == 'joint_angle':
            self.label_dim = self.label.shape[1]

        print('image shape:', self.image.shape)
        print('label shape:', self.label.shape)
        print('image data size: {} [MiB]'.format(self.image.detach().numpy().copy().__sizeof__()/1.049e+6))
        print('label data size: {} [MiB]'.format(self.label.detach().numpy().copy().__sizeof__()/1.049e+6))

    def __len__(self):
        return len(self.image)

    def __getitem__(self, idx):
        image = self.image[idx]
        image = self.transform(image)
        return image, self.label[idx]

    def _load_images(self, image_paths):
        transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize(self.image_size),
            transforms.CenterCrop(self.image_size),
        ])

        def load_one_frame(idx):
            if not os.path.exists(image_paths[idx]):
                return
            image = Image.open(image_paths[idx])
            image = transform(image)
            return idx, image

        image_list = []
        length = len(image_paths)
        image_list = [0] * length
        with futures.ThreadPoolExecutor(max_workers=4) as executor:
            future_images = [
                executor.submit(
                    load_one_frame,
                    idx) for idx in range(length)]
            for future in futures.as_completed(future_images):
                idx, image = future.result()
                image_list[idx] = image
        return torch.stack(image_list)
