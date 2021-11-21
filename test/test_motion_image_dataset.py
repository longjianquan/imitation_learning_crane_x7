import unittest
from torch.utils.data import DataLoader
from tqdm import tqdm
import time

import sys
sys.path.append('.')
sys.path.append('..')
from dataset.fast_dataloader import FastDataLoader
from dataset.motion_image_dataset import MotionImageDataset
from model.AutoEncoder import AutoEncoder
from dataset_path import datafolder


class TestDataset(unittest.TestCase):
    def test_dataset(self):
        print('\n========== test dataset ==========')
        auto_encoder = AutoEncoder(z_dim=10, image_size=64, n_channel=3)
        encoder = auto_encoder.encoder
        encoder.eval()
        # encoder = None

        dataset_image = MotionImageDataset(
            datafolder,
            data_num=50,
        )
        dataset_encoder = MotionImageDataset(
            datafolder,
            data_num=50,
            image_encoder=encoder,
        )

        datasets = [dataset_image, dataset_encoder]
        batch_sizes = [8, 256]
        for dataset, batch_size in zip(datasets, batch_sizes):
            dataloader = FastDataLoader(
                dataset,
                batch_size=batch_size,
                shuffle=True,
            )
            print('data length:', len(dataset))
            print('batch size:', batch_size)
            print('\n---------- my data loader test ----------')
            for e in range(3):
                start = time.time()
                for i, (x, y) in enumerate(tqdm(dataloader)):
                    pass
                    # print('#', i)
                    # print('x state shape:', x['state'].shape)
                    # print('y state shape:', y['state'].shape)
                    # print('x image shape:', x['image'].shape)
                    # print('y image shape:', y['image'].shape)
                    # print(x['state'][0][0])
                    # print(y['state'][0][0])
                end = time.time()
                print('elapsed time:', end - start)
                    # start = time.time()
                    # dataset.denormalization(x['state'])

            torchdataloader = DataLoader(
                dataset,
                batch_size=batch_size,
                shuffle=True,
                num_workers=8,
                # pin_memory=True,
            )
            print('\n---------- pytorch data loader test ----------')
            for e in range(3):
                start = time.time()
                for i, (x, y) in enumerate(tqdm(torchdataloader)):
                    pass
                    # print('#', i)
                    # print('x state shape:', x['state'].shape)
                    # print('y state shape:', y['state'].shape)
                    # print('x image shape:', x['image'].shape)
                    # print('y image shape:', y['image'].shape)
                end = time.time()
                print('elapsed time:', end - start)
                    # start = time.time()


if __name__ == '__main__':
    unittest.main()
