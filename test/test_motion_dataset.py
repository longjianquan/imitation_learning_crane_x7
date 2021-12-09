import unittest
from tqdm import tqdm
import time

import sys
sys.path.append('.')
sys.path.append('..')
from dataset.fast_dataloader import FastDataLoader
from dataset.motion_dataset import MotionDataset
from dataset_path import datafolder


class TestDataset(unittest.TestCase):
    def test_dataset(self):
        print('\n========== test dataset ==========')
        dataset = MotionDataset(
            datafolder,
            data_num=50,
        )
        dataloader = FastDataLoader(
            dataset,
            batch_size=100,
            shuffle=True,
        )

        print('data length:', len(dataset))
        print('\n---------- my data loader test ----------')
        for e in range(3):
            start = time.time()
            for i, (x, y) in enumerate(tqdm(dataloader)):
                # pass
                # print('#', i)
                print('x state shape:', x['state'].shape)
                print('y state shape:', y['state'].shape)
                # print('x image shape:', x['image'].shape)
                # print('y image shape:', y['image'].shape)
                # print(x['state'][0][0])
                # print(y['state'][0][0])
            end = time.time()
            print('elapsed time:', end - start)
                # start = time.time()
                # dataset.denormalization(x['state'])

if __name__ == '__main__':
    unittest.main()
