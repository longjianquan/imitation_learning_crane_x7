import unittest
from tqdm import tqdm
import time

import sys
sys.path.append('.')
sys.path.append('..')
from dataset.fast_dataloader import FastDataLoader
from dataset.dob_dataset import DOBDataset
from dataset_path import datafolder


class TestDataset(unittest.TestCase):
    def test_dataset(self):
        print('\n========== test dataset ==========')
        dataset = DOBDataset('../../datasets/freemotion/')
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
                pass
                # print('#', i)
                # print('x state shape:', x.shape)
                # print('y state shape:', y.shape)
            end = time.time()
            print('elapsed time:', end - start)

if __name__ == '__main__':
    unittest.main()
