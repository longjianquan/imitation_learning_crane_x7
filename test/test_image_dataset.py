import unittest
from torch.utils.data import DataLoader
from tqdm import tqdm

import sys
sys.path.append('.')
sys.path.append('..')
from dataset.fast_dataloader import FastDataLoader
from dataset.image_dataset import ImageDataset
from dataset_path import datafolder


class TestImageDataset(unittest.TestCase):
    def test_dataset(self):
        print('\n========== test dataset ==========')
        # dataset = ImageDataset(datafolder, data_num=50, target_type='joint_angle')
        dataset = ImageDataset(datafolder, data_num=50, target_type='task')
        print('data length:', len(dataset))
        dataloader = FastDataLoader(
            dataset,
            batch_size=256,
            shuffle=True,
        )
        print('\n---------- my data loader test ----------')
        for e in range(3):
            for i, (image, label) in enumerate(tqdm(dataloader)):
                pass
                # print('#', i)
                # print('shape:', image.shape)

        torchdataloader = DataLoader(
            dataset,
            batch_size=256,
            shuffle=True,
            pin_memory=True,
        )
        print('\n---------- pytorch data loader test ----------')
        for e in range(3):
            for i, (image, label) in enumerate(tqdm(torchdataloader)):
                pass
                # print('#', i)
                # print('shape:', image.shape)


if __name__ == '__main__':
    unittest.main()
