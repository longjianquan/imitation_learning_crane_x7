import unittest

import torch
from torch.utils.data import DataLoader

from tqdm import tqdm

import sys
sys.path.append('.')
sys.path.append('..')
from model.TransformerImitation import TransformerImitation
from dataset.motion_image_dataset import MotionImageDataset
from dataset_path import datafolder


class TestModel(unittest.TestCase):
    def test_Model(self):
        print('\n========== test model ==========')
        dataset = MotionImageDataset(datafolder)
        dataloader = DataLoader(
            dataset,
            batch_size=100,
            shuffle=True,
            num_workers=8,
            pin_memory=True,
        )

        model = TransformerImitation(dim=24)
        model.train()

        # device setting
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print('device:', device)
        model.to(device)

        for x, y in tqdm(dataloader):
            x = x['state'].to(device)
            y = y['state'].to(device)
            print('x:', x.shape)
            print('y:', y.shape)
            pred = model(x)
            print('pred:', pred.shape)


if __name__ == "__main__":
    unittest.main()
