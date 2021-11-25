from typing import Tuple
import wandb

import torch.nn as nn
import torch

import matplotlib.pyplot as plt
import seaborn as sns
sns.set()

import sys
sys.path.append('.')
from train.trainer import Tranier
from model.SpatialAE import SpatialAE
from dataset.motion_image_dataset import MotionImageDataset
from dataset.fast_dataloader import FastDataLoader
from model.TransformerImitation import TransformerImitation
from util.plot_result import *


class TransformerTrainer(Tranier):
    def __init__(
        self,
        data_path: str,
        out_dir: str,
        batch_size: int,
        image_size: int,
        learning_rate: float,
        wandb_flag: bool,
        gpu_num: list = [0],
    ):
        image_encoder = SpatialAE(
            feature_point_num=16,
            image_size=args.image_size,
        )
        image_encoder.load_state_dict(torch.load(
            './model_param/SpatialAE_param.pt'))

        train_dataset = MotionImageDataset(
            data_path,
            train=True,
            image_size=image_size,
            image_encoder=image_encoder.encoder,
            normalization=False,
        )
        valid_dataset = MotionImageDataset(
            data_path,
            train=False,
            image_size=image_size,
            image_encoder=image_encoder.encoder,
            normalization=False,
        )

        train_loader = FastDataLoader(
            train_dataset,
            batch_size=batch_size,
            shuffle=True,
        )
        valid_loader = FastDataLoader(
            valid_dataset,
            batch_size=batch_size,
            shuffle=True,
            drop_last=True,
        )

        print('train data num:', len(train_dataset))
        print('valid data num:', len(valid_dataset))

        model = TransformerImitation(dim=train_dataset.state_m.shape[-1])

        optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

        super().__init__(
            train_loader=train_loader,
            valid_loader=valid_loader,
            model=model,
            calc_loss=self.calc_loss,
            optimizer=optimizer,
            out_dir=out_dir,
            wandb_flag=wandb_flag,
            gpu_num=gpu_num,
        )

        # figure
        self.fig_reconstructed_image = plt.figure(figsize=(20, 10))
        self.fig_latent_space = plt.figure(figsize=(10, 10))
        self.fig_feature_map = plt.figure(figsize=(10, 10))

        self.loss_fn = nn.MSELoss()

        # data for plot
        self.valid_encoded = []
        self.valid_label = []

        if wandb_flag:
            wandb.init(project='TransformerImitation')
            config = wandb.config
            config.data_path = data_path
            config.batch_size = batch_size
            config.learning_rate = learning_rate
            config.train_data_num = len(train_dataset)
            config.valid_data_num = len(valid_dataset)
            wandb.watch(model)

    def calc_loss(
        self,
        batch: Tuple[torch.Tensor, torch.Tensor],
        valid: bool = False,
    ) -> torch.Tensor:
        x, y = batch
        x = x['state'].to(self.device)
        y = y['state'].to(self.device)

        pred = self.model(x)
        # print(pred)
        loss = self.loss_fn(y, pred)

        return loss

    def train(self, n_epochs: int):
        return super().train(n_epochs)


def main(args):
    TransformerTrainer(
        data_path=args.data_path,
        out_dir=args.output_path,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        image_size=args.image_size,
        wandb_flag=args.wandb,
        gpu_num=args.gpu_num,
    ).train(args.epoch)


def argparse():
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--data_path', type=str)
    parser.add_argument('--output_path', type=str, default='./results/test')
    parser.add_argument('--epoch', type=int, default=10000)
    parser.add_argument('--batch_size', type=int, default=16)
    parser.add_argument('--learning_rate', type=float, default=0.001)
    parser.add_argument('--image_size', type=int, default=64)
    parser.add_argument('--wandb', action='store_true')
    tp = lambda x:list(map(int, x.split(',')))
    parser.add_argument('--gpu_num', type=tp, default='0')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = argparse()
    main(args)
