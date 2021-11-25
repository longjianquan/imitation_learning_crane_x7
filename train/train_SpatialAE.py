import os
from typing import Tuple
from torchvision import transforms
from torchvision.transforms.transforms import ColorJitter
import wandb

import torch.nn as nn
import torch

import matplotlib.pyplot as plt
import seaborn as sns
sns.set()

import sys
sys.path.append('.')
from train.trainer import Tranier
from dataset.image_dataset import ImageDataset
from dataset.fast_dataloader import FastDataLoader
from model.SpatialAE import SpatialAE
from util.plot_result import *


class AddGaussianNoise():
    def __init__(self, mean: float = 0.0, std: float = 0.1):
        self.std = std
        self.mean = mean

    def __call__(self, tensor: torch.Tensor):
        return tensor + torch.randn_like(tensor) * self.std + self.mean


class SpatialAETrainer(Tranier):
    def __init__(
        self,
        data_path: str,
        out_dir: str,
        batch_size: int,
        image_size: int,
        learning_rate: float,
        wandb_flag: bool,
        gpu_num: list = [0],
        early_stopping_count: int = 1000,
    ):
        self.out_dir = out_dir

        train_dataset = ImageDataset(
            data_path,
            train=True,
            image_size=image_size,
        )
        valid_dataset = ImageDataset(
            data_path,
            train=False,
            image_size=image_size,
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

        model = SpatialAE(feature_point_num=16, image_size=args.image_size)

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
            early_stopping_count=early_stopping_count,
        )

        # figure
        self.fig_reconstructed_image = plt.figure(figsize=(20, 10))
        self.fig_latent_space = plt.figure(figsize=(10, 10))
        self.fig_feature_map = plt.figure(figsize=(10, 10))

        self.loss_fn = nn.MSELoss()

        # data for plot
        self.valid_encoded = []
        self.valid_label = []

        self.transform = transforms.Compose([
            AddGaussianNoise(mean=0.0, std=0.1),
            ColorJitter(brightness=0.5, contrast=0.5, saturation=0.5),
        ])

        if wandb_flag:
            wandb.init(project='AutoEncoder')
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
        image, label = batch
        self.image_ans = image.to(self.device)

        image_input = self.transform(self.image_ans)
        self.image_hat, image_feature = self.model(image_input)
        loss = self.loss_fn(self.image_ans, self.image_hat)

        # save data for plot
        if valid:
            self.valid_encoded.append(image_feature.cpu().detach().numpy())
            self.valid_label.append(label.cpu().detach().numpy())

        return loss

    def plot_results(self, epoch: int):
        if epoch % 10 == 0:
            # reconstructed image
            image_ans = formatImages(self.image_ans)
            image_hat = formatImages(self.image_hat)
            self.fig_reconstructed_image.clf()
            plot_reconstructed_image(
                self.fig_reconstructed_image,
                image_ans,
                image_hat,
                col=4,
                epoch=epoch,
            )
            path_reconstructed_image_png = os.path.join(
                self.out_dir, 'reconstructed_image.png')
            self.fig_reconstructed_image.savefig(
                path_reconstructed_image_png)

            # latent space
            self.fig_latent_space.clf()
            plot_latent_space(
                self.fig_latent_space,
                np.concatenate(self.valid_encoded, axis=0),
                np.concatenate(self.valid_label, axis=0),
                epoch=epoch,
            )
            path_latent_space = os.path.join(self.out_dir, 'latent_space.png')
            self.fig_latent_space.savefig(path_latent_space)

            # feature map
            feature_map = self.model.encoder.feature_map[0].unsqueeze(1)
            feature_map = formatImages(feature_map)
            feature_points = self.model.encoder.feature_points[0]
            feature_points = feature_points.cpu().detach().numpy()
            self.fig_feature_map.clf()
            plot_images(
                self.fig_feature_map,
                feature_map,
                epoch=epoch,
                feature_points=feature_points,
            )
            path_feature_map = os.path.join(self.out_dir, 'feature_map.png')
            self.fig_feature_map.savefig(path_feature_map)

            # upload to wandb
            if self.wandb_flag:
                wandb.log({
                    'epoch': epoch,
                    'generated_image': wandb.Image(
                        self.fig_reconstructed_image),
                    'latent_space': wandb.Image(self.fig_latent_space),
                    'feature_map': wandb.Image(self.fig_feature_map),
                })
                wandb.save(path_reconstructed_image_png)

        self.valid_encoded = []
        self.valid_label = []

    def train(self, n_epochs: int):
        return super().train(n_epochs, callback=self.plot_results)



def main(args):
    SpatialAETrainer(
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
    parser.add_argument('--batch_size', type=int, default=100)
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
