from typing import Tuple
import wandb
import os

import torch.nn as nn
import torch

import matplotlib.pyplot as plt
import seaborn as sns
sns.set()

import sys
sys.path.append('.')
from train.trainer import Tranier
# from model.SpatialAE import SpatialAE
# from dataset.motion_image_dataset import MotionImageDataset
from dataset.motion_dataset import MotionDataset
from dataset.fast_dataloader import FastDataLoader
from model.LSTMImitation import LSTMImitation
from util.plot_result import *


class LSTMTrainer(Tranier):
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
        self.out_dir = out_dir
        self.loss_fn = nn.MSELoss()
        self.fig_state = plt.figure(figsize=(20, 20))

        # image_encoder = SpatialAE(
        #     feature_point_num=16,
        #     image_size=args.image_size,
        # )
        # image_encoder.load_state_dict(torch.load(
        #     './model_param/SpatialAE_param.pt'))

        train_dataset = MotionDataset(
            data_path,
            train=True,
            # image_size=image_size,
            # image_encoder=image_encoder.encoder,
            normalization=False,
        )
        valid_dataset = MotionDataset(
            data_path,
            train=False,
            # image_size=image_size,
            # image_encoder=image_encoder.encoder,
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

        model = LSTMImitation(
            input_dim=train_dataset.state_m.shape[-1],
            output_dim=train_dataset.state_m.shape[-1],
            LSTM_dim=400,
            LSTM_layer_num=5,
        )

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

        if wandb_flag:
            wandb.init(project='LSTMImitation')
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
        loss = self.loss_fn(y, pred)

        self.y = y
        self.pred = pred

        return loss

    def plot_result(self, epoch: int):
        if epoch % 10 == 0:
            self.fig_state.clf()
            state_ans = self.y[0].cpu()
            pred = self.pred[0].cpu()
            state_ans = state_ans.cpu().detach().numpy().copy()
            pred = pred.cpu().detach().numpy().copy()
            plot_state(self.fig_state, state_ans, pred, col=3)
            self.fig_state.suptitle('{} epoch'.format(epoch))
            path_state_png = os.path.join(self.out_dir, 'state.png')
            self.fig_state.savefig(path_state_png)

            # upload to wandb
            if self.wandb_flag:
                wandb.log({
                    'epoch': epoch,
                    'state': wandb.Image(self.fig_state),
                })
                wandb.save(path_state_png)

    def train(self, n_epochs: int):
        return super().train(n_epochs, callback=self.plot_result)


def main(args):
    LSTMTrainer(
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
