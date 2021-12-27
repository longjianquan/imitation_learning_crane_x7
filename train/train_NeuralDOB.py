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
from util.plot_result import *
from dataset.fast_dataloader import FastDataLoader
from dataset.dob_dataset import DOBDataset


class MLP(nn.Module):
    def __init__(
        self,
        input_dim: int,
        output_dim: int,
        hidden_dim: int = 100,
        layer_num: int = 4,
        dropout: float = 0.0,
    ):
        super().__init__()

        layer_list = []
        channels = [input_dim] + ([hidden_dim] * (layer_num - 2)) + [output_dim]
        for i in range(len(channels) - 1):
            layer_list.append(nn.Linear(channels[i], channels[i + 1]))
            layer_list.append(nn.Dropout2d(dropout))
        self.mlp = nn.Sequential(*layer_list)

    def forward(self, x):
        return self.mlp(x)


class DOBTrainer(Tranier):
    def __init__(
        self,
        data_path: str,
        out_dir: str,
        batch_size: int,
        learning_rate: float,
        wandb_flag: bool,
        gpu: list = [0],
    ):
        self.out_dir = out_dir
        self.loss_fn = nn.MSELoss()
        self.device = torch.device(f'cuda:{gpu[0]}'
                                   if torch.cuda.is_available() else 'cpu')

        self.train_dataset = DOBDataset(
            data_path,
            train=True,
        )
        self.valid_dataset = DOBDataset(
            data_path,
            train=False,
        )

        self.train_loader = FastDataLoader(
            self.train_dataset,
            batch_size=batch_size,
            shuffle=True,
        )
        self.valid_loader = FastDataLoader(
            self.valid_dataset,
            batch_size=batch_size,
            shuffle=False,
            drop_last=True,
        )

        print('train data num:', len(self.train_dataset))
        print('valid data num:', len(self.valid_dataset))

        model = MLP(
            input_dim=self.train_dataset.x.shape[-1],
            output_dim=self.train_dataset.y.shape[-1],
            hidden_dim=400,
            layer_num=6,
        )
        model.to(self.device)

        optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

        super().__init__(
            train_loader=self.train_loader,
            valid_loader=self.valid_loader,
            model=model,
            calc_loss=self.calc_loss,
            optimizer=optimizer,
            out_dir=out_dir,
            wandb_flag=wandb_flag,
            gpu=gpu,
        )

        if wandb_flag:
            wandb.init(project='NeuralDOB')
            config = wandb.config
            config.data_path = data_path
            config.batch_size = batch_size
            config.learning_rate = learning_rate
            config.train_data_num = len(self.train_dataset)
            config.valid_data_num = len(self.valid_dataset)
            wandb.watch(model)

    def calc_loss(
        self,
        batch: Tuple[torch.Tensor, torch.Tensor],
        valid: bool = False,
    ) -> torch.Tensor:
        x, t = batch
        x = x.to(self.device)
        t = t.to(self.device)

        y = self.model(x)
        loss = self.loss_fn(y, t)

        return loss

    def plot_result(self, y: np.ndarray, t: np.ndarray):
        DoF = t.shape[0]
        fig, ax = plt.subplots(
            DoF // 2, 2,
            figsize=(20, 20),
            sharex=True,
            sharey=True,
        )

        for i in range(DoF):
            ax[i // 2, i % 2].plot(
                t[i], color='tab:grey', label='teacher', alpha=0.8)
            ax[i // 2, i % 2].plot(
                y[i], color='tab:blue', label='predict', alpha=0.8)
            ax[i // 2, i % 2].set_ylabel(r'$\tau_' + str(i) + '$ [Nm]')

        ax[-1, 0].set_xlabel('time step')
        ax[0, 0].legend(loc='lower left')
        fig.align_labels()
        fig.tight_layout(rect=[0, 0, 1, 0.96])

        return fig


    def callback(self, epoch: int):
        if epoch % 100 == 0 or (epoch % 10 == 0 and epoch <= 100):
            x = self.train_dataset.x.to(self.device)
            y = self.model(x).detach().cpu().numpy().transpose()
            t = self.train_dataset.y.detach().cpu().numpy().transpose()

            fig_train = self.plot_result(y, t)
            fig_train.savefig(os.path.join(self.out_dir, 'training.png'))

            x = self.valid_dataset.x.to(self.device)
            y = self.model(x).detach().cpu().numpy().transpose()
            t = self.valid_dataset.y.detach().cpu().numpy().transpose()

            fig_valid = self.plot_result(y, t)
            fig_valid.savefig(os.path.join(self.out_dir, 'validation.png'))

            # upload to wandb
            if self.wandb_flag:
                wandb.log({
                    'epoch': epoch,
                    'training data': wandb.Image(fig_train),
                    'validation data': wandb.Image(fig_valid),
                })

            plt.close()

    def train(self, n_epochs: int):
        return super().train(n_epochs, callback=self.callback)


def main(args):
    DOBTrainer(
        data_path=args.data,
        out_dir=args.output,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        wandb_flag=args.wandb,
        gpu=args.gpu,
    ).train(args.epoch)


def argparse():
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--data', type=str, default='../datasets/freemotion/')
    parser.add_argument('--output', type=str, default='./results/DOB_test/')
    parser.add_argument('--epoch', type=int, default=10000)
    parser.add_argument('--batch_size', type=int, default=256)
    parser.add_argument('--learning_rate', type=float, default=0.001)
    parser.add_argument('--wandb', action='store_true')
    def tp(x): return list(map(int, x.split(',')))
    parser.add_argument('--gpu', type=tp, default='0')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = argparse()
    main(args)
