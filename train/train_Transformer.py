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
from model.CNNImitation import CNNImitation
from model.TransformerImitation import TransformerImitation
from dataset.fast_dataloader import FastDataLoader
from dataset.motion_dataset import MotionDataset
from dataset.sin_wave_dataset import SinWaveDataset
# from model.SpatialAE import SpatialAE
# from dataset.motion_image_dataset import MotionImageDataset


class BCTrainer(Tranier):
    def __init__(
        self,
        data_path: str,
        out_dir: str,
        batch_size: int,
        image_size: int,
        learning_rate: float,
        wandb_flag: bool,
        gpu: list = [0],
    ):
        self.out_dir = out_dir
        self.loss_fn = nn.MSELoss()

        # image_encoder = SpatialAE(
        #     feature_point_num=16,
        #     image_size=args.image_size,
        # )
        # image_encoder.load_state_dict(torch.load(
        #     './model_param/SpatialAE_param.pt'))
        self.device = torch.device(f'cuda:{gpu[0]}'
            if torch.cuda.is_available() else 'cpu')
        # image_encoder.to(self.device)

        train_dataset = MotionDataset(
            data_path,
            train=True,
            # image_size=image_size,
            # image_encoder=image_encoder.encoder,
            # normalization=False,
            # max_length=1000,
        )
        valid_dataset = MotionDataset(
            data_path,
            train=False,
            # image_size=image_size,
            # image_encoder=image_encoder.encoder,
            # normalization=False,
            # max_length=1000,
        )
        # train_dataset = SinWaveDataset(data_num=1000)
        # valid_dataset = SinWaveDataset(data_num=100)

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

        self.dim = train_dataset.state.shape[-1]
        model = TransformerImitation(dim=self.dim)
        # model = CNNImitation(dim=train_dataset.state_m.shape[-1])

        optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

        super().__init__(
            train_loader=train_loader,
            valid_loader=valid_loader,
            model=model,
            calc_loss=self.calc_loss,
            optimizer=optimizer,
            out_dir=out_dir,
            wandb_flag=wandb_flag,
            gpu=gpu,
        )

        if wandb_flag:
            wandb.init(project='ImitationLearning')
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

    def generate(
        self,
        length: int = 500,
        init_state: torch.Tensor = None,
    ):
        seq_length = length
        memory_length = length
        if init_state is None:
            memory = [torch.zeros(size=(1, 1, self.dim)).to(self.device)]
        else:
            memory = [init_state.unsqueeze(0).unsqueeze(0)]
        from tqdm import tqdm
        for _ in tqdm(range(seq_length), 'generate'):
            memory_tensor = torch.cat(memory[:memory_length], dim=1)
            pred = self.model(memory_tensor)[:, -1]
            memory.append(pred.unsqueeze(1))
        memory_tensor = torch.cat(memory, dim=1)
        return memory_tensor

    def plot_result(self, epoch: int):
        if epoch % 100 == 0 or (epoch % 10 == 0 and epoch <= 100):
            state_ans_tensor = self.y[0]
            pred_tensor = self.pred[0]
            state_ans = state_ans_tensor.cpu().detach().numpy().copy()
            pred = pred_tensor.cpu().detach().numpy().copy()

            fig_state = plot_state(state_ans, pred)
            fig_state.suptitle('{} epoch'.format(epoch))
            path_state_png = os.path.join(self.out_dir, 'state.png')
            fig_state.savefig(path_state_png)

            fig_state_slave = plot_state(state_ans[:, 24:], pred[:, 24:])
            fig_state_slave.suptitle('{} epoch'.format(epoch))
            path_state_slave_png = os.path.join(self.out_dir, 'state_slave.png')
            fig_state_slave.savefig(path_state_slave_png)

            generated = self.generate(
                init_state=state_ans_tensor[0],
                length=state_ans_tensor.shape[0],
            )
            generated = generated.cpu().detach().numpy().copy()
            fig_generated = plot_state(state_ans, generated)
            fig_generated.suptitle('{} epoch'.format(epoch))
            path_generated = os.path.join(self.out_dir, 'state_generated.png')
            fig_generated.savefig(path_generated)

            # upload to wandb
            if self.wandb_flag:
                wandb.log({
                    'epoch': epoch,
                    'state': wandb.Image(fig_state),
                    'state_slave': wandb.Image(fig_state_slave),
                    'generated': wandb.Image(fig_generated),
                })
                wandb.save(path_state_png)
                wandb.save(path_state_slave_png)
                wandb.save(path_generated)

            plt.close()

    def train(self, n_epochs: int):
        return super().train(n_epochs, callback=self.plot_result)


def main(args):
    BCTrainer(
        data_path=args.data,
        out_dir=args.output,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        image_size=args.image_size,
        wandb_flag=args.wandb,
        gpu=args.gpu,
    ).train(args.epoch)


def argparse():
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--data', type=str)
    parser.add_argument('--output', type=str, default='./results/BC_test/')
    parser.add_argument('--epoch', type=int, default=10000)
    parser.add_argument('--batch_size', type=int, default=16)
    parser.add_argument('--learning_rate', type=float, default=0.001)
    parser.add_argument('--image_size', type=int, default=64)
    parser.add_argument('--wandb', action='store_true')
    def tp(x): return list(map(int, x.split(',')))
    parser.add_argument('--gpu', type=tp, default='0')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = argparse()
    main(args)
