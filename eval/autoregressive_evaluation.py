import argparse
import numpy as np
import torch
import matplotlib.pyplot as plt
import seaborn as sns
from torch._C import device
sns.set()

import sys
sys.path.append('.')
from model.TransformerImitation import TransformerImitation


def load_model_param(path: str):
        state_dict = torch.load(path)
        from collections import OrderedDict
        new_state_dict = OrderedDict()
        for k, v in state_dict.items():
            if 'module' in k:
                k = k.replace('module.', '')
            new_state_dict[k] = v
        return new_state_dict


def rad2deg(rad):
    return rad * 180 / np.pi


def plot_state(
    state: np.ndarray,
    DoF: int = 8,
):
    fig, ax = plt.subplots(
        DoF, 3,
        figsize=(20, 20),
        sharex=True,
        sharey='col',
    )

    state = state.transpose()

    theta = rad2deg(state[:DoF])
    omega = rad2deg(state[DoF:2*DoF])
    tau = state[2*DoF:3*DoF]

    for i in range(DoF):
        ax[i, 0].plot(theta[i])
        ax[i, 0].set_ylabel(r'$\theta_' + str(i) + '$ [deg]')
        ax[i, 0].set_ylim([-10, 370])
        ax[i, 0].set_yticks(range(0, 370, 90))

        ax[i, 1].plot(omega[i])
        ax[i, 1].set_ylabel(r'$\.{\theta}_' + str(i) + '$ [deg/s]')

        ax[i, 2].plot(tau[i])
        ax[i, 2].set_ylabel(r'$\tau_' + str(i) + '$ [Nm]')

    for i in range(3):
        ax[DoF-1, i].set_xlabel('time step')
    fig.align_labels()
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    return fig


def main(args: argparse):
    dim = 24
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    policy = TransformerImitation(dim=dim)
    state_dict = load_model_param(args.model)
    policy.load_state_dict(state_dict)
    policy = policy.to(device)

    x_list = []
    length = args.length
    x = torch.zeros(size=(1, 1, dim)).to(device)
    for _ in range(length):
        x = policy(x)
        x_list.append(x)
    x = torch.cat(x_list, dim=1)
    fig = plot_state(x.detach().cpu().numpy())
    fig.savefig('./results/autoregressive_evaluation.png')


if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--model', type=str,
        default='./model_param/Transformer_param.pt')
    parser.add_argument('--length', type=int, default=1000)
    args = parser.parse_args()

    main(args)
