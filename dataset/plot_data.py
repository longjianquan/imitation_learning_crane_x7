import numpy as np
import pandas as pd

import sys
sys.path.append('.')
from train.util.plot_result import plot_state

def main(data_path: str):
    df = pd.read_csv(data_path)
    df = df.set_index('time')

    # data shaping
    col_names_s = []
    col_names_s += [f's_presentposition[{i}]' for i in range(8)]
    col_names_s += [f's_presentvelocity[{i}]' for i in range(8)]
    col_names_s += [f's_tau_res[{i}]' for i in range(8)]
    col_names_m = []
    col_names_m += [f'm_presentposition[{i}]' for i in range(8)]
    col_names_m += [f'm_presentvelocity[{i}]' for i in range(8)]
    col_names_m += [f'm_tau_res[{i}]' for i in range(8)]
    df = df[df.index > 4.0]
    df = df[df.index < 20.0]
    print(df.head())
    # print(df.tail())
    df_s = df[col_names_s]
    df_m = df[col_names_m]
    data_s = np.array(df_s)
    data_m = np.array(df_m)

    # plot figure
    fig = plot_state(
        data_s,
        data_m,
        color1='b',
        color2='r',
        label1='slave',
        label2='master',
    )
    fig.savefig('./results/plot_data.png')


if __name__ == '__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--data', type=str,
        default='./crane_x7/src/build_autonomous/slave.csv')
    args = parser.parse_args()

    main(data_path=args.data)