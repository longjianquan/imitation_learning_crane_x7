import numpy as np
import pandas as pd

from SocketServer import SocketServer


class MotionCopyServer(SocketServer):
    def __init__(
        self,
        data_path: str,
        host: str = '127.0.0.1',
        port: int = 10051,
        input_dim: int = 24,
    ):
        self.input_dim = input_dim
        self.count = 0
        # self.motion_data = np.loadtxt(
        #     data_path,
        #     delimiter=',',
        #     dtype=np.float32,
        #     skiprows=1,
        # )

        df = pd.read_csv(data_path, dtype=np.float32)
        df = df.set_index('time')

        col_names = []
        col_names += [f'm_presentposition[{i}]' for i in range(8)]
        col_names += [f'm_presentvelocity[{i}]' for i in range(8)]
        col_names += [f'm_tau_res[{i}]' for i in range(8)]

        self.motion_data = df.loc[::20, col_names]


        print('motion_data:', self.motion_data.shape)

        super().__init__(host, port)

    def standby(self):
        return super().standby(self.NN_callback)

    def NN_callback(self, msg: str) -> str:
        # data = np.fromstring(msg, dtype=np.float32 , sep=' ')
        state_hat = np.array(self.motion_data.iloc[self.count])
        # temporary
        state_hat = np.delete(state_hat, [2, 10, 18])
        print('state_hat:', state_hat)
        print(f'{self.count} / {self.motion_data.shape[0] - 1}')

        if self.count < self.motion_data.shape[0] - 1:
            self.count += 1

        # to string
        msg = ''
        for y_element in state_hat[:self.input_dim]:
            msg += f'{y_element.item()},'

        return msg


def main(args):
    server = MotionCopyServer(
        data_path=args.data_path,
    )
    server.standby()


def argparse():
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--data_path', type=str,
        default='./crane_x7/build/slave1.csv')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = argparse()
    main(args)
