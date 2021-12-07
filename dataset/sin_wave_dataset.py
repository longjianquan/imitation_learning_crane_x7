import torch
from torch.utils.data import Dataset


class SinWaveDataset(Dataset):
    def __init__(
        self,
        data_num: int = 100,
        length: int = 1000,
    ):
        self.state_m = [[i / 100 for i in range(0, length)]] * data_num
        self.state_m = torch.sin(torch.Tensor(self.state_m))
        self.state_m = self.state_m.repeat(24, 1, 1).permute(1, 2, 0)
        self.state_m += 0.1 * torch.randn_like(self.state_m)

        print('state shape:', self.state_m.shape)
        print('state data size: {} [MiB]'.format(
            self.state_m.detach().numpy().copy().__sizeof__() / 1.049e+6))

    def __len__(self):
        return len(self.state_m)

    def __getitem__(self, idx):
        state = self.state_m[idx]

        # for pytorch dataloader
        if isinstance(idx, int):
            x_state = state[:-1]
            y_state = state[1:]
        else:
            x_state = state[:, :-1]
            y_state = state[:, 1:]

        x = {
            'state': x_state,
        }
        y = {
            'state': y_state,
        }
        return x, y
