# import torch
from torch import nn
from torch.functional import Tensor


class Conv4Imitation(nn.Module):
    def __init__(
        self,
        in_channels,
        out_channels,
        kernel_size: int = 3,
        stride: int = 1,
        # padding: int = 2,
        dropout: float = 0.0,
    ):
        super().__init__()

        # self.n_padding = padding

        self.layer = nn.Sequential(
            nn.Conv1d(
                in_channels,
                out_channels,
                kernel_size,
                stride,
                padding = kernel_size // 2,
                dilation=1,
            ),
            nn.Dropout(dropout),
            nn.ReLU(),
        )

    # def padding(self, x: Tensor):
    #     replicate = torch.stack([x[:, :, 0]] * self.n_padding)
    #     replicate = replicate.permute(1, 2, 0)
    #     y = torch.cat([replicate, x], dim=2)
    #     return y

    def forward(self, x: Tensor):
        """
        Args:
            x: Tensor, shape [batch_size, embedding_dim, seq_len]
        """
        # x = self.padding(x)
        y = self.layer(x)
        return y


class CNNImitation(nn.Module):
    def __init__(
        self,
        dim: int,
        kernel_size: int = 5,
        dropout: float = 0.1,
    ):
        super().__init__()

        channels = [dim, 16, 32, 16, dim]
        conv_list = []
        for i in range(len(channels)-1):
            conv_list.append(
                Conv4Imitation(
                    channels[i],
                    channels[i+1],
                    kernel_size=kernel_size,
                    dropout=dropout,
                ))
        self.conv = nn.Sequential(*conv_list)
        self.linear = nn.Linear(dim, dim)

    def forward(self, x: Tensor):
        """
        Args:
            x: Tensor, shape [batch_size, seq_len, embedding_dim]
        """
        x = x.permute(0, 2, 1) # (batch, dim, time)
        x = self.conv(x)
        x = x.permute(0, 2, 1) # (batch, time, dim)
        y = self.linear(x)

        return y
