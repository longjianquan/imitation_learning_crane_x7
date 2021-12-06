from torch import nn
from torch.functional import Tensor


class Conv4Imitation(nn.Module):
    def __init__(
        self,
        in_channels,
        out_channels,
        kernel_size: int = 3,
        stride: int = 1,
        dropout: float = 0.0,
        dilation: int = 1,
    ):
        super().__init__()

        self.dilated_convolution = nn.Sequential(
            nn.ReplicationPad1d((dilation * (kernel_size - 1), 0)),
            nn.Conv1d(
                in_channels,
                out_channels,
                kernel_size=kernel_size,
                stride=stride,
                dilation=dilation,
            ),
            nn.Dropout(dropout),
            nn.ReLU(),
        )

        self.point_wise_convolution = nn.Sequential(
            nn.Conv1d(
                out_channels,
                out_channels,
                kernel_size=1,
                stride=stride,
            ),
            nn.Dropout(dropout),
            nn.ReLU(),
        )

    def forward(self, x: Tensor):
        """
        Args:
            x: Tensor, shape [batch_size, embedding_dim, seq_len]
        """
        x = self.dilated_convolution(x)
        y = self.point_wise_convolution(x)
        return y


class CNNImitation(nn.Module):
    def __init__(
        self,
        dim: int,
        kernel_size: int = 5,
        dropout: float = 0.0,
    ):
        super().__init__()

        channels = [dim, 16, 32, 64, 32, 16, dim]
        conv_list = []
        for i in range(len(channels)-1):
            conv_list.append(
                Conv4Imitation(
                    channels[i],
                    channels[i+1],
                    kernel_size=kernel_size,
                    dropout=dropout,
                    dilation=2**i,
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
