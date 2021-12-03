from torch import nn


class CNNImitation(nn.Module):
    def __init__(
        self,
        dim: int,
        dropout: float = 0.0,
    ):
        super().__init__()

        channels = [dim, 16, 32, 16, dim]
        conv_list = []
        for i in range(len(channels)-1):
            conv_list.append(
                nn.Conv1d(
                    channels[i],
                    channels[i+1],
                    kernel_size=3,
                    stride=1,
                    padding=1,
                    padding_mode='replicate',
                ))
            conv_list.append(nn.Dropout(dropout))
            conv_list.append(nn.ReLU())
        self.conv = nn.Sequential(*conv_list)
        self.linear = nn.Linear(dim, dim)

    def forward(self, x):
        x = x.permute(0, 2, 1) # (batch, dim, time)
        x = self.conv(x)
        x = x.permute(0, 2, 1) # (batch, time, dim)
        y = self.linear(x)

        return y
