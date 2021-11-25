import torch
from torch import nn
import torch.nn.functional as F
from torch import Tensor
import math


class PositionalEncoding(nn.Module):
    def __init__(self, d_model: int, dropout: float = 0.1, max_len: int = 5000):
        super().__init__()
        self.dropout = nn.Dropout(p=dropout)

        position = torch.arange(max_len).unsqueeze(1)
        div_term = torch.exp(
            torch.arange(0, d_model, 2) * (-math.log(10000.0) / d_model))
        pe = torch.zeros(max_len, d_model)
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        pe = pe.unsqueeze(0)
        self.register_buffer('pe', pe)
        # print(pe.shape)

    def forward(self, x: Tensor) -> Tensor:
        """
        Args:
            x: Tensor, shape [batch_size, seq_len, embedding_dim]
        """
        x = x + self.pe[:, :x.shape[1]]
        return self.dropout(x)


class TransformerImitation(nn.Module):
    def __init__(self, dim: int):
        super().__init__()

        self.pos_encoder = PositionalEncoding(d_model=dim)
        transformer_encoder_layer = nn.TransformerEncoderLayer(
            d_model=dim,
            nhead=1,
            dim_feedforward=256,
            # batch_first=True,
        )
        self.transformer_encoder = nn.TransformerEncoder(
            encoder_layer=transformer_encoder_layer,
            num_layers=2,
        )

    def forward(self, x: Tensor, mask: Tensor = None) -> Tensor:
        """
        Args:
            x: Tensor, shape [batch_size, seq_len, embedding_dim]
            mask: Tensor, shape [batch_size, seq_len]
        """
        x = self.pos_encoder(x)
        x = x.permute(1, 0, 2) # [seq_len, batch_size, embedding_dim]
        # print(x)
        y = self.transformer_encoder(x, mask=mask)
        # print(y)
        y = y.permute(1, 0, 2) # [batch_size, seq_len, embedding_dim]

        return y
