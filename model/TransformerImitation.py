from re import S
import torch
from torch import nn
import torch.nn.functional as F
from torch import Tensor
import math


class PositionalEncoding(nn.Module):
    def __init__(
        self,
        d_model: int,
        dropout: float = 0.1,
        max_len: int = 5000,
    ):
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
            # nhead=8,
            nhead=1,
            # dim_feedforward=2048,
            dim_feedforward=256,
            # batch_first=True,
        )
        self.transformer_encoder = nn.TransformerEncoder(
            encoder_layer=transformer_encoder_layer,
            # num_layers=2,
            num_layers=1,
        )
        self.linear = nn.Linear(dim, dim)
        self.mask = None

    def _generate_square_subsequent_mask(self, sz: int) -> Tensor:
        r"""Generate a square mask for the sequence. The masked positions are filled with float('-inf').
            Unmasked positions are filled with float(0.0).
        """
        return torch.triu(torch.full((sz, sz), float('-inf')), diagonal=1)

    def forward(self, x: Tensor) -> Tensor:
        """
        Args:
            x: Tensor, shape [batch_size, seq_len, embedding_dim]
        """
        if self.mask is None or self.mask.shape[0] != x.shape[1]:
            self.mask = self._generate_square_subsequent_mask(x.shape[1])
            self.mask = self.mask.to(x.device)
        x = self.pos_encoder(x)
        x = x.permute(1, 0, 2) # (time, batch, dim)
        y = self.transformer_encoder(x, mask=self.mask)
        y = y.permute(1, 0, 2) # (batch, time, dim)
        y = self.linear(y)

        return y
