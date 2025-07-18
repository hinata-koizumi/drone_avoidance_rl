import torch
import torch.nn as nn

class Actor(nn.Module):
    def __init__(self, latent_size, action_size):
        super().__init__()
        # TODO: 方策ネットワークの定義
        pass

    def forward(self, latent):
        # TODO: アクション分布の出力
        pass

class Critic(nn.Module):
    def __init__(self, latent_size):
        super().__init__()
        # TODO: 価値関数ネットワークの定義
        pass

    def forward(self, latent):
        # TODO: 価値の出力
        pass 