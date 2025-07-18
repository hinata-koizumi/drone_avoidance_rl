import torch
import torch.nn as nn

class WorldModel(nn.Module):
    def __init__(self, obs_shape, action_size, latent_size=128):
        super().__init__()
        # TODO: エンコーダ・リカレント・デコーダ等の定義
        pass

    def forward(self, obs, action):
        """観測と行動から次状態の潜在表現を予測"""
        # TODO: forward計算
        pass

    def imagine(self, latent, policy, horizon):
        """方策に従い潜在空間上で軌道を生成"""
        # TODO: imagination rollouts
        pass 