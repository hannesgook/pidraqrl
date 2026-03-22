# Copyright (c) 2025-2026 Hannes Göök
# MIT License - PidraQRL
# https://github.com/hannesgook/pidraqrl

import torch
import torch.nn as nn

class QNetwork(nn.Module):
    def __init__(self, obs_dim=2, action_dim=3, hidden_dim=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim + action_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
        )
        self._init_weights()

    def _init_weights(self):
        for layer in self.net:
            if isinstance(layer, nn.Linear):
                nn.init.orthogonal_(layer.weight, gain=1.0)
                nn.init.zeros_(layer.bias)

    def forward(self, obs, action):
        x = torch.cat([obs, action], dim=-1)
        return self.net(x)

class TwinCritic(nn.Module):
    def __init__(self, obs_dim=2, action_dim=3, hidden_dim=128):
        super().__init__()
        self.q1 = QNetwork(obs_dim, action_dim, hidden_dim)
        self.q2 = QNetwork(obs_dim, action_dim, hidden_dim)

    def forward(self, obs, action):
        return self.q1(obs, action), self.q2(obs, action)

    def min_q(self, obs, action):
        q1, q2 = self.forward(obs, action)
        return torch.min(q1, q2)