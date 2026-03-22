# Copyright (c) 2025-2026 Hannes Göök
# MIT License - PidraQRL
# https://github.com/hannesgook/pidraqrl

import numpy as np
import torch

class ReplayBuffer:
    def __init__(self, obs_dim=2, action_dim=3, capacity=100_000):
        self.capacity = capacity
        self.ptr = 0 # write pointer
        self.size = 0 # current fill level

        # Pre-allocate arrays
        self.obs = np.zeros((capacity, obs_dim), dtype=np.float32)
        self.actions = np.zeros((capacity, action_dim), dtype=np.float32)
        self.rewards = np.zeros((capacity, 1), dtype=np.float32)
        self.next_obs = np.zeros((capacity, obs_dim), dtype=np.float32)
        self.dones = np.zeros((capacity, 1), dtype=np.float32)

    def add(self, obs, action, reward, next_obs, done):
        self.obs[self.ptr] = obs
        self.actions[self.ptr] = action
        self.rewards[self.ptr] = reward
        self.next_obs[self.ptr] = next_obs
        self.dones[self.ptr] = float(done)

        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size=256, device="cpu"):
        idx = np.random.randint(0, self.size, size=batch_size)

        def t(arr):
            return torch.FloatTensor(arr[idx]).to(device)

        return {
            "obs":      t(self.obs),
            "actions":  t(self.actions),
            "rewards":  t(self.rewards),
            "next_obs": t(self.next_obs),
            "dones":    t(self.dones),
        }

    def __len__(self):
        return self.size