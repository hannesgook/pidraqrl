# Copyright (c) 2025-2026 Hannes Göök
# MIT License - PidraQRL
# https://github.com/hannesgook/pidraqrl

# sac_agent.py - SAC with 6-dim obs, 3-dim action

import torch, torch.nn.functional as F, torch.optim as optim
import numpy as np, copy

from rl.actor        import Actor
from rl.critic       import TwinCritic
from rl.replay_buffer import ReplayBuffer

OBS_DIM = 6   # [roll, rate, target, error, angleKp_norm, rateKp_norm]
ACTION_DIM = 3   # [stickRoll, angleKp_delta, rateKp_delta]


class SACAgent:
    def __init__(self, obs_dim=OBS_DIM, action_dim=ACTION_DIM, hidden_dim=128, lr=3e-4, gamma=0.99, tau=0.005, alpha_init=0.2, batch_size=256, buffer_size=100_000, device="cpu"):
        self.device = device
        self.gamma = gamma
        self.tau = tau
        self.batch_size = batch_size
        self.action_dim = action_dim

        self.actor = Actor(obs_dim, action_dim, hidden_dim).to(device)
        self.critic = TwinCritic(obs_dim, action_dim, hidden_dim).to(device)
        self.critic_target = copy.deepcopy(self.critic).to(device)
        for p in self.critic_target.parameters():
            p.requires_grad = False

        self.actor_opt = optim.Adam(self.actor.parameters(),  lr=lr)
        self.critic_opt = optim.Adam(self.critic.parameters(), lr=lr)

        self.target_entropy = -float(action_dim)
        self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
        self.alpha_opt = optim.Adam([self.log_alpha], lr=lr)
        self.alpha = alpha_init

        self.buffer = ReplayBuffer(obs_dim, action_dim, buffer_size)
        self.total_updates = 0
        self.metrics = {"critic_loss": [], "actor_loss": [], "alpha": [], "entropy": []}

    def select_action(self, obs: np.ndarray, deterministic=False) -> np.ndarray:
        obs_t = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
        if deterministic:
            action = self.actor.get_action_deterministic(obs_t)
        else:
            action, _, _ = self.actor(obs_t)
        return action.cpu().detach().numpy()[0]

    def store(self, obs, action, reward, next_obs, done):
        self.buffer.add(obs, action, reward, next_obs, done)

    def update(self):
        if len(self.buffer) < self.batch_size:
            return None
        b = self.buffer.sample(self.batch_size, self.device)
        obs, actions, rewards, next_obs, dones = b["obs"], b["actions"], b["rewards"], b["next_obs"], b["dones"]

        with torch.no_grad():
            na, nlp, _ = self.actor(next_obs)
            q1t, q2t = self.critic_target(next_obs, na)
            td_target = rewards + self.gamma * (1 - dones) * (torch.min(q1t, q2t) - self.alpha * nlp)

        q1, q2 = self.critic(obs, actions)
        critic_loss = F.mse_loss(q1, td_target) + F.mse_loss(q2, td_target)
        self.critic_opt.zero_grad(); critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_opt.step()

        new_a, lp, _ = self.actor(obs)
        actor_loss = (self.alpha * lp - self.critic.min_q(obs, new_a)).mean()
        self.actor_opt.zero_grad(); actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_opt.step()

        alpha_loss = -(self.log_alpha * (lp + self.target_entropy).detach()).mean()
        self.alpha_opt.zero_grad(); alpha_loss.backward(); self.alpha_opt.step()
        self.alpha = self.log_alpha.exp().item()

        for p, tp in zip(self.critic.parameters(), self.critic_target.parameters()):
            tp.data.copy_(self.tau * p.data + (1 - self.tau) * tp.data)

        self.total_updates += 1
        result = {"critic_loss": critic_loss.item(), "actor_loss": actor_loss.item(), "alpha": self.alpha, "entropy": -lp.mean().item()}
        for k, v in result.items():
            self.metrics[k].append(v)
        return result

    def save(self, path="checkpoints/sac_checkpoint.pt"):
        import os; os.makedirs(os.path.dirname(path), exist_ok=True)
        torch.save({"actor": self.actor.state_dict(), "critic": self.critic.state_dict(), "critic_target": self.critic_target.state_dict(), "log_alpha": self.log_alpha, "total_updates": self.total_updates}, path)

    def load(self, path="checkpoints/sac_checkpoint.pt"):
        ckpt = torch.load(path, map_location=self.device)
        self.actor.load_state_dict(ckpt["actor"])
        self.critic.load_state_dict(ckpt["critic"])
        self.critic_target.load_state_dict(ckpt["critic_target"])
        self.log_alpha = ckpt["log_alpha"]
        self.alpha = self.log_alpha.exp().item()
        self.total_updates = ckpt["total_updates"]