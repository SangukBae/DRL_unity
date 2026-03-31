#!/usr/bin/env python3
"""
커스텀 Policy 설정.

초기에는 방법 A(policy_kwargs)로 시작한다.
성능 개선이 필요하면 방법 B(ActorCriticPolicy 상속)로 전환한다.
"""
import torch.nn as nn

# ── 방법 A: policy_kwargs로 네트워크 크기만 변경 ──────────────────────────

SAC_POLICY_KWARGS = dict(
    net_arch=[256, 256],
    activation_fn=nn.ReLU,
)

PPO_POLICY_KWARGS = dict(
    net_arch=dict(pi=[256, 256], vf=[256, 256]),
    activation_fn=nn.ReLU,
)

# ── 방법 B: 커스텀 특징 추출기 (LiDAR → 1D CNN) ─────────────────────────
# 향후 성능 개선 시 아래 클래스를 활성화하고 policy_kwargs에 추가한다.
#
# import gymnasium as gym
# import torch
# import torch.nn as nn
# from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
#
# class LiDARCNNExtractor(BaseFeaturesExtractor):
#     """
#     LiDAR 부분(lidar_dim)은 1D Conv로, 나머지(dist, angle)는 MLP로 처리.
#     두 특징을 합산하여 출력한다.
#     """
#     def __init__(self, observation_space: gym.spaces.Box, lidar_dim: int = 360,
#                  cnn_out: int = 128):
#         features_dim = cnn_out + 64
#         super().__init__(observation_space, features_dim=features_dim)
#
#         self.lidar_cnn = nn.Sequential(
#             nn.Conv1d(1, 32, kernel_size=5, stride=2, padding=2),
#             nn.ReLU(),
#             nn.Conv1d(32, 64, kernel_size=5, stride=2, padding=2),
#             nn.ReLU(),
#             nn.Flatten(),
#             nn.LazyLinear(cnn_out),
#             nn.ReLU(),
#         )
#         self.state_mlp = nn.Sequential(
#             nn.Linear(2, 64),
#             nn.ReLU(),
#         )
#         self._lidar_dim = lidar_dim
#
#     def forward(self, observations: torch.Tensor) -> torch.Tensor:
#         lidar = observations[:, :self._lidar_dim].unsqueeze(1)
#         state = observations[:, self._lidar_dim:]
#         return torch.cat([self.lidar_cnn(lidar), self.state_mlp(state)], dim=1)
