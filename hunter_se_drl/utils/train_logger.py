#!/usr/bin/env python3
"""
drl_agent 패키지와 동일한 형식으로 학습 로그를 출력하는 SB3 커스텀 콜백.
"""
import time
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

BORDER = "+" + "-" * 80 + "+"


def print_training_header(algo: str, file_name: str, save_dirs: list, state_dim: int, action_dim: int):
    """학습 시작 시 drl_agent 스타일 헤더 출력."""
    print(BORDER)
    print(f"| {algo} Agent Training")
    print(f"| File name: {file_name}")
    print(BORDER)
    print("| Results will be saved in:")
    for d in save_dirs:
        print(f"|  {d}")
    print(BORDER)
    print("| Environment")
    print(BORDER)
    print(f"| State Dim:  {state_dim}")
    print(f"| Action Dim: {action_dim}")
    print(BORDER)
    print()


class DRLLogCallback(BaseCallback):
    """
    에피소드 종료마다 drl_agent 형식으로 로그를 출력하고,
    eval_freq 스텝마다 최근 에피소드 통계를 출력한다.

    Args:
        algo:       알고리즘 이름 (예: "PPO", "SAC")
        eval_freq:  몇 스텝마다 통계 블록을 출력할지 (0이면 비활성)
        verbose:    0이면 SB3 기본 출력 억제
    """

    def __init__(self, algo: str = "RL", eval_freq: int = 10000, verbose: int = 0):
        super().__init__(verbose=verbose)
        self.algo = algo
        self.eval_freq = eval_freq

        self._ep_num = 0
        self._recent_rewards: list = []
        self._start_time = None
        self._epoch = 1
        self._last_eval_step = 0

    def _on_training_start(self) -> None:
        self._start_time = time.time()

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            if "episode" in info:
                self._ep_num += 1
                ep_reward = float(info["episode"]["r"])
                ep_len    = int(info["episode"]["l"])
                self._recent_rewards.append(ep_reward)

                print(
                    f"Total T: {self.num_timesteps:>8}  "
                    f"Episode Num: {self._ep_num:>5}  "
                    f"Episode T: {ep_len:>5}  "
                    f"Reward: {ep_reward:>10.3f}"
                )

        # eval_freq 스텝마다 통계 블록 출력
        if (
            self.eval_freq > 0
            and self.num_timesteps - self._last_eval_step >= self.eval_freq
            and len(self._recent_rewards) > 0
        ):
            self._print_eval_block()
            self._last_eval_step = self.num_timesteps
            self._recent_rewards = []
            self._epoch += 1

        return True

    def _print_eval_block(self):
        elapsed_min = round((time.time() - self._start_time) / 60.0, 2)
        rewards = np.array(self._recent_rewards)
        avg = rewards.mean()
        std = rewards.std()

        print(BORDER)
        print(f"| {self.algo} Evaluation at epoch: {self._epoch}")
        print(f"| Total time passed: {elapsed_min} min(s)")
        print(f"| Average reward over {len(rewards)} episodes: {avg:.3f} (+/- {std:.3f})")
        print(BORDER)
        print()
