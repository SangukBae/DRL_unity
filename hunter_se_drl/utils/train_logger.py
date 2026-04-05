#!/usr/bin/env python3
"""
drl_agent 패키지와 동일한 형식으로 학습 로그를 출력하는 SB3 커스텀 콜백.
"""
import sys
import shutil
import time
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

BORDER = "+" + "-" * 80 + "+"

# ── GPU 모니터링 초기화 ───────────────────────────────────────────────────────
try:
    import pynvml
    pynvml.nvmlInit()
    _NVML_AVAILABLE = True
except Exception:
    _NVML_AVAILABLE = False

try:
    import torch
    _TORCH_AVAILABLE = True
except Exception:
    _TORCH_AVAILABLE = False


def _get_gpu_stats() -> str:
    """GPU 사용량 문자열 반환. pynvml 우선, 없으면 torch.cuda 폴백."""
    if _NVML_AVAILABLE:
        try:
            handle = pynvml.nvmlDeviceGetHandleByIndex(0)
            util   = pynvml.nvmlDeviceGetUtilizationRates(handle)
            mem    = pynvml.nvmlDeviceGetMemoryInfo(handle)
            temp   = pynvml.nvmlDeviceGetTemperature(handle, pynvml.NVML_TEMPERATURE_GPU)
            mem_used  = mem.used  / 1024 ** 2
            mem_total = mem.total / 1024 ** 2
            return f"GPU {util.gpu:>3}% | Mem {mem_used:>6.0f}/{mem_total:.0f}MB | Temp {temp}°C"
        except Exception:
            pass

    if _TORCH_AVAILABLE and torch.cuda.is_available():
        try:
            used  = torch.cuda.memory_allocated(0) / 1024 ** 2
            total = torch.cuda.get_device_properties(0).total_memory / 1024 ** 2
            return f"GPU Mem {used:.0f}/{total:.0f}MB"
        except Exception:
            pass

    return "GPU N/A"


def print_training_header(algo: str, file_name: str, save_dirs: list, state_dim: int, action_dim: int):
    """학습 시작 시 drl_agent 스타일 헤더 출력."""
    gpu_info = ""
    if _NVML_AVAILABLE:
        try:
            handle = pynvml.nvmlDeviceGetHandleByIndex(0)
            name   = pynvml.nvmlDeviceGetName(handle)
            mem    = pynvml.nvmlDeviceGetMemoryInfo(handle)
            gpu_info = f"{name}  ({mem.total / 1024**2:.0f} MB)"
        except Exception:
            pass
    elif _TORCH_AVAILABLE and torch.cuda.is_available():
        gpu_info = torch.cuda.get_device_name(0)

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
    if gpu_info:
        print(f"| GPU: {gpu_info}")
        print(BORDER)
    print()


class DRLLogCallback(BaseCallback):
    """
    - 매 스텝: 보상 항목 + GPU 사용량을 같은 줄에 실시간 덮어쓰기
    - 에피소드 종료: 에피소드 요약 출력
    - eval_freq 스텝마다: 통계 블록 출력
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
        info = self.locals.get("infos", [{}])[0]

        # ── 실시간 상태 줄 (매 스텝 덮어쓰기) ──────────────────────────────
        prog     = info.get("rwc_progress",  0.0)
        fwd      = info.get("rwc_forward",   0.0)
        head     = info.get("rwc_heading",   0.0)
        curv     = info.get("rwc_curv_pen",  0.0)
        obs      = info.get("rwc_obstacle",  0.0)
        s_pen    = info.get("rwc_step_pen",  0.0)
        dist     = info.get("dist_to_goal",  0.0)
        gpu      = _get_gpu_stats()
        live = (
            f"T:{self.num_timesteps:>7} "
            f"P:{prog:+.2f} F:{fwd:+.2f} H:{head:+.2f} "
            f"C:{-curv:.2f} O:{-obs:.2f} "
            f"D:{dist:>4.1f}m {gpu}"
        )
        # 터미널 너비에 맞춰 자르고 \r로 덮어쓰기
        width = shutil.get_terminal_size((80, 20)).columns
        live = live[:width - 1].ljust(width - 1)
        sys.stdout.write(f"\r{live}")
        sys.stdout.flush()

        # ── 에피소드 종료 ────────────────────────────────────────────────────
        if "episode" in info:
            self._ep_num += 1
            ep_reward = float(info["episode"]["r"])
            ep_len    = int(info["episode"]["l"])
            self._recent_rewards.append(ep_reward)

            # 실시간 줄을 지우고 에피소드 요약을 새 줄에 출력
            sys.stdout.write("\r" + " " * (width - 1) + "\r")
            print(
                f"Total T: {self.num_timesteps:>8}  "
                f"Episode Num: {self._ep_num:>5}  "
                f"Episode T: {ep_len:>5}  "
                f"Reward: {ep_reward:>10.3f}"
            )

        # ── eval_freq 마다 통계 블록 ─────────────────────────────────────────
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
        print(f"| {self.algo} Training Statistics at epoch: {self._epoch}")
        print(f"| Total time passed: {elapsed_min} min(s)")
        print(f"| Average reward over {len(rewards)} episodes: {avg:.3f} (+/- {std:.3f})")
        print(f"| {_get_gpu_stats()}")
        print(BORDER)
        print()
