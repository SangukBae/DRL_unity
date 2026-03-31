#!/usr/bin/env python3
"""
SB3 SAC로 Hunter SE 장애물 회피 + 목표 도달 학습.

실행:
    cd /autodrive/hunter_se_drl
    python train/train_sac.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

from envs.hunter_se_env import HunterSEEnv
from models.custom_policy import SAC_POLICY_KWARGS
from utils.file_manager import ensure_dir, load_yaml

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def main():
    # 1. config 로드
    cfg = load_yaml(os.path.join(CONFIG_DIR, "train_sac_config.yaml"))["train"]

    total_timesteps  = cfg["total_timesteps"]
    learning_starts  = cfg["learning_starts"]
    learning_rate    = cfg["learning_rate"]
    buffer_size      = cfg["buffer_size"]
    batch_size       = cfg["batch_size"]
    tau              = cfg["tau"]
    gamma            = cfg["gamma"]
    train_freq       = cfg["train_freq"]
    gradient_steps   = cfg["gradient_steps"]
    ent_coef         = cfg["ent_coef"]
    target_entropy   = cfg["target_entropy"]
    save_freq        = cfg["save_freq"]
    model_save_dir   = cfg["model_save_dir"]
    log_dir          = cfg["log_dir"]

    ensure_dir(model_save_dir)
    ensure_dir(log_dir)

    # 2. 환경 생성
    env = HunterSEEnv(config_path=os.path.join(CONFIG_DIR, "env_config.yaml"))

    # 3. 모델 생성
    model = SAC(
        policy="MlpPolicy",
        env=env,
        learning_rate=learning_rate,
        buffer_size=buffer_size,
        batch_size=batch_size,
        learning_starts=learning_starts,
        tau=tau,
        gamma=gamma,
        train_freq=train_freq,
        gradient_steps=gradient_steps,
        ent_coef=ent_coef,
        target_entropy=target_entropy,
        policy_kwargs=SAC_POLICY_KWARGS,
        tensorboard_log=log_dir,
        verbose=1,
    )

    # 4. 콜백
    checkpoint_cb = CheckpointCallback(
        save_freq=save_freq,
        save_path=model_save_dir,
        name_prefix="sac_hunter_se",
    )

    # 5. 학습
    model.learn(
        total_timesteps=total_timesteps,
        callback=checkpoint_cb,
        reset_num_timesteps=True,
    )

    # 6. 최종 모델 저장
    final_path = os.path.join(model_save_dir, "final_model")
    model.save(final_path)
    print(f"\n학습 완료. 최종 모델 저장: {final_path}")

    env.close()


if __name__ == "__main__":
    main()
