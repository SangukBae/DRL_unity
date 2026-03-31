#!/usr/bin/env python3
"""
SB3 PPO로 Hunter SE 장애물 회피 + 목표 도달 학습.

실행:
    cd /autodrive/hunter_se_drl
    python train/train_ppo.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback

from envs.hunter_se_env import HunterSEEnv
from models.custom_policy import PPO_POLICY_KWARGS
from utils.file_manager import ensure_dir, load_yaml

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def main():
    # 1. config 로드
    cfg = load_yaml(os.path.join(CONFIG_DIR, "train_ppo_config.yaml"))["train"]

    total_timesteps = cfg["total_timesteps"]
    learning_rate   = cfg["learning_rate"]
    n_steps         = cfg["n_steps"]
    batch_size      = cfg["batch_size"]
    n_epochs        = cfg["n_epochs"]
    gamma           = cfg["gamma"]
    gae_lambda      = cfg["gae_lambda"]
    clip_range      = cfg["clip_range"]
    ent_coef        = cfg["ent_coef"]
    vf_coef         = cfg["vf_coef"]
    max_grad_norm   = cfg["max_grad_norm"]
    save_freq       = cfg["save_freq"]
    model_save_dir  = cfg["model_save_dir"]
    log_dir         = cfg["log_dir"]

    ensure_dir(model_save_dir)
    ensure_dir(log_dir)

    # 2. 환경 생성
    env = HunterSEEnv(config_path=os.path.join(CONFIG_DIR, "env_config.yaml"))

    # 3. 모델 생성
    model = PPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=learning_rate,
        n_steps=n_steps,
        batch_size=batch_size,
        n_epochs=n_epochs,
        gamma=gamma,
        gae_lambda=gae_lambda,
        clip_range=clip_range,
        ent_coef=ent_coef,
        vf_coef=vf_coef,
        max_grad_norm=max_grad_norm,
        policy_kwargs=PPO_POLICY_KWARGS,
        tensorboard_log=log_dir,
        verbose=1,
    )

    # 4. 콜백
    checkpoint_cb = CheckpointCallback(
        save_freq=save_freq,
        save_path=model_save_dir,
        name_prefix="ppo_hunter_se",
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
