#!/usr/bin/env python3
"""
SB3 PPO로 Hunter SE 장애물 회피 + 목표 도달 학습.

실행:
    cd /autodrive/hunter_se_drl
    python train/train_ppo.py
"""
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList

from envs.hunter_se_env import HunterSEEnv
from models.custom_policy import PPO_POLICY_KWARGS
from utils.file_manager import ensure_dir, load_yaml
from utils.train_logger import DRLLogCallback, print_training_header

CONFIG_DIR  = os.path.join(os.path.dirname(__file__), "..", "config")
RVIZ_CONFIG = "/autodrive/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_ros2/autodrive_hunter_se/rviz/simulator.rviz"
RVIZ_SETUP  = "/autodrive/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_ros2/install/setup.bash"


def _preflight_check() -> bool:
    """학습 시작 전 필수 항목을 점검하고 결과를 출력한다. FAIL 항목이 있으면 False 반환."""
    import importlib
    import socket as _socket

    print("=" * 60)
    print("  학습 준비 상태 점검")
    print("=" * 60)

    fail = False

    # 1. Config 파일
    for name in ("env_config", "train_ppo_config"):
        path = os.path.join(CONFIG_DIR, f"{name}.yaml")
        if os.path.isfile(path):
            print(f"  [OK]   config/{name}.yaml")
        else:
            print(f"  [FAIL] config/{name}.yaml 없음  →  {path}")
            fail = True

    # 2. Python 패키지
    for pkg, label in [
        ("torch",             "PyTorch"),
        ("stable_baselines3", "Stable-Baselines3"),
        ("gymnasium",         "Gymnasium"),
        ("eventlet",          "eventlet"),
        ("numpy",             "numpy"),
    ]:
        try:
            mod = importlib.import_module(pkg)
            ver = getattr(mod, "__version__", "?")
            print(f"  [OK]   {label} {ver}")
        except ImportError:
            print(f"  [FAIL] {label} import 실패  →  pip install {pkg}")
            fail = True

    # CUDA
    try:
        import torch
        if torch.cuda.is_available():
            print(f"  [OK]   CUDA  ({torch.cuda.get_device_name(0)})")
        else:
            print(f"  [WARN] CUDA 없음  →  CPU 학습 (느릴 수 있음)")
    except Exception:
        pass

    # 3. ROS2 / RViz2
    try:
        importlib.import_module("rclpy")
        print("  [OK]   rclpy  (RViz2 시각화 활성화)")
    except ImportError:
        print("  [WARN] rclpy 없음  →  RViz2 시각화 비활성화 (학습은 정상)")

    for label, path in [
        ("RViz setup.bash", RVIZ_SETUP),
        ("RViz .rviz config", RVIZ_CONFIG),
    ]:
        if os.path.isfile(path):
            print(f"  [OK]   {label}")
        else:
            print(f"  [WARN] {label} 없음  →  RViz2 자동 실행 불가")

    # 4. 포트 4567 점유 여부
    try:
        with _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM) as s:
            s.settimeout(0.3)
            if s.connect_ex(("127.0.0.1", 4567)) == 0:
                print("  [FAIL] 포트 4567 이미 사용 중  →  다른 프로세스를 종료하세요")
                fail = True
            else:
                print("  [OK]   포트 4567 사용 가능")
    except Exception:
        print("  [OK]   포트 4567 사용 가능")

    print("-" * 60)
    if fail:
        print("  [FAIL] 항목을 해결한 후 다시 실행하세요.")
    else:
        print("  모든 필수 항목 통과.  Unity Play 후 학습이 시작됩니다.")
    print("=" * 60)
    print()
    return not fail


def _launch_rviz():
    """RViz2를 백그라운드로 실행한다. 실패해도 학습을 중단하지 않는다."""
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source '{RVIZ_SETUP}' && "
        f"ros2 run rviz2 rviz2 -d '{RVIZ_CONFIG}'"
    )
    log_path = "/tmp/hunter_se_rviz2_ppo.log"
    try:
        env = os.environ.copy()
        # OpenCV 내장 Qt 플러그인이 시스템 Qt를 덮어쓰는 문제 방지
        env["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/opt/ros/humble/lib/x86_64-linux-gnu/qt5/plugins"
        with open(log_path, "ab") as log_fp:
            proc = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=log_fp,
                stderr=log_fp,
                env=env,
            )

        time.sleep(1.0)
        exit_code = proc.poll()
        if exit_code is not None:
            try:
                with open(log_path, "r", encoding="utf-8", errors="ignore") as fp:
                    lines = fp.readlines()[-20:]
                tail = "".join(lines).strip()
            except Exception:
                tail = ""

            print(f"[RViz2] 실행 직후 종료됨 (exit={exit_code})")
            print(f"[RViz2] 로그: {log_path}")
            if tail:
                print("[RViz2] 최근 로그:")
                print(tail)
            return None

        print(f"[RViz2] 시작됨 (PID {proc.pid})")
        print(f"[RViz2] 로그: {log_path}")
        return proc
    except Exception as e:
        print(f"[RViz2] 실행 실패 (학습은 계속됩니다): {e}")
        return None


def main():
    if not _preflight_check():
        sys.exit(1)

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

    # 2. 환경 생성 + RViz2 자동 실행
    env = HunterSEEnv(config_path=os.path.join(CONFIG_DIR, "env_config.yaml"))
    rviz_proc = _launch_rviz()

    # 3. 씬 제어 검증 (리셋 + 주행 가능 여부 확인)
    if not env.smoke_test():
        env.close()
        if rviz_proc is not None:
            rviz_proc.terminate()
        sys.exit(1)

    # 4. 헤더 출력
    env_obs = env.observation_space.shape[0]
    env_act = env.action_space.shape[0]
    print_training_header(
        algo="PPO",
        file_name="ppo_hunter_se",
        save_dirs=[model_save_dir, log_dir],
        state_dim=env_obs,
        action_dim=env_act,
    )

    # 5. 모델 생성
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
        verbose=0,
    )

    # 6. 콜백
    checkpoint_cb = CheckpointCallback(
        save_freq=save_freq,
        save_path=model_save_dir,
        name_prefix="ppo_hunter_se",
    )
    log_cb = DRLLogCallback(algo="PPO", eval_freq=save_freq)

    # 7. 학습
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=CallbackList([checkpoint_cb, log_cb]),
            reset_num_timesteps=True,
        )

        # 6. 최종 모델 저장
        final_path = os.path.join(model_save_dir, "final_model")
        model.save(final_path)
        print(f"\n학습 완료. 최종 모델 저장: {final_path}")
    finally:
        env.close()
        if rviz_proc is not None:
            rviz_proc.terminate()
            print("[RViz2] 종료됨")


if __name__ == "__main__":
    main()
