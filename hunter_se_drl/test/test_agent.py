#!/usr/bin/env python3
"""
학습된 SB3 모델로 AutoDRIVE Unity 시뮬레이터에서 테스트 에피소드 실행.

실행 예시:
    cd /autodrive/hunter_se_drl
    python test/test_agent.py --model_path models/saved/sac/final_model --algorithm sac
    python test/test_agent.py --model_path models/saved/ppo/final_model --algorithm ppo --num_episodes 20
"""
import argparse
import json
import os
import sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from stable_baselines3 import PPO, SAC

from envs.hunter_se_env import HunterSEEnv
from utils.file_manager import ensure_dir, save_json

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def parse_args():
    parser = argparse.ArgumentParser(description="Hunter SE 테스트 에이전트")
    parser.add_argument("--model_path",   type=str,  required=True,  help="모델 파일 경로")
    parser.add_argument("--algorithm",    type=str,  default="sac",  choices=["sac", "ppo"])
    parser.add_argument("--num_episodes", type=int,  default=10,     help="테스트 에피소드 수")
    parser.add_argument("--result_dir",   type=str,  default="results", help="결과 저장 디렉토리")
    return parser.parse_args()


def main():
    args = parse_args()

    # 1. 환경 생성
    env = HunterSEEnv(config_path=os.path.join(CONFIG_DIR, "env_config.yaml"))

    # 2. 모델 로드
    ModelClass = SAC if args.algorithm == "sac" else PPO
    model = ModelClass.load(args.model_path, env=env)
    print(f"모델 로드 완료: {args.model_path} ({args.algorithm.upper()})")

    # 3. 테스트 루프
    results = []
    for ep in range(args.num_episodes):
        obs, info = env.reset()
        episode_reward = 0.0
        step_count     = 0
        success        = False

        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward
            step_count     += 1

            if info.get("target_reached"):
                success = True

            if terminated or truncated:
                break

        result = {
            "episode":        ep + 1,
            "reward":         round(episode_reward, 3),
            "steps":          step_count,
            "success":        success,
            "collision":      info.get("collision", False),
        }
        results.append(result)
        print(f"에피소드 {ep+1:3d} | 보상: {episode_reward:7.2f} | "
              f"스텝: {step_count:4d} | {'성공' if success else '실패'}")

    # 4. 요약 출력
    total      = len(results)
    success_n  = sum(r["success"]  for r in results)
    avg_reward = sum(r["reward"]   for r in results) / total
    avg_steps  = sum(r["steps"]    for r in results) / total

    print("\n" + "=" * 50)
    print(f"  총 에피소드: {total}")
    print(f"  성공률:      {success_n}/{total} ({100 * success_n / total:.1f}%)")
    print(f"  평균 보상:   {avg_reward:.3f}")
    print(f"  평균 스텝:   {avg_steps:.1f}")
    print("=" * 50)

    # 5. 결과 저장
    ensure_dir(args.result_dir)
    timestamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_path = os.path.join(args.result_dir, f"test_{args.algorithm}_{timestamp}.json")
    save_json(result_path, {
        "model_path":    args.model_path,
        "algorithm":     args.algorithm,
        "num_episodes":  total,
        "success_rate":  success_n / total,
        "avg_reward":    avg_reward,
        "avg_steps":     avg_steps,
        "episodes":      results,
    })
    print(f"결과 저장: {result_path}")

    env.close()


if __name__ == "__main__":
    main()
