#!/usr/bin/env python3
#
# sac_agent.py
#
# ============================================================
# [역할]
# SAC(Soft Actor-Critic) 알고리즘의 신경망 정의 및 업데이트 로직.
# Gazebo 의존성이 전혀 없는 순수 PyTorch 코드이므로
# drl_agent 패키지의 sac_agent.py를 그대로 복사한다.
# ============================================================
#
# ============================================================
# [구현 내용] - drl_agent/scripts/policy/sac_agent.py 그대로 복사
# ============================================================
#
# class Actor(nn.Module):
#   - 입력: state
#   - 출력: mean, log_std (가우시안 분포 파라미터)
#   - sample(): Reparameterization trick + Tanh → action ∈ [-1,1], log_prob 반환
#
# class Critic(nn.Module):
#   - Twin Q-networks (Q1, Q2)
#   - 입력: (state, action)
#   - 출력: Q1, Q2 값
#
# class SACAgent:
#   - Actor, Critic, TargetCritic 초기화
#   - select_action(state, evaluate=False): 학습 시 샘플링, 테스트 시 mean 반환
#   - train(batch): Actor/Critic 손실 계산 및 역전파, 엔트로피 온도 자동 튜닝
#   - save(path) / load(path): 모델 저장/로드
