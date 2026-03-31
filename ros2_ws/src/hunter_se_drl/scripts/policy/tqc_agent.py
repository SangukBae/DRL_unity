#!/usr/bin/env python3
#
# tqc_agent.py
#
# ============================================================
# [역할]
# TQC(Truncated Quantile Critics) 알고리즘의 신경망 정의 및 업데이트 로직.
# Gazebo 의존성이 전혀 없는 순수 PyTorch 코드이므로
# drl_agent 패키지의 tqc_agent.py를 그대로 복사한다.
# ============================================================
#
# ============================================================
# [구현 내용] - drl_agent/scripts/policy/tqc_agent.py 그대로 복사
# ============================================================
#
# class Actor(nn.Module):
#   - 입력: state
#   - 출력: mean, log_std → action ∈ [-1,1], log_prob 반환
#
# class Critic(nn.Module):
#   - n_critics개의 분위수 회귀 네트워크
#   - 입력: (state, action)
#   - 출력: (batch_size, n_critics, n_quantiles) 텐서
#
# def quantile_huber_loss(current_quantiles, target_quantiles):
#   - Quantile Huber Loss 계산
#   - 분위수: τ = (i + 0.5) / n_quantiles
#
# class TQCAgent:
#   - Actor, Critic, TargetCritic 초기화
#   - select_action(state, evaluate=False)
#   - train(batch): 상위 분위수 제거(Truncation) 후 손실 계산 및 역전파
#   - save(path) / load(path)
