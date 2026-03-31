#!/usr/bin/env python3
#
# buffer.py
#
# ============================================================
# [역할]
# 오프폴리시 학습(SAC/TQC)에 사용하는 리플레이 버퍼.
# Gazebo 의존성이 전혀 없는 순수 Python/NumPy 코드이므로
# drl_agent 패키지의 buffer.py를 그대로 복사한다.
# ============================================================
#
# ============================================================
# [구현 내용] - drl_agent/scripts/policy/buffer.py 그대로 복사
# ============================================================
#
# class ReplayBuffer (또는 LAP):
#
#   __init__(state_dim, action_dim, max_size, device):
#     - state, action, next_state, reward, not_done 배열 초기화
#     - ptr (현재 저장 위치), size (현재 저장 수) 관리
#
#   add(state, action, next_state, reward, done):
#     - 순환 버퍼 방식으로 데이터 저장 (ptr % max_size)
#
#   sample(batch_size) → (states, actions, next_states, rewards, not_dones):
#     - 균등 샘플링 (우선순위 샘플링은 LAP 클래스에서 지원)
#     - GPU 텐서로 변환하여 반환
#
#   update_priority(priority):  [선택]
#     - PER(Prioritized Experience Replay) 우선순위 갱신
