#!/usr/bin/env python3
#
# reward.py
#
# ============================================================
# [역할]
# Hunter SE 전용 보상 함수.
# autodrive_env.py의 step_callback에서 매 스텝마다 호출된다.
# 보상 함수를 별도 파일로 분리하여 실험적 조정이 용이하도록 한다.
# ============================================================
#
# ============================================================
# [구현 내용]
# ============================================================
#
# def compute_reward(
#     dist_prev,       : 이전 스텝에서 목표까지 거리 (m)
#     dist_curr,       : 현재 스텝에서 목표까지 거리 (m)
#     min_lidar_dist,  : 현재 스텝의 LiDAR 최솟값 (m) - 가장 가까운 장애물
#     collision,       : 충돌 여부 (bool) - Unity "V1 Collision"
#     target_reached,  : 목표 도달 여부 (bool)
#     linear_vel,      : 실제 실행된 선형 속도 (m/s)
# ) → float:
#
#   [보상 구성]
#
#   1. 거리 기반 보상 (매 스텝)
#      reward += -(dist_curr - dist_prev)
#      → 목표에 가까워지면 양의 보상, 멀어지면 음의 보상
#
#   2. 목표 도달 보상 (에피소드 종료)
#      if target_reached:
#          reward += REWARD_TARGET   # 예: +10.0
#
#   3. 충돌 패널티 (에피소드 종료)
#      if collision:
#          reward += REWARD_COLLISION  # 예: -1.0
#
#   4. 생존 보너스 (매 스텝, 선택)
#      reward += REWARD_STEP  # 예: +0.01
#      → 에피소드를 오래 유지하도록 유도
#
#   5. 후진 패널티 (Hunter SE 전용, 선택)
#      if linear_vel < 0:
#          reward += REWARD_BACKWARD  # 예: -0.05
#      → 불필요한 후진 억제
#
#   [보상 상수]
#   REWARD_TARGET    = +10.0
#   REWARD_COLLISION = -1.0
#   REWARD_STEP      = +0.01
#   REWARD_BACKWARD  = -0.05
#
#   [주의]
#   상수값은 학습 결과를 보며 조정이 필요하다.
#   거리 기반 보상의 스케일이 너무 크면 목표 도달/충돌 보상이 묻힐 수 있다.
