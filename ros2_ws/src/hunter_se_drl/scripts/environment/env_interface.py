#!/usr/bin/env python3
#
# env_interface.py
#
# ============================================================
# [역할]
# train_*.py(에이전트 측)에서 autodrive_env.py(환경 서버)의
# ROS2 서비스를 호출하는 클라이언트 래퍼.
#
# drl_agent 패키지의 environment_interface.py를 그대로 복사한다.
# autodrive_env.py가 동일한 서비스 인터페이스(/step, /reset 등)를
# 제공하므로 수정 없이 재사용 가능하다.
# ============================================================
#
# ============================================================
# [구현 내용]
# ============================================================
# drl_agent/scripts/environment/environment_interface.py를 그대로 복사.
#
# class EnvInterface(Node):
#
#   __init__:
#     - ROS2 서비스 클라이언트 5개 생성
#       /step                (Step.srv)
#       /reset               (Reset.srv)
#       /get_dimensions      (GetDimensions.srv)
#       /seed                (Seed.srv)
#       /action_space_sample (SampleActionSpace.srv)
#     - 각 서비스가 준비될 때까지 wait_for_service() 대기
#
#   reset() → state: list[float]
#     - /reset 서비스 호출
#     - response.state 반환
#
#   step(action) → (state, reward, done, target)
#     - action[0]을 [-1,1] → [0,1]로 정규화 (선형속도 후진 방지)
#     - /step 서비스 호출
#     - (response.state, response.reward, response.done, response.target) 반환
#
#   get_dimensions() → (state_dim, action_dim, max_action)
#     - /get_dimensions 서비스 호출
#     - (response.state_dim, response.action_dim, response.max_action) 반환
#
#   sample_action_space() → action: list[float]
#     - /action_space_sample 서비스 호출
#     - response.action 반환
#
#   set_env_seed(seed)
#     - /seed 서비스 호출
