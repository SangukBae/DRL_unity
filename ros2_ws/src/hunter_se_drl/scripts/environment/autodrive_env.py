#!/usr/bin/env python3
#
# autodrive_env.py
#
# ============================================================
# [역할]
# 이 패키지의 핵심 파일.
# AutoDRIVE Unity 시뮬레이터와 ROS2 에이전트 사이의 브릿지 역할을 한다.
#
# 크게 두 가지 역할을 동시에 수행한다:
#   1. Socket.IO 클라이언트: AutoDRIVE Unity와 실시간 송수신
#   2. ROS2 서비스 서버: train_*.py에 /step, /reset 등 서비스 제공
#
# drl_agent 패키지의 environment.py가 Gazebo 토픽/서비스로 하던 일을
# AutoDRIVE Socket.IO 통신으로 대체한 파일이다.
# ============================================================
#
# ============================================================
# [AutoDRIVE Socket.IO 수신 데이터] (Unity → Python)
# ============================================================
# "V1 LiDAR"      : "[r0, r1, ..., rN]"  LiDAR 거리 배열 (문자열)
# "V1 PosX"       : float                로봇 X 위치 (m)
# "V1 PosY"       : float                로봇 Y 위치 (m)
# "V1 PosZ"       : float                로봇 Z 위치 (m)
# "V1 Collision"  : bool                 충돌 여부
# "Goal PosX"     : float                목표 X 위치 (m)
# "Goal PosZ"     : float                목표 Z 위치 (m)
#
# ============================================================
# [AutoDRIVE Socket.IO 송신 데이터] (Python → Unity)
# ============================================================
# "V1 Linear Velocity"  : float  선형 속도 명령 (m/s)
# "V1 Angular Velocity" : float  각속도 명령 (rad/s)
# "V1 Reset"            : bool   에피소드 리셋 명령
# "Spawn PosX"          : float  로봇 스폰 X 위치
# "Spawn PosZ"          : float  로봇 스폰 Z 위치
# "Goal PosX"           : float  목표 X 위치
# "Goal PosZ"           : float  목표 Z 위치
#
# ============================================================
# [제공하는 ROS2 서비스] (→ train_*.py / env_interface.py가 호출)
# ============================================================
# /step                 Step.srv             액션 실행 → (state, reward, done, target) 반환
# /reset                Reset.srv            환경 리셋 → 초기 state 반환
# /get_dimensions       GetDimensions.srv    상태/액션 공간 크기 반환
# /seed                 Seed.srv             난수 시드 설정
# /action_space_sample  SampleActionSpace.srv 랜덤 액션 샘플 반환
#
# ============================================================
# [상태 공간 구성]
# ============================================================
# environment_state (lidar_dim,)   : LiDAR 거리값 배열 (정규화: 0~1)
# agent_state      (4,)            : [dist_to_goal, x, y, theta]
# 전체 state       (lidar_dim + 4,): 두 배열을 concatenate하여 에이전트에 전달
#
# ============================================================
# [구현 순서]
# ============================================================
# 1. __init__
#    - utils/file_manager.py로 environment.yaml 로드
#    - Socket.IO 클라이언트 초기화 (socketio.Client())
#    - ROS2 서비스 서버 5개 생성 (/step, /reset, /get_dimensions, /seed, /action_space_sample)
#    - threading.Lock 생성 (state 동시 접근 보호)
#    - AutoDRIVE Socket.IO 서버에 연결 (socketio_host:socketio_port)
#
# 2. on_telemetry(data) - Socket.IO 수신 콜백
#    - "V1 LiDAR" 파싱 → environment_state 갱신
#    - "V1 PosX/Y/Z" 파싱 → 로봇 위치 갱신
#    - "V1 Collision" 파싱 → collision 플래그 갱신
#    - "Goal PosX/Z" 파싱 → 목표 위치 갱신
#    - agent_state 계산 [dist_to_goal, x, y, theta]
#    - telemetry_event.set() (step 핸들러의 대기 해제)
#
# 3. step_callback(request, response) - /step 서비스 핸들러
#    - request.action에서 [linear_vel, angular_vel] 추출
#    - telemetry_event.clear()
#    - Socket.IO로 "V1 Linear Velocity", "V1 Angular Velocity" 전송
#    - telemetry_event.wait(timeout) 로 다음 텔레메트리 대기
#    - reward.py의 compute_reward() 호출
#    - done 판정 (collision 또는 max_episode_steps 초과)
#    - target 판정 (dist_to_goal < goal_threshold)
#    - response에 (state, reward, done, target) 채워서 반환
#
# 4. reset_callback(request, response) - /reset 서비스 핸들러
#    - 랜덤 스폰 위치 생성 (아레나 범위 내, 중앙 근처 제외)
#    - 랜덤 목표 위치 생성 (스폰 위치와 최소 거리 이상)
#    - telemetry_event.clear()
#    - Socket.IO로 "V1 Reset", "Spawn PosX/Z", "Goal PosX/Z" 전송
#    - telemetry_event.wait(timeout) 로 초기 텔레메트리 대기
#    - response.state에 초기 state 채워서 반환
#
# 5. get_dimensions_callback(request, response)
#    - response.state_dim  = lidar_dim + agent_state_dim (전체 상태 차원)
#    - response.action_dim = action_dim
#    - response.max_action = max_action
#
# 6. seed_callback / action_space_sample_callback
#    - drl_agent/environment.py의 동일 핸들러와 동일하게 구현
