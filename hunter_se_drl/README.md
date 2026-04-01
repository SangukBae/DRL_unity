# hunter_se_drl

Hunter SE 로봇 + AutoDRIVE Unity 시뮬레이터를 위한 강화학습 패키지.
SB3(Stable-Baselines3)를 기반으로 하며, 학습 중 RViz2로 실시간 시각화가 가능하다.

## 패키지 구조

```
hunter_se_drl/
├── envs/
│   └── hunter_se_env.py      # Gymnasium 환경 + eventlet WebSocket 서버 + ROS2 퍼블리셔
├── models/
│   └── custom_policy.py      # SB3 커스텀 Policy (모델 구조 변경 시 여기만 수정)
├── train/
│   ├── train_sac.py          # SB3 SAC으로 학습 (준비 점검 + RViz2 자동 실행 포함)
│   └── train_ppo.py          # SB3 PPO으로 학습 (준비 점검 + RViz2 자동 실행 포함)
├── test/
│   └── test_agent.py         # 학습된 모델 시뮬레이터 테스트
├── deploy/
│   └── run_real_robot.py     # 실제 Hunter SE 로봇에서 모델 실행 (ROS2)
├── config/
│   ├── env_config.yaml       # 환경 파라미터 (LiDAR·속도·아레나·포트 등)
│   ├── train_sac_config.yaml
│   └── train_ppo_config.yaml
└── utils/
    ├── lidar_utils.py        # base64 + gzip LiDAR 디코딩 + 전처리
    └── file_manager.py       # 모델·로그 저장 경로 관리, YAML 로드·저장
```

## 시스템 구조

```
[호스트 - Unity AutoDRIVE]              [Docker - hunter_se_drl]
  Random Obstacles.unity
  Socket.IO (Engine.IO) 클라이언트
         |                                        |
         |  WebSocket (port 4567)                 |
         └──────────────────────────────────────→ eventlet WebSocket 서버
                                                  hunter_se_env.py
                                                  (Gymnasium Env)
                                                       |
                                              ┌────────┴────────┐
                                         SB3 학습           ROS2 퍼블리셔
                                     (SAC / PPO)          (선택적, rclpy)
                                    train_sac.py                |
                                    train_ppo.py           RViz2 시각화
```

## 통신 방식

Python이 **서버**, Unity가 **클라이언트**로 접속한다.
eventlet WebSocket + Engine.IO 프로토콜 사용 (port 4567).

**Unity → Python (텔레메트리)**
| 키 | 형식 | 설명 |
|----|------|------|
| `"V1 LIDAR Range Array"` | base64+gzip | LiDAR 거리 배열 (360개) |
| `"V1 Position"` | `"x y z"` (공백) | 로봇 위치 |
| `"V1 Orientation Euler Angles"` | `"pitch yaw roll"` (공백, 도) | 로봇 방향 |
| `"V1 Collisions"` | int 문자열 | 누적 충돌 카운터 |
| `"V1 Speed"` | float 문자열 | 선속도 (m/s) |
| `"V1 Angular Velocity"` | `"x y z"` (공백) | 각속도 |

**Python → Unity (제어)**
| 키 | 설명 |
|----|------|
| `"V1 Linear Velocity"` | 선속도 (m/s) |
| `"V1 Angular Velocity"` | 각속도 (rad/s) |
| `"V1 Reset"` | 에피소드 리셋 (`"true"` / `"false"`) |
| `"Goal PosX"` / `"Goal PosZ"` | 목표 위치 (Unity RLVisualizer 시각화용) |

## ROS2 시각화 (학습 중 RViz2)

`hunter_se_env.py`는 텔레메트리 수신 시마다 아래 ROS2 토픽을 동시에 발행한다.
rclpy가 없어도 학습은 정상 동작한다.

| 토픽 | 타입 |
|------|------|
| `/autodrive/hunter_se_1/lidar` | `sensor_msgs/LaserScan` |
| `/autodrive/hunter_se_1/imu` | `sensor_msgs/Imu` |
| `/autodrive/hunter_se_1/ips` | `geometry_msgs/Point` |
| `/autodrive/hunter_se_1/linear_vel` | `std_msgs/Float32` |
| `/autodrive/hunter_se_1/angular_vel` | `std_msgs/Float32` |
| `/autodrive/hunter_se_1/collisions` | `std_msgs/Int32` |
| `/autodrive/hunter_se_1/goal` | `visualization_msgs/Marker` |
| TF: `map → hunter_se_1 → lidar / imu` | |

## 실행 방법

### 시뮬레이터 학습 (Docker)

```bash
cd /autodrive

# SAC 학습 (권장)
python hunter_se_drl/train/train_sac.py

# PPO 학습
python hunter_se_drl/train/train_ppo.py
```

학습 파일 실행 시 자동으로:
1. 준비 상태 점검 (config·패키지·포트 확인)
2. RViz2 백그라운드 실행
3. Unity 연결 대기 후 학습 시작

### 학습 테스트

```bash
python hunter_se_drl/test/test_agent.py \
    --model_path hunter_se_drl/models/saved/sac/final_model \
    --algorithm sac \
    --num_episodes 10
```

### 실제 로봇 실행 (ROS2)

```bash
# 터미널 1: CAN 드라이버
ros2 launch hunter_base hunter_base.launch.py port_name:=can0

# 터미널 2: LiDAR 드라이버
ros2 launch <lidar_package> <lidar_launch>

# 터미널 3: RL 에이전트
python hunter_se_drl/deploy/run_real_robot.py \
    --model_path hunter_se_drl/models/saved/sac/final_model \
    --algorithm sac \
    --goal_x 3.0 --goal_z 3.0
```

## 의존성

| 패키지 | 용도 |
|--------|------|
| `stable-baselines3` | SAC / PPO 알고리즘 |
| `gymnasium` | 강화학습 환경 인터페이스 |
| `eventlet` | WebSocket 서버 (Unity 통신) |
| `numpy` | LiDAR 데이터 처리 |
| `torch` | 신경망 백엔드 |
| `rclpy` (선택) | RViz2 시각화 토픽 발행 |
