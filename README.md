# AutoDRIVE Hunter SE DRL

Hunter SE 로봇을 사용한 LiDAR 센서 기반 장애물 회피 + 목표 지점 도달 강화학습 프로젝트.

## 환경 구성

### 호스트 (Unity 시뮬레이터)
- Unity 2022.3.62f3
- HDRP (High-Definition Render Pipeline) + PhysX
- Hunter SE 스케일드 자율주행 로봇
- Socket.IO (Engine.IO) 브리지로 Python과 통신
- 씬: `Random Obstacles.unity`

### Docker (RL 학습)
- Base image: `nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04`
- Python 가상환경: `/opt/venv/rl` (자동 활성화)
- Stable-Baselines3 (editable 설치)
- ROS2 Humble + CycloneDDS
- X11 디스플레이 포워딩 지원 (gedit, RViz2 등 GUI 실행 가능)

## 시스템 구조

```
[호스트 - Unity AutoDRIVE]           [Docker - hunter_se_drl]
  Random Obstacles.unity
  Socket.IO 클라이언트
         |                                    |
         |  WebSocket (port 4567)              |
         └──────────────────────────────────→ eventlet WebSocket 서버
                                              hunter_se_env.py (Gymnasium)
                                                    |
                                              SB3 (SAC / PPO)
                                                    |
                                              ROS2 토픽 발행 ──→ RViz2
```

## 통신 프로토콜

**Unity → Python (텔레메트리)**
```json
{
  "V1 LIDAR Range Array": "base64+gzip 인코딩 float 배열",
  "V1 Position":          "x y z (공백 구분)",
  "V1 Orientation Euler Angles": "pitch yaw roll (공백 구분, 도)",
  "V1 Collisions":        "누적 충돌 카운터 (int)",
  "V1 Speed":             "선속도 (float)",
  "V1 Angular Velocity":  "x y z (공백 구분)"
}
```

**Python → Unity (제어)**
```json
{
  "V1 Linear Velocity":  "float (m/s)",
  "V1 Angular Velocity": "float (rad/s)",
  "V1 Reset":            "true / false",
  "Goal PosX":           "float",
  "Goal PosZ":           "float"
}
```

## 실행 방법

### 1. Docker 시작

```bash
# X11 디스플레이 허용 (GUI 앱 사용 시)
xhost +local:docker

cd /home/sangukbae/autodrive
docker compose up -d
docker compose exec autodrive bash
```

### 2. autodrive_hunter_se ROS2 패키지 빌드 (최초 1회)

```bash
# Docker 내부
cd "/autodrive/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_ros2"
source /opt/ros/humble/setup.bash
colcon build --packages-select autodrive_hunter_se
```

### 3. Unity 시뮬레이터 실행

```
Unity Hub → AutoDRIVE-Simulator 열기
→ Random Obstacles.unity 씬 열기
→ Play 버튼 클릭
```

### 4. RL 학습 시작 (Docker 내부)

```bash
cd /autodrive

# SAC 학습 (권장)
python hunter_se_drl/train/train_sac.py

# PPO 학습
python hunter_se_drl/train/train_ppo.py
```

학습 시작 시 자동으로:
- 준비 상태 점검 (패키지·config·포트 확인)
- RViz2 백그라운드 실행
- Unity 연결 대기 후 학습 시작

### 5. 학습 결과 확인

```bash
# 테스트
python hunter_se_drl/test/test_agent.py \
    --model_path hunter_se_drl/models/saved/sac/final_model \
    --algorithm sac \
    --num_episodes 10
```

### Docker 재시작 순서

```bash
# 종료
exit
docker compose stop

# 재시작
docker compose start
docker compose exec autodrive bash
```

## 프로젝트 구조

```
autodrive/
├── AutoDRIVE-Simulator/          # Unity 프로젝트
│   └── Assets/Scripts/
│       ├── Socket.cs             # Socket.IO 브리지
│       └── RLVisualizer.cs       # Unity 내 경로·목표 시각화
├── AutoDRIVE-Devkit/
│   └── ADSS Toolkit/autodrive_ros2/
│       └── autodrive_hunter_se/  # ROS2 시각화 패키지 (RViz2)
├── hunter_se_drl/                # RL 학습 패키지 (메인)
│   ├── envs/                     # Gymnasium 환경
│   ├── train/                    # 학습 스크립트
│   ├── test/                     # 테스트 스크립트
│   ├── deploy/                   # 실제 로봇 실행
│   ├── config/                   # 하이퍼파라미터
│   ├── models/                   # 커스텀 정책
│   └── utils/                    # LiDAR 유틸, 파일 관리
├── ros2_ws/                      # 실제 로봇용 ROS2 워크스페이스
│   └── src/hunter_ros2/          # Hunter SE CAN 드라이버
├── Dockerfile
└── docker-compose.yml
```

## 의존성

| 패키지 | 용도 |
|--------|------|
| `stable-baselines3` | SAC / PPO 알고리즘 |
| `gymnasium` | 강화학습 환경 인터페이스 |
| `eventlet` | WebSocket 서버 (Unity 통신) |
| `torch` | 신경망 백엔드 (CUDA 12.1) |
| `numpy` | LiDAR 데이터 처리 |
| `rclpy` + ROS2 Humble | RViz2 시각화 토픽 발행 |
| `gevent` + `geventwebsocket` | autodrive_hunter_se 브리지 |
