# DRL_unity

Hunter SE 로봇을 사용한 LiDAR 센서 기반 장애물 회피 + 목표 지점 도달 강화학습 프로젝트.

## 환경 구성

### 호스트 (Unity 시뮬레이터)
- Unity 2022.3.62f3
- HDRP (High-Definition Render Pipeline) + PhysX
- Hunter SE 스케일드 자율주행 로봇
- Socket.IO 브리지로 Python과 통신

### Docker (RL 학습)
- Base image: `nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04`
- Python 가상환경: `/opt/venv/rl` (자동 활성화)
- Stable-Baselines3 (editable 설치)
- ROS2 Humble + CycloneDDS

## 시스템 구조

```
[호스트 - Unity 시뮬레이터]                 [Docker - Python RL]
  RLEnvironmentManager.cs                   HunterSEEnv (Gymnasium)
  - 로봇 랜덤 스폰                            - reset(): 랜덤 위치 생성 → Unity 전송
  - 장애물 랜덤 배치        ←── Socket.IO ──→  - step(action): 제어 명령 전송
  - 목표 위치 랜덤 배치                        - reward(): LiDAR 기반 보상 계산
  - LiDAR 데이터 송신                         - SB3 PPO/SAC으로 학습
  - 충돌/도달 감지
```

### 통신 프로토콜

**Unity → Python (텔레메트리)**
```json
{
  "V1 LiDAR": "[r0, r1, ..., rN]",
  "V1 PosX/Y/Z": "float",
  "V1 Collision": "bool",
  "Goal PosX/Z": "float"
}
```

**Python → Unity (제어)**
```json
{
  "V1 Throttle": "float",
  "V1 Steering": "float",
  "V1 Reset": "bool",
  "Spawn PosX/Z": "float",
  "Goal PosX/Z": "float",
  "Obstacle[N] PosX/Z": "float"
}
```

## 실행 방법

### 1. Docker 시작

```bash
xhost +local:
sudo systemctl restart docker

cd /autodrive
docker compose up -d
docker compose exec autodrive bash
```

### 2. Unity 시뮬레이터 실행

Unity Hub에서 프로젝트를 열고 `RL_HunterSE` 씬을 실행합니다.

### 3. RL 학습 시작 (Docker 내부)

```bash
cd /autodrive
python rl/train.py
```

### Docker 종료

```bash
docker compose down
```
