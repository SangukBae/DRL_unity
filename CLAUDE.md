# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AutoDRIVE Simulator is a Unity-based digital twin simulator for scaled autonomous vehicles, targeting researchers and students developing autonomous driving algorithms. It is built on Unity 2022.3.52f1 LTS with HDRP (High-Definition Render Pipeline) and PhysX for realistic physics.

**현재 목표**: Hunter SE 로봇을 사용하여 LiDAR 센서 기반 장애물 회피 + 목표 지점 도달 강화학습 구현
- 호스트: Unity 시뮬레이터 실행 (GUI)
- Docker: Python RL 학습 코드 실행 (SB3 + CUDA 12.1)
- 통신: Socket.IO (network_mode: host, localhost)

## 환경 구성

### 호스트 (Unity)
- Unity 2022.3.62f3 (호환 버전)
- 프로젝트 경로: `/home/sangukbae/autodrive/AutoDRIVE-Simulator`
- Hunter SE 모델 복원 완료: `Assets/Models/Vehicle/Hunter SE/`
- 도로 인프라 모델 복원 완료: `Assets/Models/Infrastructure/Road Kits/`

### Docker (RL 학습)
- Base image: `nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04`
- Python 가상환경: `/opt/venv/rl` (자동 활성화)
- SB3: `/autodrive/stable-baselines3` (editable 설치)
- ROS2 Humble + CycloneDDS
- 작업 폴더: `/autodrive` (호스트 `/home/sangukbae/autodrive`와 동기화)

### Docker 실행
```bash
xhost +local:docker
docker compose up -d
docker compose exec autodrive bash
```

## RL 시스템 구조

```
[호스트 - Unity 시뮬레이터]                 [Docker - Python RL]
  RLEnvironmentManager.cs                   HunterSEEnv (Gymnasium)
  - 로봇 랜덤 스폰                            - reset(): 랜덤 위치 생성 → Unity 전송
  - 장애물 랜덤 배치        ←── Bridge ──→    - step(action): 제어 명령 전송
  - 목표 위치 랜덤 배치                        - reward(): LiDAR 기반 보상 계산
  - LiDAR 데이터 송신                         - SB3 PPO/SAC으로 학습
  - 충돌/도달 감지
```

### Socket.IO 통신 프로토콜

**Unity → Python (텔레메트리)**
```json
{
  "V1 LiDAR": "[r0, r1, ..., rN]",   // LiDAR 거리 배열
  "V1 PosX/Y/Z": "float",            // 로봇 위치
  "V1 Collision": "bool",            // 충돌 여부
  "Goal PosX/Z": "float"             // 목표 위치
}
```

**Python → Unity (제어)**
```json
{
  "V1 Throttle": "float",            // -1.0 ~ 1.0
  "V1 Steering": "float",            // -1.0 ~ 1.0
  "V1 Reset": "bool",                // 에피소드 리셋
  "Spawn PosX/Z": "float",           // 로봇 스폰 위치
  "Goal PosX/Z": "float",            // 목표 위치
  "Obstacle[N] PosX/Z": "float"      // 장애물 위치
}
```

## RL 구현 진행 과정

### Phase 1: Unity 씬 구성
1. 새 씬 생성 (`RL_HunterSE.unity`)
2. Hunter SE 프리팹 배치 (`Assets/Prefabs/Hunter SE/Hunter SE.prefab`)
3. 평지 환경 + 장애물 오브젝트 배치
4. LiDAR 센서 설정 확인 (`Assets/Scripts/LIDAR.cs`)
5. Socket.IO 브리지 설정 (`Assets/Scripts/Socket.cs`)

### Phase 2: Unity C# 스크립트 작성
- `RLEnvironmentManager.cs` (신규): 에피소드 리셋 시 로봇/장애물/목표 랜덤 배치
  - Python으로부터 `reset` 명령 수신
  - 랜덤 위치로 로봇 스폰, 장애물 배치, 목표 설정
  - 충돌/목표 도달 감지 후 Python에 신호 전송

### Phase 3: Python Gymnasium 환경 작성
- `rl/envs/hunter_se_env.py`: 커스텀 Gymnasium 환경
  - observation: LiDAR 배열 + 목표까지 거리/방향
  - action: [throttle, steering] (Box space)
  - reward: 목표 접근 보상 + 충돌 패널티 + 생존 보너스

### Phase 4: 학습 실행
- `rl/train.py`: SB3 PPO 또는 SAC으로 학습
- Docker 내부에서 실행, 호스트 Unity와 Socket.IO 통신

## Building and Running

Unity 프로젝트는 CLI 빌드 시스템 없음. Unity Hub에서 직접 빌드.

**프로젝트 열기:**
```bash
# 대용량 zip 파일 압축 해제 (최초 1회)
cd AutoDRIVE-Simulator/Tools
./unzip-and-clean.sh
```
Unity Hub → ADD → `/home/sangukbae/autodrive/AutoDRIVE-Simulator` 선택

**복원된 에셋 (git checkout으로 복원):**
```bash
# Hunter SE 모델
git checkout 60c59d73 -- "Assets/Models/Vehicle/Hunter SE"
# 도로 인프라 모델
git checkout 4848d69f -- "Assets/Models/Infrastructure"
```

## Architecture

### Code Organization (`Assets/Scripts/`)

**Vehicle Control**
- `VehicleController.cs` — Generic vehicle with Ackermann steering; supports drive modes: IRWD, IFWD, IAWD, CRWD, CFWD, CAWD, SkidSteer
- `TwistController.cs` — Velocity/angular velocity interface (ROS Twist-compatible)
- `SkidSteerController.cs` — Tank-like vehicle control (Hunter SE 사용)

**Sensor Suite**
- `LIDAR.cs` / `LIDAR3D.cs` — 2D/3D laser scanners via ray casting
- `GPS.cs`, `IMU.cs` — GNSS and inertial sensors
- `WheelEncoder.cs` — Wheel rotation tracking

**Communication Bridge**
- `Socket.cs` — SocketIO bi-directional bridge; `OnBridge` 이벤트로 Python 명령 수신, `EmitTelemetry`로 센서 데이터 송신
- `CoSimManager.cs` — Co-simulation state updates with smooth interpolation
- `ResetManager.cs` — 에피소드 리셋 (ResetFlag=true 시 초기 위치로 복귀)

**Simulation Management**
- `RandomizePose.cs` — 오브젝트 위치/회전 랜덤화 (장애물에 적용 가능)
- `CollisionDetector.cs` — 충돌 감지
- `WeatherManager.cs` / `TimeOfDay.cs` — Dynamic weather and day/night cycle

### Communication Architecture

Socket.IO 브리지 (`Socket.cs`):
- Python → Unity: `"Bridge"` 이벤트로 제어 명령 수신
- Unity → Python: `EmitTelemetry()`로 센서 데이터 송신
- 리셋: `"V1 Reset": "true"` 전송 시 `ResetManager.ResetFlag = true`
- Hunter SE는 `TwistController` 사용: `"V1 Linear Velocity"`, `"V1 Angular Velocity"`

### Key Configuration Files

| File | Purpose |
|------|---------|
| `ProjectSettings/ProjectVersion.txt` | Unity version: 2022.3.52f1 (호환: 2022.3.62f3) |
| `Packages/manifest.json` | Unity package dependencies |
| `cyclonedds_config.xml` (repo root) | DDS inter-process communication config |
| `Dockerfile` | Docker 환경 (CUDA 12.1, ROS2 Humble, SB3) |
| `docker-compose.yml` | 볼륨 마운트, GPU, 네트워크 설정 |

### Physics Loop vs. Rendering Loop
- **FixedUpdate (~50 Hz):** Vehicle dynamics, LIDAR ray casting, GPS/IMU/encoder updates
- **Update (60+ Hz):** Camera rendering, visual effects, UI, Socket.IO communication
