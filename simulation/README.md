# Hunter SE - Random Obstacles Scene

<video src="Hunter_SE_LIDAR_TEST_OS1_128.webm" controls width="100%"></video>

이 문서는 TQC 강화학습 학습에 사용되는 `hunter_se_unity/Hunter SE - Random Obstacles.unity` 씬의 구조를 정리한다.

---

## Scene Overview

| 항목 | 내용 |
| --- | --- |
| 씬 파일 | `hunter_se_unity/Hunter SE - Random Obstacles.unity` |
| 주 용도 | Hunter SE 차량의 LiDAR 기반 장애물 회피 + 목표 도달 강화학습 |
| 물리 업데이트 | 1000 Hz (`Fixed Timestep: 0.001s`) |
| LiDAR | Ouster OS1-128 (`modelPreset: 9`, `horizontalSteps: 1024`, `maxRange: 200m`, `AutomaticCaptureHz: 10`) |
| 아레나 크기 | 30 × 30 m (`arenaSize: 30`) |
| 정적 장애물 수 | 35 (`staticObstacleCount: 35`) |
| 동적 장애물 수 | 10 (`dynamicObstacleCount: 10`) |
| 랜덤 시드 | 12345 (`randomizeSeed: 1` — 매 에피소드마다 재시드) |

---

## Top-Level GameObject 구조

```
Hunter SE - Random Obstacles
├── Floor                          ← 아레나 바닥
├── Infrastructure (1/2)           ← 테스트 코스 구조물
├── Waypoints                      ← 경로 포인트 (기본 비활성)
├── StaticLightingSky              ← 정적 조명
├── HDRP Compositor                ← HDRP 후처리
├── SceneIDMap                     ← 씬 식별용
├── RGL Scene Manager              ← RGL LiDAR 물리 씬 관리
├── Unity Main Thread Dispatcher   ← 메인 스레드 디스패치 유틸
├── Random Obstacle Arena Manager  ← 장애물 랜덤 배치 + 아레나 생성
├── RL Reset Manager               ← 에피소드 리셋 처리
├── RLVisualizer                   ← 경로/목표 시각화 (Unity 내)
└── Hunter SE                      ← 차량 (아래 구조 참조)
```

---

## 차량 계층 구조

```
Hunter SE                          ← 차량 루트 (Rigidbody, VehicleController)
├── Frame                          ← 차체 메쉬
├── Front Left Wheel               ← 앞 왼쪽 바퀴 + WheelCollider
├── Front Right Wheel              ← 앞 오른쪽 바퀴 + WheelCollider
├── Rear Left Wheel                ← 뒤 왼쪽 바퀴 + WheelCollider
├── Rear Right Wheel               ← 뒤 오른쪽 바퀴 + WheelCollider
├── Center of Mass                 ← 무게중심 오프셋
├── Front Axle Center              ← 전축 중심
├── Rear Axle Center               ← 후축 중심
├── Sensors
│   ├── GNSS                       ← GPS 위치 센서
│   ├── IMU                        ← 9-DOF 관성 센서
│   ├── Left Wheel Encoder         ← 왼쪽 휠 엔코더
│   ├── Right Wheel Encoder        ← 오른쪽 휠 엔코더
│   └── LIDAR                      ← LiDAR 루트
│       └── Ouster OS1_128         ← LiDAR 외관 메쉬
├── Cameras
│   ├── Driver's Eye               ← 운전자 시점
│   ├── Front Camera               ← 전방 카메라
│   └── Rear Camera                ← 후방 카메라
└── Actuators                      ← TwistController / VehicleController
```

---

## 핵심 컴포넌트 상세

### Random Obstacle Arena Manager

매 에피소드 시작 시 아레나를 생성하고 장애물을 배치한다.

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `arenaSize` | 30 m | 아레나 한 변 길이 |
| `wallThickness` | 0.5 m | 외벽 두께 |
| `wallHeight` | 2.5 m | 외벽 높이 |
| `staticObstacleCount` | 35 | 고정 장애물 수 |
| `dynamicObstacleCount` | 10 | 이동 장애물 수 |
| `spawnMargin` | 2 m | 아레나 경계와 장애물 간 최소 여백 |
| `vehicleClearRadius` | 4 m | 차량 스폰 지점 주변 장애물 배제 반경 |
| `obstacleMinSpacing` | 2.2 m | 장애물 간 최소 간격 |
| `seed` | 12345 | 기본 랜덤 시드 |
| `randomizeSeed` | 1 (true) | 매 에피소드마다 시드 재랜덤화 |

장애물 프리팹은 `staticObstaclePrefabs`(고정)과 `dynamicObstaclePrefabs`(이동) 두 종류로 구분되며, Traffic Cone 등 다양한 형태가 포함된다.

### RL Reset Manager

Python에서 `"V1 Reset": "true"` 명령을 받으면 차량을 스폰 위치로 복귀시킨다.

| 연결 대상 | 역할 |
| --- | --- |
| `Vehicles[0]` | 차량 Transform — 위치/회전 초기화 |
| `VehicleRigidBodies[0]` | Rigidbody — 속도/관성 초기화 |
| `LeftWheelEncoders[0]` | 휠 엔코더 카운터 초기화 |
| `RightWheelEncoders[0]` | 휠 엔코더 카운터 초기화 |

### RLVisualizer

학습 중 Unity 뷰포트에 주행 경로와 목표 위치를 시각화한다.

| 파라미터 | 값 |
| --- | --- |
| `PathColor` | 초록 (r:0, g:1, b:0) |
| `PathWidth` | 0.05 m |
| `MaxPathPoints` | 2000 |
| `GoalColor` | 노랑 (r:1, g:0.85, b:0) |
| `GoalRadius` | 0.4 m |

### LiDAR (Ouster OS1-128)

| 파라미터 | 값 |
| --- | --- |
| 모델 | Ouster OS1-128 (`modelPreset: 9`) |
| 수평 해상도 | 1024 steps/회전 |
| 최대 측정 거리 | 200 m |
| 캡처 주파수 | 10 Hz (`AutomaticCaptureHz: 10`) |
| 수직 오프셋 | 0 mm |
| 수평 오프셋 | 0 mm |
| LiDAR 파이프라인 | RGL (NVIDIA RobotecGPULidar) |

RGL 포인트클라우드 출력 포맷:
- `PCL12`: XYZ (12 bytes/pt)
- `PCL24`: XYZ + Intensity + RingID (24 bytes/pt)
- `PCL48`: PCL24 + Azimuth + Distance + Timestamp (48 bytes/pt)

Python `hunter_se_env.py`는 Socket.IO로 받은 포인트클라우드를 `preprocess_lidar()`에서 80-bin LaserScan으로 리샘플링해 관측 입력으로 사용한다.

---

## 통신 구조 (Socket.IO)

```
Python RL 환경 (HunterSEEnvTQC)
  │  "V1 Linear Velocity": float (m/s)
  │  "V1 Angular Velocity": float (rad/s)
  │  "V1 Reset": "true" / "false"
  │  "Goal PosX": float
  │  "Goal PosZ": float
  ▼
Socket.cs → VehicleController → WheelCollider ×4
          → ResetManager (에피소드 리셋)

Unity → Python (텔레메트리)
  │  "V1 LiDAR": 포인트클라우드 base64+gzip
  │  "V1 PosX/Y/Z": 위치
  │  "V1 Rot Yaw": yaw (degrees)
  │  "V1 Speed": 선속도
  │  "V1 Collision": 누적 충돌 카운터
  │  "Goal PosX/Z": 목표 위치
  ▼
hunter_se_env.py
```

---

## 관련 파일

| 구분 | 경로 |
| --- | --- |
| 씬 파일 | `hunter_se_unity/Hunter SE - Random Obstacles.unity` |
| 복사본 (Simulator) | `AutoDRIVE-Simulator/Assets/Scenes/Hunter SE - Random Obstacles.unity` |
| 차량 프리팹 | `AutoDRIVE-Simulator/Assets/Prefabs/Hunter SE/Hunter SE.prefab` |
| VehicleController | `AutoDRIVE-Simulator/Assets/Scripts/VehicleController.cs` |
| Socket 브리지 | `AutoDRIVE-Simulator/Assets/Scripts/Socket.cs` |
| ResetManager | `AutoDRIVE-Simulator/Assets/Scripts/ResetManager.cs` |
| RLVisualizer | `AutoDRIVE-Simulator/Assets/Scripts/RLVisualizer.cs` |
| RGL LidarSensor | `AutoDRIVE-Simulator/Assets/Plugins/RGLUnity Plugin/Scripts/LidarSensor.cs` |
| Python 환경 | `hunter_se_drl/envs/hunter_se_env.py` |
| TQC 래퍼 | `hunter_se_drl/envs/hunter_se_env_tqc.py` |
