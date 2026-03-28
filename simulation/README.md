# Hunter SE Scenes

이 문서는 `AutoDRIVE-Simulator/Assets/Scenes` 아래의 Hunter SE 씬 구조를 빠르게 파악하기 위한 요약이다.

대상 씬:

- `Hunter SE.unity`
- `Hunter SE - New.unity`
- `Hunter SE - PID.unity`
- `Hunter SE - CBF.unity`
- `Hunter SE - Data Collection.unity`
- `Hunter SE - LIDAR Test.unity`
- `Hunter SE - Tiny Town.unity`
- `Hunter SE - Greensward.unity`
- `Hunter SE - Greensward - SRS.unity`
- `Hunter SE - Greensward Variable Friction.unity`
- `Hunter SE - Greensward with Obstacles.unity`

## Common Structure

Hunter SE 계열 씬은 대부분 공통 `Hunter SE` 차량 프리팹을 사용하고, 씬별로 다음 중 하나를 추가한다.

- 환경 루트: `Terrain`, `Skybox`, `Water`, `Trees`, `Rocks`, `Volumes`, `Lights`
- 테스트 코스 루트: `Floor`, `Waypoints`, `Infrastructure`
- 검증용 오브젝트: `Cube`, `Cones`, `CLI Manager`, `iPhone`

차량 내부 공통 트리는 보통 아래와 같다.

- `Hunter SE`
- `Sensors`
- `Cameras`
- `Actuators`
- 바퀴/차체 메쉬

주의:

- 모든 씬이 센서 구성을 씬 파일에 직접 풀어쓰는 것은 아니다.
- 일부 씬은 프리팹 인스턴스를 그대로 쓰고, 일부 씬은 LiDAR만 씬에서 override 한다.

## Scene Summary

| Scene | 주 용도 | 최상위 루트 특징 | LiDAR / 센서 특징 |
| --- | --- | --- | --- |
| `Hunter SE.unity` | 기본 베이스 씬 | `Terrain`, `Skybox`, `Water`, `Rocks`, `Lights`, `Volumes` | LiDAR 루트는 있으나 기본 상태. 씬 안에는 `Velodyne VLP-32C` 구조와 VLP-32C 계열 파라미터가 남아 있음 |
| `Hunter SE - New.unity` | 베이스 환경 변형 | `Terrain`, `Water`, `Trees`, `Bushes`, `Rocks`, `Skybox` | 센서 구조는 주로 프리팹 상속. 환경만 바꾼 일반 주행용 변형에 가까움 |
| `Hunter SE - PID.unity` | 제어 실험용 | `Floor`, `Infrastructure (1/2)`, `Waypoints` | Hunter SE 루트와 `Sensors`, `Cameras`, `LIDAR`가 씬에 직접 드러남. Ouster 계열 파라미터 `37.7 / 0` 사용 |
| `Hunter SE - CBF.unity` | 제어 실험용 | `Floor`, `Infrastructure (1/2)`, `Waypoints` | PID 씬과 거의 같은 구조. 제어 로직만 다른 변형으로 보면 됨 |
| `Hunter SE - Data Collection.unity` | 데이터 수집용 | `Floor`, `Waypoints`, `Infrastructure`, `fishhook`, `skidpad`, `slalom`, `straight` | 트랙 요소가 가장 다양함. Ouster 계열 파라미터 `37.7 / 0` 사용 |
| `Hunter SE - LIDAR Test.unity` | LiDAR 시각/포인트클라우드 검증 | `Floor`, `Waypoints`, `Infrastructure`, `Cube (0~7)`, `fishhook`, `skidpad`, `slalom`, `straight` | Hunter SE 계열 중 LiDAR 검증 목적이 가장 분명함. 씬 안에 `Ouster OS1_64` 이름이 직접 나타남 |
| `Hunter SE - Tiny Town.unity` | 소규모 도심/인프라 테스트 | `Floor`, `Infrastructure`, `StaticLightingSky` | 프리팹 기반 구조. 도시형 레이아웃 실험용으로 보기 쉬움 |
| `Hunter SE - Greensward.unity` | 오프로드/잔디 환경 | `Trees`, `Volumes`, `Wind`, `Reflections`, `Lights`, `iPhone` | 환경 중심 변형. 센서 구조는 프리팹 상속 비중이 큼 |
| `Hunter SE - Greensward - SRS.unity` | Greensward + SRS 변형 | `Trees`, `Volumes`, `Wind`, `Reflections`, `Lights`, `iPhone` | Greensward 계열의 기능 변형 |
| `Hunter SE - Greensward Variable Friction.unity` | 마찰 계수 변화 테스트 | `Trees`, `Volumes`, `Wind`, `Reflections`, `CLI Manager`, `iPhone` | Greensward 환경에 제어/물성 실험 요소가 추가된 형태 |
| `Hunter SE - Greensward with Obstacles.unity` | 장애물 포함 환경 테스트 | `Trees`, `Volumes`, `Wind`, `Reflections`, `Cones`, `iPhone` | Greensward에 장애물 검증 요소가 들어간 씬 |

## Structural Notes

### 1. Base Scene vs Test Scene

`Hunter SE.unity`와 `Hunter SE - LIDAR Test.unity`의 차이는 분명하다.

- `Hunter SE.unity`
  - 기본 환경 루트가 풍부하다
  - LiDAR가 씬에 있으나 베이스 상태에 가깝다
  - 기존 Velodyne 계열 흔적이 남아 있다
- `Hunter SE - LIDAR Test.unity`
  - 테스트 코스와 장애물 큐브가 루트에 직접 배치된다
  - LiDAR 검증을 위해 차량/센서 구성이 더 명시적으로 드러난다
  - Ouster 구성을 확인하기 가장 좋은 씬이다

### 2. Hunter SE 계열에서 직접 손대기 쉬운 씬

다음 씬들은 차량/센서가 씬 안에 직접 드러나므로 수정 포인트를 찾기 쉽다.

- `Hunter SE.unity`
- `Hunter SE - PID.unity`
- `Hunter SE - CBF.unity`
- `Hunter SE - Data Collection.unity`
- `Hunter SE - LIDAR Test.unity`

### 3. 환경만 바꾼 씬

다음 씬들은 구조 자체보다 환경 루트 차이가 핵심이다.

- `Hunter SE - New.unity`
- `Hunter SE - Tiny Town.unity`
- `Hunter SE - Greensward.unity`
- `Hunter SE - Greensward - SRS.unity`
- `Hunter SE - Greensward Variable Friction.unity`
- `Hunter SE - Greensward with Obstacles.unity`

## Hunter SE - LIDAR Test Detail

파일:

- `AutoDRIVE-Simulator/Assets/Scenes/Hunter SE - LIDAR Test.unity`

### 1. Scene Overview

| 항목 | 내용 |
| --- | --- |
| 씬 이름 | `Hunter SE - LIDAR Test.unity` |
| 주 용도 | Hunter SE 차량에서 LiDAR 외관, point cloud, 카메라 프리뷰, HUD/UI 동작을 같이 검증하는 테스트 씬 |
| 차량 루트 | `Hunter SE` 차량 1대가 메인 테스트 대상 |
| 센서 구조 | `Sensors` 아래에 `LIDAR`가 직접 드러나며, Ouster 계열 설정이 씬 안에 명시돼 있음 |
| UI 구조 | 공용 `User Interface` 프리팹을 씬에서 override 해서 카메라 프리뷰, HUD, Menu, Driving Mode, Rendering Quality를 사용 |
| 카메라 구조 | `Rear Camera`, `Driver's Eye`, `Front Camera`와 UI용 기본 카메라 전환 로직을 함께 사용 |
| 분석 포인트 | Hunter SE 계열에서 Ouster LiDAR와 HUD/Menu 상호작용을 보기 가장 쉬운 씬 |

### 2. Top-Level Roots

| 루트 오브젝트 | 역할 |
| --- | --- |
| `Floor` | 기본 바닥 |
| `Infrastructure (1)` | 테스트용 구조물 1 |
| `Infrastructure (2)` | 테스트용 구조물 2 |
| `Waypoints` | 경로 포인트 묶음. 기본 상태는 비활성 |
| `Cube`, `Cube (1)` ~ `Cube (7)` | LiDAR 반응과 시야를 보기 위한 장애물 큐브 |
| `fishhook_ccw_t_0_2_run_01` | fishhook 코스 |
| `skidpad_ccw_t_0_2_s_0_3142` | skidpad 코스 |
| `slalom_ccw_t_0_2_s_0_5236` | slalom 코스 |
| `straight_even_t_0_2_run_01` | 직선 코스 |
| `StaticLightingSky` | 정적 조명용 하늘 |
| `HDRP Compositor` | HDRP 관련 구성 |
| `SceneIDMap` | 씬 식별/매핑용 루트 |
| `Hunter SE` | 차량 본체와 센서/카메라의 루트 |

### 3. Vehicle And Sensor Structure

| 항목 | 내용 |
| --- | --- |
| 차량 계열 | Hunter SE |
| 차량 하위 트리 | 차량 루트 아래에 `Cameras`, `Sensors`, 차체/휠 메쉬, 액추에이터 관련 트리가 함께 존재 |
| `Sensors` 오브젝트 | 활성 상태 |
| `LIDAR` 오브젝트 | 활성 상태 |
| LiDAR 외관 이름 | `Ouster OS1_64` |
| LiDAR 모델 설정 | `modelPreset: 8` |
| 측정 중심 오프셋 | `centerOfMeasurementVerticalLinearOffsetMm: 37.7`, `centerOfMeasurementHorizontalLinearOffsetMm: 0` |
| LiDAR 장착 위치 | `LIDAR` Transform은 센서 루트 기준 위쪽에 배치되어 있고 yaw 90도 회전 상태 |
| 확인 포인트 | Hunter SE 계열 씬 중 Ouster 적용 상태를 가장 직접적으로 확인하기 좋음 |

### 4. Camera Configuration

| 카메라 | 용도 | 특징 |
| --- | --- | --- |
| `Rear Camera` | 차량 뒤 시점 | RenderTexture 출력용으로도 사용됨 |
| `Driver's Eye` | 운전자 시점 | 전체 레이어 렌더링에 가깝고 기본 차량 시점 역할 |
| `Front Camera` | 차량 앞 시점 | RenderTexture 출력용으로도 사용됨 |
| `Free Camera` | 공용 카메라 전환 기본값 | `CameraSwitch`의 기본 카메라 이름이 `Free Camera`로 override 되어 있음 |

### 5. UI / HUD Configuration

| 항목 | 내용 |
| --- | --- |
| UI 프리팹 | 공용 `User Interface` 프리팹을 씬에서 override |
| 기본 카메라 이름 | `Free Camera` |
| HUD 패널 | 차량 상태 텍스트, 카메라 프리뷰, 데이터 기록 버튼 포함 |
| Menu 패널 | Driving Mode, Camera Switch, Rendering Quality, Scene Light, Scene Reset, Quit 등 포함 |
| 카메라 프리뷰 | HUD 내부에 `Vehicle Front Camera Preview`, `Vehicle Rear Camera Preview` 형태로 배치 |
| Data Recorder | HUD 내부 버튼으로 배치되어 있음 |
| Socket / Connection | Menu 내부에서 연결 토글 가능하도록 override 되어 있음 |

### 6. Behaviour Notes

| 항목 | 설명 |
| --- | --- |
| HUD / Menu 기본 상태 | 공용 UI 프리팹 기준으로 `HUD`, `Menu` 패널은 기본 비활성 |
| Menu를 켜고 시작할 때 | Menu 하위 스크립트들도 시작부터 활성화되므로 Driving Mode 같은 UI 로직이 바로 실행될 수 있음 |
| Driving Mode 이슈 | 기존에는 Menu가 활성 시작되면 `DrivingMode.Start()`가 차량을 `Autonomous`로 강제해 입력이 먹지 않는 것처럼 보였음. 현재는 시작 시 기존 모드를 유지하도록 수정됨 |
| Rendering Quality 이슈 | Menu 활성 시작 시 `RenderingQuality`가 null 참조로 죽던 문제가 있었고, 현재는 null guard를 넣어 방어함 |
| HUD 표시 효과 | HUD를 켜면 텍스트와 차량 카메라 프리뷰가 같이 열리므로 화면 우측 영역이 크게 바뀌는 것이 정상 |
| Waypoints | 씬에는 존재하지만 기본 비활성이라 바로 보이지 않을 수 있음 |

### 7. Practical Interpretation

| 질문 | 이 씬에서 보면 되는 것 |
| --- | --- |
| Ouster가 실제로 적용됐는가 | `LIDAR`와 `Ouster OS1_64`, `modelPreset: 8`, `37.7 / 0` 파라미터를 확인 |
| point cloud와 외관을 같이 검증할 수 있는가 | 가능. 이 씬은 장애물 큐브와 테스트 코스가 같이 있어 확인이 쉬움 |
| UI 영향까지 같이 볼 수 있는가 | 가능. HUD/Menu를 켠 상태에서 카메라 프리뷰, Driving Mode, Rendering Quality 영향을 한 번에 볼 수 있음 |
| Hunter SE 계열 수정 기준 씬으로 적합한가 | 적합. LiDAR와 UI가 모두 직접 드러나 있어 구조 파악이 쉽다 |

### 8. Related Files And Paths

주의:

- 아래 목록은 `Hunter SE - LIDAR Test.unity`를 분석하거나 수정할 때 직접 확인해야 하는 주요 파일들이다.
- 씬이 참조하는 모든 자산을 완전히 열거한 목록은 아니고, 구조 파악과 문제 추적에 중요한 파일 위주다.

| 구분 | 파일 경로 | 역할 |
| --- | --- | --- |
| 씬 파일 | `AutoDRIVE-Simulator/Assets/Scenes/Hunter SE - LIDAR Test.unity` | LiDAR Test 씬 본체 |
| 공용 Hunter SE 프리팹 | `AutoDRIVE-Simulator/Assets/Prefabs/Hunter SE/Hunter SE.prefab` | Hunter SE 차량 공통 구조의 기준 프리팹 |
| 공용 UI 프리팹 | `AutoDRIVE-Simulator/Assets/Prefabs/Utilities/User Interface.prefab` | HUD, Menu, CameraSwitch, Driving Mode, Rendering Quality, Data Recorder가 들어 있는 UI 프리팹 |
| Ouster 모델 | `AutoDRIVE-Simulator/Assets/Models/Sensors/Ouster OS1-64/os1_64.dae` | Ouster 외관 mesh 자산 |
| 전방 프리뷰 RenderTexture | `AutoDRIVE-Simulator/Assets/Render Textures/Vehicle 1 Front Camera.renderTexture` | HUD의 차량 전방 카메라 프리뷰 출력 |
| 후방 프리뷰 RenderTexture | `AutoDRIVE-Simulator/Assets/Render Textures/Vehicle 1 Rear Camera.renderTexture` | HUD의 차량 후방 카메라 프리뷰 출력 |
| 차량 제어 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/VehicleController.cs` | Hunter SE 수동/자율 주행 입력 처리 |
| Driving Mode 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/DrivingMode.cs` | Menu의 Manual / Autonomous 전환 로직 |
| CameraSwitch 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/CameraSwitch.cs` | Driver's Eye / Front / Rear / Free Camera 전환 |
| HUD 토글 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/HUDPanel.cs` | HUD 패널 표시/숨김 |
| Menu 토글 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/MenuPanel.cs` | Menu 패널 표시/숨김 |
| HUD 텍스트 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/HUDText.cs` | 속도, IMU, GNSS, LiDAR 측정값 텍스트 갱신 |
| 데이터 기록 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/DataRecorder.cs` | HUD의 데이터 기록 버튼 및 카메라 프레임 저장 |
| 렌더링 품질 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/RenderingQuality.cs` | Low / High / Ultra Quality 전환 |
| 장면 조명 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/SceneLighting.cs` | Menu의 Scene Light 토글 |
| 장면 리셋 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/SceneReset.cs` | Menu의 씬 재로드 |
| 종료 스크립트 | `AutoDRIVE-Simulator/Assets/Scripts/Quit.cs` | Menu의 애플리케이션 종료 |
| RGL SceneManager | `AutoDRIVE-Simulator/Assets/Plugins/RGLUnity Plugin/Scripts/SceneManager.cs` | collider/mesh를 RGL 객체로 변환해 LiDAR 스캔 대상 구성 |
| RGL LidarSensor | `AutoDRIVE-Simulator/Assets/Plugins/RGLUnity Plugin/Scripts/LidarSensor.cs` | LiDAR capture 실행 |
| RGL LaserArrayLibrary | `AutoDRIVE-Simulator/Assets/Plugins/RGLUnity Plugin/Scripts/LidarModels/LaserArrayLibrary.cs` | Ouster preset과 center-of-measurement 오프셋 정의 |
| RGL LaserArray | `AutoDRIVE-Simulator/Assets/Plugins/RGLUnity Plugin/Scripts/LidarModels/LaserArray.cs` | LiDAR 측정 중심 오프셋을 실제 좌표계에 적용 |

## Hunter SE Robot Structure

`Hunter SE - LIDAR Test.unity`에서 사용되는 Hunter SE 로봇의 계층 구조와 구성 파일을 정리한다.

### 1. GameObject 계층 구조

```
Hunter SE                          ← 차량 루트 (Rigidbody, SkidSteerController)
├── Body                           ← 차체 메쉬 (Hunter SE.fbx → 차체 파트)
├── Wheel_FL                       ← 앞 왼쪽 바퀴 메쉬 + WheelCollider
├── Wheel_FR                       ← 앞 오른쪽 바퀴 메쉬 + WheelCollider
├── Wheel_RL                       ← 뒤 왼쪽 바퀴 메쉬 + WheelCollider
├── Wheel_RR                       ← 뒤 오른쪽 바퀴 메쉬 + WheelCollider
├── Sensors                        ← 센서 루트
│   ├── GPS                        ← GNSS 위치 센서 (GPS.cs)
│   ├── IMU                        ← 9-DOF 관성 센서 (IMU.cs)
│   ├── Encoder_L                  ← 왼쪽 휠 엔코더 (WheelEncoder.cs)
│   ├── Encoder_R                  ← 오른쪽 휠 엔코더 (WheelEncoder.cs)
│   └── LIDAR                      ← LiDAR 루트 (LIDAR3D.cs + LidarSensor)
│       └── Ouster OS1_64          ← LiDAR 외관 메쉬 (os1_64.dae)
├── Cameras                        ← 카메라 루트
│   ├── Driver's Eye               ← 운전자 시점 카메라
│   ├── Front Camera               ← 전방 카메라 (RenderTexture 출력)
│   ├── Rear Camera                ← 후방 카메라 (RenderTexture 출력)
│   └── Free Camera                ← 자유 시점 카메라 (FreeLookCamera.cs)
└── Actuators                      ← 액추에이터 루트 (TwistController.cs)
```

### 2. 로봇 물리 구조

| 항목 | 값 |
| --- | --- |
| 구동 방식 | 4WD 스키드 스티어 (전/후 좌우 독립 토크) |
| 트랙 폭 (trackWidth) | 141.54 mm |
| 최대 선속도 (linVelLimit) | 0.26 m/s |
| 최대 각속도 (angVelLimit) | 0.42 rad/s |
| 모터 토크 (MotorTorque) | 2.352 N·m |
| 선속도 게인 (linearGain) | 6.8 |
| 각속도 게인 (angularGain) | 0.008 |
| 제어 루프 | FixedUpdate (~50 Hz) |

스키드 스티어 구동식 — 좌우 바퀴에 서로 다른 토크를 줘서 방향을 바꾼다. 조향 각도가 없고, 4개 WheelCollider가 모두 구동축이다.

```
토크 공식:
  좌측 = linearGain * v - angularGain * w * trackWidth
  우측 = linearGain * v + angularGain * w * trackWidth
```

Socket.IO를 통해 Python에서 `"V1 Linear Velocity"`, `"V1 Angular Velocity"` 명령을 받으면 `TwistController`가 PID 루프로 `SkidSteerController`의 목표값을 추종한다.

### 3. 센서 구성

| 센서 | 스크립트 | 출력 데이터 | 업데이트 루프 |
| --- | --- | --- | --- |
| GNSS (GPS) | `GPS.cs` | 위치 [x, y, z] (m) | FixedUpdate |
| IMU | `IMU.cs` | 자세 Quaternion/Euler, 선가속도, 각속도 | FixedUpdate |
| 휠 엔코더 (×2) | `WheelEncoder.cs` | 틱 수, 회전각, 휠 속도 | FixedUpdate |
| 3D LiDAR | `LIDAR3D.cs` | 포인트클라우드 바이트 배열 (PCL12/24/48) | RGL 콜백 |

LiDAR 파이프라인:

```
LidarSensor (RGL)
  └─ ConnectToLidarFrame()
       └─ rglSubgraphLidar  (좌표 변환: Unity → Robotics 좌표계)
            ├─ rglSubgraphPcl12  (XYZ만, 12 bytes/pt)
            ├─ rglSubgraphPcl24  (XYZ + Intensity + RingID, 24 bytes/pt)
            └─ rglSubgraphPcl48  (PCL24 + Azimuth + Distance + Timestamp, 48 bytes/pt)
```

좌표 변환 행렬 (Unity → ROS 관례):

```
Robotics X = Unity Z
Robotics Y = Unity -X
Robotics Z = Unity Y
```

### 4. 통신 구조

`Socket.cs`가 차량 전체의 통신 허브 역할을 한다.

```
Python (Socket.IO "Bridge" 이벤트)
  │  "V1 Linear Velocity": float
  │  "V1 Angular Velocity": float
  │  "V1 Reset": "true"
  ▼
Socket.cs  →  TwistController  →  SkidSteerController  →  WheelCollider ×4
           →  ResetManager (리셋 플래그)

Unity (EmitTelemetry)
  │  "V1 LiDAR": 포인트클라우드 base64
  │  "V1 PosX/Y/Z": 위치
  │  "V1 AngR/P/Y": 자세
  │  "V1 Linear/Angular Velocity": 속도
  │  "V1 Collision": 충돌 여부
  ▼
Python RL 환경 (HunterSEEnv)
```

### 5. 구성 파일 목록

#### 3D 모델 (메쉬/재질)

| 파일 | 역할 |
| --- | --- |
| `Assets/Models/Vehicle/Hunter SE/Hunter SE.fbx` | 차체 + 바퀴 전체 메쉬 |
| `Assets/Models/Vehicle/Hunter SE/Frame.fbx` | 차체 프레임 단독 메쉬 |
| `Assets/Models/Vehicle/Hunter SE/Materials/Material #*.mat` | 차체 재질 (금속, 고무, 플라스틱 등) |
| `Assets/Models/Sensors/Ouster OS1-64/os1_64.dae` | Ouster OS1-64 LiDAR 외관 메쉬 |
| `Assets/Models/Sensors/Ouster OS1-64/os1_64.stl` | STL 포맷 동일 외관 |

#### 프리팹

| 파일 | 역할 |
| --- | --- |
| `Assets/Prefabs/Hunter SE/Hunter SE.prefab` | 차량 전체 구조의 기준 프리팹 |

#### 제어 스크립트

| 파일 | 역할 |
| --- | --- |
| `Assets/Scripts/SkidSteerController.cs` | 4WD 스키드 스티어 구동 — WheelCollider 토크 계산 |
| `Assets/Scripts/TwistController.cs` | 선/각속도 명령을 PID로 추종하여 SkidSteerController에 전달 |
| `Assets/Scripts/VehicleController.cs` | 범용 차량 제어 인터페이스 (수동/자율 모드 전환) |
| `Assets/Scripts/ResetManager.cs` | 에피소드 리셋 플래그 처리 — 초기 위치로 복귀 |

#### 센서 스크립트

| 파일 | 역할 |
| --- | --- |
| `Assets/Scripts/LIDAR3D.cs` | RGL 포인트클라우드 수신 및 PCL12/24/48 포맷 변환 |
| `Assets/Scripts/GPS.cs` | Unity Transform → ROS 좌표계 위치 변환 |
| `Assets/Scripts/IMU.cs` | 9-DOF IMU — 자세/각속도/선가속도 측정 |
| `Assets/Scripts/WheelEncoder.cs` | WheelCollider RPM → 엔코더 틱/속도 변환 |

#### 통신 스크립트

| 파일 | 역할 |
| --- | --- |
| `Assets/Scripts/Socket.cs` | Socket.IO 브리지 — Python 명령 수신 / 텔레메트리 송신 |
| `Assets/Scripts/Telemetry.cs` | SimRacingStudio API용 텔레메트리 (선택적) |

#### RGL 플러그인 (LiDAR 물리 시뮬레이션)

| 파일 | 역할 |
| --- | --- |
| `Assets/Plugins/RGLUnity Plugin/Scripts/LidarSensor.cs` | LiDAR 스캔 실행 및 콜백 |
| `Assets/Plugins/RGLUnity Plugin/Scripts/LidarModels/LidarModels.cs` | LidarModel 열거형 (OusterOS1_64 등) |
| `Assets/Plugins/RGLUnity Plugin/Scripts/LidarModels/LaserArrayLibrary.cs` | 채널별 수직/수평 각도 오프셋 정의 |
| `Assets/Plugins/RGLUnity Plugin/Scripts/LidarModels/LidarConfigurationLibrary.cs` | 모델별 horizontalSteps, maxRange 설정 |
| `Assets/Plugins/RGLUnity Plugin/Scripts/LidarModels/LaserArray.cs` | centerOfMeasurement 오프셋 적용 |

## Recommended Reference Scenes

Hunter SE 계열을 분석하거나 수정할 때는 아래 순서로 보는 것이 가장 효율적이다.

1. `Hunter SE.unity`
   - 기본 차량/환경 구조 확인
2. `Hunter SE - PID.unity`
   - 제어 실험용 구조 확인
3. `Hunter SE - Data Collection.unity`
   - 다양한 트랙 요소 확인
4. `Hunter SE - LIDAR Test.unity`
   - LiDAR 시각화와 센서 적용 확인
