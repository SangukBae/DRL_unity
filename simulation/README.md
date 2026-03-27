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
