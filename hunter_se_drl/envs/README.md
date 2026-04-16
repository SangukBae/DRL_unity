# envs/

AutoDRIVE Unity 시뮬레이터와 통신하는 Gymnasium 환경 모듈.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `hunter_se_env.py` | 기저 Gymnasium 환경 — eventlet WebSocket 서버 + ROS2 퍼블리셔 |
| `hunter_se_env_tqc.py` | TQC 전용 래퍼 — PI 속도 제어기 + 확장 관측 + TQC 보상 함수 |

---

## hunter_se_env.py — HunterSEEnv

### 핵심 설계

```
                    HunterSEEnv
                          |
         ┌────────────────┼────────────────┐
         │                │                │
 Gymnasium Env    eventlet WebSocket    ROS2 노드 (선택)
 - reset()        - Unity 접속 수신    - 토픽 발행
 - step(action)   - 텔레메트리 파싱   - TF 브로드캐스트
 - observation    - 제어 명령 송신    - Goal Marker 발행
 - action_space   - ping/pong 처리
```

### 관측 공간 (SAC / PPO 기준)

```
shape = (lidar_dim + 3,) = (363,)

[ LiDAR 거리 배열 (360,) | dist_norm | angle_norm | theta_err ]
  └── Base64 decode → XYZ 파싱       └── 목표까지 거리/각도     └── 헤딩 오차
      → 높이 필터 → 방위각 투영             dist:  [0, 1]              [-1, 1]
      → 80-bin 리샘플 → 정규화             angle: [0, 1]
```

| 요소 | 인덱스 | 범위 | 설명 |
|------|--------|------|------|
| `lidar_norm` | `[0:360]` | `[0, 1]` | arena 대각선(`≈42.4m`)으로 클리핑 후 정규화 |
| `dist_norm` | `[360]` | `[0, 1]` | 목표까지 거리 / arena 대각선 |
| `angle_norm` | `[361]` | `[0, 1]` | 목표 방위각 / 2π |
| `theta_err` | `[362]` | `[-1, 1]` | 헤딩 오차 / π |

### 행동 공간 (SAC / PPO 기준)

```
Box([0, -1], [1, 1])
  action[0] × max_linear_vel  → V1 Linear Velocity  (m/s)   전진 전용
  action[1] × max_angular_vel → V1 Angular Velocity (rad/s)
```

### 보상 함수 (SAC / PPO 기준)

| 항목 | 내용 |
|------|------|
| 진행 보상 | 목표에 가까워진 거리 × k_p |
| 헤딩 보너스 | cos(theta_err) × k_h |
| 곡률 패널티 | 원운동 억제 (회전 / 속도) |
| 장애물 패널티 | LiDAR 최소 거리 기반 |
| 시간 패널티 | 매 스텝 -step_pen |
| 터미널 | 목표 도달 +10, 충돌 -10 |

결과는 `[-1, 1]`로 클리핑.

### Zone 정의 (장애물 패널티용)

LiDAR index 0 = 전방, CCW 증가

| Zone | 각도 범위 | 임계 거리 |
|------|-----------|-----------|
| Z0 Front Center | ±20° | 0.70 m |
| Z1 Front Left | 20°~70° | 0.64 m |
| Z2 Front Right | -70°~-20° | 0.64 m |
| Z3 Side Left | 70°~130° | 0.40 m |
| Z4 Side Right | -130°~-70° | 0.40 m |
| Z5 Rear | 130°~230° | 0.32 m |

### 스레드 구조

```
[OS 스레드 - SB3 학습]          [eventlet 데몬 스레드]
    reset() / step()    ←──── _telemetry_event (threading.Event)
    _send_control()     ────→ _send_queue (queue.Queue) ──→ ws.send()
                                   ↑
                              ws_handler만 ws.send() 호출 (스레드 안전)
```

### ROS2 발행 토픽

rclpy가 설치된 환경에서 자동 활성화. 없어도 학습 정상 동작.

| 토픽 | 타입 |
|------|------|
| `/autodrive/hunter_se_1/lidar` | `sensor_msgs/LaserScan` |
| `/autodrive/hunter_se_1/imu` | `sensor_msgs/Imu` |
| `/autodrive/hunter_se_1/ips` | `geometry_msgs/Point` |
| `/autodrive/hunter_se_1/linear_vel` | `std_msgs/Float32` |
| `/autodrive/hunter_se_1/angular_vel` | `std_msgs/Float32` |
| `/autodrive/hunter_se_1/collisions` | `std_msgs/Int32` |
| `/autodrive/hunter_se_1/goal` | `visualization_msgs/Marker` |
| TF | `map → hunter_se_1 → lidar / imu` |

---

## hunter_se_env_tqc.py — HunterSEEnvTQC

`HunterSEEnv`를 `gym.Wrapper`로 감싸 TQC 학습 전용으로 확장한다.  
기저 `HunterSEEnv`는 수정하지 않으므로 SAC / PPO / COX-Q에 영향 없음.

### 관측 공간

```
shape = (lidar_dim + 8,) = (368,)

[ lidar_norm(360) | dist_norm | theta_err_norm
  | goal_x_body_norm | goal_y_body_norm
  | prev_throttle_exec | prev_steering_exec
  | throttle_exec | steering_exec ]
```

| 요소 | 인덱스 | 범위 | 설명 |
|------|--------|------|------|
| `lidar_norm` | `[0:360]` | `[0, 1]` | arena 대각선으로 클리핑 후 정규화 |
| `dist_norm` | `[360]` | `[0, 1]` | 목표까지 거리 / arena 대각선 |
| `theta_err_norm` | `[361]` | `[-1, 1]` | 헤딩 오차 / π |
| `goal_x_body_norm` | `[362]` | `[-1, 1]` | body frame 전방 목표 오차 / arena 대각선 |
| `goal_y_body_norm` | `[363]` | `[-1, 1]` | body frame 측방 목표 오차 / arena 대각선 |
| `prev_throttle_exec` | `[364]` | `[-1, 1]` | 직전 스텝의 throttle 액추에이터 상태 |
| `prev_steering_exec` | `[365]` | `[-1, 1]` | 직전 스텝의 steering 액추에이터 상태 |
| `throttle_exec` | `[366]` | `[-1, 1]` | 현재 throttle 액추에이터 상태 (Unity `Applied Throttle`) |
| `steering_exec` | `[367]` | `[-1, 1]` | 현재 steering 액추에이터 상태 (Unity `Applied Steering`) |

`*_exec`: Unity telemetry의 `Applied Throttle/Steering`이 있으면 해당 값, 없으면 Python이 보낸 rate-limited command 사용.

### 행동 공간

```
Box([-1, -1], [1, 1])
  action[0]: target_speed_norm ∈ [-1, 1]  후진 허용
  action[1]: steering_norm     ∈ [-1, 1]
```

정책 출력은 target speed / steering이며, 래퍼 내부 PI 제어기가 throttle actuator 명령으로 변환한다.

### PI 속도 제어기

| 파라미터 | 값 | 설명 |
|---------|----|----|
| `TARGET_SPEED_KP` | 0.55 | 비례 게인 |
| `TARGET_SPEED_KI` | 0.08 | 적분 게인 |
| `TARGET_SPEED_INTEGRAL_CLAMP` | 0.75 | 적분 항 클램프 |
| `MAX_TARGET_SPEED_DELTA` | 0.25 / step | target speed rate-limit |

피드포워드(FF): 데이터셋 기반 throttle-velocity 비선형 보간 테이블 적용.

### 보상 함수

터미널 보상:

| 조건 | 보상 |
|------|------|
| 목표 도달 | `+50.0` |
| Hard collision | `-10.0` |
| Soft collision | `-8.0` |

비터미널 보상 (합산 후 `[-5, 5]` 클리핑):

| 항목 | 계수 | 설명 |
|------|------|------|
| 진행 | k_p=6.0 | Euclidean 거리 감소량 × k_p |
| Maneuver progress | k_shape=1.0 | Body-frame shaped distance 감소 (좌우 오차 1.5× 가중) |
| 헤딩 개선 | k_h=0.5 | 전진 중 theta 감소량 기반 (개선형) |
| Steering 정규화 | λ=0.02 | commanded steering_norm 크기 억제 |
| Yaw rate 패널티 | λ=0.05 | 실제 yaw rate 억제 |
| 후진 패널티 | λ=0.03 | 불필요한 후진 억제 |
| 장애물 근접 | w=0.8 | danger_ratio³ 기반 (3순위 폴백 시스템) |
| 고속 접근 패널티 | λ=2.5 + 1.0 | 장애물 근처 전진 속도 억제 |
| 제동 보상 | λ=1.25 | 장애물 근처 감속 시 보너스 |
| 시간 패널티 | 0.02 / step | 매 스텝 |
| Timeout 패널티 | 8.0 + dist×0.8 | 타임아웃 종료 시 명시적 패널티 |

### 장애물 위험도 판정 (3순위 폴백)

```
1순위: Rectangular Safety Region gap
       gap = LiDAR 거리 - 해당 방향 차체 경계 거리
       gap ≤ 0 → danger_linear = 1.0

2순위: 6-Zone threshold (Z0~Z5)
       speed-dependent margin 포함

3순위: 전방 / 전체 min_laser 기반
       d_safe = 0.50 + 0.15 × |v|
```

danger_ratio = danger_linear³ (위험구간 안에서만 패널티 가파르게 증가)  
safety_gate = max(0.20, 1 - danger_ratio) → 위험 시 진행/헤딩 보상 감쇠

---

## 공통 설정 파일

`config/env_config.yaml`

| 키 | 기본값 | 설명 |
|----|--------|------|
| `port` | 4567 | Socket.IO 포트 |
| `lidar_dim` | 360 | LiDAR 빔 수 |
| `lidar_max_range` | 200.0 m | LiDAR 최대 거리 |
| `max_linear_vel` | 1.333 m/s | 최대 선속도 |
| `max_angular_vel` | 0.702 rad/s | 최대 각속도 |
| `arena_size` | 30.0 m | 아레나 한 변 길이 |
| `goal_threshold` | 1.0 m | 목표 도달 판정 거리 |
| `max_episode_steps` | — | 에피소드 최대 스텝 |
| `step_timeout` | — | 스텝당 Unity 응답 타임아웃 (s) |
