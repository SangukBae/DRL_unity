# envs/

AutoDRIVE Unity 시뮬레이터와 통신하는 Gymnasium 환경 모듈.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `hunter_se_env.py` | Gymnasium 환경 + eventlet WebSocket 서버 + ROS2 퍼블리셔 |

## 핵심 설계

`hunter_se_env.py`는 세 가지 역할을 동시에 수행한다.

```
                    hunter_se_env.py
                          |
         ┌────────────────┼────────────────┐
         │                │                │
 Gymnasium Env    eventlet WebSocket    ROS2 노드 (선택)
 - reset()        - Unity 접속 수신    - 토픽 발행
 - step(action)   - 텔레메트리 파싱   - TF 브로드캐스트
 - observation    - 제어 명령 송신    - Goal Marker 발행
 - action_space   - ping/pong 처리
```

## 상태 공간 (Observation Space)

```
[ LiDAR 거리 배열 (360,) | dist_to_goal | angle_to_goal ]  → shape=(362,)
  └── base64+gzip 디코딩      └── 로봇 위치 + 목표 위치로 계산
      → 정규화 (0~1)              dist: 0~1, angle: 0~1
```

## 액션 공간 (Action Space)

```
Box([-1, -1], [1, 1])
  action[0] × max_linear_vel  → V1 Linear Velocity  (m/s)
  action[1] × max_angular_vel → V1 Angular Velocity (rad/s)
```

## 보상 함수

6가지 항목의 합산, [-1, 1]로 클리핑.

| 항목 | 내용 |
|------|------|
| 진행 보상 | 목표에 가까워진 거리 × k_p |
| 헤딩 보너스 | cos(theta_err) × k_h |
| 곡률 패널티 | 원운동 억제 (회전 / 속도) |
| 장애물 패널티 | LiDAR 최소 거리 기반 |
| 시간 패널티 | 매 스텝 -step_pen |
| 터미널 | 목표 도달 +10, 충돌 -10 |

## 스레드 구조

```
[OS 스레드 - SB3 학습]          [eventlet 데몬 스레드]
    reset() / step()    ←──── _telemetry_event (threading.Event)
    _send_control()     ────→ _send_queue (queue.Queue) ──→ ws.send()
                                   ↑
                              ws_handler만 ws.send() 호출 (스레드 안전)
```

## ROS2 발행 토픽

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
