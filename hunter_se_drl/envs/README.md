# envs/

AutoDRIVE Unity 시뮬레이터와 통신하는 Gymnasium 환경 모듈.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `hunter_se_env.py` | Gymnasium 환경 구현 + eventlet WebSocket 서버 통합 |

## 핵심 설계

`hunter_se_env.py`는 두 가지 역할을 동시에 수행한다.

```
                    hunter_se_env.py
                         |
          ┌──────────────┴──────────────┐
          │                             │
  Gymnasium Env                 eventlet WebSocket 서버
  - reset()                     - Unity 접속 수신
  - step(action)                - 텔레메트리 파싱
  - observation_space           - 제어 명령 송신
  - action_space
```

## 상태 공간 (Observation Space)

```
[ LiDAR 거리 배열 (N,) | dist_to_goal | angle_to_goal ]
  └── lidar_utils.py로 디코딩     └── 로봇 위치 + 목표 위치로 계산
```

## 액션 공간 (Action Space)

```
Box([-1, -1], [1, 1])
  action[0] → Linear Velocity  (정규화 후 실제 속도로 변환)
  action[1] → Angular Velocity (정규화 후 실제 각속도로 변환)
```
