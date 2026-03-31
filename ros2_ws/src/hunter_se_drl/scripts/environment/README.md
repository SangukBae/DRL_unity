# scripts/environment/

AutoDRIVE Unity 시뮬레이터와의 통신을 담당하는 모듈.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `autodrive_env.py` | 핵심 브릿지: Socket.IO로 AutoDRIVE 수신·송신 + ROS2 서비스 제공 |
| `env_interface.py` | 에이전트 측 ROS2 서비스 클라이언트 (/step·/reset 호출) |

## 통신 흐름

```
[Unity AutoDRIVE]                        [autodrive_env.py]                    [train_*.py]
      |                                        |                                     |
      |-- Socket.IO "V1 LiDAR" ----------→     |                                     |
      |-- Socket.IO "V1 PosX/Y/Z" ------→     | state 갱신                           |
      |-- Socket.IO "V1 Collision" -----→     |                                     |
      |-- Socket.IO "Goal PosX/Z" ------→     |                                     |
      |                                        |←── ROS2 /step(action) ─────────────|
      |                                        |    reward 계산                       |
      |←-- Socket.IO "V1 Linear Vel" ─────────|    (state, reward, done) 반환 ──────→|
      |←-- Socket.IO "V1 Angular Vel" ────────|                                     |
      |                                        |←── ROS2 /reset ────────────────────|
      |←-- Socket.IO "V1 Reset" ──────────────|    초기 state 반환 ─────────────────→|
      |←-- Socket.IO "Spawn PosX/Z" ──────────|                                     |
      |←-- Socket.IO "Goal PosX/Z" ───────────|                                     |
```

## 제공 ROS2 서비스 (autodrive_env.py → train_*.py)

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/step` | Step.srv | action 수신 → Unity 전송 → (state, reward, done) 반환 |
| `/reset` | Reset.srv | Unity 리셋 명령 → 초기 state 반환 |
| `/get_dimensions` | GetDimensions.srv | 상태/액션 공간 크기 반환 |
| `/seed` | Seed.srv | 환경 난수 시드 설정 |
| `/action_space_sample` | SampleActionSpace.srv | 랜덤 액션 샘플 반환 |
