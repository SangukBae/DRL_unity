# Hunter SE - Random Obstacles 시뮬레이션

TQC 강화학습에 사용되는 `hunter_se_unity/Hunter SE - Random Obstacles.unity` 씬 문서 목록.

## 문서 구성

| 파일 | 내용 |
| --- | --- |
| [SCENE.md](SCENE.md) | 씬 구조, 최상위 GameObjects, 물리 설정 |
| [ROBOT.md](ROBOT.md) | Hunter SE 로봇 사양, 차량 계층 구조 |
| [LIDAR.md](LIDAR.md) | Ouster OS1-128 LiDAR 상세, RGL 파이프라인 |
| [ARENA.md](ARENA.md) | 장애물 아레나, 랜덤 배치 시스템 |
| [COMMUNICATION.md](COMMUNICATION.md) | Socket.IO 프로토콜, 텔레메트리 전체 키 목록 |
| [RL_COMPONENTS.md](RL_COMPONENTS.md) | RL 전용 컴포넌트 (Reset Manager, RLVisualizer) |

## 빠른 참조

| 항목 | 값 |
| --- | --- |
| 씬 파일 | `hunter_se_unity/Hunter SE - Random Obstacles.unity` |
| 물리 업데이트 | 1000 Hz (Fixed Timestep 0.001 s) |
| LiDAR | Ouster OS1-128, 1024 steps, 200 m, 10 Hz |
| 아레나 | 30 × 30 m, 정적 35 + 동적 10 장애물 |
| 최대 속도 | 1.33 m/s (4.8 km/h) |
| 통신 포트 | 4567 (Socket.IO) |
