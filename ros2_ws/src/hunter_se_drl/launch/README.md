# launch/

ROS2 노드 동시 실행 설정 파일 모음.
각 launch 파일은 `autodrive_env.py`(환경 서버)와 학습/테스트 스크립트를 함께 실행한다.

## 파일 목록

| 파일 | 실행 노드 | 용도 |
|------|-----------|------|
| `train_sac.launch.py` | autodrive_env + train_sac | SAC 학습 |
| `train_tqc.launch.py` | autodrive_env + train_tqc | TQC 학습 |
| `test.launch.py` | autodrive_env + test | 학습된 모델 테스트 |

## 실행 순서

```
1. autodrive_env.py 노드 시작
   → Socket.IO로 AutoDRIVE 연결 대기
   → ROS2 서비스 (/step, /reset 등) 준비

2. train_*.py 또는 test.py 노드 시작
   → autodrive_env 서비스가 준비될 때까지 대기
   → 학습 또는 테스트 루프 시작
```

## 실행 방법

```bash
ros2 launch hunter_se_drl train_sac.launch.py
ros2 launch hunter_se_drl train_tqc.launch.py
ros2 launch hunter_se_drl test.launch.py
```
