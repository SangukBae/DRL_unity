# deploy/

학습된 SB3 모델을 실제 Hunter SE 로봇에서 실행하는 스크립트.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `run_real_robot.py` | 학습된 모델을 ROS2 노드로 실행. `/scan` + `/odom` 구독 → `/cmd_vel` 발행 |

## 시스템 구조

```
[실제 Hunter SE 로봇]
  CAN 버스 (can0)
       ↕
  hunter_base_node      ← hunter_ros2 패키지
  /cmd_vel (Twist)      ← run_real_robot.py 가 publish
  /odom (Odometry)      → run_real_robot.py 가 subscribe

[실제 LiDAR]
  /scan (LaserScan)     → run_real_robot.py 가 subscribe
```

## 실행 순서

```bash
# 터미널 1: CAN 드라이버 기동
ros2 launch hunter_base hunter_base.launch.py port_name:=can0

# 터미널 2: LiDAR 드라이버 (실제 LiDAR 종류에 맞게 교체)
# 예) RPLIDAR
ros2 launch rplidar_ros rplidar_a3_launch.py

# 터미널 3: RL 에이전트 실행
cd /home/sangukbae/autodrive/hunter_se_drl
python deploy/run_real_robot.py \
    --model_path models/saved/sac/final_model \
    --algorithm  sac \
    --goal_x 3.0 --goal_z 3.0
```

## 인자 설명

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `--model_path` | (필수) | 학습된 SB3 모델 파일 경로 |
| `--algorithm` | `sac` | 알고리즘 (`sac` 또는 `ppo`) |
| `--goal_x` | `3.0` | 목표 위치 X 좌표 (m) |
| `--goal_z` | `3.0` | 목표 위치 Z(Y) 좌표 (m) |

## 주의사항

- `env_config.yaml`의 `max_linear_vel`, `max_angular_vel`이 실제 로봇 안전 범위 내인지 확인
- LiDAR 빔 수·최대 거리가 달라도 `preprocess_lidar()`가 자동 리샘플링·정규화
- 처음 실행 시 낮은 속도(max_linear_vel: 0.5 이하)로 테스트 권장
