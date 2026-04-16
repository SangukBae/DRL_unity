# config/

환경 및 학습 파라미터 설정 파일 모음.
모든 하이퍼파라미터와 환경 설정은 코드가 아닌 이 폴더의 yaml 파일에서 관리한다.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `env_config.yaml` | 환경 파라미터 (LiDAR·속도·아레나·포트 등) |
| `train_sac_config.yaml` | SAC 학습 파라미터 |
| `train_ppo_config.yaml` | PPO 학습 파라미터 |

## 주요 env_config 항목

| 항목 | 기본값 | 설명 |
|------|--------|------|
| `port` | 4567 | Unity Socket.IO 연결 포트 |
| `lidar_dim` | 360 | LiDAR 빔 수 |
| `lidar_max_range` | 200.0 | LiDAR 최대 거리 (m) |
| `max_linear_vel` | 1.5 | 최대 선속도 (m/s) |
| `max_angular_vel` | 1.0 | 최대 각속도 (rad/s) |
| `arena_size` | 30.0 | 아레나 크기 (m) |
| `goal_threshold` | 0.5 | 목표 도달 판정 거리 (m) |
| `max_episode_steps` | 500 | 에피소드 최대 스텝 수 |
| `collision_debounce_sec` | 1.0 | 충돌 디바운싱 (초) |

## 로드 방법

`utils/file_manager.py`의 `load_yaml()` 함수로 로드한다.

```python
from utils.file_manager import load_yaml
config = load_yaml("config/env_config.yaml")
```
