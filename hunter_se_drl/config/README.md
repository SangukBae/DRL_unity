# config/

환경 및 학습 파라미터 설정 파일 모음.
모든 하이퍼파라미터와 환경 설정은 코드가 아닌 이 폴더의 yaml 파일에서 관리한다.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `env_config.yaml` | 환경 파라미터 (LiDAR·속도·아레나·포트 등) |
| `train_sac_config.yaml` | SAC 학습 파라미터 |
| `train_ppo_config.yaml` | PPO 학습 파라미터 |

## 로드 방법

`utils/file_manager.py`의 `load_yaml()` 함수로 로드한다.

```python
from utils.file_manager import load_yaml
config = load_yaml("config/env_config.yaml")
```
