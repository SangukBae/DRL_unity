# utils/

공통 유틸리티 모듈.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `lidar_utils.py` | AutoDRIVE LiDAR 데이터 디코딩 (base64 + gzip) |
| `file_manager.py` | 모델·로그 저장 경로 관리, YAML 로드·저장 |

## 사용처

| 파일 | 사용하는 utils |
|------|---------------|
| `envs/hunter_se_env.py` | `lidar_utils.decode_lidar()`, `file_manager.load_yaml()` |
| `train/train_sac.py` | `file_manager.load_yaml()` |
| `train/train_ppo.py` | `file_manager.load_yaml()` |
| `test/test_agent.py` | `file_manager.load_yaml()`, `file_manager.save_json()` |
