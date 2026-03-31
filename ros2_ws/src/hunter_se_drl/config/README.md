# config/

환경 및 학습 파라미터 설정 파일 모음.
모든 하이퍼파라미터와 환경 설정은 코드가 아닌 이 폴더의 yaml 파일에서 관리한다.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `environment.yaml` | Hunter SE 환경 파라미터 (LiDAR·속도·아레나·Socket.IO) |
| `hyperparameters_sac.yaml` | SAC 알고리즘 하이퍼파라미터 |
| `hyperparameters_tqc.yaml` | TQC 알고리즘 하이퍼파라미터 |
| `train_sac_config.yaml` | SAC 학습 루프 설정 |
| `train_tqc_config.yaml` | TQC 학습 루프 설정 |
| `test_config.yaml` | 테스트 실행 설정 |

## 로드 방법

`scripts/utils/file_manager.py`의 `load_yaml()` 함수로 로드한다.
`autodrive_env.py`와 `train_*.py`가 시작 시 각자 필요한 파일을 로드한다.
