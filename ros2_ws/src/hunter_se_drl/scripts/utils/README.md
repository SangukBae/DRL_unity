# scripts/utils/

공통 유틸리티 모듈.

## 파일 목록

| 파일 | 역할 | 작업 |
|------|------|------|
| `file_manager.py` | 모델·로그 저장 경로 관리, YAML 로드·저장 | drl_agent에서 그대로 복사 |
| `reward.py` | Hunter SE 전용 보상 함수 | 신규 작성 |

## 사용처

- `file_manager.py`: autodrive_env.py, train_sac.py, train_tqc.py에서 config 로드 시 사용
- `reward.py`: autodrive_env.py의 step_callback에서 매 스텝마다 호출
