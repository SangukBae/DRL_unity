# test/

학습된 모델을 AutoDRIVE Unity 시뮬레이터에서 테스트하는 모듈.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `test_agent.py` | 학습된 모델 로드 후 테스트 에피소드 실행 및 결과 저장 |

## 실행 방법

```bash
python test/test_agent.py --model_path models/saved/final_model.zip
```

## 학습과의 차이점

| 항목 | 학습 (train_*.py) | 테스트 (test_agent.py) |
|------|-------------------|----------------------|
| 액션 선택 | 확률적 (탐험 포함) | 결정론적 (deterministic=True) |
| 모델 업데이트 | O | X |
| 결과 저장 | 모델 체크포인트 | 에피소드 보상·성공률 |
