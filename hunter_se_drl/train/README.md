# train/

SB3 알고리즘으로 Hunter SE를 학습시키는 학습 루프 모듈.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `train_sac.py` | SB3 SAC 알고리즘으로 학습 |
| `train_ppo.py` | SB3 PPO 알고리즘으로 학습 |

## 알고리즘 선택 기준

| 알고리즘 | 특징 | 권장 상황 |
|---------|------|-----------|
| SAC | 오프폴리시, 샘플 효율 높음, 연속 액션에 강함 | 기본 권장 |
| PPO | 온폴리시, 안정적, 하이퍼파라미터 튜닝 쉬움 | SAC 불안정 시 대안 |

## 실행 방법

```bash
# SAC 학습
python train/train_sac.py

# PPO 학습
python train/train_ppo.py
```

## 학습 흐름

```
config 로드
    ↓
HunterSEEnv 생성 (envs/hunter_se_env.py)
    ↓
SB3 모델 생성 (SAC 또는 PPO + HunterSEPolicy)
    ↓
model.learn(total_timesteps=...)
    ↓                    ← SB3가 내부적으로 reset() / step() 반복 호출
모델 저장
```
