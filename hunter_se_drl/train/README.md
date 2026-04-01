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
cd /autodrive

# SAC 학습
python hunter_se_drl/train/train_sac.py

# PPO 학습
python hunter_se_drl/train/train_ppo.py
```

## 학습 흐름

```
_preflight_check()        ← 준비 상태 점검 (config·패키지·포트·ROS2)
    ↓ FAIL이면 종료
config 로드
    ↓
HunterSEEnv 생성          ← eventlet 서버 + ROS2 퍼블리셔 초기화
    ↓
_launch_rviz()            ← RViz2 백그라운드 자동 실행
    ↓
SB3 모델 생성 (SAC 또는 PPO)
    ↓
model.learn(total_timesteps=...)
    ↓                     ← SB3가 내부적으로 reset() / step() 반복 호출
모델 저장 (checkpoints/ + models/)
    ↓
env.close() + RViz2 종료
```

## 준비 상태 점검 항목

학습 파일 실행 시 가장 먼저 출력된다.

| 상태 | 의미 |
|------|------|
| `[OK]` | 정상 |
| `[WARN]` | 없어도 학습 가능 (RViz2 관련) |
| `[FAIL]` | 반드시 해결 필요 — 자동으로 종료됨 |

점검 항목:
- config yaml 파일 존재 여부
- PyTorch, SB3, Gymnasium, eventlet, numpy import
- CUDA 사용 가능 여부
- rclpy import (RViz2 시각화)
- RViz2 setup.bash / .rviz config 파일 존재
- 포트 4567 점유 여부
