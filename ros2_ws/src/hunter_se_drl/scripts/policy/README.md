# scripts/policy/

강화학습 알고리즘 및 학습 루프를 담당하는 모듈.
`sac_agent.py`, `tqc_agent.py`, `buffer.py`는 drl_agent 패키지에서 그대로 복사한다.
`train_sac.py`, `train_tqc.py`는 복사 후 config 경로만 수정한다.

## 파일 목록

| 파일 | 역할 | 작업 |
|------|------|------|
| `sac_agent.py` | SAC Actor·Critic 신경망 및 업데이트 로직 | 그대로 복사 |
| `tqc_agent.py` | TQC 분위수 Critic 및 업데이트 로직 | 그대로 복사 |
| `buffer.py` | 리플레이 버퍼 (균등·우선순위 샘플링) | 그대로 복사 |
| `train_sac.py` | SAC 학습 루프 전체 | 복사 후 config 경로 수정 |
| `train_tqc.py` | TQC 학습 루프 전체 | 복사 후 config 경로 수정 |

## 학습 루프 흐름 (train_*.py 공통)

```
1. config 로드 (train_*_config.yaml, hyperparameters_*.yaml)
2. EnvInterface 초기화 → /get_dimensions 호출로 state_dim, action_dim 획득
3. Agent 초기화 (SAC 또는 TQC)
4. Buffer 초기화

[워밍업 단계]
5. timesteps_before_training 동안 랜덤 액션으로 버퍼 채우기
   reset() → for each step: sample_action_space() → step(action) → buffer.add()

[학습 단계]
6. 매 스텝마다:
   a. agent.select_action(state)
   b. env.step(action) → (next_state, reward, done, target)
   c. buffer.add(state, action, next_state, reward, done)
   d. agent.train(buffer.sample())
   e. done이면 env.reset()

[평가]
7. eval_freq 스텝마다:
   - 탐험 없이 deterministic 액션으로 eval_episodes 에피소드 실행
   - 평균 보상, 성공률 로깅

[저장]
8. checkpoint_freq 스텝마다 모델 저장
```
