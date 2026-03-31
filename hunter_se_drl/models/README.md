# models/

SB3 커스텀 Policy 모델 정의 모듈.
모델 구조를 변경하고 싶을 때 이 폴더의 파일만 수정하면 된다.

## 파일 목록

| 파일 | 역할 |
|------|------|
| `custom_policy.py` | SB3 커스텀 Actor-Critic Policy 정의 |

## 설계 의도

SB3의 기본 MlpPolicy 대신 커스텀 Policy를 사용하면
신경망 구조(레이어 수, 크기, 활성화 함수 등)를 자유롭게 변경할 수 있다.

```python
# train_sac.py에서 이렇게 사용
from models.custom_policy import HunterSEPolicy

model = SAC(
    policy=HunterSEPolicy,   # 커스텀 Policy 지정
    env=env,
    ...
)
```

## 모델 구조 변경 방법

`custom_policy.py`의 네트워크 파라미터만 수정하면
`train_sac.py`, `train_ppo.py` 코드 변경 없이 구조가 바뀐다.
