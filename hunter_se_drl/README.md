# hunter_se_drl

Hunter SE 로봇 + AutoDRIVE Unity 시뮬레이터를 위한 강화학습 패키지.
SB3(Stable-Baselines3)를 기반으로 하며, 향후 모델 구조 변경이 용이하도록 설계되었다.

## 패키지 구조

```
hunter_se_drl/
├── envs/
│   └── hunter_se_env.py     # Gymnasium 환경 + eventlet WebSocket 서버 통합
├── models/
│   └── custom_policy.py     # SB3 커스텀 Policy (모델 구조 변경 시 여기만 수정)
├── train/
│   ├── train_sac.py         # SB3 SAC으로 학습
│   └── train_ppo.py         # SB3 PPO으로 학습
├── test/
│   └── test_agent.py        # 학습된 모델 테스트
├── config/
│   ├── env_config.yaml      # 환경 파라미터 (LiDAR·속도·아레나·포트 등)
│   ├── train_sac_config.yaml
│   └── train_ppo_config.yaml
└── utils/
    ├── lidar_utils.py       # base64 + gzip LiDAR 디코딩
    └── file_manager.py      # 모델·로그 저장 경로 관리, YAML 로드·저장
```

## 시스템 구조

```
[호스트 - Unity AutoDRIVE]              [Docker - hunter_se_drl]
  Hunter SE - Random Obstacles 씬
  Socket.IO (Engine.IO) 클라이언트
         |                                        |
         |  WebSocket (port 4567)                 |
         └──────────────────────────────────────→ eventlet WebSocket 서버
                                                  hunter_se_env.py
                                                  (Gymnasium Env)
                                                       |
                                                  SB3 (SAC / PPO)
                                                  train_sac.py
                                                  train_ppo.py
```

## 통신 방식

Python이 **서버**, Unity가 **클라이언트**로 접속한다.
`test_collision.py`와 동일한 eventlet WebSocket + Engine.IO 프로토콜을 사용한다.

**Unity → Python (텔레메트리)**
- `"V1 LIDAR Range Array"` : base64+gzip 인코딩된 LiDAR 거리 배열
- `"V1 Collisions"`        : 누적 충돌 카운터 (int)
- `"V1 Position"`          : 로봇 위치 문자열

**Python → Unity (제어)**
- `"V1 Linear Velocity"`   : 선형 속도 (m/s)
- `"V1 Angular Velocity"`  : 각속도 (rad/s)
- `"V1 Reset"`             : 에피소드 리셋 ("true" / "false")

## 실행 방법

```bash
# Docker 내부에서 실행
cd /home/sangukbae/autodrive/hunter_se_drl

# SAC 학습
python train/train_sac.py

# PPO 학습
python train/train_ppo.py

# 테스트
python test/test_agent.py
```

## 의존성

- `stable-baselines3` : `/home/sangukbae/autodrive/stable-baselines3` (editable 설치)
- `gymnasium`         : Gymnasium 환경 인터페이스
- `eventlet`          : WebSocket 서버
- `numpy`             : LiDAR 데이터 처리
- `torch`             : 신경망 (SB3 백엔드)
