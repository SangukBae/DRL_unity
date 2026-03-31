# hunter_se_drl

Hunter SE 로봇 + AutoDRIVE Unity 시뮬레이터를 위한 강화학습 ROS2 패키지.

## 패키지 구조

```
hunter_se_drl/
├── CMakeLists.txt                      # ROS2 빌드 설정, Python 스크립트 설치 경로 등록
├── package.xml                         # 패키지 의존성 선언 (rclpy, drl_agent_interfaces)
│
├── config/                             # 환경/학습 파라미터 설정 파일
│   ├── environment.yaml                # Hunter SE LiDAR·속도·아레나·Socket.IO 파라미터
│   ├── hyperparameters_sac.yaml        # SAC 신경망 크기·학습률·엔트로피 온도
│   ├── hyperparameters_tqc.yaml        # TQC 분위수 개수·Critic 수·학습률
│   ├── train_sac_config.yaml           # SAC 학습 루프 설정 (총 스텝 수·평가 주기 등)
│   ├── train_tqc_config.yaml           # TQC 학습 루프 설정
│   └── test_config.yaml                # 테스트 에피소드 수·모델 경로
│
├── launch/                             # 노드 동시 실행 설정
│   ├── train_sac.launch.py             # autodrive_env + train_sac 동시 실행
│   ├── train_tqc.launch.py             # autodrive_env + train_tqc 동시 실행
│   └── test.launch.py                  # autodrive_env + test 동시 실행
│
└── scripts/
    ├── environment/                    # 시뮬레이터 연동 담당
    │   ├── autodrive_env.py            # 핵심: Socket.IO ↔ ROS2 서비스 브릿지
    │   └── env_interface.py            # 에이전트 측 ROS2 서비스 클라이언트
    │
    ├── policy/                         # 강화학습 알고리즘 담당
    │   ├── sac_agent.py                # SAC Actor·Critic 신경망 및 업데이트 로직
    │   ├── tqc_agent.py                # TQC 분위수 Critic 및 업데이트 로직
    │   ├── buffer.py                   # 리플레이 버퍼 (균등·우선순위 샘플링)
    │   ├── train_sac.py                # SAC 학습 루프
    │   └── train_tqc.py                # TQC 학습 루프
    │
    └── utils/                          # 공통 유틸리티
        ├── file_manager.py             # 모델·로그 저장 경로 관리, YAML 로드·저장
        └── reward.py                   # 보상 함수
```

## 시스템 구조

```
[호스트 - Unity AutoDRIVE]                    [Docker - hunter_se_drl]
  Random Obstacles 씬                            autodrive_env.py
  Hunter SE 로봇                   Socket.IO         ↕
  RLEnvironmentManager.cs    ←────────────────→  ROS2 Services
  - LiDAR 데이터 송신                           (/step /reset ...)
  - 위치/충돌/목표 송신                              ↕
  - 제어 명령 수신                             train_sac.py / train_tqc.py
  - 리셋/스폰 수신                                   ↕
                                              sac_agent.py / tqc_agent.py
```

## 실행 방법

```bash
# SAC 학습
ros2 launch hunter_se_drl train_sac.launch.py

# TQC 학습
ros2 launch hunter_se_drl train_tqc.launch.py

# 테스트
ros2 launch hunter_se_drl test.launch.py
```

## 의존 패키지

- `drl_agent_interfaces`: Step/Reset/GetDimensions 등 ROS2 서비스 정의
- `drl_agent`: SAC/TQC/buffer 코드 참조 (scripts/policy에 복사)
