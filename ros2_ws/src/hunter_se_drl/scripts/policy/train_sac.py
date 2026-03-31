#!/usr/bin/env python3
#
# train_sac.py
#
# ============================================================
# [역할]
# SAC 알고리즘으로 Hunter SE를 학습시키는 메인 학습 루프.
# drl_agent의 train_sac_agent.py를 기반으로,
# config 파일 경로를 hunter_se_drl 패키지 경로로 수정한 버전.
# ============================================================
#
# ============================================================
# [구현 내용]
# ============================================================
#
# 1. 초기화
#    - rclpy.init()
#    - utils/file_manager.py로 train_sac_config.yaml, hyperparameters_sac.yaml 로드
#    - EnvInterface 노드 생성
#    - /get_dimensions 호출 → state_dim, action_dim, max_action 획득
#    - SACAgent(state_dim, action_dim, max_action, hyperparams) 초기화
#    - ReplayBuffer(state_dim, action_dim, buffer_size) 초기화
#    - 난수 시드 설정 (/seed 서비스 호출)
#
# 2. 워밍업 (timesteps_before_training 스텝)
#    - env.reset() → state
#    - for each step:
#        action = env.sample_action_space()
#        next_state, reward, done, target = env.step(action)
#        buffer.add(state, action, next_state, reward, done)
#        done이면 env.reset()
#
# 3. 학습 루프 (max_timesteps 스텝)
#    - action = agent.select_action(state)
#    - next_state, reward, done, target = env.step(action)
#    - buffer.add(...)
#    - batch = buffer.sample(batch_size)
#    - agent.train(batch)
#    - done이면 env.reset()
#
# 4. 평가 (eval_freq 스텝마다)
#    - evaluate=True로 eval_episodes 에피소드 실행
#    - 평균 보상, 성공률 출력 및 로깅
#
# 5. 저장 (checkpoint_freq 스텝마다)
#    - agent.save(model_save_dir)
#
# [수정 필요 항목] (drl_agent 코드 대비)
# - config 파일 경로: hunter_se_drl 패키지의 config/ 디렉토리로 변경
# - import 경로: 같은 패키지 내의 sac_agent, buffer, env_interface import
