#!/usr/bin/env python3
#
# train_tqc.py
#
# ============================================================
# [역할]
# TQC 알고리즘으로 Hunter SE를 학습시키는 메인 학습 루프.
# drl_agent의 train_tqc_agent.py를 기반으로,
# config 파일 경로를 hunter_se_drl 패키지 경로로 수정한 버전.
# ============================================================
#
# ============================================================
# [구현 내용]
# ============================================================
# train_sac.py와 동일한 구조.
# SACAgent 대신 TQCAgent를 사용하고,
# hyperparameters_tqc.yaml, train_tqc_config.yaml을 로드한다.
#
# TQC는 SAC 대비 더 보수적인 Q-value 추정으로
# 과대추정 문제를 완화하므로 안정적인 학습이 가능하다.
#
# [수정 필요 항목] (drl_agent 코드 대비)
# - config 파일 경로: hunter_se_drl 패키지의 config/ 디렉토리로 변경
# - import 경로: 같은 패키지 내의 tqc_agent, buffer, env_interface import
