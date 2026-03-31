#!/usr/bin/env python3
#
# file_manager.py
#
# ============================================================
# [역할]
# 모델/로그 저장 경로 관리 및 YAML 파일 로드/저장 유틸리티.
# Gazebo 의존성이 없는 순수 Python 코드이므로
# drl_agent 패키지의 file_manager.py를 그대로 복사한다.
# ============================================================
#
# ============================================================
# [구현 내용] - drl_agent/scripts/utils/file_manager.py 그대로 복사
# ============================================================
#
# def load_yaml(path) → dict:
#   - yaml.safe_load()로 yaml 파일 읽기
#   - 파일이 없으면 FileNotFoundError 발생
#
# def save_yaml(path, data):
#   - yaml.dump()로 dict를 yaml 파일로 저장
#
# def save_json(path, data):
#   - json.dump()로 dict를 json 파일로 저장
#
# class DirectoryManager:
#   __init__(path): 관리할 디렉토리 경로 설정
#   create(exist_ok=True): 디렉토리 생성 (os.makedirs)
#   remove_if_present(): 디렉토리가 있으면 삭제 (shutil.rmtree)
