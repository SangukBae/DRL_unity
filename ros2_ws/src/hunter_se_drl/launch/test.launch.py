# test.launch.py
#
# [역할]
# 학습된 모델을 테스트하기 위한 두 ROS2 노드를 동시에 실행한다.
#   1. autodrive_env  : Socket.IO ↔ ROS2 서비스 브릿지 (환경 서버)
#   2. test           : 테스트 루프 (모델 로드 → 에피소드 실행 → 결과 저장)
#
# [구현 내용]
# train_sac.launch.py와 동일한 구조.
# test_config.yaml의 algorithm 항목을 읽어 SAC 또는 TQC 모델을 로드한다.
# 테스트 노드에 model_path 파라미터를 전달하는 방식으로 구현한다.
