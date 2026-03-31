# train_tqc.launch.py
#
# [역할]
# TQC 학습에 필요한 두 ROS2 노드를 동시에 실행한다.
#   1. autodrive_env  : Socket.IO ↔ ROS2 서비스 브릿지 (환경 서버)
#   2. train_tqc      : TQC 학습 루프 (에이전트 클라이언트)
#
# [구현 내용]
# train_sac.launch.py와 동일한 구조.
# train_sac_node 대신 train_tqc_node를 실행한다.
#
# [주의]
# train_sac.launch.py 주의사항 동일.
