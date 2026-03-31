# train_sac.launch.py
#
# [역할]
# SAC 학습에 필요한 두 ROS2 노드를 동시에 실행한다.
#   1. autodrive_env  : Socket.IO ↔ ROS2 서비스 브릿지 (환경 서버)
#   2. train_sac      : SAC 학습 루프 (에이전트 클라이언트)
#
# [구현 내용]
# from launch import LaunchDescription
# from launch_ros.actions import Node
#
# def generate_launch_description():
#     autodrive_env_node = Node(
#         package='hunter_se_drl',
#         executable='autodrive_env.py',
#         name='autodrive_env',
#         output='screen',
#     )
#     train_sac_node = Node(
#         package='hunter_se_drl',
#         executable='train_sac.py',
#         name='train_sac',
#         output='screen',
#     )
#     return LaunchDescription([autodrive_env_node, train_sac_node])
#
# [주의]
# autodrive_env_node가 먼저 Socket.IO 연결과 ROS2 서비스 준비를 완료해야
# train_sac_node가 /step, /reset 서비스를 호출할 수 있다.
# train_sac.py 내부에서 서비스 준비 완료를 기다리는 로직이 필요하다.
