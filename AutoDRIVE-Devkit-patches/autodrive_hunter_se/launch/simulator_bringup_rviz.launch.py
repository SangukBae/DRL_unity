from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Hunter SE 브릿지 노드 (Unity 텔레메트리 → ROS 2 토픽)
        Node(
            package='autodrive_hunter_se',
            executable='autodrive_bridge',
            name='autodrive_hunter_se_bridge',
            emulate_tty=True,
            output='screen',
        ),
        # RViz2 시각화
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [FindPackageShare('autodrive_hunter_se'), '/rviz/simulator.rviz']],
        ),
    ])
