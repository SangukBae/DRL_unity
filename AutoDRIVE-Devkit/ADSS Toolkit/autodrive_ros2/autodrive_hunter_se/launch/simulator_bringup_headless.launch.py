from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Hunter SE 브릿지 노드만 실행 (RViz 없음)
        Node(
            package='autodrive_hunter_se',
            executable='autodrive_bridge',
            name='autodrive_hunter_se_bridge',
            emulate_tty=True,
            output='screen',
        ),
    ])
