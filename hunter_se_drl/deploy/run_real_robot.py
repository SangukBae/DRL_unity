#!/usr/bin/env python3
"""
학습된 SB3 모델을 실제 Hunter SE 로봇에서 실행하는 ROS2 노드.

사전 조건:
    # 터미널 1: CAN 드라이버
    ros2 launch hunter_base hunter_base.launch.py port_name:=can0

    # 터미널 2: LiDAR 드라이버 (실제 LiDAR 종류에 맞게 교체)
    ros2 launch <lidar_package> <lidar_launch_file>

실행:
    # 터미널 3: RL 에이전트
    cd /home/sangukbae/autodrive/hunter_se_drl
    python deploy/run_real_robot.py \\
        --model_path models/saved/sac/final_model \\
        --algorithm  sac \\
        --goal_x 3.0 --goal_z 3.0
"""
import argparse
import math
import os
import sys
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from stable_baselines3 import PPO, SAC

from utils.file_manager import load_yaml
from utils.lidar_utils import preprocess_lidar

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "env_config.yaml")


class RealRobotAgent(Node):
    """
    학습된 SB3 모델을 실제 Hunter SE에서 실행하는 ROS2 노드.

    구독:
        /scan  (sensor_msgs/LaserScan)  - 실제 LiDAR
        /odom  (nav_msgs/Odometry)      - 로봇 위치·자세

    발행:
        /cmd_vel (geometry_msgs/Twist)  - hunter_base_node가 CAN으로 전달
    """

    def __init__(self, model, cfg: dict, goal_pos: tuple):
        super().__init__("hunter_se_rl_agent")

        self._model          = model
        self._lidar_dim      = cfg["lidar_dim"]
        self._lidar_max      = cfg["lidar_max_range"]
        self._max_lin        = cfg["max_linear_vel"]
        self._max_ang        = cfg["max_angular_vel"]
        self._goal_threshold = cfg["goal_threshold"]
        self._goal_x, self._goal_z = goal_pos  # (x, z) — ROS: x forward, z unused → 실제: x,y

        # 내부 상태
        self._lidar_ranges  = np.full(self._lidar_dim, self._lidar_max, dtype=np.float32)
        self._robot_x       = 0.0
        self._robot_y       = 0.0   # ROS2 좌표계: Y = 전진 방향 (수평면)
        self._robot_yaw     = 0.0
        self._odom_received = False
        self._lock          = threading.Lock()

        # 구독
        self.create_subscription(LaserScan, "/scan",  self._scan_cb,  10)
        self.create_subscription(Odometry,  "/odom",  self._odom_cb,  10)

        # 발행
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # 제어 루프 (10 Hz)
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f"RL 에이전트 시작 | 목표: ({self._goal_x:.2f}, {self._goal_z:.2f})"
        )

    # ── 콜백 ──────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        with self._lock:
            self._lidar_ranges = ranges

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        q   = msg.pose.pose.orientation
        with self._lock:
            self._robot_x    = pos.x
            self._robot_y    = pos.y
            self._robot_yaw  = self._quat_to_yaw(q.x, q.y, q.z, q.w)
            self._odom_received = True

    # ── 제어 루프 ──────────────────────────────────────────────────────────

    def _control_loop(self):
        with self._lock:
            lidar_ranges = self._lidar_ranges.copy()
            rx, ry, yaw  = self._robot_x, self._robot_y, self._robot_yaw

        obs = self._build_observation(lidar_ranges, rx, ry, yaw)

        # 목표 도달 확인
        dist = math.hypot(self._goal_x - rx, self._goal_z - ry)
        if dist < self._goal_threshold:
            self.get_logger().info(f"목표 도달! 거리: {dist:.3f}m — 정지합니다.")
            self._publish_cmd(0.0, 0.0)
            return

        # 모델 추론
        action, _ = self._model.predict(obs, deterministic=True)
        linear_vel  = float(action[0]) * self._max_lin
        angular_vel = float(action[1]) * self._max_ang

        self._publish_cmd(linear_vel, angular_vel)

        self.get_logger().debug(
            f"목표까지 {dist:.2f}m | lin={linear_vel:.2f} ang={angular_vel:.2f}"
        )

    # ── 헬퍼 ──────────────────────────────────────────────────────────────

    def _build_observation(
        self, lidar_ranges: np.ndarray, rx: float, ry: float, yaw: float
    ) -> np.ndarray:
        """시뮬레이터 학습과 동일한 관측 벡터 생성."""
        lidar_norm = preprocess_lidar(lidar_ranges, self._lidar_dim, self._lidar_max)

        # ROS2 좌표계: x=전진, y=좌측
        # 목표 방향 계산 (goal_x, goal_z → ROS x, y로 매핑)
        dx       = self._goal_x - rx
        dy       = self._goal_z - ry
        dist     = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx) - yaw
        # 각도 정규화 [-π, π]
        angle_to_goal = (angle_to_goal + math.pi) % (2 * math.pi) - math.pi

        arena_diag = self._lidar_max * 2 * math.sqrt(2)
        dist_norm  = np.clip(dist / arena_diag, 0.0, 1.0)
        angle_norm = (angle_to_goal / math.pi + 1.0) / 2.0

        return np.concatenate(
            [lidar_norm, [dist_norm, angle_norm]]
        ).astype(np.float32)

    def _publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def parse_args():
    parser = argparse.ArgumentParser(description="실제 Hunter SE RL 에이전트")
    parser.add_argument("--model_path", type=str,  required=True, help="SB3 모델 파일 경로")
    parser.add_argument("--algorithm",  type=str,  default="sac", choices=["sac", "ppo"])
    parser.add_argument("--goal_x",     type=float, default=3.0,  help="목표 X 좌표 (m)")
    parser.add_argument("--goal_z",     type=float, default=3.0,  help="목표 Z(Y) 좌표 (m)")
    # rclpy 인자 필터링
    return parser.parse_args(args=[
        a for a in sys.argv[1:] if not a.startswith("__")
    ])


def main():
    args = parse_args()

    # 환경 설정 로드
    cfg = load_yaml(CONFIG_PATH)["env"]

    # 모델 로드
    ModelClass = SAC if args.algorithm == "sac" else PPO
    model = ModelClass.load(args.model_path)
    print(f"모델 로드 완료: {args.model_path} ({args.algorithm.upper()})")

    # ROS2 초기화 및 실행
    rclpy.init()
    node = RealRobotAgent(
        model    = model,
        cfg      = cfg,
        goal_pos = (args.goal_x, args.goal_z),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 요청 — 로봇 정지")
        node._publish_cmd(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
