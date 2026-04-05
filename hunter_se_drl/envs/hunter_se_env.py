#!/usr/bin/env python3
import json
import math
import os
import queue
import threading
import time

import eventlet
import eventlet.wsgi
import gymnasium as gym
import numpy as np
from eventlet import websocket

from utils.file_manager import load_yaml
from utils.lidar_utils import decode_lidar, preprocess_lidar

# ── ROS2 (선택적 — 없어도 학습은 정상 동작) ───────────────────────────────────
try:
    import rclpy
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    import tf2_ros
    from std_msgs.msg import Float32, Int32, Header
    from geometry_msgs.msg import Point, TransformStamped
    from sensor_msgs.msg import Imu, LaserScan
    from visualization_msgs.msg import Marker
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False

# ── Engine.IO / Socket.IO 패킷 상수 ────────────────────────────────────────
EIO_OPEN    = "0"
EIO_PING    = "2"
EIO_PONG    = "3"
EIO_MESSAGE = "4"
SIO_EVENT   = "2"

OPEN_PACKET = EIO_OPEN + json.dumps({
    "sid": "autodrive",
    "upgrades": [],
    "pingInterval": 25000,
    "pingTimeout": 60000,
})

DEFAULT_CONTROL = {
    "V1 Reset": "false",
    "V1 CoSim": "0",
    "V1 Linear Velocity": "0",
    "V1 Angular Velocity": "0",
    "V1 Throttle": "0",
    "V1 Steering": "0",
    "V1 Brake": "0",
    "V1 Handbrake": "0",
    "V1 Headlights": "0",
    "V1 Indicators": "0",
    "Goal PosX": "0",
    "Goal PosZ": "0",
}

LIDAR_RANGE_MIN = 0.15   # m
LIDAR_OFFSET    = [0.10, 0.0, 0.25]
IMU_OFFSET      = [0.0,  0.0, 0.10]

# ── Hunter SE 물리 상수 (데이터셋 + 공식 메뉴얼 기준) ───────────────────────
# Throttle → 속도 비선형 매핑 (AutoDRIVE-Hunter-SE-Dataset/vehicle_parameters)
_THROTTLE_PTS = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
_VELOCITY_PTS = np.array([0.000, 0.318, 0.626, 0.927, 1.223, 1.516,
                           1.809, 2.109, 2.500, 3.016, 3.561])

# 최소 회전 반경 (공식 메뉴얼: 1.9 m)
R_MIN = 1.9  # m

# 액션 변화율 제한 (데이터셋 프레임 간 delta std ~0.022/frame @30Hz → RL ~4Hz 환산)
MAX_THROTTLE_DELTA = 0.25   # 정규화 단위 /step
MAX_STEERING_DELTA = 0.20   # 정규화 단위 /step


class HunterSEEnv(gym.Env):
    """
    AutoDRIVE Hunter SE 강화학습 환경.

    관측 공간: LiDAR(lidar_dim) + [dist_to_goal, angle_to_goal] → shape=(lidar_dim+2,)
    행동 공간: [linear_vel, angular_vel] ∈ [-1, 1]  (실제 속도로 스케일링 후 전송)

    ROS2가 설치된 환경에서는 학습 중에도 RViz2로 센서 데이터를 시각화할 수 있다.
    발행 토픽:
      /autodrive/hunter_se_1/lidar       (sensor_msgs/LaserScan)
      /autodrive/hunter_se_1/imu         (sensor_msgs/Imu)
      /autodrive/hunter_se_1/ips         (geometry_msgs/Point)
      /autodrive/hunter_se_1/linear_vel  (std_msgs/Float32)
      /autodrive/hunter_se_1/angular_vel (std_msgs/Float32)
      /autodrive/hunter_se_1/collisions  (std_msgs/Int32)
      /autodrive/hunter_se_1/goal        (visualization_msgs/Marker)
    TF: map → hunter_se_1 → lidar / imu
    """

    metadata = {"render_modes": []}

    def __init__(self, config_path: str = None):
        super().__init__()

        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(__file__), "..", "config", "env_config.yaml"
            )
        cfg = load_yaml(config_path)["env"]

        # 설정값
        self.host                 = cfg["host"]
        self.port                 = cfg["port"]
        self.lidar_dim            = cfg["lidar_dim"]
        self.lidar_max_range      = cfg["lidar_max_range"]
        self.max_linear_vel       = cfg["max_linear_vel"]
        self.max_angular_vel      = cfg["max_angular_vel"]
        self.arena_size           = cfg["arena_size"]
        self.goal_threshold       = cfg["goal_threshold"]
        self.max_episode_steps    = cfg["max_episode_steps"]
        self.step_timeout         = cfg["step_timeout"]
        self.collision_debounce   = cfg["collision_debounce_sec"]

        # Gymnasium 공간 정의
        obs_dim = self.lidar_dim + 2
        self.observation_space = gym.spaces.Box(
            low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32
        )
        # action[0]: 스로틀 [-1, 1] — 전진(+) / 후진(-), 최대 속도 동일 (±4.8 km/h)
        # action[1]: 조향   [-1, 1] — 좌(-) / 우(+)
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([ 1.0,  1.0], dtype=np.float32),
        )

        # 내부 상태
        self._ws                = None          # 현재 WebSocket 연결
        self._telemetry         = {}            # 최신 텔레메트리
        self._telemetry_event   = threading.Event()
        self._connected_event   = threading.Event()
        self._send_queue        = queue.Queue() # ws.send() 요청을 ws_handler에 전달
        self._prev_unity_count  = 0             # 이전 Unity 누적 충돌 카운터
        self._last_collision_at = 0.0
        self._ignore_spawn      = True
        self._goal_pos          = (0.0, 0.0)   # (x, z)
        self._robot_pos         = (0.0, 0.0)   # (x, z)
        self._robot_yaw         = 0.0          # 로봇 헤딩 (rad, 표준 수학 좌표계)
        self._prev_dist         = None
        self._min_laser         = None         # 보상 계산용 LiDAR 최소 거리
        self._prev_v            = 0.0          # 스무딩 보상용 이전 속도
        self._prev_w            = 0.0
        self._prev_throttle_norm = 0.0         # 변화율 제한용 이전 정규화 액션
        self._prev_steering_norm = 0.0
        self._step_count        = 0
        self._step_timeout_count = 0
        self._episode_count     = 0

        # 제어 명령 로그
        self._ctrl_log_file   = None
        self._ctrl_log_writer = None
        if cfg.get("control_log_enabled", False):
            log_dir = cfg.get("control_log_dir", "logs/control")
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(
                log_dir,
                time.strftime("control_%Y%m%d_%H%M%S.csv"),
            )
            self._ctrl_log_file = open(log_path, "w", buffering=1, encoding="utf-8")
            self._ctrl_log_file.write(
                "episode,step,timestamp,"
                "throttle,steering,linear_vel,angular_vel,reset\n"
            )
            print(f"[HunterSEEnv] 제어 명령 로그: {log_path}")

        # ROS2 퍼블리셔 초기화 (선택적)
        self._ros_node       = None
        self._ros_pubs       = {}
        self._tf_broadcaster = None
        self._ros_enabled    = False
        self._ros_warn_logged = False   # ROS 퍼블리시 첫 실패 시 1회 경고 출력용
        if _ROS2_AVAILABLE:
            self._init_ros()

        # eventlet WebSocket 서버를 데몬 스레드로 기동
        self._server_thread = threading.Thread(target=self._start_server, daemon=True)
        self._server_thread.start()

    # ── Gymnasium 인터페이스 ────────────────────────────────────────────────

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # 새 목표 위치 랜덤 생성 (아레나 내부, 로봇과 최소 3m 이상)
        half = self.arena_size / 2 - 2.0
        while True:
            gx = float(np.random.uniform(-half, half))
            gz = float(np.random.uniform(-half, half))
            if math.hypot(gx, gz) > 3.0:
                break
        self._goal_pos = (gx, gz)

        self._episode_count      += 1
        self._step_count          = 0
        self._prev_dist           = None
        self._ignore_spawn        = True
        self._last_collision_at   = 0.0
        self._prev_throttle_norm  = 0.0
        self._prev_steering_norm  = 0.0
        self._telemetry_event.clear()

        # Unity가 연결될 때까지 무한 대기 (Play 버튼을 누를 때까지 블로킹)
        if not self._connected_event.is_set():
            print("[HunterSEEnv] Unity 연결 대기 중... (Unity에서 Play 버튼을 눌러주세요)")
            self._connected_event.wait()
            print("[HunterSEEnv] Unity 연결됨, 학습을 시작합니다.")

        # Unity에 리셋 명령 전송
        self._send_control(0.0, 0.0, reset=True)

        # 새 목표 위치 마커 발행
        self._publish_goal_marker()

        # 첫 텔레메트리 대기
        if not self._telemetry_event.wait(timeout=self.step_timeout):
            print(
                "[HunterSEEnv] reset timeout: Unity 텔레메트리를 받지 못했습니다. "
                "Unity Play 상태와 Socket.IO 연결을 확인하세요."
            )
            obs = np.zeros(self.lidar_dim + 2, dtype=np.float32)
            return obs, {}

        # 스폰 직후 충돌 카운터 기준값 설정
        try:
            self._prev_unity_count = int(self._telemetry.get("V1 Collisions", 0))
        except (ValueError, TypeError):
            self._prev_unity_count = 0

        obs = self._get_observation()
        self._prev_dist = self._dist_to_goal()
        return obs, {}

    def step(self, action):
        # ── Fix 1: 액션 변화율 제한 ──────────────────────────────────────────
        # 한 스텝에 변화할 수 있는 정규화 액션의 폭을 제한한다.
        # (데이터셋 기준: throttle std ~0.022/frame @30Hz, RL ~4Hz 환산 → 0.25/step)
        throttle_norm = float(np.clip(
            action[0],
            self._prev_throttle_norm - MAX_THROTTLE_DELTA,
            self._prev_throttle_norm + MAX_THROTTLE_DELTA,
        ))
        steering_norm = float(np.clip(
            action[1],
            self._prev_steering_norm - MAX_STEERING_DELTA,
            self._prev_steering_norm + MAX_STEERING_DELTA,
        ))
        self._prev_throttle_norm = throttle_norm
        self._prev_steering_norm = steering_norm

        # ── Fix 3: 비선형 Throttle-Velocity 매핑 (전진/후진 대칭) ────────────
        # 부호를 분리해 절댓값으로 보간한 뒤 부호를 복원한다.
        # max_linear_vel(1.333 m/s)에 해당하는 실제 throttle: (1.333+0.010)/3.097 ≈ 0.434
        sign = 1.0 if throttle_norm >= 0.0 else -1.0
        throttle_real = abs(throttle_norm) * (self.max_linear_vel + 0.010) / 3.097
        linear_vel = sign * float(np.clip(
            np.interp(throttle_real, _THROTTLE_PTS, _VELOCITY_PTS),
            0.0, self.max_linear_vel,
        ))

        angular_vel = steering_norm * self.max_angular_vel

        # ── Fix 2: 속도 비례 각속도 상한 (애커만 물리 제약: ω ≤ |v| / R_min) ─
        w_cap = abs(linear_vel) / R_MIN
        angular_vel = float(np.clip(angular_vel, -w_cap, w_cap))

        self._telemetry_event.clear()
        self._send_control(linear_vel, angular_vel)

        # 다음 텔레메트리 대기
        if not self._telemetry_event.wait(timeout=self.step_timeout):
            self._step_timeout_count += 1
            if self._step_timeout_count == 1 or self._step_timeout_count % 20 == 0:
                print(
                    f"[HunterSEEnv] step timeout x{self._step_timeout_count}: "
                    "제어 명령에 대한 Unity 응답이 없습니다."
                )
            obs = np.zeros(self.lidar_dim + 2, dtype=np.float32)
            return obs, 0.0, False, True, {"timeout": True}

        self._step_timeout_count = 0
        self._step_count += 1

        collision      = self._detect_collision()
        dist           = self._dist_to_goal()
        target_reached = dist < self.goal_threshold
        terminated     = collision or target_reached
        truncated      = self._step_count >= self.max_episode_steps

        obs  = self._get_observation()  # min_laser 갱신 포함

        reward = self.get_reward(
            target=target_reached,
            collision=collision,
            v=linear_vel,
            w=angular_vel,
            prev_goal_dist=self._prev_dist if self._prev_dist is not None else dist,
            curr_goal_dist=dist,
            theta_err=self._theta_err(),
            zmins=None, zthrs=None,
            min_laser=self._min_laser,
            v_max=self.max_linear_vel,
            w_max=self.max_angular_vel,
            prev_v=self._prev_v,
            prev_w=self._prev_w,
        )
        self._prev_dist = dist
        self._prev_v    = linear_vel
        self._prev_w    = angular_vel
        info = {
            "collision":      collision,
            "target_reached": target_reached,
            "dist_to_goal":   dist,
            "step":           self._step_count,
        }
        return obs, reward, terminated, truncated, info

    def smoke_test(self) -> bool:
        """학습 전 씬 리셋과 로봇 제어 동작을 검증한다.

        검증 항목:
          1. 리셋 명령 → Unity 텔레메트리 수신 확인
          2. 전진 명령 10 스텝 → 로봇 실제 이동 확인
          3. 재리셋 → 로봇·목표 위치 변경 확인 (랜덤 배치)

        Returns:
            True  : 모든 검증 통과
            False : 하나 이상 실패 (학습 중단 권장)
        """
        SEP = "=" * 55

        print(f"\n{SEP}")
        print("  씬 제어 검증 (SmokeTest)")
        print(SEP)

        # ── 1) 첫 번째 리셋 ─────────────────────────────────────
        print("  [1/3] 씬 리셋 명령 전송...")
        obs, _ = self.reset()

        if obs.sum() == 0.0:
            print("  [FAIL] 리셋 후 Unity 텔레메트리를 받지 못했습니다.")
            print(f"{SEP}\n")
            return False

        pos1  = self._robot_pos
        goal1 = self._goal_pos
        tele_keys = list(self._telemetry.keys())
        print(f"  [OK]   리셋 완료")
        print(f"         텔레메트리 키: {tele_keys}")
        print(f"         V1 Position raw: {self._telemetry.get('V1 Position', '(없음)')}")
        print(f"         로봇 위치 : ({pos1[0]:.4f}, {pos1[1]:.4f})")
        print(f"         목표 위치 : ({goal1[0]:.2f}, {goal1[1]:.2f})")

        # ── 2) 제어 테스트 (전진 10 스텝) ────────────────────────
        print("  [2/3] 전진 제어 명령 전송 (10 스텝)...")
        for _ in range(10):
            _, _, terminated, truncated, info = self.step(np.array([0.5, 0.0], dtype=np.float32))
            if info.get("timeout"):
                print("  [FAIL] 제어 명령에 대한 Unity 응답이 없습니다.")
                print("         V1 CoSim, Socket.IO 연결, 씬 스크립트를 확인하세요.")
                print(f"{SEP}\n")
                return False
            if terminated or truncated:
                break

        pos2  = self._robot_pos
        moved = math.hypot(pos2[0] - pos1[0], pos2[1] - pos1[1])

        if moved < 0.05:
            print(f"  [FAIL] 제어 명령을 보냈지만 로봇이 움직이지 않았습니다.")
            print(f"         이동 거리: {moved:.3f} m (기준: 0.05 m)")
            print(f"         리셋 후 위치: ({pos1[0]:.4f}, {pos1[1]:.4f})")
            print(f"         10스텝 후 위치: ({pos2[0]:.4f}, {pos2[1]:.4f})")
            print(f"         V1 Position raw: {self._telemetry.get('V1 Position', '(없음)')}")
            print("         점검 사항:")
            print("           1) Unity 씬에서 VehicleController.DrivingMode = 1 확인")
            print("           2) Socket.cs Inspector: TwistControllers 비어있는지 확인")
            print("           3) UnityMainThreadDispatcher 오브젝트가 씬에 있는지 확인")
            print("           4) Hunter SE - Random Obstacles 씬이 실행 중인지 확인")
            print(f"{SEP}\n")
            return False

        print(f"  [OK]   로봇 이동 확인 — 이동 거리: {moved:.3f} m")

        # ── 3) 두 번째 리셋 (랜덤 배치 확인) ─────────────────────
        print("  [3/3] 재리셋 → 랜덤 위치 변경 확인...")
        obs, _ = self.reset()
        pos3  = self._robot_pos
        goal2 = self._goal_pos

        pos_changed  = math.hypot(pos3[0] - pos1[0], pos3[1] - pos1[1]) > 0.1
        goal_changed = math.hypot(goal2[0] - goal1[0], goal2[1] - goal1[1]) > 0.1

        print(f"  [{'OK' if pos_changed else 'WARN'}]   로봇 위치 변경: {pos_changed}  "
              f"({pos1} → {pos3})")
        print(f"  [{'OK' if goal_changed else 'WARN'}]   목표 위치 변경: {goal_changed}  "
              f"({goal1} → {goal2})")

        print("-" * 55)
        print("  모든 검증 통과 — 학습을 시작합니다.")
        print(f"{SEP}\n")
        return True

    def close(self):
        if self._ctrl_log_file is not None:
            try:
                self._ctrl_log_file.flush()
                self._ctrl_log_file.close()
            except Exception:
                pass
            self._ctrl_log_file = None

        if self._ros_enabled and self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
                self._ros_node = None
            except Exception:
                pass
            try:
                import time as _time
                _time.sleep(0.3)   # DDS 스레드 정리 대기
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    # ── ROS2 초기화 및 퍼블리시 ────────────────────────────────────────────

    def _init_ros(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._ros_node = rclpy.create_node('hunter_se_rl_bridge')
            self._ros_pubs = {
                'lidar':       self._ros_node.create_publisher(LaserScan, '/autodrive/hunter_se_1/lidar',       qos),
                'imu':         self._ros_node.create_publisher(Imu,       '/autodrive/hunter_se_1/imu',         qos),
                'ips':         self._ros_node.create_publisher(Point,     '/autodrive/hunter_se_1/ips',         qos),
                'linear_vel':  self._ros_node.create_publisher(Float32,   '/autodrive/hunter_se_1/linear_vel',  qos),
                'angular_vel': self._ros_node.create_publisher(Float32,   '/autodrive/hunter_se_1/angular_vel', qos),
                'collisions':  self._ros_node.create_publisher(Int32,     '/autodrive/hunter_se_1/collisions',  qos),
                'goal':        self._ros_node.create_publisher(Marker,    '/autodrive/hunter_se_1/goal',        qos),
            }
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self._ros_node)
            self._ros_enabled = True
            print('[HunterSEEnv] ROS2 퍼블리셔 초기화 완료 — RViz2 시각화 가능')
        except Exception as e:
            print(f'[HunterSEEnv] ROS2 초기화 실패 (학습은 계속됩니다): {e}')
            self._ros_enabled = False

    def _broadcast_tf(self, child: str, parent: str, translation: list, quaternion: list, stamp):
        tf = TransformStamped()
        tf.header.stamp            = stamp
        tf.header.frame_id         = parent
        tf.child_frame_id          = child
        tf.transform.translation.x = float(translation[0])
        tf.transform.translation.y = float(translation[1])
        tf.transform.translation.z = float(translation[2])
        tf.transform.rotation.x    = float(quaternion[0])
        tf.transform.rotation.y    = float(quaternion[1])
        tf.transform.rotation.z    = float(quaternion[2])
        tf.transform.rotation.w    = float(quaternion[3])
        self._tf_broadcaster.sendTransform(tf)

    def _publish_ros(self):
        """텔레메트리를 ROS2 토픽으로 발행한다. 실패해도 학습을 중단하지 않는다."""
        if not self._ros_enabled:
            return
        try:
            now = self._ros_node.get_clock().now().to_msg()
            tel = self._telemetry

            # 쿼터니언 파싱 (공통)
            try:
                q = [float(v) for v in tel.get('V1 Orientation Quaternion', '0 0 0 1').split()]
                if len(q) < 4:
                    q = [0.0, 0.0, 0.0, 1.0]
            except (ValueError, TypeError):
                q = [0.0, 0.0, 0.0, 1.0]

            # 1) IPS + TF
            pos_str = tel.get('V1 Position', '')
            if pos_str:
                try:
                    parts = [float(v) for v in pos_str.split()]
                    if len(parts) >= 3:
                        ips = Point()
                        ips.x, ips.y, ips.z = parts[0], parts[1], parts[2]
                        self._ros_pubs['ips'].publish(ips)

                        self._broadcast_tf('hunter_se_1', 'map',        parts,       q,         now)
                        self._broadcast_tf('lidar',       'hunter_se_1', LIDAR_OFFSET, [0,0,0,1], now)
                        self._broadcast_tf('imu',         'hunter_se_1', IMU_OFFSET,   [0,0,0,1], now)
                except (ValueError, TypeError):
                    pass

            # 2) IMU
            try:
                av = [float(v) for v in tel.get('V1 Angular Velocity', '0 0 0').split()]
                if len(av) < 3:
                    av = [0.0, 0.0, 0.0]
            except (ValueError, TypeError):
                av = [0.0, 0.0, 0.0]
            try:
                la = [float(v) for v in tel.get('V1 Linear Acceleration', '0 0 0').split()]
                if len(la) < 3:
                    la = [0.0, 0.0, 0.0]
            except (ValueError, TypeError):
                la = [0.0, 0.0, 0.0]

            imu = Imu()
            imu.header.stamp    = now
            imu.header.frame_id = 'imu'
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q[0], q[1], q[2], q[3]
            imu.orientation_covariance         = [0.0025,0.0,0.0,0.0,0.0025,0.0,0.0,0.0,0.0025]
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = av[0], av[1], av[2]
            imu.angular_velocity_covariance    = [0.0025,0.0,0.0,0.0,0.0025,0.0,0.0,0.0,0.0025]
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = la[0], la[1], la[2]
            imu.linear_acceleration_covariance = [0.0025,0.0,0.0,0.0,0.0025,0.0,0.0,0.0,0.0025]
            self._ros_pubs['imu'].publish(imu)

            # 3) LiDAR
            lidar_raw = tel.get('V1 LIDAR Range Array', '')
            ranges = decode_lidar(lidar_raw) if lidar_raw else np.full(self.lidar_dim, self.lidar_max_range, dtype=np.float32)
            try:
                scan_rate = float(tel.get('V1 LIDAR Scan Rate', '10'))
                if scan_rate <= 0:
                    scan_rate = 10.0
            except (ValueError, TypeError):
                scan_rate = 10.0

            ls = LaserScan()
            ls.header.stamp      = now
            ls.header.frame_id   = 'lidar'
            ls.angle_min         = -math.pi
            ls.angle_max         =  math.pi
            ls.angle_increment   = 2 * math.pi / max(len(ranges), 1)
            ls.time_increment    = (1.0 / scan_rate) / max(len(ranges), 1)
            ls.scan_time         = 1.0 / scan_rate
            ls.range_min         = LIDAR_RANGE_MIN
            ls.range_max         = self.lidar_max_range
            ls.ranges            = ranges.tolist()
            ls.intensities       = []
            self._ros_pubs['lidar'].publish(ls)

            # 4) 선속도
            lv = Float32()
            lv.data = float(tel.get('V1 Speed', 0))
            self._ros_pubs['linear_vel'].publish(lv)

            # 5) 각속도 (IMU Y축 = Unity 수직 회전축 = yaw rate)
            aw_msg = Float32()
            aw_msg.data = float(av[1])
            self._ros_pubs['angular_vel'].publish(aw_msg)

            # 6) 충돌 카운터
            col = Int32()
            col.data = int(tel.get('V1 Collisions', 0))
            self._ros_pubs['collisions'].publish(col)

        except Exception as e:
            if not self._ros_warn_logged:
                print(f'[HunterSEEnv] ROS2 퍼블리시 실패 (이후 경고 생략): {e}')
                self._ros_warn_logged = True

    def _publish_goal_marker(self):
        """현재 목표 위치를 RViz2 Marker로 발행한다."""
        if not self._ros_enabled:
            return
        try:
            m = Marker()
            m.header.stamp    = self._ros_node.get_clock().now().to_msg()
            m.header.frame_id = 'map'
            m.ns              = 'rl_goal'
            m.id              = 0
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD
            m.pose.position.x = float(self._goal_pos[0])
            m.pose.position.y = 0.5   # 높이 중앙 (실린더 높이 1m 기준)
            m.pose.position.z = float(self._goal_pos[1])
            m.pose.orientation.w = 1.0
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 1.0
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.8
            self._ros_pubs['goal'].publish(m)
        except Exception as e:
            if not self._ros_warn_logged:
                print(f'[HunterSEEnv] Goal marker 퍼블리시 실패 (이후 경고 생략): {e}')
                self._ros_warn_logged = True

    # ── 내부 헬퍼 ──────────────────────────────────────────────────────────

    def _get_observation(self) -> np.ndarray:
        lidar_raw = self._telemetry.get("V1 LIDAR Range Array", "")
        if lidar_raw:
            ranges = decode_lidar(lidar_raw)
        else:
            ranges = np.full(self.lidar_dim, self.lidar_max_range, dtype=np.float32)

        # 보상 함수용 min_laser (정규화 전 원시 거리값)
        valid = ranges[ranges > 0.01]
        self._min_laser = float(valid.min()) if len(valid) > 0 else None

        lidar_norm = preprocess_lidar(ranges, self.lidar_dim, self.lidar_max_range)

        dist       = self._dist_to_goal()
        angle      = self._angle_to_goal()
        dist_norm  = np.clip(dist / (self.arena_size * math.sqrt(2)), 0.0, 1.0)
        angle_norm = (angle / math.pi + 1.0) / 2.0   # [-π, π] → [0, 1]

        return np.concatenate([lidar_norm, [dist_norm, angle_norm]]).astype(np.float32)

    def _detect_collision(self) -> bool:
        now = time.monotonic()
        collision = False

        try:
            unity_count = int(self._telemetry.get("V1 Collisions", self._prev_unity_count))
        except (ValueError, TypeError):
            unity_count = self._prev_unity_count

        if unity_count > self._prev_unity_count:
            if self._ignore_spawn:
                self._ignore_spawn     = False
                self._prev_unity_count = unity_count
            elif now - self._last_collision_at > self.collision_debounce:
                collision              = True
                self._last_collision_at = now
            self._prev_unity_count = unity_count

        return collision

    def _dist_to_goal(self) -> float:
        rx, rz = self._robot_pos
        gx, gz = self._goal_pos
        return math.hypot(gx - rx, gz - rz)

    def _angle_to_goal(self) -> float:
        rx, rz = self._robot_pos
        gx, gz = self._goal_pos
        return math.atan2(gz - rz, gx - rx)

    @staticmethod
    def get_reward(
        target, collision,
        v, w,
        prev_goal_dist, curr_goal_dist,
        theta_err=None,
        zmins=None, zthrs=None,
        min_laser=None,
        v_max=1.5, w_max=1.0,
        k_p=2.0, progress_clip=0.25,
        lambda_k=0.35,
        z_weights=(0.6, 0.85, 1.0, 0.85, 0.6),
        safety_margin=1.5, w_obs=0.8,
        d_safe_base=0.55, d_safe_speed=0.30,
        k_h=0.3, step_pen=0.02,
        k_smooth=0.0, prev_v=None, prev_w=None,
        k_forward=0.15,
    ) -> float:
        # 터미널
        if target:    return 10.0
        if collision: return -10.0

        # 정규화
        v_n = v / max(v_max, 1e-6)
        w_n = w / max(w_max, 1e-6)

        # 1) 진행 보상
        delta_d  = np.clip(prev_goal_dist - curr_goal_dist, -progress_clip, progress_clip)
        progress = k_p * delta_d

        # 2) 전진 보너스 / 후진 페널티
        # v_n > 0이면 보너스, v_n < 0이면 페널티 (후진 억제)
        forward = k_forward * v_n

        # 3) 곡률 페널티 (원운동 억제)
        kappa    = abs(w_n) / (abs(v_n) + 1e-3)
        curv_pen = lambda_k * kappa

        # 4) 장애물 근접 페널티
        obstacle = 0.0
        if zmins is not None and zthrs is not None and len(zmins) == 5 and len(zthrs) == 5:
            deficits = []
            for i in range(5):
                thr_expanded = max(1e-6, safety_margin * float(zthrs[i]))
                d = max(0.0, 1.0 - (float(zmins[i]) / thr_expanded))
                deficits.append(d)
            wsum     = sum(z_weights)
            weighted = sum(wi * di for wi, di in zip(z_weights, deficits)) / max(wsum, 1e-6)
            obstacle = w_obs * weighted
        else:
            if min_laser is not None and np.isfinite(min_laser):
                d_safe = d_safe_base + d_safe_speed * abs(v)
                if min_laser < d_safe:
                    obstacle = w_obs * (1.0 - min_laser / max(d_safe, 1e-6))

        # 5) 헤딩 보너스
        heading = k_h * math.cos(theta_err) if theta_err is not None else 0.0

        # 6) 스무딩 페널티 (선택)
        smooth = 0.0
        if k_smooth > 0.0 and prev_v is not None and prev_w is not None:
            dv = abs(v - prev_v) / max(v_max, 1e-6)
            dw = abs(w - prev_w) / max(w_max, 1e-6)
            smooth = k_smooth * 0.5 * (dv + dw)

        # 7) 합산: 시간 페널티(step_pen)로 빠른 경로 탐색 유도
        reward = progress + heading + forward - curv_pen - obstacle - step_pen - smooth
        return float(np.clip(reward, -1.0, 1.0))

    def _parse_position(self):
        """텔레메트리에서 로봇 위치(x, z)와 yaw를 파싱한다."""
        pos_str = self._telemetry.get("V1 Position", "")
        if pos_str:
            try:
                parts = [float(v) for v in pos_str.split()]
                if len(parts) >= 3:
                    self._robot_pos = (parts[0], parts[1])
            except (ValueError, TypeError):
                pass

        euler_str = self._telemetry.get("V1 Orientation Euler Angles", "")
        if euler_str:
            try:
                parts = [float(v) for v in euler_str.split()]
                if len(parts) >= 3:
                    euler_y_deg = parts[1]
                    self._robot_yaw = math.pi / 2.0 - math.radians(euler_y_deg)
            except (ValueError, TypeError):
                pass

    def _theta_err(self) -> float:
        """목표 방향과 로봇 헤딩의 차이 (rad), [-π, π] 정규화."""
        angle = self._angle_to_goal() - self._robot_yaw
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _send_control(self, linear_vel: float, angular_vel: float, reset: bool = False):
        """제어 패킷을 Queue에 넣어 ws_handler(eventlet green thread)가 전송하게 한다."""
        control = dict(DEFAULT_CONTROL)
        throttle = 0.0
        steering = 0.0
        if self.max_linear_vel > 1e-6:
            throttle = float(np.clip(linear_vel / self.max_linear_vel, -1.0, 1.0))
        if self.max_angular_vel > 1e-6:
            steering = float(np.clip(angular_vel / self.max_angular_vel, -1.0, 1.0))

        control["V1 CoSim"]            = "0"           # 속도 제어 모드 (CoSim=1은 위치 직접 제어)
        control["V1 Linear Velocity"]  = str(linear_vel)
        control["V1 Angular Velocity"] = str(angular_vel)
        control["V1 Throttle"]         = str(throttle)
        control["V1 Steering"]         = str(steering)
        control["V1 Reset"]            = "true" if reset else "false"
        control["Goal PosX"]           = str(self._goal_pos[0])
        control["Goal PosZ"]           = str(self._goal_pos[1])
        packet = EIO_MESSAGE + SIO_EVENT + json.dumps(["Bridge", control])

        if self._ctrl_log_file is not None:
            self._ctrl_log_file.write(
                f"{self._episode_count},{self._step_count},"
                f"{time.monotonic():.4f},"
                f"{throttle:.4f},{steering:.4f},"
                f"{linear_vel:.4f},{angular_vel:.4f},"
                f"{'true' if reset else 'false'}\n"
            )

        # 최신 제어 명령만 의미가 있으므로 연결 대기 중 누적된 stale 패킷은 버린다.
        while True:
            try:
                self._send_queue.get_nowait()
            except queue.Empty:
                break
        self._send_queue.put(packet)

    # ── WebSocket 서버 ──────────────────────────────────────────────────────

    def _start_server(self):
        @websocket.WebSocketWSGI
        def ws_handler(ws):
            send_lock = threading.Lock()

            def safe_send(packet: str):
                with send_lock:
                    ws.send(packet)

            def send_loop():
                while self._ws is ws:
                    sent = False
                    while True:
                        try:
                            packet = self._send_queue.get_nowait()
                        except queue.Empty:
                            break
                        safe_send(packet)
                        sent = True
                    if not sent:
                        eventlet.sleep(0.01)

            self._ws = ws
            self._connected_event.set()
            print("[HunterSEEnv] Unity Socket.IO 연결됨")
            sender_gt = None
            try:
                safe_send(OPEN_PACKET)
                safe_send(EIO_MESSAGE + SIO_EVENT + json.dumps(["Bridge", DEFAULT_CONTROL]))
                sender_gt = eventlet.spawn(send_loop)

                while True:
                    msg = ws.wait()
                    if msg is None:
                        break
                    if msg == EIO_PING:
                        safe_send(EIO_PONG)
                    elif msg.startswith(EIO_MESSAGE + SIO_EVENT):
                        try:
                            payload = json.loads(msg[2:])
                            if (
                                isinstance(payload, list)
                                and len(payload) >= 2
                                and payload[0] == "Bridge"
                                and isinstance(payload[1], dict)
                            ):
                                self._telemetry = payload[1]
                                self._parse_position()
                                self._publish_ros()         # ROS2 토픽 발행
                                self._telemetry_event.set()
                        except (json.JSONDecodeError, IndexError):
                            pass
            except Exception:
                pass
            finally:
                if sender_gt is not None:
                    sender_gt.kill()
                self._ws = None
                self._connected_event.clear()
                print("[HunterSEEnv] Unity Socket.IO 연결 종료")

        def app(environ, start_response):
            if environ.get("PATH_INFO", "").startswith("/socket.io/"):
                return ws_handler(environ, start_response)
            start_response("404 Not Found", [("Content-Type", "text/plain")])
            return [b"Not Found"]

        eventlet.wsgi.server(
            eventlet.listen((self.host, self.port)), app, log_output=False
        )
