#!/usr/bin/env python3
import json
import math
import os
import threading
import time

import eventlet
import eventlet.wsgi
import gymnasium as gym
import numpy as np
from eventlet import websocket

from utils.file_manager import load_yaml
from utils.lidar_utils import decode_lidar, preprocess_lidar

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
}


class HunterSEEnv(gym.Env):
    """
    AutoDRIVE Hunter SE 강화학습 환경.

    관측 공간: LiDAR(lidar_dim) + [dist_to_goal, angle_to_goal] → shape=(lidar_dim+2,)
    행동 공간: [linear_vel, angular_vel] ∈ [-1, 1]  (실제 속도로 스케일링 후 전송)
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
        self.reward_target        = cfg["reward_target"]
        self.reward_collision     = cfg["reward_collision"]
        self.reward_step          = cfg["reward_step"]
        self.reward_backward      = cfg["reward_backward"]

        # Gymnasium 공간 정의
        obs_dim = self.lidar_dim + 2
        self.observation_space = gym.spaces.Box(
            low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
        )

        # 내부 상태
        self._ws                = None          # 현재 WebSocket 연결
        self._telemetry         = {}            # 최신 텔레메트리
        self._telemetry_event   = threading.Event()
        self._prev_unity_count  = 0             # 이전 Unity 누적 충돌 카운터
        self._last_collision_at = 0.0
        self._ignore_spawn      = True
        self._goal_pos          = (0.0, 0.0)   # (x, z)
        self._robot_pos         = (0.0, 0.0)   # (x, z)
        self._prev_dist         = None
        self._step_count        = 0

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

        self._step_count        = 0
        self._prev_dist         = None
        self._ignore_spawn      = True
        self._last_collision_at = 0.0
        self._telemetry_event.clear()

        # Unity에 리셋 명령 전송
        self._send_control(0.0, 0.0, reset=True)

        # 첫 텔레메트리 대기
        if not self._telemetry_event.wait(timeout=self.step_timeout):
            # 타임아웃 시 zero 관측 반환
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
        linear_vel  = float(action[0]) * self.max_linear_vel
        angular_vel = float(action[1]) * self.max_angular_vel

        self._telemetry_event.clear()
        self._send_control(linear_vel, angular_vel)

        # 다음 텔레메트리 대기
        if not self._telemetry_event.wait(timeout=self.step_timeout):
            obs = np.zeros(self.lidar_dim + 2, dtype=np.float32)
            return obs, 0.0, False, True, {"timeout": True}

        self._step_count += 1

        collision      = self._detect_collision()
        dist           = self._dist_to_goal()
        target_reached = dist < self.goal_threshold
        terminated     = collision or target_reached
        truncated      = self._step_count >= self.max_episode_steps

        reward = self._compute_reward(
            linear_vel, collision, target_reached, dist
        )
        self._prev_dist = dist

        obs  = self._get_observation()
        info = {
            "collision":      collision,
            "target_reached": target_reached,
            "dist_to_goal":   dist,
            "step":           self._step_count,
        }
        return obs, reward, terminated, truncated, info

    def close(self):
        pass

    # ── 내부 헬퍼 ──────────────────────────────────────────────────────────

    def _get_observation(self) -> np.ndarray:
        lidar_raw = self._telemetry.get("V1 LIDAR Range Array", "")
        if lidar_raw:
            ranges = decode_lidar(lidar_raw)
        else:
            ranges = np.full(self.lidar_dim, self.lidar_max_range, dtype=np.float32)

        lidar_norm = preprocess_lidar(ranges, self.lidar_dim, self.lidar_max_range)

        dist       = self._dist_to_goal()
        angle      = self._angle_to_goal()
        dist_norm  = np.clip(dist / (self.arena_size * math.sqrt(2)), 0.0, 1.0)
        angle_norm = (angle / math.pi + 1.0) / 2.0   # [-π, π] → [0, 1]

        return np.concatenate([lidar_norm, [dist_norm, angle_norm]]).astype(np.float32)

    def _detect_collision(self) -> bool:
        """test_collision.py의 디바운싱 로직을 적용한 충돌 감지."""
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
        """로봇에서 목표까지의 방향각 (rad). 단순 XZ 평면 기준."""
        rx, rz = self._robot_pos
        gx, gz = self._goal_pos
        return math.atan2(gz - rz, gx - rx)

    def _compute_reward(
        self, linear_vel: float, collision: bool, target_reached: bool, dist: float
    ) -> float:
        if target_reached:
            return self.reward_target
        if collision:
            return self.reward_collision

        reward = self.reward_step

        # 목표 접근 보상
        if self._prev_dist is not None:
            reward += (self._prev_dist - dist) * 2.0

        # 후진 패널티
        if linear_vel < 0:
            reward += self.reward_backward

        return float(reward)

    def _parse_position(self):
        """텔레메트리에서 로봇 (x, z) 파싱."""
        pos_str = self._telemetry.get("V1 Position", "")
        if pos_str:
            try:
                parts = [float(v) for v in pos_str.split(",")]
                if len(parts) >= 3:
                    self._robot_pos = (parts[0], parts[2])  # Unity: Y = 높이
            except (ValueError, TypeError):
                pass

    def _send_control(self, linear_vel: float, angular_vel: float, reset: bool = False):
        if self._ws is None:
            return
        control = dict(DEFAULT_CONTROL)
        control["V1 Linear Velocity"]  = str(linear_vel)
        control["V1 Angular Velocity"] = str(angular_vel)
        control["V1 Reset"]            = "true" if reset else "false"
        packet = EIO_MESSAGE + SIO_EVENT + json.dumps(["Bridge", control])
        try:
            self._ws.send(packet)
        except Exception:
            pass

    # ── WebSocket 서버 ──────────────────────────────────────────────────────

    def _start_server(self):
        @websocket.WebSocketWSGI
        def ws_handler(ws):
            self._ws = ws
            try:
                ws.send(OPEN_PACKET)
                packet = EIO_MESSAGE + SIO_EVENT + json.dumps(
                    ["Bridge", DEFAULT_CONTROL]
                )
                ws.send(packet)

                while True:
                    msg = ws.wait()
                    if msg is None:
                        break
                    if msg == EIO_PING:
                        ws.send(EIO_PONG)
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
                                self._telemetry_event.set()
                        except (json.JSONDecodeError, IndexError):
                            pass
            except Exception:
                pass
            finally:
                self._ws = None

        def app(environ, start_response):
            if environ.get("PATH_INFO", "").startswith("/socket.io/"):
                return ws_handler(environ, start_response)
            start_response("404 Not Found", [("Content-Type", "text/plain")])
            return [b"Not Found"]

        eventlet.wsgi.server(
            eventlet.listen((self.host, self.port)), app, log_output=False
        )
