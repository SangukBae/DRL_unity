#!/usr/bin/env python3
"""
AutoDRIVE Hunter SE ROS 2 Bridge (시각화 전용).

Unity → ROS 2 토픽 발행:
  /autodrive/hunter_se_1/lidar          (sensor_msgs/LaserScan)
  /autodrive/hunter_se_1/imu            (sensor_msgs/Imu)
  /autodrive/hunter_se_1/ips            (geometry_msgs/Point)
  /autodrive/hunter_se_1/linear_vel     (std_msgs/Float32)
  /autodrive/hunter_se_1/angular_vel    (std_msgs/Float32)  -- IMU yaw rate
  /autodrive/hunter_se_1/collisions     (std_msgs/Int32)

TF 브로드캐스트:
  map → hunter_se_1
  hunter_se_1 → lidar
  hunter_se_1 → imu

[주의]
이 브릿지는 포트 4567에서 Socket.IO 서버로 실행된다.
강화학습 환경(hunter_se_env.py)도 같은 포트를 사용하므로 동시 실행 불가.
- 시각화 전용: 이 브릿지 실행 (hunter_se_env.py 중지)
- RL 학습:     hunter_se_env.py 실행 (이 브릿지 중지)

[Goal 마커에 대하여]
Goal 위치는 RL 학습(hunter_se_env.py)이 Unity로 보내는 값이다.
시각화 전용 브릿지에는 RL이 없으므로 Goal 토픽은 발행하지 않는다.
Goal 마커 시각화는 Unity 씬의 RLVisualizer.cs가 RL 학습 중에 담당한다.
"""

import base64
import gzip
import math

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import tf2_ros
from std_msgs.msg import Float32, Int32, Header
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import Imu, LaserScan

import numpy as np
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
import socketio

# ── 전역 ROS 2 노드 (bridge 이벤트 핸들러에서 접근) ─────────────────────────
bridge_node    = None
publishers     = {}
tf_broadcaster = None

# ── LiDAR 파라미터 (Hunter SE - Random Obstacles 씬 기준) ────────────────────
LIDAR_RANGE_MIN = 0.15   # m
LIDAR_RANGE_MAX = 12.0   # m

# ── Hunter SE 센서 오프셋 (hunter_se_1 frame 기준, 단위: m) ──────────────────
LIDAR_OFFSET = np.array([0.10, 0.0, 0.25])   # 전방 0.10m, 상방 0.25m
IMU_OFFSET   = np.array([0.0,  0.0, 0.10])   # 차체 중앙 상방

# ── Socket.IO 서버 ────────────────────────────────────────────────────────────
sio = socketio.Server(async_mode='gevent')

# 시각화 모드 제어 명령: 속도 0, Goal 위치 필드 없음
# Goal PosX/Z를 보내지 않으면 Socket.cs의 HasField 검사에서 통과되지 않아
# Unity 씬의 RLVisualizer goal 마커 위치가 변경되지 않는다.
ZERO_CONTROL = {
    'V1 Reset':           'false',
    'V1 CoSim':           '0',
    'V1 Linear Velocity': '0',
    'V1 Angular Velocity':'0',
}


def _decode_lidar(b64_data: str) -> np.ndarray:
    """base64 + gzip 압축된 LiDAR 데이터를 float 배열로 변환한다.

    Unity DataCompressor.CompressArray(): 값 배열 → '\\n' join → GZip → Base64
    """
    try:
        raw          = base64.b64decode(b64_data)
        decompressed = gzip.decompress(raw).decode('utf-8')
        values       = [float(v) for v in decompressed.split('\n') if v.strip()]
        return np.array(values, dtype=np.float32)
    except Exception:
        return np.full(360, LIDAR_RANGE_MAX, dtype=np.float32)


def _broadcast_tf(child: str, parent: str, translation: np.ndarray, quaternion: np.ndarray):
    tf = TransformStamped()
    tf.header.stamp             = bridge_node.get_clock().now().to_msg()
    tf.header.frame_id          = parent
    tf.child_frame_id           = child
    tf.transform.translation.x  = float(translation[0])
    tf.transform.translation.y  = float(translation[1])
    tf.transform.translation.z  = float(translation[2])
    tf.transform.rotation.x     = float(quaternion[0])
    tf.transform.rotation.y     = float(quaternion[1])
    tf.transform.rotation.z     = float(quaternion[2])
    tf.transform.rotation.w     = float(quaternion[3])
    tf_broadcaster.sendTransform(tf)


# ── Socket.IO 이벤트 핸들러 ────────────────────────────────────────────────────

@sio.on('connect')
def on_connect(sid, environ):
    print('[Hunter SE Bridge] Unity 연결됨')


@sio.on('disconnect')
def on_disconnect(sid):
    print('[Hunter SE Bridge] Unity 연결 끊김')


@sio.on('Bridge')
def on_bridge(sid, data):
    if not data:
        sio.emit('Bridge', data=ZERO_CONTROL)
        return

    now = bridge_node.get_clock().now().to_msg()

    # 1) 위치 (IPS)
    pos = np.fromstring(data.get('V1 Position', '0 0 0'), dtype=float, sep=' ')
    if len(pos) < 3:
        pos = np.zeros(3)
    ips_msg       = Point()
    ips_msg.x, ips_msg.y, ips_msg.z = float(pos[0]), float(pos[1]), float(pos[2])
    publishers['ips'].publish(ips_msg)

    # 2) IMU
    quat    = np.fromstring(data.get('V1 Orientation Quaternion', '0 0 0 1'), dtype=float, sep=' ')
    ang_vel = np.fromstring(data.get('V1 Angular Velocity',       '0 0 0'),   dtype=float, sep=' ')
    lin_acc = np.fromstring(data.get('V1 Linear Acceleration',    '0 0 0'),   dtype=float, sep=' ')
    if len(quat)    < 4: quat    = np.array([0.0, 0.0, 0.0, 1.0])
    if len(ang_vel) < 3: ang_vel = np.zeros(3)
    if len(lin_acc) < 3: lin_acc = np.zeros(3)

    imu_msg = Imu()
    imu_msg.header = Header()
    imu_msg.header.stamp                   = now
    imu_msg.header.frame_id                = 'imu'
    imu_msg.orientation.x                  = float(quat[0])
    imu_msg.orientation.y                  = float(quat[1])
    imu_msg.orientation.z                  = float(quat[2])
    imu_msg.orientation.w                  = float(quat[3])
    imu_msg.orientation_covariance         = [0.0025,0.0,0.0,0.0,0.0025,0.0,0.0,0.0,0.0025]
    imu_msg.angular_velocity.x             = float(ang_vel[0])
    imu_msg.angular_velocity.y             = float(ang_vel[1])
    imu_msg.angular_velocity.z             = float(ang_vel[2])
    imu_msg.angular_velocity_covariance    = [0.0025,0.0,0.0,0.0,0.0025,0.0,0.0,0.0,0.0025]
    imu_msg.linear_acceleration.x          = float(lin_acc[0])
    imu_msg.linear_acceleration.y          = float(lin_acc[1])
    imu_msg.linear_acceleration.z          = float(lin_acc[2])
    imu_msg.linear_acceleration_covariance = [0.0025,0.0,0.0,0.0,0.0025,0.0,0.0,0.0,0.0025]
    publishers['imu'].publish(imu_msg)

    # 3) TF
    _broadcast_tf('hunter_se_1', 'map',        pos,         quat)
    _broadcast_tf('lidar',       'hunter_se_1', LIDAR_OFFSET, np.array([0, 0, 0, 1]))
    _broadcast_tf('imu',         'hunter_se_1', IMU_OFFSET,   np.array([0, 0, 0, 1]))

    # 4) LiDAR (base64+gzip 디코딩)
    lidar_raw = data.get('V1 LIDAR Range Array', '')
    ranges    = _decode_lidar(lidar_raw) if lidar_raw else np.full(360, LIDAR_RANGE_MAX, dtype=np.float32)

    try:
        scan_rate = float(data.get('V1 LIDAR Scan Rate', '10'))
        if scan_rate <= 0:
            scan_rate = 10.0
    except (ValueError, TypeError):
        scan_rate = 10.0

    ls = LaserScan()
    ls.header = Header()
    ls.header.stamp    = now
    ls.header.frame_id = 'lidar'
    ls.angle_min       = -math.pi
    ls.angle_max       =  math.pi
    ls.angle_increment = 2 * math.pi / max(len(ranges), 1)
    ls.time_increment  = (1.0 / scan_rate) / max(len(ranges), 1)
    ls.scan_time       = 1.0 / scan_rate
    ls.range_min       = LIDAR_RANGE_MIN
    ls.range_max       = LIDAR_RANGE_MAX
    ls.ranges          = ranges.tolist()
    ls.intensities     = []
    publishers['lidar'].publish(ls)

    # 5) 선속도 (V1 Speed: 로컬 좌표 Z축 방향 속도 크기)
    lin_vel_msg      = Float32()
    lin_vel_msg.data = float(data.get('V1 Speed', 0))
    publishers['linear_vel'].publish(lin_vel_msg)

    # 6) 각속도 (IMU angular velocity Y축 = Unity 수직축 = 로봇 yaw rate)
    ang_vel_msg      = Float32()
    ang_vel_msg.data = float(ang_vel[1])   # Unity Y = 수직 회전축
    publishers['angular_vel'].publish(ang_vel_msg)

    # 7) 충돌 카운터
    col_msg      = Int32()
    col_msg.data = int(data.get('V1 Collisions', 0))
    publishers['collisions'].publish(col_msg)

    # Unity에 제어 명령 송신 (시각화 모드: 속도 0, Goal 필드 없음)
    sio.emit('Bridge', data=ZERO_CONTROL)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    global bridge_node, publishers, tf_broadcaster

    rclpy.init()
    bridge_node = rclpy.create_node('autodrive_hunter_se_bridge')

    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )

    publishers = {
        'lidar':       bridge_node.create_publisher(LaserScan, '/autodrive/hunter_se_1/lidar',       qos),
        'imu':         bridge_node.create_publisher(Imu,       '/autodrive/hunter_se_1/imu',         qos),
        'ips':         bridge_node.create_publisher(Point,     '/autodrive/hunter_se_1/ips',         qos),
        'linear_vel':  bridge_node.create_publisher(Float32,   '/autodrive/hunter_se_1/linear_vel',  qos),
        'angular_vel': bridge_node.create_publisher(Float32,   '/autodrive/hunter_se_1/angular_vel', qos),
        'collisions':  bridge_node.create_publisher(Int32,     '/autodrive/hunter_se_1/collisions',  qos),
    }

    tf_broadcaster = tf2_ros.TransformBroadcaster(bridge_node)

    print('[Hunter SE Bridge] 서버 시작 — 포트 4567 대기 중...')
    print('[Hunter SE Bridge] Unity에서 Play 버튼을 누르면 연결됩니다.')

    app = socketio.WSGIApp(sio)
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever()


if __name__ == '__main__':
    main()
