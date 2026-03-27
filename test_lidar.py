import socketio
import eventlet
import eventlet.wsgi
import base64
import numpy as np
import struct

sio = socketio.Server(cors_allowed_origins='*')
app = socketio.WSGIApp(sio)

def decode_pointcloud(b64_data):
    """base64 → (N, 3) float32 numpy array (x, y, z)"""
    raw = base64.b64decode(b64_data)
    n_points = len(raw) // 12  # 각 포인트 = float32 x3 = 12 bytes
    if n_points == 0:
        return np.zeros((0, 3), dtype=np.float32)
    points = np.frombuffer(raw, dtype=np.float32).reshape(n_points, 3)
    return points

@sio.event
def connect(sid, environ):
    print(f"\n[연결됨] Unity 시뮬레이터 접속: sid={sid}")

@sio.event
def disconnect(sid):
    print(f"\n[연결 끊김] sid={sid}")

@sio.event
def Bridge(sid, data):
    # LiDAR point cloud 키 찾기
    lidar_key = None
    for key in data:
        if 'LIDAR Pointcloud' in key or 'Pointcloud' in key:
            lidar_key = key
            break

    if lidar_key and data[lidar_key]:
        pcl = decode_pointcloud(data[lidar_key])
        print(f"\r[{lidar_key}] {len(pcl)} points | "
              f"x: [{pcl[:,0].min():.2f}, {pcl[:,0].max():.2f}] "
              f"y: [{pcl[:,1].min():.2f}, {pcl[:,1].max():.2f}] "
              f"z: [{pcl[:,2].min():.2f}, {pcl[:,2].max():.2f}]",
              end="", flush=True)
    else:
        # 수신된 모든 키 출력
        keys = list(data.keys())
        print(f"\r[Bridge] keys: {keys[:5]}...", end="", flush=True)

print("=" * 50)
print("LiDAR 수신 테스트 서버: 0.0.0.0:4567")
print("Unity Play 후 데이터 수신 대기 중...")
print("=" * 50)
eventlet.wsgi.server(eventlet.listen(('0.0.0.0', 4567)), app)
