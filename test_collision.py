"""
Unity SocketIO 플러그인과 직접 통신하는 최소 Engine.IO 구현.

충돌 카운트 참고사항:
  Unity의 OnCollisionEnter는 물리 콜라이더 하나당 1씩 증가.
  Hunter SE는 다중 콜라이더(차체 + 바퀴 4개 등)를 가져서
  한 번 충돌 시 카운터가 3~5씩 오르는 것은 Unity 정상 동작.
  Python에서 카운터 증가 여부만 감지해 충돌 이벤트 1회로 처리.
"""

import base64
import gzip
import json
import time
from datetime import datetime

import eventlet
import eventlet.wsgi
import numpy as np
from eventlet import websocket

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
}

BRIDGE_PACKET = EIO_MESSAGE + SIO_EVENT + json.dumps(["Bridge", DEFAULT_CONTROL])

# 세션 상태
prev_unity_count   = 0     # Unity 누적 카운터 (다중 콜라이더로 +N씩 증가)
collision_events   = 0     # Python이 감지한 실제 충돌 이벤트 횟수
last_collision_at  = 0.0   # 디바운스용 마지막 충돌 시각
DEBOUNCE_SEC       = 1.0   # 1초 내 재발생은 같은 충돌로 간주
first_message      = True
ignore_spawn       = True  # 스폰 직후 접촉 무시


def decode_lidar(b64_data):
    try:
        raw = base64.b64decode(b64_data)
        decompressed = gzip.decompress(raw).decode("utf-8")
        values = [float(v) for v in decompressed.split("\n") if v.strip()]
        return np.array(values, dtype=np.float32)
    except Exception:
        return np.array([], dtype=np.float32)


def process_bridge(ws, telemetry):
    global prev_unity_count, collision_events, last_collision_at
    global first_message, ignore_spawn

    ws.send(BRIDGE_PACKET)

    if first_message:
        first_message = False
        print(f"\n[수신 키 목록]: {sorted(telemetry.keys())}\n")
        if "V1 Collisions" not in telemetry:
            print("[경고] 'V1 Collisions' 키 없음 → LiDAR 감지만 동작합니다.\n")

    now = time.monotonic()
    collision_this_step = False

    # ── Unity 충돌 카운터 ──────────────────────────────────────
    collision_str = telemetry.get("V1 Collisions")
    if collision_str is not None:
        try:
            unity_count = int(collision_str)
        except (ValueError, TypeError):
            unity_count = prev_unity_count

        if unity_count > prev_unity_count:
            if ignore_spawn:
                # 스폰 직후 접촉은 기준값으로만 저장
                ignore_spawn = False
                prev_unity_count = unity_count
                print(f"[초기화] 스폰 접촉 무시 (Unity 카운터 기준값: {unity_count})\n")
            elif now - last_collision_at > DEBOUNCE_SEC:
                # 충돌 이벤트 1회 카운트 (Unity +N이어도 1회로 처리)
                collision_events += 1
                last_collision_at = now
                collision_this_step = True

        prev_unity_count = unity_count

    # ── LiDAR 근접 감지 (보조) ─────────────────────────────────
    lidar_raw = telemetry.get("V1 LIDAR Range Array", "")
    lidar_min = None
    if lidar_raw:
        ranges = decode_lidar(lidar_raw)
        if len(ranges) > 0:
            valid = ranges[ranges > 0.01]
            if len(valid) > 0:
                lidar_min = float(valid.min())
                if lidar_min < 0.15 and now - last_collision_at > DEBOUNCE_SEC:
                    collision_events += 1
                    last_collision_at = now
                    collision_this_step = True

    # ── 출력 ───────────────────────────────────────────────────
    if collision_this_step:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"\n{'=' * 50}")
        print(f"  충돌! [{ts}]  (이번 세션 {collision_events}번째)")
        print(f"  위치: {telemetry.get('V1 Position', '?')}")
        if lidar_min is not None:
            print(f"  LiDAR 최근접: {lidar_min:.3f}m")
        print(f"{'=' * 50}")
    else:
        parts = [f"충돌 이벤트: {collision_events}회"]
        if lidar_min is not None:
            parts.append(f"LiDAR min: {lidar_min:.2f}m")
        pos = telemetry.get("V1 Position", "")
        if pos:
            parts.append(f"위치: {pos}")
        print(f"\r[상태] {' | '.join(parts)}    ", end="", flush=True)


@websocket.WebSocketWSGI
def ws_handler(ws):
    global first_message, prev_unity_count, collision_events
    global last_collision_at, ignore_spawn

    # 세션 초기화
    first_message     = True
    prev_unity_count  = 0
    collision_events  = 0
    last_collision_at = 0.0
    ignore_spawn      = True

    print(f"\n[연결됨] Unity 접속")
    print("키보드로 로봇을 움직이고 벽에 충돌시켜보세요.\n")

    try:
        ws.send(OPEN_PACKET)
        ws.send(BRIDGE_PACKET)

        while True:
            msg = ws.wait()
            if msg is None:
                break

            if msg == EIO_PING:
                ws.send(EIO_PONG)
            elif msg.startswith(EIO_MESSAGE + SIO_EVENT):
                try:
                    payload = json.loads(msg[2:])
                    if (isinstance(payload, list)
                            and len(payload) >= 2
                            and payload[0] == "Bridge"
                            and isinstance(payload[1], dict)):
                        process_bridge(ws, payload[1])
                except (json.JSONDecodeError, IndexError, KeyError):
                    pass

    except Exception as e:
        print(f"\n[오류]: {e}")

    print(f"\n[연결 끊김]")


def app(environ, start_response):
    path = environ.get("PATH_INFO", "")
    if path.startswith("/socket.io/"):
        return ws_handler(environ, start_response)
    start_response("404 Not Found", [("Content-Type", "text/plain")])
    return [b"Not Found"]


print("=" * 50)
print("충돌 감지 테스트 서버: 0.0.0.0:4567")
print("Unity Play 후 키보드로 로봇을 움직이세요.")
print("충돌 시 터미널에 알림이 표시됩니다.")
print("=" * 50)
eventlet.wsgi.server(eventlet.listen(("0.0.0.0", 4567)), app)
