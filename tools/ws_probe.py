#!/usr/bin/env python3
"""Host-side probe for the on-device pose WebSocket server (ARCHITECTURE.md §6).

Connects to ws://<host>:<port>/pose, performs the RFC 6455 handshake by hand
(no third-party deps), then reads "pose_frame" JSON messages and reports:

  - connection / handshake success
  - frame rate (Hz) and inter-frame jitter
  - per-frame keypoint count and mean confidence
  - end-to-end arrival latency proxy (host recv gaps)

Typical use against a USB device:

  adb forward tcp:9090 tcp:8080
  python3 tools/ws_probe.py --host 127.0.0.1 --port 9090 --seconds 10

Exit code 0 if at least one valid pose_frame arrived, else non-zero.
"""
import argparse
import base64
import json
import os
import socket
import struct
import sys
import time


def handshake(sock: socket.socket, host: str, port: int, path: str) -> None:
    key = base64.b64encode(os.urandom(16)).decode()
    req = (
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n"
    )
    sock.sendall(req.encode())
    resp = b""
    while b"\r\n\r\n" not in resp:
        chunk = sock.recv(1024)
        if not chunk:
            raise ConnectionError("server closed during handshake")
        resp += chunk
    if b" 101 " not in resp.split(b"\r\n", 1)[0]:
        raise ConnectionError(f"bad handshake response: {resp[:80]!r}")


def read_frame(sock: socket.socket) -> str:
    """Read one unmasked server->client text frame, return its payload."""
    hdr = recvn(sock, 2)
    b0, b1 = hdr[0], hdr[1]
    opcode = b0 & 0x0F
    length = b1 & 0x7F
    if length == 126:
        length = struct.unpack("!H", recvn(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", recvn(sock, 8))[0]
    if b1 & 0x80:  # server frames are not masked, but be safe
        recvn(sock, 4)
    payload = recvn(sock, length) if length else b""
    if opcode == 0x8:
        raise ConnectionError("server sent close")
    return payload.decode("utf-8", "replace")


def recvn(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("server closed mid-frame")
        buf += chunk
    return buf


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--path", default="/pose")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    print(f"connecting to ws://{args.host}:{args.port}{args.path} ...")
    sock = socket.create_connection((args.host, args.port), timeout=5.0)
    sock.settimeout(5.0)
    handshake(sock, args.host, args.port, args.path)
    print("handshake OK (HTTP 101)")

    t0 = time.monotonic()
    last = None
    gaps = []
    frames = 0
    sample = None
    kp_counts = []
    confs = []
    while time.monotonic() - t0 < args.seconds:
        try:
            msg = read_frame(sock)
        except (socket.timeout, TimeoutError):
            continue
        now = time.monotonic()
        try:
            obj = json.loads(msg)
        except json.JSONDecodeError:
            continue
        if obj.get("type") != "pose_frame":
            continue
        frames += 1
        if last is not None:
            gaps.append(now - last)
        last = now
        if sample is None:
            sample = obj
        kps = obj.get("keypoints", [])
        kp_counts.append(sum(1 for k in kps if k.get("conf", 0) > 0.0))
        confs.append(
            sum(k.get("conf", 0) for k in kps) / max(1, len(kps))
        )
    sock.close()

    dur = time.monotonic() - t0
    print("\n=== device -> host link report ===")
    print(f"duration            : {dur:.1f} s")
    print(f"pose_frames         : {frames}")
    if frames == 0:
        print("NO pose frames received (is a body in view / camera granted?)")
        return 1
    rate = frames / dur
    print(f"frame rate          : {rate:.1f} Hz")
    if gaps:
        gaps.sort()
        mean = sum(gaps) / len(gaps)
        p50 = gaps[len(gaps) // 2]
        p95 = gaps[min(len(gaps) - 1, int(len(gaps) * 0.95))]
        print(f"inter-frame mean    : {mean * 1000:.1f} ms")
        print(f"inter-frame p50/p95 : {p50 * 1000:.1f} / {p95 * 1000:.1f} ms")
    print(f"mean visible kp     : {sum(kp_counts) / len(kp_counts):.1f} / 17")
    print(f"mean confidence     : {sum(confs) / len(confs):.2f}")
    if sample is not None:
        print(f"sample observer_cnt : {sample.get('observer_count')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
