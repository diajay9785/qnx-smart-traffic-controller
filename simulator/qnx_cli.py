"""Tiny command-line client for the QNX controller - handy to check the board is alive
before starting the simulator.

    python qnx_cli.py PING
    python qnx_cli.py STATUS --host 192.168.1.50
    python qnx_cli.py COUNTS 12 9 4
    python qnx_cli.py EMERG N
"""
import argparse
import socket
import sys

ap = argparse.ArgumentParser()
ap.add_argument("command", nargs="+", help="e.g. PING | STATUS | COUNTS 5 3 8 | EMERG N")
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=5000)
args = ap.parse_args()

try:
    with socket.create_connection((args.host, args.port), timeout=4) as s:
        s.sendall((" ".join(args.command) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(256)
            if not chunk:
                break
            buf += chunk
    print(buf.decode().strip())
except OSError as exc:
    sys.exit(f"cannot reach {args.host}:{args.port} - {exc}")
