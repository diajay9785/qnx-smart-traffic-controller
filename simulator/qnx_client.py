"""TCP client for qnx_traffic_controller.c.

The controller speaks a one-command-per-connection line protocol on port 5000:

    PING                    -> PONG
    COUNTS <N> <S> <E>      -> PLAN <G0> <G1>
    EMERG N|S|E             -> OK
    STATUS                  -> STATE <state> N=<R/Y/G> S=.. E=.. G0=.. G1=.. FB=<0/1> EM=<N/S/E/->

This client runs in a background thread so the Pygame loop never blocks on the
network. All commands are serialised on one thread, because the C server handles
one connection at a time.
"""
import queue
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import config as C


@dataclass
class NetSnapshot:
    host: str = ""
    port: int = 0
    link_ok: bool = False
    error: str = "connecting..."
    rtt_ms: float = 0.0
    status_time: float = 0.0          # monotonic time of last good STATUS
    state: str = "UNKNOWN"
    lights: dict = field(default_factory=lambda: {"N": "R", "S": "R", "E": "R"})
    g0: int = 30
    g1: int = 30
    fallback: bool = False
    emergency: str = "-"
    state_since: float = 0.0          # monotonic time the current state began
    state_duration: float = 0.0       # expected length of the current state (s)
    plan_time: float = 0.0            # monotonic time of last PLAN reply
    counts_sent: tuple = (0, 0, 0)
    log: list = field(default_factory=list)
    time_scale: float = 1.0
    cycle: int = 0                    # Webster cycle length (s) from the QNX controller
    flow_ratio: float = 0.0           # Webster total flow ratio Y

    @property
    def fresh(self):
        return self.link_ok and (time.monotonic() - self.status_time) < C.STALE_AFTER_S

    def remaining(self):
        """Estimated seconds left, refreshed from the controller REM field."""
        if self.state_duration <= 0:
            return 0.0
        elapsed = (time.monotonic() - self.state_since) * self.time_scale
        return max(0.0, self.state_duration - elapsed)


def parse_status(line):
    """Parse 'STATE GREEN N=G S=G E=R G0=30 G1=30 FB=0 EM=-' -> dict or None."""
    parts = line.split()
    if len(parts) < 3 or parts[0] != "STATE":
        return None
    kv = dict(p.split("=", 1) for p in parts[2:] if "=" in p)
    try:
        return {
            "state": parts[1],
            "lights": {"N": kv["N"], "S": kv["S"], "E": kv["E"]},
            "g0": int(kv["G0"]), "g1": int(kv["G1"]),
            "fallback": kv.get("FB", "0") == "1",
            "emergency": kv.get("EM", "-"),
            "cycle": int(kv.get("C", 0)),
            "remaining": int(kv.get("REM", 0)),
        }
    except (KeyError, ValueError):
        return None


class QnxClient(threading.Thread):
    def __init__(self, host, port, time_scale=1.0):
        super().__init__(daemon=True, name="qnx-client")
        self.host, self.port = host, port
        self.time_scale = time_scale
        self._lock = threading.Lock()
        self._cmds = queue.Queue()
        self._stop_evt = threading.Event()
        self._counts = (0, 0, 0)
        self._log = deque(maxlen=40)
        self._snap = NetSnapshot(host=host, port=port, time_scale=time_scale)
        self._state_key = None

    # ---- public API (called from the Pygame thread) -------------------------
    def set_counts(self, n, s, e):
        c = tuple(max(0, min(C.MAX_COUNT, int(v))) for v in (n, s, e))
        with self._lock:
            self._counts = c

    def emergency(self, approach):
        self._cmds.put(f"EMERG {approach}")

    def snapshot(self):
        with self._lock:
            snap = NetSnapshot(**{k: getattr(self._snap, k) for k in self._snap.__dataclass_fields__})
            snap.lights = dict(self._snap.lights)
            snap.log = list(self._log)
        return snap

    def stop(self):
        self._stop_evt.set()

    # ---- internals -----------------------------------------------------------
    def _note(self, text, kind="info"):
        with self._lock:
            self._log.append((time.time(), text, kind))

    def _txn(self, cmd):
        t0 = time.monotonic()
        with socket.create_connection((self.host, self.port), timeout=C.SOCKET_TIMEOUT_S) as s:
            s.settimeout(C.SOCKET_TIMEOUT_S)
            s.sendall((cmd + "\n").encode("ascii"))
            buf = b""
            while b"\n" not in buf:
                chunk = s.recv(256)
                if not chunk:
                    break
                buf += chunk
        line = buf.split(b"\n")[0].decode("ascii", "replace").strip()
        if not line:
            raise ConnectionError("empty reply")
        return line, (time.monotonic() - t0) * 1000.0

    def _link_up(self, rtt):
        with self._lock:
            was = self._snap.link_ok
            self._snap.link_ok = True
            self._snap.error = ""
            self._snap.rtt_ms = rtt
        if not was:
            self._note(f"link up  {self.host}:{self.port}", "ok")

    def _link_down(self, err):
        with self._lock:
            was = self._snap.link_ok
            self._snap.link_ok = False
            self._snap.error = str(err)
        if was or not self._log:
            self._note(f"link down: {err}", "err")

    def _apply_status(self, st):
        now = time.monotonic()
        with self._lock:
            snap = self._snap
            snap.state, snap.lights = st["state"], st["lights"]
            snap.g0, snap.g1 = st["g0"], st["g1"]
            snap.fallback, snap.emergency = st["fallback"], st["emergency"]
            snap.cycle = st["cycle"] or snap.cycle
            snap.status_time = now
            key = (st["state"], st["lights"]["N"], st["lights"]["S"], st["lights"]["E"], st["emergency"])
            if key != self._state_key:
                self._state_key = key
                snap.state_since = now
                lt = st["lights"]
                if st["state"] == "GREEN":
                    snap.state_duration = st["g0"] if (lt["N"] == "G" or lt["S"] == "G") else st["g1"]
                elif st["state"] == "YELLOW":
                    snap.state_duration = C.YELLOW_S
                elif st["state"] == "ALL_RED":
                    snap.state_duration = st.get("remaining", 0) or C.ALL_RED_S
                elif st["state"] == "EMERGENCY_GREEN":
                    snap.state_duration = st.get("remaining", 0) or C.EMERGENCY_HOLD_S
                else:
                    snap.state_duration = C.FAILSAFE_S
            elif st.get("remaining", 0) > 0:
                # The controller reports its real timer deadline. Refresh the
                # local estimate only when a material extension occurred (e.g.
                # an extra ambulance added five seconds).
                predicted = snap.remaining()
                reported = float(st["remaining"])
                if abs(reported - predicted) > 1.5:
                    snap.state_since = now
                    snap.state_duration = reported

    def _do_status(self):
        line, rtt = self._txn("STATUS")
        st = parse_status(line)
        if st is None:
            raise ValueError(f"bad STATUS reply: {line!r}")
        self._link_up(rtt)
        self._apply_status(st)

    def _do_counts(self):
        with self._lock:
            n, s, e = self._counts
        cmd = f"COUNTS {n} {s} {e}"
        line, rtt = self._txn(cmd)
        self._link_up(rtt)
        with self._lock:
            self._snap.counts_sent = (n, s, e)
        parts = line.split()
        if parts and parts[0] == "PLAN" and len(parts) >= 3:
            with self._lock:
                self._snap.g0, self._snap.g1 = int(parts[1]), int(parts[2])
                if len(parts) >= 5:                      # PLAN g0 g1 cycle Y  (Webster)
                    self._snap.cycle, self._snap.flow_ratio = int(parts[3]), float(parts[4])
                self._snap.plan_time = time.monotonic()
            self._note(f"{cmd}  ->  {line}", "ok")
        else:
            self._note(f"{cmd}  ->  {line}", "err")

    def _do_cmd(self, cmd):
        line, rtt = self._txn(cmd)
        self._link_up(rtt)
        self._note(f"{cmd}  ->  {line}", "emerg" if line == "OK" else "err")

    def run(self):
        next_status = next_counts = 0.0
        while not self._stop_evt.is_set():
            now = time.monotonic()
            try:
                try:
                    cmd = self._cmds.get_nowait()
                except queue.Empty:
                    cmd = None
                if cmd:
                    self._do_cmd(cmd)
                elif now >= next_counts:
                    next_counts = now + C.COUNTS_INTERVAL_S
                    self._do_counts()
                elif now >= next_status:
                    next_status = now + C.STATUS_POLL_S
                    self._do_status()
                else:
                    time.sleep(0.02)
            except (OSError, ValueError) as exc:
                self._link_down(exc)
                time.sleep(0.5)
