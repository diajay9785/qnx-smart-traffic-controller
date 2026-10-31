"""Pure-Python stand-in for qnx_traffic_controller.c.

The real controller only builds on QNX (mqueue, neutrino). This mock reproduces its
TCP protocol and its state machine line-for-line so you can develop and demo the
simulator on any laptop. It is NOT a replacement for testing against the real thing.

    python mock_qnx_controller.py                 # listen on port 5000
    python mock_qnx_controller.py --speed 3       # run controller time 3x faster (testing only)
"""
import argparse
import socketserver
import threading
import time

CYCLE, MIN_GREEN, MAX_GREEN = 60, 5, 40
YELLOW, ALL_RED, EMERG_HOLD, FALLBACK_AFTER = 3, 1, 10, 15


def calculate_split(n, s, e):
    """Same count-weighted split as calculate_split() in the C file."""
    d0, d1 = n + s, e
    total = d0 + d1
    if total == 0:
        return 30, 30
    x = max(MIN_GREEN, min(MAX_GREEN, (CYCLE * d0) // total))
    g0, g1 = x, CYCLE - x
    if g1 < MIN_GREEN:
        g1 = MIN_GREEN
        g0 = CYCLE - g1
    if g1 > MAX_GREEN:
        g1 = MAX_GREEN
        g0 = CYCLE - g1
    return g0, g1


class MockController:
    def __init__(self, port=5000, speed=1.0, host="0.0.0.0", verbose=True):
        self.port, self.host, self.speed, self.verbose = port, host, speed, verbose
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.green0 = self.green1 = 30
        self.fallback = False
        self.state, self.phase = "GREEN", 0
        self.resume_phase = 0
        self.resume_after = False
        self.em_active = self.em_pending = "-"
        self.last_plan = self.now()
        self.deadline = self.now() + self.green0
        self._running = True
        self._server = None

    # -- scaled clock (speed != 1 is for automated tests only) ------------------
    def now(self):
        return (time.monotonic() - self.t0) * self.speed

    def log(self, msg):
        if self.verbose:
            print(msg, flush=True)

    # -- lights -------------------------------------------------------------------
    def lights(self):
        n = s = e = "R"
        if self.state == "GREEN":
            if self.phase == 0:
                n = s = "G"
            else:
                e = "G"
        elif self.state == "YELLOW":
            if self.phase == 0:
                n = s = "Y"
            else:
                e = "Y"
        elif self.state == "EMERGENCY_GREEN":
            if self.em_active == "N":
                n = "G"
            elif self.em_active == "S":
                s = "G"
            elif self.em_active == "E":
                e = "G"
        if e != "R" and (n != "R" or s != "R"):          # conflict guard
            n = s = e = "R"
            self.state = "FAILSAFE"
            self.deadline = self.now() + 3
            self.log("[SAFETY] CONFLICT detected; forcing ALL-RED failsafe")
        return n, s, e

    def enter(self, state, duration):
        self.state = state
        self.deadline = self.now() + duration
        n, s, e = self.lights()
        self.log(f"[SIGNAL] state={self.state} phase={self.phase} N={n} S={s} E={e} "
                 f"G0={self.green0} G1={self.green1} fallback={int(self.fallback)} emergency={self.em_active}")

    # -- controller_worker() state machine ---------------------------------------------
    def tick(self):
        with self.lock:
            now = self.now()
            if not self.fallback and now - self.last_plan >= FALLBACK_AFTER:
                self.fallback = True
                self.green0 = self.green1 = 10
                self.log(f"[WATCHDOG] no new traffic plan for {FALLBACK_AFTER} seconds; fallback timing enabled")
            if self.em_pending != "-" and self.state == "GREEN":
                self.resume_phase = self.phase
                self.em_active, self.em_pending = self.em_pending, "-"
                self.enter("YELLOW", YELLOW)
            elif now >= self.deadline:
                if self.state == "GREEN":
                    self.enter("YELLOW", YELLOW)
                elif self.state == "YELLOW":
                    self.enter("ALL_RED", ALL_RED)
                elif self.state == "ALL_RED":
                    if self.em_active != "-":
                        self.enter("EMERGENCY_GREEN", EMERG_HOLD)
                    elif self.resume_after:
                        self.resume_after = False
                        self.phase = self.resume_phase
                        self.enter("GREEN", self.green0 if self.phase == 0 else self.green1)
                    else:
                        self.phase = 1 - self.phase
                        self.enter("GREEN", self.green0 if self.phase == 0 else self.green1)
                elif self.state == "EMERGENCY_GREEN":
                    self.em_active = "-"
                    self.phase = self.resume_phase
                    self.resume_after = True
                    self.enter("ALL_RED", ALL_RED)
                elif self.state == "FAILSAFE":
                    self.phase = 0
                    self.em_active = "-"
                    self.enter("ALL_RED", ALL_RED)

    def _loop(self):
        with self.lock:
            self.enter("GREEN", self.green0)
        while self._running:
            self.tick()
            time.sleep(0.05)

    # -- TCP command handling (handle_command() in the C file) ----------------------------
    def handle(self, line):
        if line == "PING":
            return "PONG"
        if line == "STATUS":
            with self.lock:
                n, s, e = self.lights()
                return (f"STATE {self.state} N={n} S={s} E={e} G0={self.green0} G1={self.green1} "
                        f"FB={int(self.fallback)} EM={self.em_active}")
        if line.startswith("COUNTS "):
            try:
                n, s, e = (int(x) for x in line[7:].split()[:3])
            except ValueError:
                return "ERR bad_counts"
            if not all(0 <= v <= 200 for v in (n, s, e)) or len(line[7:].split()) < 3:
                return "ERR bad_counts"
            g0, g1 = calculate_split(n, s, e)
            with self.lock:
                self.green0, self.green1, self.fallback = g0, g1, False
                self.last_plan = self.now()
            self.log(f"[ANALYTICS] counts N={n} S={s} E={e} -> plan G0={g0} G1={g1}")
            return f"PLAN {g0} {g1}"
        if line.startswith("EMERG "):
            a = line[6:7].upper()
            if a not in ("N", "S", "E") or len(line) != 7:
                return "ERR bad_approach"
            with self.lock:
                if self.em_pending == "-" or a == "E":
                    self.em_pending = a
            self.log(f"[EMERGENCY] request forwarded for approach {a}")
            return "OK"
        return "ERR unknown_command"

    def start(self):
        outer = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.request.settimeout(3)
                try:
                    raw = self.rfile.readline(300)
                except OSError:
                    return
                line = raw.decode("ascii", "replace").replace("\r", "").strip("\n")
                self.wfile.write((outer.handle(line) + "\n").encode())

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        self._server = Server((self.host, self.port), Handler)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        threading.Thread(target=self._loop, daemon=True).start()
        self.log(f"[TCP] mock controller listening on port {self.port}")
        return self

    def stop(self):
        self._running = False
        if self._server:
            self._server.shutdown()
            self._server.server_close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Mock of qnx_traffic_controller.c")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--speed", type=float, default=1.0, help="controller clock multiplier (tests only)")
    args = ap.parse_args()
    MockController(port=args.port, speed=args.speed).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
