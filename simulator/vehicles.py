"""Road geometry, vehicle physics and the traffic world (no drawing here)."""
import bisect
import math
import random

import config as C

L = 1 if C.LEFT_HAND_TRAFFIC else -1
SB_X = C.CX + L * C.LANE_OFF     # southbound lane (vehicles from the North road)
NB_X = C.CX - L * C.LANE_OFF     # northbound lane (vehicles from the South road)
WB_Y = C.CY + L * C.LANE_OFF     # westbound lane  (vehicles from the East road)
TOP, BOT, RIGHT = -C.MARGIN, C.WIN_H + C.MARGIN, C.SIM_W + C.MARGIN

STOP_LINE = {"N": C.CY - C.STOP_OFF, "S": C.CY + C.STOP_OFF, "E": C.CX + C.STOP_OFF}
# shift from an approach lane to the road centre line (ambulances overtake the queue there)
CENTER_SHIFT = {"N": (-L * C.LANE_OFF, 0), "S": (L * C.LANE_OFF, 0), "E": (0, -L * C.LANE_OFF)}

ACC = 150.0        # px/s^2
B_COMF = 170.0     # comfortable braking
B_HARD = 450.0     # limit braking
GAP = 9.0          # standstill gap between vehicles

PALETTE = [(231, 76, 90), (66, 153, 225), (245, 190, 60), (110, 200, 140),
           (170, 120, 230), (240, 140, 80), (200, 205, 215), (90, 215, 215),
           (240, 110, 170), (120, 130, 150)]


class Path:
    def __init__(self, pts):
        self.pts = pts
        self.cum = [0.0]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x1 - x0, y1 - y0))
        self.length = self.cum[-1]

    def at(self, s):
        s = max(0.0, min(s, self.length))
        i = min(bisect.bisect_right(self.cum, s) - 1, len(self.pts) - 2)
        seg = self.cum[i + 1] - self.cum[i]
        t = (s - self.cum[i]) / seg if seg > 1e-9 else 0.0
        (x0, y0), (x1, y1) = self.pts[i], self.pts[i + 1]
        return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, math.atan2(y1 - y0, x1 - x0)


def _bezier(p0, c, p1, n=16):
    out = []
    for k in range(n + 1):
        t = k / n
        out.append(((1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * c[0] + t * t * p1[0],
                    (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * c[1] + t * t * p1[1]))
    return out


def build_approaches():
    """Build movement choices for all three approaches.

    N: straight to S (70%) or turn to E (30%).
    S: straight to N (70%) or turn to E (30%).
    E: turn to N or S (50/50).
    stop_s is the distance to the stop line along the selected path.
    """
    R = C.ROAD_HALF
    n_stop = C.STOP_LINE_N if hasattr(C, "STOP_LINE_N") else C.CY - C.STOP_OFF
    s_stop = C.CY + C.STOP_OFF
    e_stop = C.CX + C.STOP_OFF

    # Straight-through movements.
    p_ns = Path([(SB_X, TOP), (SB_X, BOT)])
    p_sn = Path([(NB_X, BOT), (NB_X, TOP)])

    # Turn paths from N/S into the east arm. The two routes use visually
    # separated exit lanes so simultaneous N+S phase traffic cannot overlap.
    n_exit_y = C.CY - L * C.LANE_OFF
    s_exit_y = C.CY + L * C.LANE_OFF
    n_curve_start = (SB_X, C.CY - R - 18)
    n_curve_end = (C.CX + R + 18, n_exit_y)
    n_turn_pts = [(SB_X, TOP), n_curve_start]
    # Control point sits east of centre so the N-to-E turn stays in the upper-right quadrant.
    n_turn_pts += _bezier(
        n_curve_start,
        (C.CX + R - 4, C.CY - R - 4),
        n_curve_end,
    )[1:]
    n_turn_pts += [(RIGHT, n_exit_y)]

    s_curve_start = (NB_X, C.CY + R + 18)
    s_curve_end = (C.CX + R + 18, s_exit_y)
    s_turn_pts = [(NB_X, BOT), s_curve_start]
    # S-approach turn stays in the lower-right quadrant on its own exit lane.
    s_turn_pts += _bezier(
        s_curve_start,
        (C.CX + R - 4, C.CY + R + 4),
        s_curve_end,
    )[1:]
    s_turn_pts += [(RIGHT, s_exit_y)]

    # East approach splits into N and S. Curves are separated at the exits.
    e_to_s = [(RIGHT, WB_Y), (C.CX + R + 18, WB_Y)]
    e_to_s += _bezier(
        (C.CX + R + 18, WB_Y),
        (C.CX + R - 4, C.CY + R - 4),
        (SB_X, C.CY + R + 18),
    )[1:]
    e_to_s += [(SB_X, BOT)]
    e_to_n = [(RIGHT, WB_Y), (C.CX + R + 18, WB_Y)]
    e_to_n += _bezier(
        (C.CX + R + 18, WB_Y),
        (C.CX + R - 4, C.CY - R + 4),
        (NB_X, C.CY - R - 18),
    )[1:]
    e_to_n += [(NB_X, TOP)]

    return {
        "N": [(p_ns, 0.70, n_stop - TOP), (Path(n_turn_pts), 0.30, n_stop - TOP)],
        # S-to-E turn is disabled: it would cross N straight traffic while N and S share phase 0.
        "S": [(p_sn, 1.00, BOT - s_stop)],
        # E runs alone in phase 1, so its turns never meet N/S traffic.
        "E": [(Path(e_to_s), 0.50, RIGHT - e_stop), (Path(e_to_n), 0.50, RIGHT - e_stop)],
    }

APPROACHES = build_approaches()


class Vehicle:
    def __init__(self, approach, path, stop_s, emergency=False):
        self.approach, self.path, self.emergency = approach, path, emergency
        self.length = 48 if emergency else random.choice((36, 38, 40, 42))
        self.width = 24 if emergency else 21
        self.vmax = 150.0 if emergency else random.uniform(95, 125)
        self.v = self.vmax * 0.5
        self.s = 0.0
        self.stop_lim = stop_s - self.length / 2      # centre position when front is on the line
        self.color = (245, 245, 250) if emergency else random.choice(PALETTE)
        self.crossed = False
        self.counted = False
        self.decision = None                           # amber decision: 'go' / 'stop'
        self.wait = 0.0
        self.braking = False

    def step(self, dt, light, leader):
        v_t = self.vmax
        hard_limit = None
        if leader is not None:
            gap = (leader.s - leader.length / 2) - (self.s + self.length / 2) - (GAP + 0.25 * self.v)
            v_t = min(v_t, leader.v + math.sqrt(2 * B_COMF * max(0.0, gap)))
            hard_limit = leader.s - leader.length / 2 - GAP * 0.6 - self.length / 2

        stop_here = False
        if not self.crossed:
            dist = self.stop_lim - self.s
            if light == "G":
                self.decision = None
            elif light == "Y":
                if self.decision is None:
                    brake = self.v * self.v / (2 * B_COMF)
                    self.decision = "go" if dist < brake else "stop"
            elif self.decision is None:
                self.decision = "stop"
            if light != "G" and self.decision != "go":
                stop_here = True
                v_t = min(v_t, math.sqrt(2 * B_COMF * max(0.0, dist)))

        self.braking = self.v > v_t + 0.5 or self.v < 2.0
        if self.v < v_t:
            self.v = min(v_t, self.v + ACC * dt)
        else:
            self.v = max(v_t, self.v - B_HARD * dt)

        prev = self.s
        self.s += self.v * dt
        if hard_limit is not None and self.s > hard_limit:
            self.s = max(prev, hard_limit)
            self.v = min(self.v, leader.v)
        if stop_here and self.s > self.stop_lim:
            self.s = max(prev, self.stop_lim)
            self.v = 0.0
        if not self.crossed and self.s > self.stop_lim + 3:
            self.crossed = True
        if self.v < 4 and not self.crossed:
            self.wait += dt

    def lateral(self):
        """1.0 = on the centre line (ambulance overtaking), 0.0 = in its own lane."""
        if not self.emergency:
            return 0.0
        d = self.s - self.stop_lim
        return 1.0 if d <= 0 else max(0.0, 1.0 - d / 100.0)

    def position(self):
        x, y, ang = self.path.at(self.s)
        f = self.lateral()
        if f:
            dx, dy = CENTER_SHIFT[self.approach]
            x, y = x + dx * f, y + dy * f
        return x, y, ang


class World:
    def __init__(self):
        self.rates = dict(C.DEFAULT_RATES)
        self.reset()

    def reset(self):
        self.veh = {"N": [], "S": [], "E": []}
        self.backlog = {"N": 0, "S": 0, "E": 0}        # vehicles queued beyond the screen edge
        self.emerg_pending = {"N": 0, "S": 0, "E": 0}
        self.passed = {"N": 0, "S": 0, "E": 0}
        self.wait_sum = 0.0
        self.wait_n = 0

    def burst(self, approach, n=10):
        self.backlog[approach] += n

    def spawn_emergency(self, approach):
        self.emerg_pending[approach] += 1

    def _space(self, approach, emergency):
        mins = [v.s for v in self.veh[approach] if v.emergency == emergency]
        return (min(mins) if mins else 1e9) > 62

    def _spawn(self, approach, emergency):
        path, _, stop_s = random.choices(APPROACHES[approach], weights=[w for _, w, _ in APPROACHES[approach]])[0]
        self.veh[approach].append(Vehicle(approach, path, stop_s, emergency))

    def counts(self):
        """Vehicles waiting for / approaching each stop line (what a YOLO counter would report)."""
        out = []
        for a in "NSE":
            waiting = sum(1 for v in self.veh[a] if not v.crossed)
            out.append(min(C.MAX_COUNT, waiting + self.backlog[a] + self.emerg_pending[a]))
        return tuple(out)

    def update(self, dt, lights, emergency_approach=None, emergency_active=False):
        """Advance traffic. During emergency green, only ambulances on the
        selected approach move; all ordinary traffic is held to avoid visual
        collisions and to model an exclusive emergency corridor.
        """
        dt = min(dt, 0.05)
        for a in "NSE":
            if random.random() < self.rates[a] / 60.0 * dt:
                self.backlog[a] += 1
            if self.emerg_pending[a] and self._space(a, True):
                self.emerg_pending[a] -= 1
                self._spawn(a, True)
            if self.backlog[a] and self._space(a, False):
                self.backlog[a] -= 1
                self._spawn(a, False)

            last_norm = last_em = None
            for v in sorted(self.veh[a], key=lambda x: -x.s):
                # Ambulances wait for their own emergency-green window. The
                # emergency phase freezes all non-ambulance traffic, including
                # other vehicles queued on the same approach.
                if v.emergency:
                    allowed = emergency_active and a == emergency_approach
                    light = "G" if allowed else "R"
                elif emergency_active:
                    continue
                else:
                    light = lights.get(a, "R")

                v.step(dt, light, last_em if v.emergency else last_norm)
                if v.emergency:
                    last_em = v
                else:
                    last_norm = v
                if v.crossed and not v.counted:
                    v.counted = True
                    self.passed[a] += 1
                    self.wait_sum += v.wait
                    self.wait_n += 1
            self.veh[a] = [v for v in self.veh[a] if v.s < v.path.length - 1]

    @property
    def avg_wait(self):
        return self.wait_sum / self.wait_n if self.wait_n else 0.0
