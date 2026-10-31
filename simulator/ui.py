"""All drawing: junction, vehicles, signal heads and the HUD panel."""
import math
import time

import pygame

import config as C
import vehicles as V

GRASS = (28, 40, 34)
ASPHALT = (47, 50, 58)
BOX = (53, 56, 65)
EDGE = (190, 196, 208)
DASH = (232, 190, 60)
PANEL = (21, 24, 32)
CARD = (30, 34, 45)
CARD_LINE = (48, 54, 70)
TEXT = (232, 236, 244)
MUTED = (140, 148, 166)
GREEN = (70, 225, 125)
AMBER = (255, 200, 60)
RED = (255, 80, 80)
CYAN = (90, 205, 255)

STATE_COL = {"GREEN": GREEN, "YELLOW": AMBER, "ALL_RED": RED,
             "EMERGENCY_GREEN": CYAN, "FAILSAFE": (255, 130, 60), "UNKNOWN": MUTED}
BULB = {"R": ((255, 72, 72), (72, 28, 30)), "Y": ((255, 205, 55), (72, 58, 22)),
        "G": ((62, 232, 124), (22, 66, 42))}
ROAD_NAME = {"N": "North", "S": "South", "E": "East"}


def _font(size, bold=False):
    return pygame.font.SysFont("segoeui,helveticaneue,arial,dejavusans,freesans", size, bold=bold)


class Renderer:
    def __init__(self, screen):
        self.screen = screen
        self.f_title, self.f_big = _font(21, True), _font(30, True)
        self.f_med, self.f_sm, self.f_xs = _font(16, True), _font(14), _font(12)
        self.bg = self._build_bg()
        self._car_cache = {}
        x0, w = C.SIM_W + 20, (C.HUD_W - 40 - 20) // 3
        self.buttons = {a: pygame.Rect(x0 + i * (w + 10), 566, w, 38) for i, a in enumerate("NSE")}

    # ---------------------------------------------------------------- background
    def _build_bg(self):
        s = pygame.Surface((C.SIM_W, C.WIN_H))
        s.fill(GRASS)
        for y in range(0, C.WIN_H, 36):
            if (y // 36) % 2:
                pygame.draw.rect(s, (31, 44, 37), (0, y, C.SIM_W, 36))
        rw, cx, cy = C.ROAD_HALF, C.CX, C.CY
        pygame.draw.rect(s, ASPHALT, (cx - rw, 0, 2 * rw, C.WIN_H))
        pygame.draw.rect(s, ASPHALT, (cx, cy - rw, C.SIM_W - cx, 2 * rw))
        pygame.draw.rect(s, BOX, (cx - rw, cy - rw, 2 * rw, 2 * rw))
        # road edges (not across the junction mouth)
        pygame.draw.line(s, EDGE, (cx - rw, 0), (cx - rw, C.WIN_H), 3)
        pygame.draw.line(s, EDGE, (cx + rw, 0), (cx + rw, cy - rw), 3)
        pygame.draw.line(s, EDGE, (cx + rw, cy + rw), (cx + rw, C.WIN_H), 3)
        pygame.draw.line(s, EDGE, (cx + rw, cy - rw), (C.SIM_W, cy - rw), 3)
        pygame.draw.line(s, EDGE, (cx + rw, cy + rw), (C.SIM_W, cy + rw), 3)
        pygame.draw.line(s, EDGE, (cx - rw, cy - rw), (cx - rw, cy + rw), 3)   # solid west kerb
        # centre dashes
        for y in list(range(0, cy - rw - 20, 34)) + list(range(cy + rw + 12, C.WIN_H, 34)):
            pygame.draw.rect(s, DASH, (cx - 2, y, 4, 18))
        for x in range(cx + rw + 12, C.SIM_W, 34):
            pygame.draw.rect(s, DASH, (x, cy - 2, 18, 4))
        # zebra crossings + stop lines
        zc = (222, 226, 235)
        for k in range(cx - rw + 6, cx + rw - 6, 16):
            pygame.draw.rect(s, zc, (k, cy - C.STOP_OFF + 6, 8, 16))
            pygame.draw.rect(s, zc, (k, cy + C.STOP_OFF - 22, 8, 16))
        for k in range(cy - rw + 6, cy + rw - 6, 16):
            pygame.draw.rect(s, zc, (cx + C.STOP_OFF - 22, k, 16, 8))
        half = lambda sign: (min(cx, cx + sign * rw), rw)       # (start, size) of one half-road
        x_n, wn = half(V.L)
        x_s, ws = half(-V.L)
        pygame.draw.rect(s, (255, 255, 255), (x_n, V.STOP_LINE["N"] - 2, wn, 4))
        pygame.draw.rect(s, (255, 255, 255), (x_s, V.STOP_LINE["S"] - 2, ws, 4))
        y_e, we = half(V.L)
        pygame.draw.rect(s, (255, 255, 255), (V.STOP_LINE["E"] - 2, min(cy, cy + V.L * rw), 4, rw))
        return s

    # ---------------------------------------------------------------- helpers
    def text(self, surf, msg, font, color, pos, anchor="topleft"):
        img = font.render(msg, True, color)
        rect = img.get_rect()
        setattr(rect, anchor, pos)
        surf.blit(img, rect)
        return rect

    def pill(self, msg, center, color, font=None):
        font = font or self.f_sm
        img = font.render(msg, True, TEXT)
        r = img.get_rect(center=center).inflate(22, 10)
        box = pygame.Surface(r.size, pygame.SRCALPHA)
        pygame.draw.rect(box, (14, 17, 24, 215), box.get_rect(), border_radius=r.h // 2)
        pygame.draw.rect(box, color, box.get_rect(), 2, border_radius=r.h // 2)
        self.screen.blit(box, r)
        self.screen.blit(img, img.get_rect(center=r.center))

    def _glow(self, center, color, radius=22, alpha=70):
        g = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        for r in range(radius, 0, -3):
            pygame.draw.circle(g, (*color, int(alpha * (1 - r / radius) + 8)), (radius, radius), r)
        self.screen.blit(g, g.get_rect(center=center))

    # ---------------------------------------------------------------- signal heads
    def head_pos(self, a):
        rw = C.ROAD_HALF + 22
        if a == "N":
            return C.CX + V.L * rw, V.STOP_LINE["N"] - 40
        if a == "S":
            return C.CX - V.L * rw, V.STOP_LINE["S"] + 40
        return V.STOP_LINE["E"] + 40, C.CY + V.L * rw

    def draw_head(self, a, ch, blink_on):
        x, y = self.head_pos(a)
        pole_to = {"N": (C.CX + V.L * C.ROAD_HALF, y), "S": (C.CX - V.L * C.ROAD_HALF, y),
                   "E": (x, C.CY + V.L * C.ROAD_HALF)}[a]
        pygame.draw.line(self.screen, (90, 96, 110), (x, y), pole_to, 3)
        r = pygame.Rect(0, 0, 26, 70)
        r.center = (x, y)
        pygame.draw.rect(self.screen, (12, 13, 17), r, border_radius=8)
        pygame.draw.rect(self.screen, (70, 76, 90), r, 2, border_radius=8)
        for i, key in enumerate("RYG"):
            on = (ch == key) and (blink_on or ch != "R" or True)
            col = BULB[key][0] if on else BULB[key][1]
            c = (x, y - 22 + i * 22)
            if on:
                self._glow(c, BULB[key][0])
            pygame.draw.circle(self.screen, col, c, 8)

    # ---------------------------------------------------------------- vehicles
    def _car(self, v, t):
        flash = int(t * 6) % 2 if v.emergency else 0
        key = (v.color, v.length, v.width, v.braking, flash, v.emergency)
        surf = self._car_cache.get(key)
        if surf is None:
            surf = pygame.Surface((v.length, v.width), pygame.SRCALPHA)
            body = pygame.Rect(0, 0, v.length, v.width)
            pygame.draw.rect(surf, v.color, body, border_radius=6)
            pygame.draw.rect(surf, tuple(int(c * 0.72) for c in v.color), body, 1, border_radius=6)
            if v.emergency:
                pygame.draw.rect(surf, (225, 40, 50), (4, v.width // 2 - 2, v.length - 8, 4))
                pygame.draw.rect(surf, (215, 235, 255), (v.length - 17, 3, 8, v.width - 6), border_radius=2)
                bar = pygame.Rect(v.length // 2 - 4, 3, 7, v.width - 6)
                pygame.draw.rect(surf, (255, 60, 60) if flash else (70, 120, 255), bar, border_radius=2)
            else:
                pygame.draw.rect(surf, (28, 34, 46), (v.length - 17, 3, 9, v.width - 6), border_radius=3)
                pygame.draw.rect(surf, tuple(int(c * 0.82) for c in v.color), (8, 4, v.length - 28, v.width - 8), border_radius=4)
            for dy in (2, v.width - 5):
                pygame.draw.rect(surf, (255, 240, 170), (v.length - 3, dy, 3, 3))
                pygame.draw.rect(surf, (255, 60, 60) if v.braking else (120, 30, 34), (0, dy, 3, 3))
            self._car_cache[key] = surf
        return surf

    def draw_vehicles(self, world, t):
        for a in "NSE":
            for v in world.veh[a]:
                x, y, ang = v.position()
                img = pygame.transform.rotate(self._car(v, t), -math.degrees(ang))
                if v.emergency:
                    self._glow((x, y), (255, 70, 70) if int(t * 6) % 2 else (80, 130, 255), 34, 90)
                self.screen.blit(img, img.get_rect(center=(x, y)))

    # ---------------------------------------------------------------- main draw
    def draw(self, world, snap, ui):
        t = time.time()
        self.screen.blit(self.bg, (0, 0))
        fresh = snap.fresh
        lights = snap.lights if fresh else {"N": "R", "S": "R", "E": "R"}
        blink = int(t * 2) % 2 == 0
        self.draw_vehicles(world, t)
        for a in "NSE":
            ch = lights[a]
            if not fresh and not blink:
                ch = "-"
            self.draw_head(a, ch, True)

        if snap.emergency != "-" and fresh:                    # pulsing ring around the junction
            pulse = 120 + int(110 * abs(math.sin(t * 5)))
            ring = pygame.Surface((C.SIM_W, C.WIN_H), pygame.SRCALPHA)
            pygame.draw.rect(ring, (*CYAN, pulse), (C.CX - 66, C.CY - 66, 132, 132), 4, border_radius=14)
            self.screen.blit(ring, (0, 0))

        counts = world.counts()
        for a, c in zip("NSE", counts):
            pos = {"N": (C.CX + (-V.L) * 150, 26), "S": (C.CX + V.L * 150, C.WIN_H - 26),
                   "E": (C.SIM_W - 96, C.CY - V.L * 96)}[a]
            colour = AMBER if ui["selected"] == a else CARD_LINE
            extra = f"  +{world.backlog[a]}" if world.backlog[a] else ""
            self.pill(f"{ROAD_NAME[a]}  ·  {c} waiting{extra}", pos, colour)

        banners = []
        if not fresh:
            banners.append((RED, "NO LINK TO CONTROLLER - signals held RED (fail-safe)"))
        if snap.fallback and fresh:
            banners.append((AMBER, "WATCHDOG FALLBACK - controller has no fresh traffic plan"))
        if snap.emergency != "-" and fresh:
            banners.append((CYAN, f"EMERGENCY PRIORITY - {ROAD_NAME.get(snap.emergency, snap.emergency)} road"))
        if ui["paused"]:
            banners.append((MUTED, "PAUSED (traffic only - the controller keeps running)"))
        for i, (col, msg) in enumerate(banners):
            self.pill(msg, (C.SIM_W // 2, 70 + i * 38), col, self.f_med)

        self.draw_hud(world, snap, ui, counts)

    # ---------------------------------------------------------------- HUD
    def card(self, rect, title=None):
        pygame.draw.rect(self.screen, CARD, rect, border_radius=12)
        pygame.draw.rect(self.screen, CARD_LINE, rect, 1, border_radius=12)
        if title:
            self.text(self.screen, title.upper(), self.f_xs, MUTED, (rect.x + 14, rect.y + 9))

    def draw_hud(self, world, snap, ui, counts):
        s = self.screen
        x0 = C.SIM_W
        pygame.draw.rect(s, PANEL, (x0, 0, C.HUD_W, C.WIN_H))
        pygame.draw.line(s, CARD_LINE, (x0, 0), (x0, C.WIN_H), 2)
        px, pw = x0 + 14, C.HUD_W - 28
        self.text(s, "QNX Traffic Simulator", self.f_title, TEXT, (px + 2, 12))
        self.text(s, "T-junction · phase 0 = N+S · phase 1 = E", self.f_xs, MUTED, (px + 2, 40))

        # link card
        r = pygame.Rect(px, 62, pw, 58)
        self.card(r, "Controller link")
        ok = snap.fresh
        pygame.draw.circle(s, GREEN if ok else RED, (r.x + 22, r.y + 38), 7)
        self.text(s, f"{snap.host}:{snap.port}", self.f_med, TEXT, (r.x + 38, r.y + 27))
        detail = f"{snap.rtt_ms:.0f} ms" if ok else (snap.error.split("] ")[-1][:22] or "waiting")
        self.text(s, detail, self.f_sm, GREEN if ok else RED, (r.right - 14, r.y + 29), "topright")

        # state card with countdown ring
        r = pygame.Rect(px, 130, pw, 150)
        self.card(r, "Signal state")
        col = STATE_COL.get(snap.state, MUTED) if ok else MUTED
        c = (r.x + 66, r.y + 84)
        rem, dur = snap.remaining(), snap.state_duration
        pygame.draw.circle(s, (42, 47, 62), c, 48, 8)
        if ok and dur > 0:
            frac = max(0.0, min(1.0, rem / dur))
            if frac > 0:
                pts = [(c[0] + 48 * math.cos(-math.pi / 2 + 2 * math.pi * frac * k / 60),
                        c[1] + 48 * math.sin(-math.pi / 2 + 2 * math.pi * frac * k / 60)) for k in range(61)]
                pygame.draw.lines(s, col, False, pts, 8)
            self.text(s, f"≈{rem:0.0f}" if rem >= 1 else f"≈{rem:0.1f}", self.f_big, TEXT, c, "center")
        else:
            self.text(s, "--", self.f_big, MUTED, c, "center")
        label = {"EMERGENCY_GREEN": "EMERGENCY", "ALL_RED": "ALL RED"}.get(snap.state, snap.state)
        self.text(s, label if ok else "OFFLINE", self.f_title, col, (r.x + 136, r.y + 34))
        lt = snap.lights
        if ok:
            if snap.state == "EMERGENCY_GREEN":
                phase = f"Emergency → {ROAD_NAME.get(snap.emergency, '?')} road"
            elif snap.state in ("ALL_RED", "FAILSAFE"):
                phase = "All-red clearance"
            else:
                phase = "Phase 0 · North + South" if "R" != lt["N"] or "R" != lt["S"] else "Phase 1 · East"
                if snap.emergency != "-":
                    phase = f"Clearing for {ROAD_NAME.get(snap.emergency, '?')} emergency"
        else:
            phase = "no data"
        self.text(s, phase, self.f_sm, TEXT, (r.x + 136, r.y + 66))
        self.text(s, "countdown is estimated from STATUS", self.f_xs, MUTED, (r.x + 136, r.y + 90))
        if snap.fallback and ok:
            self.pill("FALLBACK 10 s", (r.x + 190, r.y + 124), AMBER, self.f_xs)
        if snap.emergency != "-" and ok:
            self.pill(f"EM {snap.emergency}", (r.x + 270, r.y + 124), CYAN, self.f_xs)

        # plan bar
        r = pygame.Rect(px, 290, pw, 72)
        self.card(r, "Green plan from controller")
        g0, g1 = snap.g0, snap.g1
        tot = max(1, g0 + g1)
        bar = pygame.Rect(r.x + 14, r.y + 30, pw - 28, 16)
        pygame.draw.rect(s, (42, 47, 62), bar, border_radius=8)
        w0 = int(bar.w * g0 / tot)
        pygame.draw.rect(s, CYAN, (bar.x, bar.y, w0, bar.h), border_top_left_radius=8, border_bottom_left_radius=8)
        pygame.draw.rect(s, AMBER, (bar.x + w0, bar.y, bar.w - w0, bar.h), border_top_right_radius=8, border_bottom_right_radius=8)
        self.text(s, f"G0  N+S  {g0}s", self.f_xs, CYAN, (bar.x, bar.bottom + 5))
        self.text(s, f"G1  E  {g1}s", self.f_xs, AMBER, (bar.right, bar.bottom + 5), "topright")
        if snap.plan_time:
            age = time.monotonic() - snap.plan_time
            label = f"Webster C={snap.cycle}s Y={snap.flow_ratio:.2f} ({age:0.0f}s)" if snap.cycle else f"plan age {age:0.0f}s"
            self.text(s, label, self.f_xs, MUTED, (bar.centerx, bar.bottom + 5), "midtop")

        # approaches
        r = pygame.Rect(px, 372, pw, 128)
        self.card(r)
        for cx_, label in ((r.x + 16, "ROAD"), (r.x + 56, "SIGNAL"), (r.x + 130, "QUEUE"), (r.x + 202, "SENT"), (r.x + 272, "RATE/min")):
            self.text(s, label, self.f_xs, MUTED, (cx_, r.y + 9))
        for i, a in enumerate("NSE"):
            yy = r.y + 32 + i * 30
            if ui["selected"] == a:
                pygame.draw.rect(s, (44, 50, 68), (r.x + 6, yy - 3, pw - 12, 28), border_radius=8)
                pygame.draw.rect(s, AMBER, (r.x + 6, yy - 3, pw - 12, 28), 1, border_radius=8)
            self.text(s, ROAD_NAME[a], self.f_sm, TEXT, (r.x + 16, yy + 2))
            ch = snap.lights[a] if ok else "-"
            lc = {"R": RED, "Y": AMBER, "G": GREEN}.get(ch, (70, 76, 92))
            pygame.draw.circle(s, lc, (r.x + 70, yy + 11), 7)
            self.text(s, str(counts["NSE".index(a)]), self.f_med, TEXT, (r.x + 130, yy))
            self.text(s, str(snap.counts_sent["NSE".index(a)]), self.f_sm, MUTED, (r.x + 204, yy + 2))
            self.text(s, str(world.rates[a]), self.f_sm, TEXT, (r.x + 276, yy + 2))

        # stats
        r = pygame.Rect(px, 510, pw, 46)
        self.card(r)
        self.text(s, f"Passed {sum(world.passed.values())}", self.f_med, TEXT, (r.x + 14, r.y + 13))
        self.text(s, f"Avg wait {world.avg_wait:0.1f}s", self.f_med, TEXT, (r.right - 14, r.y + 13), "topright")

        # emergency buttons
        mouse = ui["mouse"]
        for a, br in self.buttons.items():
            hover = br.collidepoint(mouse)
            pygame.draw.rect(s, (130, 38, 48) if hover else (98, 30, 40), br, border_radius=10)
            pygame.draw.rect(s, (255, 110, 120), br, 1, border_radius=10)
            self.text(s, f"AMBULANCE {a}", self.f_xs, TEXT, br.center, "center")

        # log
        r = pygame.Rect(px, 614, pw, 126)
        self.card(r, "Protocol log")
        for i, (ts, msg, kind) in enumerate(snap.log[-6:]):
            col = {"ok": (150, 220, 175), "err": (255, 140, 140), "emerg": CYAN}.get(kind, MUTED)
            self.text(s, f"{time.strftime('%H:%M:%S', time.localtime(ts))}  {msg}"[:50], self.f_xs, col, (r.x + 12, r.y + 28 + i * 15))

        self.text(s, "TAB road · UP/DOWN rate · B burst · 1/2/3 ambulance N/S/E",
                  self.f_xs, MUTED, (px + 2, 750))
        self.text(s, "SPACE pause · R reset · P screenshot · ESC quit", self.f_xs, MUTED, (px + 2, 768))
