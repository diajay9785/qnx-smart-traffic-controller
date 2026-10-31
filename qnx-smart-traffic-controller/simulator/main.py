"""QNX Traffic Simulator - Pygame front end for qnx_traffic_controller.c

    python main.py --host <board-ip>          # real controller on QNX
    python main.py --mock                     # built-in mock controller (no board needed)
"""
import argparse
import os
import sys
import time

import config as C


def parse_args():
    ap = argparse.ArgumentParser(description="Junction simulator for the QNX RTOS traffic controller")
    ap.add_argument("--host", default=C.DEFAULT_HOST, help="IP of the QNX board (default %(default)s)")
    ap.add_argument("--port", type=int, default=C.DEFAULT_PORT)
    ap.add_argument("--mock", action="store_true", help="start the Python mock controller inside this process")
    ap.add_argument("--mock-speed", type=float, default=1.0, help=argparse.SUPPRESS)
    ap.add_argument("--right-hand", action="store_true", help="drive on the right instead of the left")
    ap.add_argument("--frames", type=int, default=0, help=argparse.SUPPRESS)      # test: exit after N frames
    ap.add_argument("--screenshot", default="", help=argparse.SUPPRESS)           # test: save on exit
    ap.add_argument("--script", default="", help=argparse.SUPPRESS)               # test: "sec:key,sec:key"
    return ap.parse_args()


def main():
    args = parse_args()
    if args.right_hand:
        C.LEFT_HAND_TRAFFIC = False          # must happen before vehicles/ui are imported
    if args.frames:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    import pygame
    import ui as UI
    import vehicles as V
    from qnx_client import QnxClient

    mock = None
    if args.mock:
        from mock_qnx_controller import MockController
        mock = MockController(port=args.port, speed=args.mock_speed, host="127.0.0.1").start()
        args.host = "127.0.0.1"

    pygame.init()
    pygame.display.set_caption("QNX Traffic Simulator")
    screen = pygame.display.set_mode((C.WIN_W, C.WIN_H))
    clock = pygame.time.Clock()
    renderer = UI.Renderer(screen)
    world = V.World()
    client = QnxClient(args.host, args.port, time_scale=args.mock_speed if mock else 1.0)
    client.start()

    state = {"selected": "N", "paused": False, "mouse": (0, 0)}
    scripted = []
    if args.script:
        for item in args.script.split(","):
            sec, key = item.split(":")
            scripted.append((float(sec), key))
    t_start = time.monotonic()
    frame = 0

    def emergency(a):
        world.spawn_emergency(a)
        client.emergency(a)

    def handle_key(name):
        if name == "tab":
            state["selected"] = "NSE"["NSE".index(state["selected"]) - 2]
        elif name == "up":
            world.rates[state["selected"]] = min(C.RATE_MAX, world.rates[state["selected"]] + C.RATE_STEP)
        elif name == "down":
            world.rates[state["selected"]] = max(0, world.rates[state["selected"]] - C.RATE_STEP)
        elif name == "b":
            world.burst(state["selected"], 10)
        elif name in ("1", "2", "3"):
            emergency("NSE"[int(name) - 1])
        elif name == "space":
            state["paused"] = not state["paused"]
        elif name == "r":
            world.reset()

    keymap = {pygame.K_TAB: "tab", pygame.K_UP: "up", pygame.K_DOWN: "down", pygame.K_b: "b",
              pygame.K_1: "1", pygame.K_2: "2", pygame.K_3: "3", pygame.K_SPACE: "space", pygame.K_r: "r"}

    running = True
    while running:
        dt = clock.tick(C.FPS) / 1000.0
        state["mouse"] = pygame.mouse.get_pos()
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_p:
                    os.makedirs("screenshots", exist_ok=True)
                    path = time.strftime("screenshots/sim_%Y%m%d_%H%M%S.png")
                    pygame.image.save(screen, path)
                    print("saved", path)
                elif ev.key in keymap:
                    handle_key(keymap[ev.key])
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                for a, rect in renderer.buttons.items():
                    if rect.collidepoint(ev.pos):
                        emergency(a)
        while scripted and time.monotonic() - t_start >= scripted[0][0]:
            handle_key(scripted.pop(0)[1])

        snap = client.snapshot()
        lights = snap.lights if snap.fresh else {"N": "R", "S": "R", "E": "R"}   # fail-safe: red
        if not state["paused"]:
            world.update(dt, lights, emergency_approach=snap.emergency if snap.state == "EMERGENCY_GREEN" else None, emergency_active=(snap.fresh and snap.state == "EMERGENCY_GREEN"))
        client.set_counts(*world.counts())

        renderer.draw(world, snap, state)
        pygame.display.flip()
        frame += 1
        if args.frames and frame >= args.frames:
            running = False

    if args.screenshot:
        pygame.image.save(screen, args.screenshot)
    client.stop()
    if mock:
        mock.stop()
    pygame.quit()


if __name__ == "__main__":
    sys.exit(main())
