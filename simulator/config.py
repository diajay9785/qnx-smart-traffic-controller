"""Shared constants for the QNX traffic simulator.

Everything you are likely to want to tweak lives here.
"""

# ---- window --------------------------------------------------------------
WIN_W, WIN_H = 1280, 800
SIM_W = 900                      # width of the junction view (rest is the HUD)
HUD_W = WIN_W - SIM_W
FPS = 60

# ---- QNX controller link (matches qnx_traffic_controller.c) ---------------
DEFAULT_HOST = "10.0.0.2"   # <-- put the Raspberry Pi (QNX) IP here, or use --host
DEFAULT_PORT = 5000
STATUS_POLL_S = 0.25             # how often STATUS is requested
COUNTS_INTERVAL_S = 2.0          # how often COUNTS n s e is sent
SOCKET_TIMEOUT_S = 3.5           # COUNTS can block up to 2 s on the QNX side
STALE_AFTER_S = 2.0              # no fresh STATUS for this long -> fail-safe red
MAX_COUNT = 200                  # controller rejects counts above 200

# ---- controller timing constants (mirror the C #defines) ------------------
YELLOW_S = 3
ALL_RED_S = 1
EMERGENCY_ALL_RED_S = 5
EMERGENCY_HOLD_S = 10
FAILSAFE_S = 3
FALLBACK_AFTER_S = 15

# ---- junction geometry -----------------------------------------------------
# T-junction: North road (top), South road (bottom), East road (right).
# N and S share phase 0, E is phase 1 - exactly like the C controller.
LEFT_HAND_TRAFFIC = True         # India drives on the left; set False for right-hand
CX, CY = 420, 400                # centre of the junction box
ROAD_HALF = 60                   # half road width (2 lanes of 60 px)
LANE_OFF = 30                    # lane centre offset from the road centre line
STOP_OFF = 88                    # stop line distance from the junction centre
MARGIN = 40                      # vehicles spawn this far outside the window

# ---- traffic ----------------------------------------------------------------
DEFAULT_RATES = {"N": 18, "S": 18, "E": 12}   # arrivals per minute per road
RATE_STEP = 3
RATE_MAX = 90
