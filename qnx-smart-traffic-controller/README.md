# QNX Smart Traffic Controller

A real-time adaptive traffic signal controller for a T-junction, written in C for the **QNX RTOS**, with a Pygame junction simulator that drives it over TCP. The controller sizes green times from live vehicle counts (Webster's method), pre-empts for ambulances, and blocks unsafe signal combinations in software.

## Overview

| Part | Runs on | Role |
|---|---|---|
| `controller/qnx_traffic_controller.c` | QNX (Raspberry Pi) | Decides the signals: Webster timing, emergency pre-emption, fail-safe, latency statistics |
| `simulator/` | Windows / Linux / macOS | Draws the junction, moves vehicles, sends vehicle counts and ambulance requests, shows the controller's lights |

```
 Pygame simulator ──COUNTS n s e──▶  QNX controller (TCP 5000)
                  ◀── PLAN g0 g1 ───
                  ──EMERG N/S/E───▶
                  ◀── STATUS poll ──  (lights, state, G0/G1, fallback, emergency, time left)
```

The lights on screen come only from the controller's `STATUS` reply. If the link drops or `STATUS` goes stale for 2 s, the simulator shows all-red and holds vehicles at the stop lines.

## Features

**Real-time (QNX)**
- Seven tasks on POSIX threads with `SCHED_FIFO` priorities
- Event-driven controller blocked on a POSIX message queue; message priorities let emergencies overtake queued traffic plans
- Single timer service on absolute `CLOCK_MONOTONIC` deadlines, immune to wall-clock changes
- `mlockall` to avoid page faults in the critical path
- Logger ring buffer so real-time threads never block on stdout
- Built-in latency histograms (min / avg / p50 / p99 / max / deadline misses) via the `STATS` command

**Traffic logic**
- Webster optimum cycle and green split from North, South and East counts
- Watchdog: no new plan for 15 s switches to fixed 10 s fallback timing
- Emergency pre-emption: yellow, 5 s all-red, exclusive ambulance green; each extra ambulance on the active approach adds 5 s; the interrupted phase then resumes
- Conflict guard between the state machine and the outputs: an unsafe combination forces an all-red failsafe

## Junction

T-junction, traffic drives on the left (use `--right-hand` to flip). North and South run together in **phase 0**; East runs alone in **phase 1**.

| Approach | Movements |
|---|---|
| North | 70% straight to South, 30% turn to East |
| South | straight to North only |
| East | 50% turn to North, 50% turn to South |

The South-to-East turn is disabled on purpose: it would cross the North straight-through lane while North and South share green. Enabling it needs a separate protected-turn phase in the controller.

## Controller tasks

| Task | Priority | Purpose |
|---|---|---|
| Signal controller | 30 | State machine: GREEN, YELLOW, ALL_RED, EMERGENCY_GREEN, FAILSAFE |
| Emergency | 30 | Forwards validated `EMERG` requests to the controller |
| Timer service | 29 | Cycle, emergency and sampling timers, plan watchdog |
| Analytics | 20 | Counts to green split (Webster), sends the plan |
| Sampling | 20 | Vehicle-sampling heartbeat, measures sampling jitter |
| TCP server | 15 | Command interface on port 5000 |
| Logger | 10 | Buffered console output and periodic statistics |

### Message queues

| Queue | Direction | Content |
|---|---|---|
| `/tc_ctrl` | all tasks to controller | state timeouts (prio 31), emergency requests (30), watchdog (20), plans (10) |
| `/tc_analytics` | TCP to analytics | vehicle counts |
| `/tc_emerg` | TCP to emergency task | validated emergency requests |

### Webster timing

```
y0 = max(N, S) / SAT      y1 = E / SAT      Y = y0 + y1
C0 = (1.5 L + 5) / (1 - Y)                  g_i = (C0 - L) * y_i / Y
```

`L` = 8 s lost time, `SAT` = 40 vehicles, cycle limited to 30–120 s, each green 7–90 s, `Y` capped at 0.90.

## TCP protocol

One newline-terminated command per connection.

| Command | Reply |
|---|---|
| `PING` | `PONG` |
| `COUNTS N S E` | `PLAN G0 G1 CYCLE Y` (counts 0–200) |
| `STATUS` | `STATE <state> N=G S=G E=R G0=.. G1=.. FB=.. EM=.. C=.. REM=..` |
| `EMERG N` / `EMERG S` / `EMERG E` | `OK` |
| `STATS` | transitions, conflicts blocked and latency statistics |

Errors: `ERR bad_counts`, `ERR bad_approach`, `ERR busy`, `ERR unknown_command`.

## Build and run

### 1. Controller on QNX

Needs a QNX SDP environment. Link the socket library and target your board's CPU (`aarch64le` for a 64-bit Raspberry Pi image).

```sh
qcc -Vgcc_ntoaarch64le -O2 -Wall -o qnx_traffic_controller controller/qnx_traffic_controller.c -lsocket
```

On the board:

```sh
mqueue &                      # message-queue manager, if your image needs it
./qnx_traffic_controller      # listens on TCP 5000
ifconfig                      # note the board's IP address
```

Options: `-p <port>`, `-q` (quiet), `-n` (no real-time scheduling, for testing without privileges).

### 2. Simulator

Python 3.9+ and pygame 2.5+.

```sh
cd simulator
pip install -r requirements.txt

python qnx_cli.py PING --host <board-ip>      # must print PONG
python main.py --host <board-ip>
```

No board handy? The mock controller is a Python port of the C state machine and protocol:

```sh
python main.py --mock
```

Default host is `10.0.0.2` (`simulator/config.py`).

### 3. Test the C controller on a Linux PC

```sh
gcc -O2 -Wall -o tc controller/qnx_traffic_controller.c -lpthread -lrt
./tc -n -p 5055 &
python simulator/qnx_cli.py STATUS --host 127.0.0.1 --port 5055
```

## Simulator controls

| Key / mouse | Action |
|---|---|
| `TAB` | select a road |
| `UP` / `DOWN` | change that road's arrival rate (vehicles/min) |
| `B` | burst of 10 extra vehicles on the selected road |
| `1` `2` `3` or buttons | ambulance on North / South / East and `EMERG` command |
| `SPACE` | pause traffic (counts keep flowing so the watchdog stays quiet) |
| `R` | reset traffic |
| `P` | screenshot to `screenshots/` |
| `ESC` | quit |

The simulator sends `COUNTS` every 2 s and polls `STATUS` 4 times a second. Ambulances overtake the queue on the centre line, wait at the stop line and move when the controller grants emergency green. Normal vehicles freeze during emergency green.

## Repository layout

```
controller/
  qnx_traffic_controller.c    QNX controller (single file)
simulator/
  main.py                     entry point, input, main loop
  qnx_client.py               background TCP client and STATUS parser
  vehicles.py                 road geometry, car-following physics, spawning, counts
  ui.py                       junction, signal heads, vehicles, HUD
  config.py                   host/port, timings, geometry, arrival rates
  mock_qnx_controller.py      Python port of the controller for laptop testing
  qnx_cli.py                  one-shot command line client
  requirements.txt
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `PING` times out | controller not running, wrong IP, or port 5000 blocked; check `ifconfig` on QNX |
| Linker error for `socket`, `bind`, `setsockopt` | add `-lsocket` |
| `mq_open` fails | start the message-queue manager: `mqueue &` |
| Simulator shows all-red and *NO LINK* | run `qnx_cli.py STATUS` first and confirm a valid `STATE ...` reply |
| Hotspot network | both devices must be on the same reachable network; some hotspots isolate clients |
| Server stops answering | the controller serves one connection at a time; do not leave `telnet` or `nc` open on port 5000 |

## Limits

- Vehicle counts come from the simulation, not from a camera model.
- Signal states are reported over TCP; the controller drives no GPIO pins.
- Development prototype, not safety-certified traffic-control software.

## Authors

Divyalakshmi Jayaprakash, Dr. N.G.P. Institute of Technology, Coimbatore
