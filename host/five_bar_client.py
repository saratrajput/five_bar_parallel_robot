#!/usr/bin/env python3
"""Interactive serial client for the five-bar IK firmware.

Commands at the REPL:
    move <x> <y>   send M command, end-effector to (x, y) mm
    home           park at home pose
    status         read last commanded servo angles
    release        detach servos (no torque)
    raw <line>     send an arbitrary line verbatim
    quit           exit

Pass --plot to open a live 2D matplotlib view of the linkage.
"""

import argparse
import math
import threading
import time

import serial


# Geometry mirrors firmware/five_bar_ik/five_bar_ik.ino
L1 = 33.9
L2 = 50.0
BASE_D = 77.144

_HOME_XY = (0.0, math.sqrt(L2 * L2 - (BASE_D / 2.0) * (BASE_D / 2.0)) + L1)


def solve_ik(x, y):
    """Elbows-out IK. Returns (theta1, theta2) in radians or None if unreachable."""
    a1x = -BASE_D / 2.0
    a2x = +BASE_D / 2.0
    dx1, dy1 = x - a1x, y
    dx2, dy2 = x - a2x, y
    r1 = math.hypot(dx1, dy1)
    r2 = math.hypot(dx2, dy2)
    rmin = abs(L1 - L2)
    rmax = L1 + L2
    if not (rmin <= r1 <= rmax and rmin <= r2 <= rmax):
        return None
    c1 = (L1 * L1 + r1 * r1 - L2 * L2) / (2.0 * L1 * r1)
    c2 = (L1 * L1 + r2 * r2 - L2 * L2) / (2.0 * L1 * r2)
    if not (-1.0 <= c1 <= 1.0 and -1.0 <= c2 <= 1.0):
        return None
    phi1 = math.atan2(dy1, dx1)
    phi2 = math.atan2(dy2, dx2)
    th1 = phi1 + math.acos(c1)
    th2 = phi2 - math.acos(c2)
    return th1, th2


def forward_kinematics(th1, th2):
    """Return joint positions (left_motor, left_elbow, ee, right_elbow, right_motor)."""
    mL = (-BASE_D / 2.0, 0.0)
    mR = (+BASE_D / 2.0, 0.0)
    eL = (mL[0] + L1 * math.cos(th1), mL[1] + L1 * math.sin(th1))
    eR = (mR[0] + L1 * math.cos(th2), mR[1] + L1 * math.sin(th2))
    # End-effector: intersection of circles (eL, L2) and (eR, L2); take the +y branch.
    dx = eR[0] - eL[0]
    dy = eR[1] - eL[1]
    d = math.hypot(dx, dy)
    if d == 0.0 or d > 2.0 * L2:
        return mL, eL, None, eR, mR
    a = d / 2.0
    h2 = L2 * L2 - a * a
    if h2 < 0.0:
        return mL, eL, None, eR, mR
    h = math.sqrt(h2)
    mx = (eL[0] + eR[0]) / 2.0
    my = (eL[1] + eR[1]) / 2.0
    # Always pick the +perp branch — matches the elbows-out IK assembly mode.
    px, py = -dy / d, dx / d
    ee = (mx + h * px, my + h * py)
    return mL, eL, ee, eR, mR


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.current = (math.pi / 2.0, math.pi / 2.0)
        self.target = (math.pi / 2.0, math.pi / 2.0)
        self.title_xy = _HOME_XY
        self.last_tick = time.time()
        self.stop = threading.Event()


def wait_for_ready(ser, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        print(f"< {line}")
        if line.startswith("READY"):
            return True
    return False


def send(ser, line, serial_lock=None):
    if serial_lock is None:
        ser.write((line + "\n").encode())
        ser.flush()
        reply = ser.readline().decode(errors="replace").strip()
    else:
        with serial_lock:
            ser.write((line + "\n").encode())
            ser.flush()
            reply = ser.readline().decode(errors="replace").strip()
    print(f"< {reply}" if reply else "< (no reply)")
    return reply


def repl(ser, state=None):
    print("Type '?' for firmware help, 'quit' to exit.")
    serial_lock = state.serial_lock if state else None
    while True:
        try:
            raw = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            send(ser, "R", serial_lock)
            if state:
                state.stop.set()
            return
        if not raw:
            continue
        parts = raw.split()
        cmd = parts[0].lower()
        if cmd in ("quit", "exit", "q"):
            send(ser, "R", serial_lock)
            if state:
                state.stop.set()
            return
        if cmd == "move" and len(parts) == 3:
            try:
                x = float(parts[1])
                y = float(parts[2])
            except ValueError:
                print("usage: move <x> <y>")
                continue
            reply = send(ser, f"M {x} {y}", serial_lock)
            if state and reply.startswith("OK"):
                sol = solve_ik(x, y)
                if sol is not None:
                    with state.lock:
                        state.target = sol
                        state.title_xy = (x, y)
        elif cmd == "home":
            reply = send(ser, "H", serial_lock)
            if state and reply.startswith("OK"):
                with state.lock:
                    state.target = (math.pi / 2.0, math.pi / 2.0)
                    state.title_xy = _HOME_XY
        elif cmd == "status":
            send(ser, "S", serial_lock)
        elif cmd == "release":
            send(ser, "R", serial_lock)
        elif cmd == "raw" and len(parts) >= 2:
            send(ser, raw[len("raw "):], serial_lock)
        elif cmd == "?":
            send(ser, "?", serial_lock)
        else:
            print("usage: move <x> <y> | home | status | release | raw <line> | ? | quit")


def run_with_plot(ser, state):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots(figsize=(7, 6))
    span_x = BASE_D / 2.0 + L1 + L2 + 10
    ax.set_xlim(-span_x, +span_x)
    ax.set_ylim(-10, L1 + L2 + 10)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")

    base_line, = ax.plot([], [], color="gray", linewidth=2)
    prox_left, = ax.plot([], [], color="tab:blue", linewidth=4)
    prox_right, = ax.plot([], [], color="tab:blue", linewidth=4)
    dist_left, = ax.plot([], [], color="tab:orange", linewidth=2.5)
    dist_right, = ax.plot([], [], color="tab:orange", linewidth=2.5)
    joints, = ax.plot([], [], "o", color="black", markersize=6)
    ee_dot, = ax.plot([], [], "o", color="tab:red", markersize=9)

    # Match firmware moveSmooth pacing: ~8 ms per degree of the larger axis.
    deg_per_sec = 1000.0 / 8.0

    def tween():
        now = time.time()
        with state.lock:
            dt = now - state.last_tick
            state.last_tick = now
            c1, c2 = state.current
            t1, t2 = state.target
            max_step_rad = math.radians(deg_per_sec * dt)
            d1 = t1 - c1
            d2 = t2 - c2
            max_d = max(abs(d1), abs(d2))
            if max_d <= max_step_rad or max_d == 0.0:
                state.current = (t1, t2)
            else:
                scale = max_step_rad / max_d
                state.current = (c1 + d1 * scale, c2 + d2 * scale)
            return state.current, state.title_xy

    def update(_frame):
        if state.stop.is_set():
            plt.close(fig)
            return ()
        (th1, th2), (tx, ty) = tween()
        mL, eL, ee, eR, mR = forward_kinematics(th1, th2)
        base_line.set_data([mL[0], mR[0]], [mL[1], mR[1]])
        prox_left.set_data([mL[0], eL[0]], [mL[1], eL[1]])
        prox_right.set_data([mR[0], eR[0]], [mR[1], eR[1]])
        if ee is not None:
            dist_left.set_data([eL[0], ee[0]], [eL[1], ee[1]])
            dist_right.set_data([eR[0], ee[0]], [eR[1], ee[1]])
            ee_dot.set_data([ee[0]], [ee[1]])
        joints.set_data(
            [mL[0], mR[0], eL[0], eR[0]],
            [mL[1], mR[1], eL[1], eR[1]],
        )
        ax.set_title(f"target ({tx:.1f}, {ty:.1f}) mm")
        return base_line, prox_left, prox_right, dist_left, dist_right, joints, ee_dot

    def on_close(_evt):
        if not state.stop.is_set():
            try:
                with state.serial_lock:
                    ser.write(b"R\n")
                    ser.flush()
            except Exception:
                pass
        state.stop.set()

    fig.canvas.mpl_connect("close_event", on_close)

    state.last_tick = time.time()
    repl_thread = threading.Thread(target=repl, args=(ser, state), daemon=True)
    repl_thread.start()

    _anim = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--plot", action="store_true", help="open a live 2D linkage view")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=1.0) as ser:
        # Nano resets on DTR; give the bootloader a moment then drain the banner.
        time.sleep(2.0)
        wait_for_ready(ser)
        if args.plot:
            run_with_plot(ser, SharedState())
        else:
            repl(ser)


if __name__ == "__main__":
    main()
