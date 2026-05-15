"""Live driver for the five-bar IK firmware.

`LiveSession` owns the serial port, a matplotlib widget figure that mirrors
the physical robot, and the move/home/release helpers. Designed for the
notebook but usable as a standalone module.
"""

import math
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))
import five_bar_client as fbc  # noqa: E402


class LiveSession:
    """One session = one open serial port + one live figure.

    Use directly or as a context manager:

        session = LiveSession("/dev/ttyUSB0")
        session.move(20, 50)
        session.close()

        # or
        with LiveSession("/dev/ttyUSB0") as session:
            session.move(20, 50)
    """

    DEG_PER_SEC = 1000.0 / 8.0  # matches firmware moveSmooth pacing
    FRAME_DT = 1.0 / 30.0

    def __init__(self, port="/dev/ttyUSB0", baud=115200, settle_s=2.0, timeout=1.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(settle_s)  # Nano DTR-resets on port open; wait for the bootloader.
        fbc.wait_for_ready(self.ser)

        self.fig, self.ax = plt.subplots(figsize=(7, 6))
        span_x = fbc.BASE_D / 2.0 + fbc.L1 + fbc.L2 + 10
        self.ax.set_xlim(-span_x, +span_x)
        self.ax.set_ylim(-10, fbc.L1 + fbc.L2 + 10)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")

        self._base_line, = self.ax.plot([], [], color="gray", linewidth=2)
        self._prox_left, = self.ax.plot([], [], color="tab:blue", linewidth=4)
        self._prox_right, = self.ax.plot([], [], color="tab:blue", linewidth=4)
        self._dist_left, = self.ax.plot([], [], color="tab:orange", linewidth=2.5)
        self._dist_right, = self.ax.plot([], [], color="tab:orange", linewidth=2.5)
        self._joints, = self.ax.plot([], [], "o", color="black", markersize=6)
        self._ee_dot, = self.ax.plot([], [], "o", color="tab:red", markersize=9)

        self.current = (math.pi / 2.0, math.pi / 2.0)
        self._redraw(*self.current, title_xy=fbc._HOME_XY)

    def _redraw(self, th1, th2, title_xy=None):
        mL, eL, ee, eR, mR = fbc.forward_kinematics(th1, th2)
        self._base_line.set_data([mL[0], mR[0]], [mL[1], mR[1]])
        self._prox_left.set_data([mL[0], eL[0]], [mL[1], eL[1]])
        self._prox_right.set_data([mR[0], eR[0]], [mR[1], eR[1]])
        if ee is not None:
            self._dist_left.set_data([eL[0], ee[0]], [eL[1], ee[1]])
            self._dist_right.set_data([eR[0], ee[0]], [eR[1], ee[1]])
            self._ee_dot.set_data([ee[0]], [ee[1]])
        self._joints.set_data(
            [mL[0], mR[0], eL[0], eR[0]],
            [mL[1], mR[1], eL[1], eR[1]],
        )
        if title_xy is not None:
            self.ax.set_title(f"target ({title_xy[0]:.1f}, {title_xy[1]:.1f}) mm")
        self.fig.canvas.draw_idle()

    def _tween(self, target, title_xy):
        c1, c2 = self.current
        t1, t2 = target
        max_d = max(abs(t1 - c1), abs(t2 - c2))
        duration = math.degrees(max_d) / self.DEG_PER_SEC
        if duration <= 0.0:
            self._redraw(t1, t2, title_xy=title_xy)
            self.current = target
            return
        t0 = time.time()
        while True:
            frac = min((time.time() - t0) / duration, 1.0)
            th1 = c1 + (t1 - c1) * frac
            th2 = c2 + (t2 - c2) * frac
            self._redraw(th1, th2, title_xy=title_xy)
            self.fig.canvas.flush_events()
            if frac >= 1.0:
                break
            time.sleep(self.FRAME_DT)
        self.current = target

    def move(self, x, y):
        sol = fbc.solve_ik(x, y)
        if sol is None:
            print(f"host: OOR ({x}, {y})")
            return
        clear = fbc.singularity_clearance(x, y)
        if clear is not None and clear < fbc.SINGULARITY_MARGIN_MM:
            print(f"warn: near singularity (clearance {clear:.1f} mm)")
        reply = fbc.send(self.ser, f"M {x} {y}")
        if not reply.startswith("OK"):
            return
        self._tween(sol, (x, y))

    def home(self):
        reply = fbc.send(self.ser, "H")
        if not reply.startswith("OK"):
            return
        self._tween((math.pi / 2.0, math.pi / 2.0), fbc._HOME_XY)

    def release(self):
        fbc.send(self.ser, "R")

    def status(self):
        fbc.send(self.ser, "S")

    def close(self):
        if self.ser.is_open:
            self.release()
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
