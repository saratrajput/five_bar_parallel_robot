"""Tutorial plotting functions for the five-bar IK notebook.

Each function builds its own figure, annotates it, and shows the result.
All kinematics come from `host/five_bar_client.py` (imported as `fbc`).
"""

import math
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
from ipywidgets import interact, FloatSlider

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))
import five_bar_client as fbc  # noqa: E402


def setup_ax(ax, *, span_x=None, y_max=None, y_min=-10):
    if span_x is None:
        span_x = fbc.BASE_D / 2.0 + fbc.L1 + fbc.L2 + 10
    if y_max is None:
        y_max = fbc.L1 + fbc.L2 + 10
    ax.set_xlim(-span_x, +span_x)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")


def draw_linkage(ax, th1, th2, *, target=None):
    mL, eL, ee, eR, mR = fbc.forward_kinematics(th1, th2)
    ax.plot([mL[0], mR[0]], [mL[1], mR[1]], color="gray", lw=2)
    ax.plot([mL[0], eL[0]], [mL[1], eL[1]], color="tab:blue", lw=4)
    ax.plot([mR[0], eR[0]], [mR[1], eR[1]], color="tab:blue", lw=4)
    if ee is not None:
        ax.plot([eL[0], ee[0]], [eL[1], ee[1]], color="tab:orange", lw=2.5)
        ax.plot([eR[0], ee[0]], [eR[1], ee[1]], color="tab:orange", lw=2.5)
        ax.plot([ee[0]], [ee[1]], "o", color="tab:red", markersize=9)
    ax.plot(
        [mL[0], mR[0], eL[0], eR[0]],
        [mL[1], mR[1], eL[1], eR[1]],
        "o", color="black", markersize=6,
    )
    if target is not None:
        ax.plot([target[0]], [target[1]], "x", color="tab:red", markersize=10, mew=2)


def plot_geometry():
    fig, ax = plt.subplots(figsize=(7, 5))
    setup_ax(ax)

    th1, th2 = math.pi / 2, math.pi / 2
    draw_linkage(ax, th1, th2)
    mL, eL, ee, eR, mR = fbc.forward_kinematics(th1, th2)

    ax.annotate("left motor", xy=mL, xytext=(mL[0] - 6, -6), ha="right")
    ax.annotate("right motor", xy=mR, xytext=(mR[0] + 6, -6), ha="left")
    ax.annotate("elbow", xy=eL, xytext=(eL[0] - 8, eL[1]), ha="right")
    ax.annotate("elbow", xy=eR, xytext=(eR[0] + 8, eR[1]), ha="left")
    ax.text((mL[0] + eL[0]) / 2 - 4, (mL[1] + eL[1]) / 2, f"L1 = {fbc.L1} mm",
            color="tab:blue", ha="right")
    ax.text((eL[0] + ee[0]) / 2 - 4, (eL[1] + ee[1]) / 2 + 2, f"L2 = {fbc.L2} mm",
            color="tab:orange", ha="right")
    ax.text(0, -7, f"BASE_D = {fbc.BASE_D} mm", color="gray", ha="center")
    ax.annotate("end-effector", xy=ee, xytext=(ee[0] + 12, ee[1] + 2),
                color="tab:red",
                arrowprops=dict(arrowstyle="->", color="tab:red", lw=1))
    ax.set_title("Five-bar geometry at home")
    plt.show()


def plot_single_arm(x=15, y=50):
    th1, th2 = fbc.solve_ik(x, y)
    mL, eL, ee, _, _ = fbc.forward_kinematics(th1, th2)

    fig, ax = plt.subplots(figsize=(7, 5))
    setup_ax(ax)
    ax.plot([mL[0], eL[0]], [mL[1], eL[1]], color="tab:blue", lw=4)
    ax.plot([eL[0], ee[0]], [eL[1], ee[1]], color="tab:orange", lw=2.5)
    ax.plot([mL[0], eL[0]], [mL[1], eL[1]], "o", color="black", markersize=7)
    ax.plot([ee[0]], [ee[1]], "o", color="tab:red", markersize=10)

    ax.annotate("M (motor)", xy=mL, xytext=(mL[0] - 3, -6), ha="right", fontsize=11)
    ax.annotate("E (elbow)", xy=eL, xytext=(eL[0] - 4, eL[1] + 3), ha="right", fontsize=11)
    ax.annotate("T (target)", xy=ee, xytext=(ee[0] + 4, ee[1] + 2),
                fontsize=11, color="tab:red")
    ax.text((mL[0] + eL[0]) / 2 - 2, (mL[1] + eL[1]) / 2 - 2, "L1",
            color="tab:blue", ha="right", fontsize=12)
    ax.text((eL[0] + ee[0]) / 2 + 1, (eL[1] + ee[1]) / 2 + 3, "L2",
            color="tab:orange", fontsize=12)
    ax.set_title(f"Left arm reaching target ({x}, {y}) — a 2R serial chain")
    plt.show()


def plot_triangle(x=15, y=50):
    th1, th2 = fbc.solve_ik(x, y)
    mL, eL, ee, _, _ = fbc.forward_kinematics(th1, th2)

    phi = math.atan2(y - mL[1], x - mL[0])
    r = math.hypot(x - mL[0], y - mL[1])
    alpha = math.acos((fbc.L1**2 + r**2 - fbc.L2**2) / (2 * fbc.L1 * r))

    fig, ax = plt.subplots(figsize=(7, 5))
    setup_ax(ax)

    ax.plot([mL[0], eL[0]], [mL[1], eL[1]], color="tab:blue", lw=4)
    ax.plot([eL[0], ee[0]], [eL[1], ee[1]], color="tab:orange", lw=2.5)
    ax.plot([mL[0], ee[0]], [mL[1], ee[1]], color="gray", lw=1.5, ls="--")
    ax.plot([mL[0], eL[0]], [mL[1], eL[1]], "o", color="black", markersize=7)
    ax.plot([ee[0]], [ee[1]], "o", color="tab:red", markersize=10)

    ax.annotate("M", xy=mL, xytext=(mL[0] - 4, mL[1] - 5), fontsize=14, fontweight="bold")
    ax.annotate("E", xy=eL, xytext=(eL[0] - 5, eL[1] + 2), fontsize=14, fontweight="bold")
    ax.annotate("T", xy=ee, xytext=(ee[0] + 3, ee[1] + 2), fontsize=14, fontweight="bold",
                color="tab:red")
    ax.text((mL[0] + eL[0]) / 2 - 2, (mL[1] + eL[1]) / 2 - 1, "L1",
            color="tab:blue", ha="right", fontsize=12)
    ax.text((eL[0] + ee[0]) / 2 + 2, (eL[1] + ee[1]) / 2 + 3, "L2",
            color="tab:orange", fontsize=12)
    ax.text((mL[0] + ee[0]) / 2 + 2, (mL[1] + ee[1]) / 2 - 4, "r",
            color="gray", fontsize=12)

    arc_r = 13
    arc = Arc((mL[0], mL[1]), 2 * arc_r, 2 * arc_r,
              angle=0,
              theta1=math.degrees(phi), theta2=math.degrees(phi + alpha),
              color="tab:purple", lw=2)
    ax.add_patch(arc)
    mid = phi + alpha / 2
    ax.text(mL[0] + (arc_r + 3) * math.cos(mid),
            mL[1] + (arc_r + 3) * math.sin(mid),
            r"$\alpha$", color="tab:purple", fontsize=14, fontweight="bold")
    phi_mid = phi / 2
    ax.text(mL[0] + 6 * math.cos(phi_mid), mL[1] + 6 * math.sin(phi_mid) - 2,
            r"$\varphi$", color="gray", fontsize=13)

    ax.set_title(r"Law of cosines gives $\alpha$; $\theta = \varphi + \alpha$")
    plt.show()


def plot_two_branches(x=15, y=50):
    mL = (-fbc.BASE_D / 2.0, 0.0)
    dx, dy = x - mL[0], y - mL[1]
    r = math.hypot(dx, dy)
    phi = math.atan2(dy, dx)
    alpha = math.acos((fbc.L1**2 + r**2 - fbc.L2**2) / (2 * fbc.L1 * r))

    elbow_plus = (mL[0] + fbc.L1 * math.cos(phi + alpha),
                  mL[1] + fbc.L1 * math.sin(phi + alpha))
    elbow_minus = (mL[0] + fbc.L1 * math.cos(phi - alpha),
                   mL[1] + fbc.L1 * math.sin(phi - alpha))

    fig, ax = plt.subplots(figsize=(7, 5))
    setup_ax(ax)

    ax.plot([mL[0], elbow_plus[0]], [mL[1], elbow_plus[1]], color="tab:blue", lw=4,
            label=r"$+\alpha$ (elbows-out)")
    ax.plot([elbow_plus[0], x], [elbow_plus[1], y], color="tab:orange", lw=2.5)

    ax.plot([mL[0], elbow_minus[0]], [mL[1], elbow_minus[1]], color="tab:blue",
            lw=2, ls="--", alpha=0.45, label=r"$-\alpha$ (the other branch)")
    ax.plot([elbow_minus[0], x], [elbow_minus[1], y], color="tab:orange",
            lw=1.5, ls="--", alpha=0.45)

    ax.plot([mL[0]], [mL[1]], "o", color="black", markersize=8)
    ax.plot([elbow_plus[0]], [elbow_plus[1]], "o", color="black", markersize=7)
    ax.plot([elbow_minus[0]], [elbow_minus[1]], "o", color="gray", markersize=6, alpha=0.7)
    ax.plot([x], [y], "o", color="tab:red", markersize=10)

    ax.legend(loc="lower right")
    ax.set_title("Two elbow positions reach the same target")
    plt.show()


def plot_multi_target(targets=((0, 60), (20, 50), (-20, 50))):
    fig, axes = plt.subplots(1, len(targets), figsize=(4.5 * len(targets), 4.5))
    if len(targets) == 1:
        axes = [axes]
    for ax, (x, y) in zip(axes, targets):
        setup_ax(ax)
        th1, th2 = fbc.solve_ik(x, y)
        draw_linkage(ax, th1, th2, target=(x, y))
        ax.set_title(f"target ({x}, {y})")
    plt.tight_layout()
    plt.show()


def interactive_solver(x0=0, y0=60, x_range=(-40, 40), y_range=(30, 75)):
    fig_i, ax_i = plt.subplots(figsize=(7, 5))
    setup_ax(ax_i)

    base_line, = ax_i.plot([], [], color="gray", lw=2)
    pL, = ax_i.plot([], [], color="tab:blue", lw=4)
    pR, = ax_i.plot([], [], color="tab:blue", lw=4)
    dL, = ax_i.plot([], [], color="tab:orange", lw=2.5)
    dR, = ax_i.plot([], [], color="tab:orange", lw=2.5)
    joints, = ax_i.plot([], [], "o", color="black", markersize=6)
    ee_dot, = ax_i.plot([], [], "o", color="tab:red", markersize=9)
    target_x, = ax_i.plot([], [], "x", color="tab:red", markersize=12, mew=2)

    def update(x, y):
        target_x.set_data([x], [y])
        sol = fbc.solve_ik(x, y)
        if sol is None:
            for ln in (base_line, pL, pR, dL, dR, joints, ee_dot):
                ln.set_data([], [])
            ax_i.set_title(f"target ({x:.0f}, {y:.0f}) — OOR")
            fig_i.canvas.draw_idle()
            return
        th1, th2 = sol
        mL, eL, ee, eR, mR = fbc.forward_kinematics(th1, th2)
        base_line.set_data([mL[0], mR[0]], [mL[1], mR[1]])
        pL.set_data([mL[0], eL[0]], [mL[1], eL[1]])
        pR.set_data([mR[0], eR[0]], [mR[1], eR[1]])
        dL.set_data([eL[0], ee[0]], [eL[1], ee[1]])
        dR.set_data([eR[0], ee[0]], [eR[1], ee[1]])
        joints.set_data([mL[0], mR[0], eL[0], eR[0]], [mL[1], mR[1], eL[1], eR[1]])
        ee_dot.set_data([ee[0]], [ee[1]])
        clear = fbc.singularity_clearance(x, y)
        flag = "  (near singularity)" if clear < fbc.SINGULARITY_MARGIN_MM else ""
        ax_i.set_title(
            f"θ1 = {math.degrees(th1):.1f}°   "
            f"θ2 = {math.degrees(th2):.1f}°   "
            f"clearance = {clear:.1f} mm{flag}"
        )
        fig_i.canvas.draw_idle()

    return interact(
        update,
        x=FloatSlider(min=x_range[0], max=x_range[1], step=1, value=x0,
                      description="x (mm)"),
        y=FloatSlider(min=y_range[0], max=y_range[1], step=1, value=y0,
                      description="y (mm)"),
    )


def plot_workspace_map(nx=220, ny=180, x_lim=(-50, 50), y_lim=(0, 85)):
    xs = np.linspace(x_lim[0], x_lim[1], nx)
    ys = np.linspace(y_lim[0], y_lim[1], ny)
    grid = np.full((ny, nx), np.nan)
    for j, yy in enumerate(ys):
        for i, xx in enumerate(xs):
            c = fbc.singularity_clearance(xx, yy)
            if c is not None:
                grid[j, i] = c

    fig, ax = plt.subplots(figsize=(7, 5.5))
    ax.imshow(np.zeros((ny, nx)), extent=[xs[0], xs[-1], ys[0], ys[-1]],
              origin="lower", cmap="Greys", vmin=0, vmax=1, alpha=0.25)
    im = ax.imshow(grid, extent=[xs[0], xs[-1], ys[0], ys[-1]],
                   origin="lower", cmap="viridis", vmin=0, vmax=30)

    mask = (grid < fbc.SINGULARITY_MARGIN_MM) & ~np.isnan(grid)
    red = np.zeros((ny, nx, 4))
    red[mask] = [1.0, 0.2, 0.2, 0.55]
    ax.imshow(red, extent=[xs[0], xs[-1], ys[0], ys[-1]], origin="lower")

    ax.plot([0], [fbc._HOME_XY[1]], marker="*", color="white", markersize=15,
            markeredgecolor="black", markeredgewidth=1.2, linestyle="None",
            label="home")
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("Parallel-singularity clearance  (red = within margin, grey = OOR)")
    ax.legend(loc="upper right")
    fig.colorbar(im, ax=ax, label="clearance (mm)")
    plt.show()
