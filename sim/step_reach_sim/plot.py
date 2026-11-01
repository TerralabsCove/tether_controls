"""Plots + summary table from results/<stance>.npz."""
import sys, json, numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from step_reach import *

OUT = "out"
TXT, TXT2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
SEQ = ["#dbe8f8", "#9cc2ee", "#4f91e0", "#1d5aa8"]   # one hue, light -> dark (nested regions)
CAT = ["#2a78d6", "#eb6834", "#1baf7a"]
NAMES = {EE_B: "B-end (EE_2) anchored", EE_A: "A-end (EE_1) anchored"}


def load(stance):
    d = np.load(f"results/{stance}.npz")
    return {k: d[k] for k in d.files}


def regions(d, s, k):
    """Nested masks (H x R): kinematic, collision-free, <= peak, <= rated."""
    return [d["kin"], d["feas"][s], d["feas"][s] & (d["peak"][s, k] <= 1), d["feas"][s] & (d["rated"][s, k] <= 1)]


def envelope(mask, RS):
    """max r per height row (nan where nothing feasible)."""
    out = np.full(mask.shape[0], np.nan)
    for i, row in enumerate(mask):
        if row.any():
            out[i] = RS[row].max()
    return out


def style(ax):
    ax.set_facecolor(SURF)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.tick_params(colors=TXT2, labelsize=8)
    ax.grid(color=GRID, lw=0.6)
    ax.set_axisbelow(True)


def map_plot(d, stance, k, kname):
    RS, HS = d["RS"], d["HS"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.6), sharey=True, facecolor=SURF)
    labels = ["Geometric reach (grippers vertical)", "+ no self / terrain collision",
              "+ static torque <= peak", "+ static torque <= rated (continuous)"]
    dr, dh = RS[1] - RS[0], HS[1] - HS[0]
    for s, ax in enumerate(axes):
        style(ax)
        for m, c, lab in zip(regions(d, s, k), SEQ, labels):
            ax.imshow(np.where(m, 1.0, np.nan), origin="lower", cmap=matplotlib.colors.ListedColormap([c]),
                      extent=[RS[0] - dr / 2, RS[-1] + dr / 2, HS[0] - dh / 2, HS[-1] + dh / 2],
                      aspect="equal", interpolation="nearest")
            ax.plot([], [], "s", color=c, ms=9, label=lab)
        ax.axhline(0, color=TXT2, lw=0.8)
        ax.set_title(["Ledge edge next to the target foot", "Ledge edge next to the stance foot"][s],
                     color=TXT, fontsize=10, loc="left")
        ax.set_xlabel("Step length r: horizontal distance between dock centres (m)", color=TXT2, fontsize=9)
        ax.set_xlim(0.1, 0.95)
    axes[0].set_ylabel("Step height h: target dock above stance dock (m)", color=TXT2, fontsize=9)
    axes[1].legend(loc="lower right", fontsize=8, frameon=False, labelcolor=TXT)
    fig.suptitle(f"Feasible footholds, {NAMES[stance]}, masses: {kname}", color=TXT, fontsize=11, x=0.06, ha="left")
    fig.tight_layout()
    fig.savefig(f"{OUT}/map_{stance[-1]}_{kname}.png", dpi=130)
    plt.close(fig)


def envelope_plot(d, stance):
    RS, HS = d["RS"], d["HS"]
    fig, ax = plt.subplots(figsize=(7.5, 5.4), facecolor=SURF)
    style(ax)
    best = lambda i: d["feas"][0][:, :] | d["feas"][1] if i is None else None
    curves = [("Geometric reach", d["kin"], CAT[0], "--"),
              ("Collision-free (best ledge layout)", d["feas"][0] | d["feas"][1], CAT[0], "-"),
              ("+ torque <= peak",
               (d["feas"] & (d["peak"][:, 1] <= 1)).any(0), CAT[1], "-"),
              ("+ torque <= rated",
               (d["feas"] & (d["rated"][:, 1] <= 1)).any(0), CAT[2], "-")]
    for lab, m, c, ls in curves:
        e = envelope(m, RS)
        ax.plot(e, HS, ls, color=c, lw=2, label=lab)
    ax.axhline(0, color=TXT2, lw=0.8)
    ax.set_xlabel("Max step length r (m)", color=TXT2, fontsize=9)
    ax.set_ylabel("Step height h (m)", color=TXT2, fontsize=9)
    ax.set_xlim(0, 0.95)
    ax.set_aspect("equal")
    ax.legend(fontsize=8, frameon=False, labelcolor=TXT, loc="lower left")
    ax.set_title(f"Max step length vs step height, {NAMES[stance]}", color=TXT, fontsize=10, loc="left")
    fig.tight_layout()
    fig.savefig(f"{OUT}/envelope_{stance[-1]}.png", dpi=130)
    plt.close(fig)


def pose_plot(stance, cases):
    """Side view (x-z) of the arm meshes for a few footholds."""
    m = StepModel(stance)
    meshes = m.arm.link_mesh
    fig, axes = plt.subplots(1, len(cases), figsize=(4.2 * len(cases), 4.8), facecolor=SURF)
    for ax, (title, r, h, scen, x) in zip(axes, cases):
        style(ax)
        X = m.link_frames(x)
        for n, mesh in meshes.items():
            if mesh is None:
                continue
            v = mesh.vertices @ X[n][:3, :3].T + X[n][:3, 3]
            tri = v[mesh.faces][:, :, [0, 2]]
            ax.add_collection(PolyCollection(tri[::3], facecolors="#9cc2ee" if "EndEff" not in n else "#eb6834",
                                             edgecolors="none", alpha=0.35))
        for xmin, xmax, zt in StepModel.terrains(r, h)[scen]:
            xmin, xmax = max(xmin, -0.6), min(xmax, 1.2)
            ax.fill_between([xmin, xmax], [zt, zt], [-1.2, -1.2], color="#d6d5d0", lw=0)
        ax.plot([0, r], [0, h], "o", color=TXT, ms=4)
        ax.set_xlim(-0.5, 1.1); ax.set_ylim(-1.0, 1.25); ax.set_aspect("equal")
        ax.set_title(title, color=TXT, fontsize=9, loc="left")
    fig.tight_layout()
    fig.savefig(f"{OUT}/poses_{stance[-1]}.png", dpi=130)
    plt.close(fig)


def summary(d):
    RS, HS = d["RS"], d["HS"]
    i0 = int(np.argmin(abs(HS)))
    res = {}
    for name, m in [("kin", d["kin"]), ("collision_free", d["feas"][0] | d["feas"][1]),
                    ("peak_urdf", (d["feas"] & (d["peak"][:, 0] <= 1)).any(0)),
                    ("peak_ds", (d["feas"] & (d["peak"][:, 1] <= 1)).any(0)),
                    ("rated_urdf", (d["feas"] & (d["rated"][:, 0] <= 1)).any(0)),
                    ("rated_ds", (d["feas"] & (d["rated"][:, 1] <= 1)).any(0))]:
        e = envelope(m, RS)
        rows = np.where(m.any(1))[0]
        res[name] = dict(flat_max_r=float(e[i0]) if not np.isnan(e[i0]) else None,
                         max_up=float(HS[rows.max()]) if len(rows) else None,
                         max_down=float(HS[rows.min()]) if len(rows) else None,
                         min_r_flat=float(RS[m[i0]].min()) if m[i0].any() else None,
                         table={f"{h:+.2f}": (None if np.isnan(e[i]) else float(e[i]))
                                for i, h in enumerate(HS) if abs(round(h * 100) % 10) == 0})
    return res


if __name__ == "__main__":
    import os
    os.makedirs(OUT, exist_ok=True)
    for stance in sys.argv[1:]:
        d = load(stance)
        for k, kname in enumerate(d["models"]):
            map_plot(d, stance, k, str(kname))
        envelope_plot(d, stance)
        json.dump(summary(d), open(f"{OUT}/summary_{stance[-1]}.json", "w"), indent=1)
        print(stance, json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "table"} for k, v in summary(d).items()}, indent=1))
