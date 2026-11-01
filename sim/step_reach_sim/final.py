import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from plot import *

d = load(EE_B); RS, HS = d["RS"], d["HS"]
dr, dh = RS[1] - RS[0], HS[1] - HS[0]
ext = [RS[0] - dr / 2, RS[-1] + dr / 2, HS[0] - dh / 2, HS[-1] + dh / 2]
LAY = [("Geometric reach, both grippers vertical", lambda s: d["kin"], "#dbe8f8"),
       ("+ no self-collision or terrain collision (also within peak torque)", lambda s: d["feas"][s], "#7fb0ea"),
       ("+ within rated (continuous) torque", lambda s: d["feas"][s] & (d["rated"][s, 1] <= 1), "#1d5aa8")]

# Fig 1: maps
fig, axes = plt.subplots(1, 2, figsize=(10.5, 6.2), sharey=True, facecolor=SURF)
for s, ax in enumerate(axes):
    style(ax)
    for lab, fn, c in LAY:
        ax.imshow(np.where(fn(s), 1.0, np.nan), origin="lower", cmap=matplotlib.colors.ListedColormap([c]),
                  extent=ext, aspect="equal", interpolation="nearest")
        if s == 0: ax.plot([], [], "s", color=c, ms=9, label=lab)
    ax.axhline(0, color=TXT2, lw=0.8)
    ax.set_title(["Step onto/off a ledge whose edge is at the TARGET foot", "...whose edge is at the STANCE foot"][s], color=TXT, fontsize=9.5, loc="left")
    ax.set_xlabel("Step length r: horizontal distance between dock centres (m)", color=TXT2, fontsize=9)
    ax.set_xlim(0.1, 0.92); ax.set_ylim(-0.92, 0.92)
axes[0].set_ylabel("Step height h: target dock above stance dock (m)", color=TXT2, fontsize=9)
fig.legend(loc="lower center", ncol=2, fontsize=8, frameon=False, labelcolor=TXT)
fig.suptitle("Where the free gripper can dock in one step (B-end anchored; A-end is the same within 2 cm)", color=TXT, fontsize=11, x=0.05, ha="left")
fig.tight_layout(rect=(0, 0.08, 1, 1))
fig.savefig("out/fig1_foothold_map.png", dpi=130); plt.close(fig)

# Fig 2: envelope
best = d["feas"][0] | d["feas"][1]
curves = [("Geometric reach (no collisions checked)", d["kin"], CAT[0], (0, (4, 3))),
          ("Collision-free, best ledge layout (= within peak torque)", best, CAT[0], "-"),
          ("+ within rated torque", (d["feas"] & (d["rated"][:, 1] <= 1)).any(0), CAT[1], "-")]
fig, ax = plt.subplots(figsize=(7.6, 6.4), facecolor=SURF); style(ax)
env = {}
for lab, m, c, ls in curves:
    e = envelope(m, RS); env[lab] = e
    ax.plot(e, HS, linestyle=ls, color=c, lw=2, label=lab)
ax.axhline(0, color=TXT2, lw=0.8)
ax.set_xlabel("Max step length r (m)", color=TXT2, fontsize=9); ax.set_ylabel("Step height h (m)", color=TXT2, fontsize=9)
ax.set_xlim(0, 0.95); ax.set_ylim(-0.92, 0.92); ax.set_aspect("equal")
ax.legend(fontsize=8, frameon=False, labelcolor=TXT, loc="upper left", bbox_to_anchor=(1.0, 1.0))
ax.set_title("Max step length vs step height (14.35 kg arm)", color=TXT, fontsize=10, loc="left")
fig.tight_layout(); fig.savefig("out/fig2_envelope.png", dpi=130, bbox_inches="tight"); plt.close(fig)

print("h     geo   coll  rated")
for i, h in enumerate(HS):
    if round(h * 100) % 10 == 0 or abs(abs(h) - 0.66) < 1e-6:
        print(f"{h:+.2f} " + " ".join("  -  " if np.isnan(env[l][i]) else f"{env[l][i]:.2f} " for l, *_ in curves))

# Fig 3: poses
def pick(r, h, s=0):
    i, j = int(np.argmin(abs(HS - h))), int(np.argmin(abs(RS - r)))
    return HS[i], RS[j], d["qbest"][s, i, j]
cases = []
for title, r, h, s in [("Longest flat step 0.84 m", 0.84, 0.0, 0), ("Flat step at rated torque 0.58 m", 0.58, 0.0, 0),
                       ("Highest step up 0.66 m (r=0.40)", 0.40, 0.66, 0), ("Deepest step down 0.66 m (r=0.40)", 0.40, -0.66, 1)]:
    hh, rr, q = pick(r, h, s)
    if title.startswith("Flat step at rated"):
        # choose rated-feasible config: re-solve and keep min rated ratio
        m_ = StepModel(EE_B); best_ = None
        for x in batch_solve(m_, rr, hh, np.random.default_rng(0).uniform(-np.pi, np.pi, (96, 5))):
            X = m_.link_frames(x)
            if m_.self_collision(X) or m_.terrain_collision(X, m_.terrains(rr, hh)["edge_at_target"]): continue
            ra = m_.ratios(m_.torques(X, x)["datasheet_motors"])[0]
            if best_ is None or ra < best_[0]: best_ = (ra, x)
        q = best_[1]
    print(title, rr, hh, np.round(np.degrees(q)).tolist())
    cases.append((title, rr, hh, ["edge_at_target", "edge_at_stance"][s], q))
pose_plot(EE_B, cases)
