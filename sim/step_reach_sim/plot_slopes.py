import json, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from plot import style, TXT, TXT2, SURF, CAT
rows = json.load(open("out/slopes.json"))
A = sorted({w["alpha"] for w in rows})
get = lambda b, k: [next((w[k] or 0) for w in rows if w["alpha"] == a and w["beta"] == b) for a in A]
fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), sharey=True, facecolor=SURF)
for ax, k, t in zip(axes, ["max_r_rated", "max_r_peak"], ["Within rated (continuous) torque", "Within peak torque"]):
    style(ax)
    for (b, lab), c in zip([(0, "Step up-slope"), (90, "Step sideways (across the slope)"), (180, "Step down-slope")], CAT):
        ax.plot(A, get(b, k), "-o", color=c, lw=2, ms=4, label=lab)
    for x, s in [(0, "floor"), (90, "wall"), (180, "ceiling")]:
        ax.axvline(x, color=TXT2, lw=0.6); ax.text(x + 2, 0.02, s, color=TXT2, fontsize=8)
    ax.set_xticks(range(0, 181, 15)); ax.set_xlim(-3, 183); ax.set_ylim(0, 0.9)
    ax.set_xlabel("Surface angle from horizontal (deg)", color=TXT2, fontsize=9)
    ax.set_title(t, color=TXT, fontsize=10, loc="left")
axes[0].set_ylabel("Max step length along the surface (m)", color=TXT2, fontsize=9)
axes[1].legend(fontsize=8, frameon=False, labelcolor=TXT, loc="lower right")
fig.suptitle("Step length on inclines, walls and ceilings (14.35 kg arm; gripper hold strength NOT included)",
             color=TXT, fontsize=10.5, x=0.05, ha="left")
fig.tight_layout(); fig.savefig("out/fig4_slopes.png", dpi=130, bbox_inches="tight")
