import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
sys.argv = ["x", "inside"]
import corners as C
from plot import style, TXT, TXT2, SURF
from step_reach import *

def best_pose(kind, xw, z):
    C.KIND = kind
    C.N = np.array([1.0, 0, 0]) if kind == "inside" else np.array([-1.0, 0, 0])
    terrain = [(-10.0, xw, 0.0), (xw, 10.0, 10.0)] if kind == "inside" else [(-10.0, xw, 0.0)]
    best = None
    for x in C.solve(np.array([xw, 0, z]), np.random.default_rng(0).uniform(-np.pi, np.pi, (192, 6))):
        q = C.M.qdict(x[:5]); X = C.M.arm.fk(q, Rz(x[5]) @ C.M.world_base(q))
        if C.M.self_collision(X) or C.M.terrain_collision(X, terrain): continue
        ra = C.M.ratios(C.M.torques(X, x, models=["datasheet_motors"])["datasheet_motors"])[0]
        if best is None or ra < best[0]: best = (ra, X)
    return best

C.init()
cases = [("Floor -> wall, wall 0.60 m away,\nwall dock 0.30 m up", "inside", 0.60, 0.30),
         ("Floor -> wall, farthest: wall 1.10 m away,\nwall dock 0.20 m up", "inside", 1.10, 0.20),
         ("Over an edge: edge 0.20 m away,\nface dock 0.30 m below the top", "outside", 0.20, -0.30)]
fig, axes = plt.subplots(1, 3, figsize=(13, 4.8), facecolor=SURF)
meshes = C.M.arm.link_mesh
for ax, (t, kind, xw, z) in zip(axes, cases):
    style(ax)
    ra, X = best_pose(kind, xw, z)
    for n, mesh in meshes.items():
        if mesh is None: continue
        v = mesh.vertices @ X[n][:3, :3].T + X[n][:3, 3]
        ax.add_collection(PolyCollection(v[mesh.faces][::3][:, :, [0, 2]], facecolors="#eb6834" if "EndEff" in n else "#9cc2ee",
                                         edgecolors="none", alpha=0.35))
    if kind == "inside":
        ax.fill_between([-0.6, xw], [0, 0], [-1, -1], color="#d6d5d0", lw=0); ax.fill_between([xw, 1.6], [1.3, 1.3], [-1, -1], color="#d6d5d0", lw=0)
    else:
        ax.fill_between([-0.6, xw], [0, 0], [-1, -1], color="#d6d5d0", lw=0)
    ax.plot([0, xw], [0, z], "o", color=TXT, ms=4)
    ax.set_xlim(-0.4, 1.4); ax.set_ylim(-0.8, 1.2); ax.set_aspect("equal")
    ax.set_title(t + f"\n(stance motor load {ra*100:.0f}% of rated)", color=TXT, fontsize=9, loc="left")
fig.tight_layout(); fig.savefig("out/fig5_corners.png", dpi=130, bbox_inches="tight")
