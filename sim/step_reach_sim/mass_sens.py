"""Flat-floor and wall step limits under alternative mass assumptions."""
import numpy as np
from multiprocessing import Pool
from step_reach import *

RS = np.round(np.arange(0.12, 0.861, 0.02), 3)
M = None
def models(m):
    base = dict(m.mass["datasheet_motors"])
    b = dict(base); b["LiPo_3200mAh_batt_3"] = b["LiPo_3200mAh_batt_4"] = 96.6 * 1.81e-3   # same density as the 5000 mAh model
    c = dict(b); c["SlipRing_1_stator"] = c["SlipRing_2_stator"] = 286.6 * 2.70e-3       # aluminium instead of steel
    return {"A datasheet motors (used so far)": base, "B + 3200 mAh packs at LiPo density": b, "C + slip-ring bases as aluminium": c}
def init():
    global M
    M = StepModel(EE_B); M.mass = models(M)
def cell(ir):
    r = RS[ir]
    out = {}
    for x in batch_solve(M, r, 0.0, np.random.default_rng(ir).uniform(-np.pi, np.pi, (300, 5))):
        X = M.link_frames(x)
        if M.self_collision(X) or M.terrain_collision(X, M.terrains(r, 0.0)["edge_at_target"]):
            continue
        for gname, g in [("floor", G), ("wall_up", np.array([-9.81, 0, 0])), ("wall_side", np.array([0, -9.81, 0]))]:
            tq = M.torques(X, x, g)
            for k, t in tq.items():
                ra, pk = M.ratios(t)
                key = (gname, k)
                o = out.setdefault(key, [np.inf, np.inf]); o[0] = min(o[0], ra); o[1] = min(o[1], pk)
    return ir, out
if __name__ == "__main__":
    with Pool(4, initializer=init) as p:
        res = dict(p.imap_unordered(cell, range(len(RS))))
    m0 = StepModel(EE_B); mm = models(m0)
    for k, M_ in mm.items():
        print(f"{k:40s} total {sum(M_.values()):5.1f} kg, batteries {sum(v for n, v in M_.items() if 'LiPo' in n):.2f} kg")
    for gname in ["floor", "wall_up", "wall_side"]:
        for k in mm:
            rr = [RS[i] for i in res if (gname, k) in res[i] and res[i][(gname, k)][0] <= 1]
            rp = [RS[i] for i in res if (gname, k) in res[i] and res[i][(gname, k)][1] <= 1]
            print(f"{gname:9s} {k:40s} max step rated {max(rr) if rr else '-'}  peak {max(rp) if rp else '-'}")
