"""Stepping along a flat surface tilted by alpha (0 floor, 90 wall, 180 ceiling).
Kinematics are the flat-floor ones in the surface frame; only gravity changes.
beta = step direction relative to up-slope (0 up, 90 sideways, 180 down)."""
import numpy as np, json, sys
from multiprocessing import Pool
from step_reach import *

RS = np.round(np.arange(0.12, 0.861, 0.02), 3)
ALPHA = list(range(0, 181, 15))
BETA = [0, 90, 180, 270]
MDL = "datasheet_motors"
M = None

def init():
    global M
    M = StepModel(EE_B)

def cell(ir):
    r = RS[ir]
    rng = np.random.default_rng(ir)
    sols = batch_solve(M, r, 0.0, rng.uniform(-np.pi, np.pi, (300, 5)))
    flat = M.terrains(r, 0.0)["edge_at_target"]
    out = {}
    for x in sols:
        X = M.link_frames(x)
        if M.self_collision(X) or M.terrain_collision(X, flat):
            continue
        for a in ALPHA:
            for b in BETA:
                ar, br = np.radians(a), np.radians(b)
                g = 9.81 * np.array([-np.sin(ar) * np.cos(br), -np.sin(ar) * np.sin(br), -np.cos(ar)])
                t = M.torques(X, x, g, models=[MDL])[MDL]
                ra, pk = M.ratios(t)
                key = (a, b)
                lim = max(t, key=lambda k: t[k] / SPEC[MOTOR[k]]["rated"])
                if key not in out or ra < out[key]["rated"]:
                    out[key] = dict(rated=ra, peak=pk, limiting=lim, wrench=M.stance_wrench(X, g),
                                    q=x.tolist())
                out[key]["best_peak"] = min(out[key].get("best_peak", np.inf), pk)
    return ir, out

if __name__ == "__main__":
    with Pool(4, initializer=init) as p:
        res = dict(p.imap_unordered(cell, range(len(RS))))
    rows = []
    for a in ALPHA:
        for b in BETA:
            rr = [RS[i] for i in range(len(RS)) if (a, b) in res[i] and res[i][(a, b)]["rated"] <= 1]
            rp = [RS[i] for i in range(len(RS)) if (a, b) in res[i] and res[i][(a, b)]["best_peak"] <= 1]
            def at(rv):
                if rv is None: return None
                i = int(np.argmin(abs(RS - rv))); c = res[i][(a, b)]
                return dict(r=float(rv), limiting=c["limiting"], rated_ratio=round(c["rated"], 2),
                            wrench=[round(float(w), 1) for w in c["wrench"]])
            rows.append(dict(alpha=a, beta=b, max_r_rated=max(rr) if rr else None,
                             max_r_peak=max(rp) if rp else None,
                             at_rated=at(max(rr) if rr else None), at_peak=at(max(rp) if rp else None),
                             at_030=at(0.30)))
    json.dump(rows, open("out/slopes.json", "w"), indent=1, default=float)
    print(f"{'alpha':>5} {'beta':>4} {'r_rated':>7} {'r_peak':>6}  limiting@rated  wrench@0.30m(N_in, shear, bend, twist)")
    for w in rows:
        print(f"{w['alpha']:>5} {w['beta']:>4} {str(w['max_r_rated']):>7} {str(w['max_r_peak']):>6}  "
              f"{(w['at_rated'] or {}).get('limiting','-'):14s} {w['at_030']['wrench'] if w['at_030'] else '-'}")
