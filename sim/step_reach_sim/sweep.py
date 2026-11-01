"""Grid sweep over (r, h) footholds for one stance end. Writes results/<stance>.npz."""
import sys, os, numpy as np
from multiprocessing import Pool
from step_reach import *

RS = np.round(np.arange(0.10, 0.951, 0.02), 3)
HS = np.round(np.arange(-0.96, 0.961, 0.02), 3)
SCEN = ["edge_at_target", "edge_at_stance"]
MODELS = ["urdf", "datasheet_motors"]
M = None

def init(stance):
    global M
    M = StepModel(stance)

def row(ih):
    h = HS[ih]
    rng = np.random.default_rng(ih)
    nR = len(RS)
    out = dict(kin=np.zeros(nR, bool), selfok=np.zeros(nR, bool),
               feas=np.zeros((len(SCEN), nR), bool),
               rated=np.full((len(SCEN), len(MODELS), nR), np.inf),
               peak=np.full((len(SCEN), len(MODELS), nR), np.inf),
               qbest=np.full((len(SCEN), nR, 5), np.nan),
               tbest=np.full((len(SCEN), nR, 5), np.nan))
    warm = []
    for ir, r in enumerate(RS):
        seeds = np.concatenate([rng.uniform(-np.pi, np.pi, (48, 5))] + ([np.array(warm)] if warm else []))
        sols = batch_solve(M, r, h, seeds)
        out["kin"][ir] = bool(sols)
        nxt = []
        terr = M.terrains(r, h)
        for x in sols:
            X = M.link_frames(x)
            if M.self_collision(X):
                continue
            out["selfok"][ir] = True
            tq = None
            for s, name in enumerate(SCEN):
                if M.terrain_collision(X, terr[name]):
                    continue
                out["feas"][s, ir] = True
                tq = tq or M.torques(X, x)
                for k, mdl in enumerate(MODELS):
                    ra, pk = M.ratios(tq[mdl])
                    out["rated"][s, k, ir] = min(out["rated"][s, k, ir], ra)
                    if pk < out["peak"][s, k, ir]:
                        out["peak"][s, k, ir] = pk
                        if mdl == "datasheet_motors":
                            out["qbest"][s, ir] = x
                            out["tbest"][s, ir] = [tq[mdl][j] for j in ARM_JOINTS]
            nxt.append(x)
        warm = nxt[:12]
    return ih, out

if __name__ == "__main__":
    stance = sys.argv[1]
    os.makedirs("results", exist_ok=True)
    with Pool(4, initializer=init, initargs=(stance,)) as pool:
        res = dict(pool.imap_unordered(row, range(len(HS))))
    agg = {k: np.stack([res[i][k] for i in range(len(HS))], -2 if k in ("kin", "selfok") else (1 if k in ("feas", "qbest", "tbest") else 2))
           for k in res[0]}
    np.savez(f"results/{stance}.npz", RS=RS, HS=HS, scen=SCEN, models=MODELS, **agg)
    print("done", stance, {k: v.shape for k, v in agg.items()})
