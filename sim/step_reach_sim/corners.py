"""Surface transitions. Stance gripper on the floor at the origin.
inside : wall at x = d (concave corner), target dock on the wall at height z, pointing +x.
outside: floor ends at an edge x = e, target dock on the face below, depth z < 0, pointing -x.
The reverse move (wall -> floor / face -> top) uses the same poses with the other gripper as stance."""
import sys, numpy as np
from multiprocessing import Pool
from step_reach import *
from step_reach import _batch_inv, _batch_chain

KIND = sys.argv[1]
XS = np.round(np.arange(0.10, 1.201, 0.02), 3)
ZS = np.round(np.arange(0.06, 0.901, 0.02), 3) if KIND == "inside" else np.round(-np.arange(0.06, 0.901, 0.02), 3)
N = np.array([1.0, 0, 0]) if KIND == "inside" else np.array([-1.0, 0, 0])
E1, E2 = np.array([0, 1.0, 0]), np.array([0, 0, 1.0])
M = M2 = None

def init():
    global M, M2
    M, M2 = StepModel(EE_B), StepModel(EE_A)

def residual(X6, p_t):
    Tw = np.eye(4); Tw[:3, :3] = R_DOWN
    T = Tw @ _batch_inv(_batch_chain(M.ch_st.ops, X6[:, :5])) @ _batch_chain(M.ch_sw.ops, X6[:, :5])
    c, s = np.cos(X6[:, 5]), np.sin(X6[:, 5])
    Rz_ = np.zeros((len(X6), 3, 3)); Rz_[:, 0, 0] = c; Rz_[:, 0, 1] = -s; Rz_[:, 1, 0] = s; Rz_[:, 1, 1] = c; Rz_[:, 2, 2] = 1
    p = np.einsum("bij,bj->bi", Rz_, T[:, :3, 3]); z = np.einsum("bij,bj->bi", Rz_, T[:, :3, 2])
    return np.concatenate([p - p_t, (z @ E1)[:, None], (z @ E2)[:, None]], 1), z @ N

def solve(p_t, seeds, iters=100, tol=1e-4, eps=1e-6, lam=1e-3):
    X = seeds.copy(); n = len(X)
    for _ in range(iters):
        P = np.concatenate([X] + [X + eps * np.eye(6)[k] for k in range(6)])
        F, _ = residual(P, p_t); f0 = F[:n]
        J = np.stack([(F[(k + 1) * n:(k + 2) * n] - f0) / eps for k in range(6)], 2)
        A = J @ np.swapaxes(J, 1, 2) + lam ** 2 * np.eye(5)
        dx = -np.einsum("bji,bj->bi", J, np.linalg.solve(A, f0[..., None])[..., 0])
        X += dx * np.minimum(1, 0.5 / np.maximum(np.linalg.norm(dx, axis=1, keepdims=True), 1e-12))
    F, nz = residual(X, p_t)
    ok = (np.linalg.norm(F, axis=1) < tol) & (nz > 0.99)
    sols = []
    for x in X[ok]:
        x = (x + np.pi) % (2 * np.pi) - np.pi
        if all(np.linalg.norm((x - y + np.pi) % (2 * np.pi) - np.pi) > 0.05 for y in sols):
            sols.append(x)
    return sols

def row(ix):
    xw = XS[ix]; rng = np.random.default_rng(ix)
    terrain = [(-10.0, xw, 0.0), (xw, 10.0, 10.0)] if KIND == "inside" else [(-10.0, xw, 0.0)]
    out = np.full((len(ZS), 6), np.nan)   # kin, feasible, fwd rated, fwd peak, rev rated, rev peak
    for iz, z in enumerate(ZS):
        sols = solve(np.array([xw, 0, z]), np.column_stack([rng.uniform(-np.pi, np.pi, (96, 6))]))
        out[iz, 0] = bool(sols); out[iz, 1] = 0
        best = [np.inf] * 4
        for x in sols:
            q = M.qdict(x[:5]); Tb = Rz(x[5]) @ M.world_base(q)
            X = M.arm.fk(q, Tb)
            if M.self_collision(X) or M.terrain_collision(X, terrain):
                continue
            out[iz, 1] = 1
            f = M.ratios(M.torques(X, x, models=["datasheet_motors"])["datasheet_motors"])
            r = M2.ratios(M2.torques(X, x, models=["datasheet_motors"])["datasheet_motors"])
            best = [min(best[0], f[0]), min(best[1], f[1]), min(best[2], r[0]), min(best[3], r[1])]
        out[iz, 2:] = best
    return ix, out

if __name__ == "__main__":
    with Pool(4, initializer=init) as p:
        res = dict(p.imap_unordered(row, range(len(XS))))
    A = np.stack([res[i] for i in range(len(XS))], 1)   # (Z, X, 6)
    np.savez(f"results/corner_{KIND}.npz", XS=XS, ZS=ZS, A=A)
    feas = A[..., 1] == 1
    lab = ["collision-free", "fwd<=rated", "fwd<=peak", "rev<=rated", "rev<=peak"]
    masks = [feas, feas & (A[..., 2] <= 1), feas & (A[..., 3] <= 1), feas & (A[..., 4] <= 1), feas & (A[..., 5] <= 1)]
    print(KIND, "kinematic cells", int((A[..., 0] == 1).sum()), "collision-free", int(feas.sum()))
    for l, m in zip(lab, masks):
        if not m.any(): print(f"  {l:15s} none"); continue
        zi, xi = np.where(m)
        print(f"  {l:15s} wall/edge dist {XS[xi].min():.2f}-{XS[xi].max():.2f} m, target height {ZS[zi].min():+.2f}..{ZS[zi].max():+.2f} m")
        for zz in ([0.1, 0.2, 0.3, 0.4, 0.5, 0.6] if KIND == "inside" else [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6]):
            iz = int(np.argmin(abs(ZS - zz)))
            xs = XS[m[iz]]
            print(f"      z={zz:+.1f}: dist " + (f"{xs.min():.2f}-{xs.max():.2f}" if len(xs) else "none"))
