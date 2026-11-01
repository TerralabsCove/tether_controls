"""Dock-to-dock step planner for the test videos.

A scenario is a set of scene boxes, a list of docks, a start pose (which dock each
gripper is in) and a list of steps (which gripper moves to which dock). Each step is
  lift   : swing dock straight out of its dock by LIFT along the surface normal
  swing  : joint-space move to an approach pose LIFT off the target dock (collision-checked,
           best of many IK solutions, random via points if the direct move collides)
  descend: straight into the target dock
Torques are quasi-static gravity torques (no dynamics)."""
import numpy as np, fcl
from step_reach import *
from step_reach import _batch_inv, _batch_chain

LIFT = 0.12
FOOT = {EE_A: {"SlipRing_1_rotor", EE_A}, EE_B: {"SlipRing_2_rotor", EE_B}}


def dock_pose(p, n, xhint=(1.0, 0, 0)):
    """Dock frame at p with +z = n (into the surface)."""
    z = np.asarray(n, float); z /= np.linalg.norm(z)
    x = np.asarray(xhint, float) - z * (z @ xhint)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0, 1.0, 0]) - z * z[1]
    x /= np.linalg.norm(x)
    T = np.eye(4); T[:3, :3] = np.column_stack([x, np.cross(z, x), z]); T[:3, 3] = p
    return T


class Box:
    def __init__(self, center, size, R=np.eye(3), color=(0.82, 0.81, 0.78, 1)):
        self.c, self.size, self.R, self.color = np.asarray(center, float), np.asarray(size, float), R, color
        self.obj = fcl.CollisionObject(fcl.Box(*self.size), fcl.Transform(R, self.c))
        corners = np.array(np.meshgrid(*[[-0.5, 0.5]] * 3)).reshape(3, -1).T * self.size
        v = corners @ R.T + self.c
        self.aabb = (v.min(0), v.max(0))


class Planner:
    def __init__(self, boxes, max_peak=None):
        self.boxes = boxes
        self.max_peak = max_peak   # reject swing paths whose static torque exceeds this x peak
        self.M = {EE_A: StepModel(EE_A), EE_B: StepModel(EE_B)}

    # ---------- kinematics with an arbitrary stance dock pose ----------
    def base(self, stance, x6, Tws):
        M = self.M[stance]
        return Tws @ Rz(x6[5]) @ inv(M.ch_st(M.qdict(x6[:5])))

    def frames(self, stance, x6, Tws):
        M = self.M[stance]
        return M.arm.fk(M.qdict(x6[:5]), self.base(stance, x6, Tws))

    def swap(self, stance, x6, Tws, new_stance, Tws_new):
        """Same physical pose, re-expressed with the other gripper as stance."""
        Tb = self.base(stance, x6, Tws)
        Mn = self.M[new_stance]
        R = inv(Tws_new) @ Tb @ Mn.ch_st(Mn.qdict(x6[:5]))
        y = x6.copy(); y[5] = np.arctan2(R[1, 0], R[0, 0])
        return y

    def _res(self, stance, X6, Tws, p, n):
        M = self.M[stance]
        T = _batch_chain(M.ch_st.ops, X6[:, :5])
        c, s = np.cos(X6[:, 5]), np.sin(X6[:, 5])
        Rz_ = np.zeros((len(X6), 4, 4)); Rz_[:, 0, 0] = c; Rz_[:, 0, 1] = -s; Rz_[:, 1, 0] = s; Rz_[:, 1, 1] = c
        Rz_[:, 2, 2] = Rz_[:, 3, 3] = 1
        T = Tws @ Rz_ @ _batch_inv(T) @ _batch_chain(M.ch_sw.ops, X6[:, :5])
        z = T[:, :3, 2]
        e1 = np.cross(n, [1.0, 0, 0]) if abs(n[0]) < 0.9 else np.cross(n, [0, 1.0, 0])
        e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
        return np.concatenate([T[:, :3, 3] - p, (z @ e1)[:, None], (z @ e2)[:, None]], 1), z @ n

    def ik(self, stance, Tws, p, n, seeds, iters=100, tol=1e-4, eps=1e-6, lam=1e-3):
        X = np.array(seeds, float).copy(); k = len(X)
        for _ in range(iters):
            P = np.concatenate([X] + [X + eps * np.eye(6)[i] for i in range(6)])
            F, _ = self._res(stance, P, Tws, p, n); f0 = F[:k]
            J = np.stack([(F[(i + 1) * k:(i + 2) * k] - f0) / eps for i in range(6)], 2)
            A = J @ np.swapaxes(J, 1, 2) + lam ** 2 * np.eye(5)
            dx = -np.einsum("bji,bj->bi", J, np.linalg.solve(A, f0[..., None])[..., 0])
            X += dx * np.minimum(1, 0.3 / np.maximum(np.linalg.norm(dx, axis=1, keepdims=True), 1e-12))
        F, nz = self._res(stance, X, Tws, p, n)
        ok = (np.linalg.norm(F, axis=1) < tol) & (nz > 0.99)
        return X[ok]

    # ---------- checks ----------
    def collides(self, stance, X, swing_foot_free):
        M = self.M[stance]
        if M.self_collision(X):
            return True
        skip = set(FOOT[stance]) | (set() if swing_foot_free else FOOT[M.swing])
        for n in M.cl:
            if n in skip:
                continue
            lo, hi = M.aabb[n]
            for b in self.boxes:
                if np.any(hi < b.aabb[0]) or np.any(lo > b.aabb[1]):
                    continue
                if fcl.collide(M.cl[n]["obj"], b.obj, fcl.CollisionRequest(), fcl.CollisionResult()):
                    return True
        return False

    def load(self, stance, X, x6):
        M = self.M[stance]
        t = M.torques(X, x6[:5], G, models=["datasheet_motors"])["datasheet_motors"]
        return t, M.stance_wrench(X, G)

    def peak_ratio(self, stance, X, x6):
        return self.M[stance].ratios(self.load(stance, X, x6)[0])[1]

    # ---------- motion pieces ----------
    @staticmethod
    def wrap_path(a, b, k):
        d = (b - a + np.pi) % (2 * np.pi) - np.pi
        s = np.linspace(0, 1, k)
        s = s * s * (3 - 2 * s)
        return [a + si * d for si in s]

    def cart_line(self, stance, Tws, x6, p0, p1, n, k):
        out, x = [], x6
        for p in np.linspace(p0, p1, k):
            sol = self.ik(stance, Tws, p, n, [x], iters=40)
            if len(sol) == 0:
                return None
            x = sol[0]; out.append(x)
        return out

    def segment_ok(self, stance, Tws, a, b, k=40):
        path = self.wrap_path(a, b, k)
        worst = 0
        for x in path[1:]:
            X = self.frames(stance, x, Tws)
            if self.collides(stance, X, swing_foot_free=True):
                return None, None
            worst = max(worst, self.peak_ratio(stance, X, x))
            if self.max_peak and worst > self.max_peak:
                return None, None
        return path, worst

    def step(self, stance, Tws, x6, dock_from, dock_to, rng, nseeds=160):
        """Returns list of x6 frames for one step, or raises."""
        pa, na = dock_from[:3, 3], dock_from[:3, 2]
        pb, nb = dock_to[:3, 3], dock_to[:3, 2]
        lift = self.cart_line(stance, Tws, x6, pa, pa - LIFT * na, na, 14)
        if lift is None:
            raise RuntimeError("lift failed")
        xa = lift[-1]
        seeds = np.concatenate([rng.uniform(-np.pi, np.pi, (nseeds, 6)), [xa]])
        cands = self.ik(stance, Tws, pb - LIFT * nb, nb, seeds)
        uniq = []
        for c in cands:
            if all(np.linalg.norm((c - u + np.pi) % (2 * np.pi) - np.pi) > 0.05 for u in uniq):
                uniq.append(c)
        scored = []
        for c in uniq:
            X = self.frames(stance, c, Tws)
            if self.collides(stance, X, swing_foot_free=True):
                continue
            travel = np.linalg.norm((c - xa + np.pi) % (2 * np.pi) - np.pi)
            scored.append((travel, c))
        scored.sort(key=lambda t: t[0])
        best = None
        # docked pose cost first (cheap), so a pose that can be held from either end within rated torque
        # is found even when it is not among the nearest approach poses
        pre = []
        for travel, c in scored[:80]:
            down = self.cart_line(stance, Tws, c, pb - LIFT * nb, pb, nb, 14)
            if down is None:
                continue
            held = self.M[stance].ratios(self.load(stance, self.frames(stance, down[-1], Tws), down[-1])[0])[0]
            other = self.M[stance].swing   # the next step may hold on with the other end
            xo = self.swap(stance, down[-1], Tws, other, dock_to)
            held = max(held, self.M[other].ratios(Planner.load(self, other, self.frames(other, xo, dock_to), xo)[0])[0])
            pre.append((0.08 * travel + 4 * max(0.0, held - 1), travel, c, down, held))
        pre.sort(key=lambda t: t[0])
        for _, travel, c, down, held in pre[:25]:
            path, worst = self.segment_ok(stance, Tws, xa, c)
            if path is not None:
                cost = worst + 0.08 * travel + 4 * max(0.0, held - 1)
                if best is None or cost < best[0]:
                    best = (cost, path, down)
        if best is None:   # via: arm sticking straight out of the stance surface (end-over-end flip)
            ps, ns = Tws[:3, 3], Tws[:3, 2]
            vias = self.ik(stance, Tws, ps - 1.25 * ns, -ns, rng.uniform(-np.pi, np.pi, (120, 6)))
            for travel, c in scored[:10]:
                down = self.cart_line(stance, Tws, c, pb - LIFT * nb, pb, nb, 14)
                if down is None:
                    continue
                for mid in vias:
                    if self.collides(stance, self.frames(stance, mid, Tws), True):
                        continue
                    p1, w1 = self.segment_ok(stance, Tws, xa, mid, 30)
                    if p1 is None:
                        continue
                    p2, w2 = self.segment_ok(stance, Tws, mid, c, 30)
                    if p2 is None:
                        continue
                    if best is None or max(w1, w2) < best[0]:
                        best = (max(w1, w2), p1 + p2[1:], down)
                if best:
                    break
        if best is None:   # random via points
            for travel, c in scored[:8]:
                down = self.cart_line(stance, Tws, c, pb - LIFT * nb, pb, nb, 14)
                if down is None:
                    continue
                for _ in range(400):
                    mid = 0.5 * (xa + c) + rng.normal(0, 0.8, 6)
                    if self.collides(stance, self.frames(stance, mid, Tws), True):
                        continue
                    p1, w1 = self.segment_ok(stance, Tws, xa, mid, 30)
                    if p1 is None:
                        continue
                    p2, w2 = self.segment_ok(stance, Tws, mid, c, 30)
                    if p2 is None:
                        continue
                    best = (max(w1, w2), p1 + p2[1:], down)
                    break
                if best:
                    break
        if best is None:
            raise RuntimeError("no collision-free swing found")
        return lift, best[1], best[2]
