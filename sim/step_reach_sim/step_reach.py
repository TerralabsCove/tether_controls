"""Inchworm step-reach envelope for terra_arm_description/urdf/arm.urdf.

One gripper (stance) is docked on a horizontal surface, gripper axis vertical.
For every candidate foothold of the other gripper (swing) at horizontal distance r
and height h relative to the stance dock, we search for joint configurations that
put the swing dock there with its axis also vertical (pointing down), then check:
  1. mesh self-collision (link level, CAD-contact pairs disabled)
  2. terrain collision for two ledge layouts (edge next to target / next to stance)
  3. static gravity torque at each motor with the swing foot unsupported
"""
import sys, itertools
import numpy as np
import fcl, trimesh
from scipy.optimize import least_squares
from model import Arm, rpy2R, axis_angle, PKG

# dock_female frame in each EE link frame (from dock_female.urdf header; EE link frames
# are world-aligned at the CAD zero pose so the world-relative rpy is the local rpy)
DOCKS = {
    "EndEffectorAssembly_v3_1": ((0.000682593492, -0.00219814401, 0.0523494243),
                                 (-0.000658808122, 0.0439345587, -1.28470452)),
    "EndEffectorAssembly_v3_2": ((-2.09228589e-11, 1.37687084e-12, -0.0524),
                                 (3.14159265, 3.79208276e-10, -0.390706101)),
}
EE_A, EE_B = "EndEffectorAssembly_v3_1", "EndEffectorAssembly_v3_2"
ARM_JOINTS = ["Revolute_73", "Revolute_29", "Revolute_44", "Revolute_57", "Revolute_22"]
MOTOR = {"Revolute_44": "10010", "Revolute_22": "10010", "Revolute_29": "10010",
         "Revolute_57": "4340", "Revolute_73": "4340",
         "SlipRing_1_spin": "4340", "SlipRing_2_spin": "4340"}  # gripper spin motors
SPEC = {"10010": dict(rated=40.0, peak=150.0, mass=1.485),   # DM-J10010-2EC manual
        "4340": dict(rated=9.0, peak=27.0, mass=0.375)}      # DM-J4340P-2EC manual
NO_TERRAIN = {"SlipRing_1_rotor", "SlipRing_2_rotor", EE_A, EE_B}  # feet, sit on the dock
TERRAIN_MARGIN = 0.005   # m clearance between arm body and terrain
LEDGE_A = 0.07           # m from a foothold centre to the ledge edge (dock radius 0.06 + 1 cm)
G = np.array([0, 0, -9.81])
R_DOWN = np.diag([1.0, -1.0, -1.0])  # dock +z (into the surface) points to world -z


class Chain:
    """Compressed transform chain from the root link to one link."""
    def __init__(self, arm, link, tail=np.eye(4)):
        path, cur = [], link
        byc = {j["child"]: j for j in arm.joints}
        while cur != arm.root:
            j = byc[cur]; path.append(j); cur = j["parent"]
        path.reverse()
        ops, acc = [], np.eye(4)
        for j in path:
            acc = acc @ j["T"]
            if j["type"] != "fixed":
                ops.append(("F", acc)); ops.append(("R", j["axis"] / np.linalg.norm(j["axis"]), j["name"]))
                acc = np.eye(4)
        ops.append(("F", acc @ tail))
        self.ops = ops

    def __call__(self, q):
        T = np.eye(4)
        for op in self.ops:
            T = T @ (op[1] if op[0] == "F" else axis_angle(op[1], q.get(op[2], 0.0)))
        return T


def inv(T):
    Ti = np.eye(4); Ti[:3, :3] = T[:3, :3].T; Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]; return Ti


def Rz(a):
    T = np.eye(4); c, s = np.cos(a), np.sin(a); T[:2, :2] = [[c, -s], [s, c]]; return T


class StepModel:
    def __init__(self, stance):
        self.arm = Arm()
        self.stance = stance
        self.swing = EE_A if stance == EE_B else EE_B
        self.dock = {k: np.eye(4) for k in DOCKS}
        for k, (xyz, rpy) in DOCKS.items():
            self.dock[k][:3, :3] = rpy2R(*rpy); self.dock[k][:3, 3] = xyz
        self.ch_st = Chain(self.arm, stance, self.dock[stance])
        self.ch_sw = Chain(self.arm, self.swing, self.dock[self.swing])
        self._init_collision()
        self._init_torque()

    # ---------- kinematics ----------
    def qdict(self, x):
        return dict(zip(ARM_JOINTS, x))

    def world_base(self, q):
        """Base transform with the stance dock at the origin, axis vertical."""
        Tw = np.eye(4); Tw[:3, :3] = R_DOWN
        return Tw @ inv(self.ch_st(q))

    def swing_dock(self, x):
        q = self.qdict(x)
        return self.world_base(q) @ self.ch_sw(q)

    def residual(self, x, r, h):
        T = self.swing_dock(x)
        p, z = T[:3, 3], T[:3, 2]
        return np.array([np.hypot(p[0], p[1]) - r, p[2] - h, z[0], z[1]])

    def solve(self, r, h, seeds, tol=1e-4):
        sols = []
        for s in seeds:
            res = least_squares(self.residual, s, args=(r, h), method="trf", xtol=1e-10,
                                ftol=1e-10, gtol=1e-10, max_nfev=200)
            if np.linalg.norm(res.fun) > tol:
                continue
            x = (res.x + np.pi) % (2 * np.pi) - np.pi
            if self.swing_dock(x)[2, 2] > -0.99:   # must point down, not up
                continue
            if all(np.linalg.norm((x - y + np.pi) % (2 * np.pi) - np.pi) > 0.05 for y in sols):
                sols.append(x)
        return sols

    def link_frames(self, x):
        """World frames of all links, yawed so the swing foot sits on +x."""
        q = self.qdict(x)
        Tb = self.world_base(q)
        p = (Tb @ self.ch_sw(q))[:3, 3]
        return self.arm.fk(q, Rz(-np.arctan2(p[1], p[0])) @ Tb)

    # ---------- collision ----------
    def _init_collision(self):
        arm = self.arm
        meshes = arm.load_meshes()
        # rigid bodies = links joined by fixed joints
        parent = {n: n for n in arm.links}
        def find(n):
            while parent[n] != n:
                n = parent[n]
            return n
        for j in arm.joints:
            if j["type"] == "fixed":
                parent[find(j["child"])] = find(j["parent"])
        self.body = {n: find(n) for n in arm.links}
        self.cl = {}
        for n, m in meshes.items():
            if m is None:
                continue
            bvh = fcl.BVHModel(); bvh.beginModel(len(m.vertices), len(m.faces))
            bvh.addSubModel(m.vertices, m.faces); bvh.endModel()
            self.cl[n] = dict(obj=fcl.CollisionObject(bvh, fcl.Transform()),
                              hull=np.asarray(m.convex_hull.vertices))
        X0 = arm.fk({})
        self._place(X0)
        names = sorted(self.cl)
        self.pairs = []
        self.disabled = []
        for a, b in itertools.combinations(names, 2):
            if self.body[a] == self.body[b]:
                continue
            if self._collide(a, b):
                self.disabled.append((a, b))  # touching in the CAD pose: treat as designed contact
            else:
                self.pairs.append((a, b))

    def _place(self, X):
        self.aabb = {}
        for n, c in self.cl.items():
            T = X[n]
            c["obj"].setTransform(fcl.Transform(T[:3, :3], T[:3, 3]))
            v = c["hull"] @ T[:3, :3].T + T[:3, 3]
            self.aabb[n] = (v.min(0), v.max(0))

    def _collide(self, a, b):
        return fcl.collide(self.cl[a]["obj"], self.cl[b]["obj"], fcl.CollisionRequest(), fcl.CollisionResult()) > 0

    def self_collision(self, X):
        self._place(X)
        for a, b in self.pairs:
            (amin, amax), (bmin, bmax) = self.aabb[a], self.aabb[b]
            if np.any(amax < bmin) or np.any(bmax < amin):
                continue
            if self._collide(a, b):
                return (a, b)
        return None

    def terrain_collision(self, X, boxes):
        """boxes: list of (xmin, xmax, ztop); solid below ztop. Call after self_collision (placed)."""
        for n in self.cl:
            if n in NO_TERRAIN:
                continue
            lo, hi = self.aabb[n]
            for xmin, xmax, ztop in boxes:
                if hi[0] < xmin or lo[0] > xmax or lo[2] > ztop + TERRAIN_MARGIN:
                    continue
                cx, hx = 0.5 * (xmin + xmax), 0.5 * (xmax - xmin)
                box = fcl.CollisionObject(fcl.Box(2 * hx, 20.0, 20.0),
                                          fcl.Transform(np.array([cx, 0.0, ztop + TERRAIN_MARGIN - 10.0])))
                if fcl.collide(self.cl[n]["obj"], box, fcl.CollisionRequest(), fcl.CollisionResult()):
                    return n
        return None

    @staticmethod
    def terrains(r, h):
        """Two ledge layouts. Flat floor when h == 0."""
        if abs(h) < 1e-9:
            flat = [(-10.0, 10.0, 0.0)]
            return {"edge_at_target": flat, "edge_at_stance": flat}
        xs_t, xs_s = r - LEDGE_A, LEDGE_A
        return {"edge_at_target": [(-10.0, xs_t, 0.0), (xs_t, 10.0, h)],
                "edge_at_stance": [(-10.0, xs_s, 0.0), (xs_s, 10.0, h)]}

    # ---------- torque ----------
    def _init_torque(self):
        arm = self.arm
        kids = {}
        for j in arm.joints:
            kids.setdefault(j["parent"], []).append(j["child"])
        def subtree(n):
            out, st = set(), [n]
            while st:
                c = st.pop(); out.add(c); st.extend(kids.get(c, []))
            return out
        self.jinfo = {}
        for j in arm.joints:
            if j["name"] not in MOTOR:
                continue
            sub = subtree(j["child"])
            side = sub if self.swing in sub else set(arm.links) - sub
            self.jinfo[j["name"]] = dict(j=j, side=sorted(side))
        self.mass = {"urdf": {n: L["mass"] for n, L in arm.links.items()}}
        corr = dict(self.mass["urdf"])
        for n in corr:
            if n.startswith("DM-J10010L"):
                corr[n] = SPEC["10010"]["mass"]
            elif n.startswith("DM-J4340P"):
                corr[n] = SPEC["4340"]["mass"]
        self.mass["datasheet_motors"] = corr

    def torques(self, X, x, g=G, models=None):
        """Static gravity torque (N m) at each arm motor, swing foot unsupported."""
        com = {n: X[n][:3, :3] @ L["com"] + X[n][:3, 3] for n, L in self.arm.links.items()}
        out = {}
        for model, M in self.mass.items():
            if models and model not in models:
                continue
            t = {}
            for name, ji in self.jinfo.items():
                j = ji["j"]
                Tj = X[j["parent"]] @ j["T"]
                a = Tj[:3, :3] @ (j["axis"] / np.linalg.norm(j["axis"]))
                p = Tj[:3, 3]
                mom = sum(M[n] * np.cross(com[n] - p, g) for n in ji["side"])
                t[name] = abs(a @ mom)
            out[model] = t
        return out

    @staticmethod
    def ratios(t):
        """max over motors of torque / rated and torque / peak."""
        rated = max(v / SPEC[MOTOR[k]]["rated"] for k, v in t.items())
        peak = max(v / SPEC[MOTOR[k]]["peak"] for k, v in t.items())
        return rated, peak


# ---------- batched IK (damped least squares, all seeds at once) ----------
def _batch_chain(ops, X):
    """X: (B, 5) in ARM_JOINTS order -> (B, 4, 4)."""
    B = X.shape[0]
    T = np.broadcast_to(np.eye(4), (B, 4, 4)).copy()
    for op in ops:
        if op[0] == "F":
            T = T @ op[1]
        else:
            a = op[1]
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            if op[2] not in ARM_JOINTS:
                continue  # slip rings held at 0
            q = X[:, ARM_JOINTS.index(op[2])]
            R = np.zeros((B, 4, 4)); R[:, 3, 3] = 1
            R[:, :3, :3] = np.eye(3) + np.sin(q)[:, None, None] * K + (1 - np.cos(q))[:, None, None] * (K @ K)
            T = T @ R
    return T


def _batch_inv(T):
    Ti = np.zeros_like(T); Rt = np.swapaxes(T[:, :3, :3], 1, 2)
    Ti[:, :3, :3] = Rt; Ti[:, :3, 3] = -np.einsum("bij,bj->bi", Rt, T[:, :3, 3]); Ti[:, 3, 3] = 1
    return Ti


def batch_residual(model, X, r, h):
    Tw = np.eye(4); Tw[:3, :3] = R_DOWN
    T = Tw @ _batch_inv(_batch_chain(model.ch_st.ops, X)) @ _batch_chain(model.ch_sw.ops, X)
    p, z = T[:, :3, 3], T[:, :3, 2]
    return np.stack([np.hypot(p[:, 0], p[:, 1]) - r, p[:, 2] - h, z[:, 0], z[:, 1]], 1), T[:, 2, 2]


def batch_solve(model, r, h, seeds, iters=80, tol=1e-4, eps=1e-6, lam=1e-3):
    X = np.array(seeds, float)
    n = len(X)
    for _ in range(iters):
        P = np.concatenate([X] + [X + eps * np.eye(5)[k] for k in range(5)])
        F, _ = batch_residual(model, P, r, h)
        f0 = F[:n]
        J = np.stack([(F[(k + 1) * n:(k + 2) * n] - f0) / eps for k in range(5)], 2)  # (n,4,5)
        A = J @ np.swapaxes(J, 1, 2) + lam ** 2 * np.eye(4)
        dx = -np.einsum("bji,bj->bi", J, np.linalg.solve(A, f0[..., None])[..., 0])
        nrm = np.linalg.norm(dx, axis=1, keepdims=True)
        X = X + dx * np.minimum(1, 0.5 / np.maximum(nrm, 1e-12))
        if np.all(np.linalg.norm(f0, axis=1) < tol * 0.1):
            break
    F, zz = batch_residual(model, X, r, h)
    ok = (np.linalg.norm(F, axis=1) < tol) & (zz < -0.99)
    sols = []
    for x in X[ok]:
        x = (x + np.pi) % (2 * np.pi) - np.pi
        if all(np.linalg.norm((x - y + np.pi) % (2 * np.pi) - np.pi) > 0.05 for y in sols):
            sols.append(x)
    return sols


def stance_wrench(self, X, g=G, model="datasheet_motors"):
    """Load the arm puts on the stance gripper, in the stance dock frame
    (+z into the surface). Returns normal force (N, + = pressing in, - = pulling out),
    shear (N), bending moment (N m), twist (N m)."""
    M = self.mass[model]
    Td = X[self.stance] @ self.dock[self.stance]
    p, R = Td[:3, 3], Td[:3, :3]
    F = sum(M[n] for n in self.arm.links) * g
    mom = np.zeros(3)
    for n, L in self.arm.links.items():
        c = X[n][:3, :3] @ L["com"] + X[n][:3, 3]
        mom += M[n] * np.cross(c - p, g)
    f, m = R.T @ F, R.T @ mom
    return f[2], np.hypot(f[0], f[1]), np.hypot(m[0], m[1]), abs(m[2])


StepModel.stance_wrench = stance_wrench
