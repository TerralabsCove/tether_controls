"""Mood clip for the Glade set designers: Tether as a curious creature in a nature set.

It walks in along a trail of floor docks through moss, stops to study a leaf, stacks a small cairn
from prop stones (each with a dock on top), steps back to look at it, and wanders off to look up at
the tree (6 ft 10 in, the Glade check-in deck's dimensions).

Usage: python glade_curious.py [--preview]
Writes videos/construction/glade_curious.mp4 (or a PNG contact sheet with --preview).

"Looking" is the free gripper hovering near something, leaning in, tilting and turning its spin motor.
Every pose is planned with the same collision checks as the other clips. Swings are held within rated
torque where the planner can find such a path; the actual worst ratio is printed and stated in the clip.
Stone sizes and masses are assumptions (lightweight props, not real rock)."""
import os, sys, pickle, subprocess, numpy as np
from construct import *          # CarryPlanner, block helpers, showcase drawing helpers

OUT = "videos/construction"
STRICT = 0.26                     # max static torque / peak on swings = within rated on every motor
TREE = np.array([1.60, 1.00])
POND = np.array([-0.55, 1.15])
D = {"D0": (-1.35, -0.35), "D1": (-0.90, -0.32), "D2": (-0.45, -0.25), "D3": (0.00, -0.15),
     "D4": (0.45, -0.02), "W": (0.90, 0.12), "D6": (0.50, 0.50), "D7": (0.78, 0.93)}
LEAF = np.array([-0.12, -0.55])
CAIRN = np.array([1.02, 0.45])
# stones: semi-axes (m), mass (kg), start xy, start yaw. Base stone is part of the set and does not move.
BASE = dict(semi=(0.19, 0.14, 0.06), xy=CAIRN, yaw=0.3)
STONES = [dict(semi=(0.15, 0.11, 0.055), kg=2.0, xy=(1.20, -0.10), yaw=0.9, to_yaw=-0.2),
          dict(semi=(0.12, 0.095, 0.05), kg=1.5, xy=(0.95, -0.25), yaw=-0.5, to_yaw=0.7),
          dict(semi=(0.09, 0.075, 0.045), kg=1.0, xy=(1.28, 0.24), yaw=0.2, to_yaw=0.0)]
STONE_RGBA = ["0.56 0.55 0.52 1", "0.62 0.60 0.56 1", "0.50 0.50 0.49 1"]


def stone_T(xy, z_center, yaw):
    T = np.eye(4); T[:3, :3] = Rz3(yaw); T[:3, 3] = (xy[0], xy[1], z_center)
    return T


def stone_box(T, semi):
    b = Box(T[:3, 3], 2 * np.asarray(semi), T[:3, :3]); b.mat = "plain"
    return b


def dockT(name):
    x, y = D[name]
    return dock_pose((x, y, 0.0), DOWN)


def scene():
    floor = floor_box(-25.0, 25.0, -25.0, 25.0)
    mesh, objs = trunk(tuple(TREE))
    v = mesh.vertices
    tree = Multi(objs, (v.min(0), v.max(0)))
    tree_solid = Solid(mesh, "0.43 0.34 0.26 1", objs)
    base_T = stone_T(BASE["xy"], BASE["semi"][2], BASE["yaw"])
    boulders = [((-1.25, 0.85), (0.30, 0.22, 0.20), 0.4), ((0.15, 1.45), (0.24, 0.20, 0.16), -0.3),
                ((2.3, 0.2), (0.28, 0.22, 0.22), 0.8)]
    bb = [stone_box(stone_T(xy, s[2], yaw), s) for xy, s, yaw in boulders]
    return dict(floor=floor, tree=tree, tree_solid=tree_solid, base_T=base_T, boulders=boulders, boulder_boxes=bb)


class Run:
    """Plans the clip as a sequence of moves; keeps state and frames."""

    def __init__(self, seed):
        self.S = scene()
        self.rng = np.random.default_rng(seed)
        self.stones = [stone_T(s["xy"], s["semi"][2], s["yaw"]) for s in STONES]
        self.P = CarryPlanner(self.static())
        self.frames, self.k, self.relaxed = [], 0, []
        self.top_z = 2 * BASE["semi"][2]

    def static(self, skip=None):
        S = self.S
        return [S["floor"], S["tree"], stone_box(S["base_T"], BASE["semi"])] + S["boulder_boxes"] + \
               [stone_box(T, STONES[i]["semi"]) for i, T in enumerate(self.stones) if i != skip]

    def snap(self, phase, x, carry=None, held=None, n=1, spin=0.0):
        stat = {i: T.copy() for i, T in enumerate(self.stones) if i != held}
        for _ in range(n):
            self.frames.append((self.stance, self.Tws, x, self.k, phase, carry, held, stat, spin))

    def start(self, b_dock, a_dock):
        P = self.P
        self.stance, self.Tws = EE_B, dockT(b_dock)
        self.at = {EE_B: dockT(b_dock), EE_A: dockT(a_dock)}
        tgt = self.at[EE_A]
        sols = P.ik(EE_B, self.Tws, tgt[:3, 3], tgt[:3, 2], self.rng.uniform(-np.pi, np.pi, (300, 6)))
        good = [(P.M[EE_B].ratios(P.load(EE_B, X, x)[0])[0], x) for x in sols
                for X in [P.frames(EE_B, x, self.Tws)] if not P.collides(EE_B, X, False)]
        self.x6 = min(good, key=lambda t: t[0])[1]
        self.snap("start", self.x6, n=24)

    def _step(self, mover, Ttgt):
        """P.step with the swing held within rated torque if possible."""
        new_st = EE_A if mover == EE_B else EE_B
        if new_st != self.stance:
            self.x6 = self.P.swap(self.stance, self.x6, self.Tws, new_st, self.at[new_st])
            self.stance, self.Tws = new_st, self.at[new_st]
        keep = self.P.max_peak
        for strict in (True, False):
            self.P.max_peak = STRICT if strict else None
            for _ in range(3):
                try:
                    out = self.P.step(self.stance, self.Tws, self.x6, self.at[mover], Ttgt, self.rng)
                    self.P.max_peak = keep
                    if not strict:
                        self.relaxed.append(self.k)
                    return out
                except RuntimeError:
                    pass
        self.P.max_peak = keep
        raise RuntimeError(f"move {self.k} failed")

    def move(self, mover, Ttgt, names=("lift", "swing", "lower"), carry=None, held=None, descend=True, label=""):
        lift, swing, down = self._step(mover, Ttgt)
        segs = (lift, swing, down) if descend else (lift, swing)
        for ph, seg in zip(names, segs):
            for x in seg:
                self.snap(ph, x, carry, held)
        self.x6 = segs[-1][-1]
        print(f"  {self.k:2d} {label}", flush=True)
        self.k += 1

    def walk(self, mover, name):
        self.P.boxes = self.static()
        self.move(mover, dockT(name), label=f"walk {mover[-1]} -> {name}")
        self.at[mover] = dockT(name)
        self.snap("hold", self.x6, n=8)

    def look(self, mover, target, n, dist, hold=40, sniff=0.0, wiggle=0.6, label="look"):
        """Free end hovers dist from target, looking along n; optional lean-in; spin wiggle while it holds."""
        n = np.asarray(n, float); n /= np.linalg.norm(n)
        p = np.asarray(target, float) - dist * n
        self.P.boxes = self.static()
        self.move(mover, dock_pose(p + LIFT * n, n), names=("lift", "look", "look"), descend=False, label=label)
        self.at[mover] = dock_pose(p, n)
        if sniff:
            M = self.P
            fwd = M.cart_line(self.stance, self.Tws, self.x6, p, p + sniff * n, n, 12)
            if fwd:
                for x in fwd:
                    self.snap("lean", x)
                self.snap("hold", fwd[-1], n=10)
                for x in fwd[::-1]:
                    self.snap("lean", x)
        if hold:
            ts = np.linspace(0, 1, hold)
            for t in ts:   # a slow head tilt: turn one way, the other way, back to centre
                s = wiggle * np.sin(2 * np.pi * t) * np.sin(np.pi * t)
                self.snap("hold", self.x6, spin=s)

    def carry_stone(self, i):
        P, st = self.P, STONES[i]
        W = np.array([D["W"][0], D["W"][1], 0.0])
        semi = np.asarray(st["semi"])
        T0 = self.stones[i]
        z_c = self.top_z + semi[2]
        Tgoal = stone_T(CAIRN, z_c, st["to_yaw"])
        c = dict(center=W, th0=np.arctan2(T0[1, 3] - W[1], T0[0, 3] - W[0]), th1=np.arctan2(CAIRN[1] - W[1], CAIRN[0] - W[0]),
                 yaw0=st["yaw"], dyaw=(st["to_yaw"] - st["yaw"] + np.pi / 2) % np.pi - np.pi / 2, size=2 * semi, mass=st["kg"])
        # reach for it (pick pose chosen counting the stone it is about to hold)
        P.carry, P.next_carry = None, c
        P.boxes = self.static()
        Tpick = dock_pose(T0[:3, 3] + (0, 0, semi[2]), DOWN)
        self.move(EE_B, Tpick, label=f"pick stone {i}")
        P.next_carry = None
        self.at[EE_B] = Tpick
        self.snap("hold", self.x6, n=10)
        # carry it to the cairn
        P.carry = c
        P.boxes = self.static(skip=i)
        Tplace = dock_pose((CAIRN[0], CAIRN[1], z_c + semi[2]), DOWN)
        self.move(EE_B, Tplace, names=("lift", "carry", "lower"), carry=c, held=i, label=f"place stone {i}")
        P.carry = None
        self.at[EE_B] = Tplace
        self.stones[i] = Tgoal
        self.top_z = z_c + semi[2]
        self.snap("hold", self.x6, n=14)


STATE = ("frames", "stance", "Tws", "x6", "at", "stones", "top_z", "k", "relaxed")
CKPT = "plans/construction/glade_ckpt.pkl"


def tree_look(R):
    d = np.array([D["D7"][0], D["D7"][1]]) - TREE; d /= np.linalg.norm(d)
    for h, gap in [(0.75, 0.20), (0.65, 0.20), (0.85, 0.22), (0.55, 0.18), (0.70, 0.26)]:
        bark = np.array([*(TREE + trunk_radius(h) * d), h])
        n = np.array([-d[0], -d[1], 0.35])
        try:
            R.look(EE_B, bark, n, gap, hold=70, wiggle=0.9, label=f"look up at the tree (h={h})")
            return
        except RuntimeError as e:
            print("   tree look failed at", h, e, flush=True)
    raise RuntimeError("no tree look")


LEAF3 = np.array([LEAF[0], LEAF[1], 0.0])
ACTIONS = [
    lambda R: R.walk(EE_B, "D2"),
    lambda R: R.walk(EE_A, "D3"),
    # a leaf by the trail: B is the free end, A holds D3
    lambda R: R.look(EE_B, LEAF3, (0.25, 0.05, -1.0), 0.17, hold=45, sniff=0.07, label="look at leaf"),
    lambda R: R.look(EE_B, LEAF3, (-0.35, -0.30, -1.0), 0.14, hold=50, wiggle=0.8, label="look again, other side"),
    lambda R: R.walk(EE_B, "D4"),
    lambda R: R.walk(EE_A, "W"),
    # the stones
    lambda R: R.carry_stone(0),
    lambda R: R.carry_stone(1),
    lambda R: R.look(EE_B, np.array([CAIRN[0], CAIRN[1], R.top_z + 0.02]), (0.55, 0.85, -0.7), 0.22, hold=45, label="look at the cairn"),
    lambda R: R.carry_stone(2),
    # step back and look at it from the trail side, so the way to the tree does not cross the cairn
    lambda R: R.walk(EE_B, "D6"),
    lambda R: R.look(EE_A, np.array([CAIRN[0], CAIRN[1], R.top_z]), (1.0, -0.1, -0.35), 0.24, hold=55, wiggle=0.7, label="step back, look at the cairn"),
    lambda R: R.walk(EE_A, "D7"),
    # wander off: look up at the tree
    tree_look,
]


def plan(seed=0):
    """Runs ACTIONS in order, checkpointing after each; a failed action is retried with a fresh rng."""
    R = Run(seed)
    def save(st):
        d = dict(stage=st, **{a: getattr(R, a) for a in STATE})
        pickle.dump(d, open(CKPT, "wb")); pickle.dump(d, open(CKPT.replace(".pkl", f"_{st:02d}.pkl"), "wb"))

    def restore():
        ck = pickle.load(open(CKPT, "rb"))
        for a in STATE:
            setattr(R, a, ck[a])
        R.P.carry = R.P.next_carry = None
        R.P.max_peak = None
        return ck["stage"]
    if os.path.exists(CKPT):
        stage = restore(); print("  resumed at stage", stage, flush=True)
    else:
        R.start("D0", "D1"); stage = 0; save(stage)
    while stage < len(ACTIONS):
        for attempt in range(5):
            try:
                ACTIONS[stage](R)
                break
            except RuntimeError as e:
                print(f"   stage {stage} attempt {attempt} failed: {e}", flush=True)
                restore()
                R.rng = np.random.default_rng([seed, stage, attempt + 1])
        else:
            raise RuntimeError(f"stage {stage} failed")
        stage += 1
        save(stage)
    R.snap("done", R.x6, n=24)
    return R


# ---------------------------------------------------------------- rendering
def build_glade(P, S, tmp):
    arm = P.M[EE_A].arm
    os.makedirs(tmp, exist_ok=True)

    def color(n):
        if "DM-J10010" in n: return "0.24 0.25 0.28 1"
        if "DM-J4340" in n: return "0.34 0.35 0.38 1"
        if "LiPo" in n: return "0.30 0.40 0.58 1"
        if "SlipRing" in n: return "0.55 0.56 0.60 1"
        if "EndEffector" in n: return "0.93 0.48 0.22 1"
        return "0.82 0.83 0.85 1"
    teal = f"{TEAL[0]/255:.3f} {TEAL[1]/255:.3f} {TEAL[2]/255:.3f} 1"
    rng = np.random.default_rng(7)
    assets, bodies, world, names = [], [], [], []
    for i, (n, m) in enumerate(arm.link_mesh.items()):
        if m is None: continue
        fn = f"{tmp}/l{i}.stl"; m.export(fn)
        assets.append(f'<mesh name="m{i}" file="{fn}" inertia="shell"/>')
        bodies.append(f'<body name="b{i}" mocap="true"><geom type="mesh" mesh="m{i}" rgba="{color(n)}" contype="0" conaffinity="0"/></body>')
        names.append(n)
    dm = Arm(PKG + "/urdf/dock_female.urdf").load_meshes()["dock_female"]
    dm.export(f"{tmp}/dock.stl")
    assets.append(f'<mesh name="dock" file="{tmp}/dock.stl" inertia="shell"/>')
    S["tree_solid"].mesh.export(f"{tmp}/tree.stl")
    assets.append(f'<mesh name="tree" file="{tmp}/tree.stl" inertia="shell"/>')
    g = lambda s: world.append(s)
    q2s = lambda q: " ".join(f"{v:.5f}" for v in q)

    def quat_yaw(a):
        return (np.cos(a / 2), 0, 0, np.sin(a / 2))
    # ground: moss
    b = S["floor"]
    g(f'<geom type="box" size="{" ".join(map(str, b.size / 2))}" pos="{" ".join(map(str, b.c))}" material="moss" contype="0" conaffinity="0"/>')
    for _ in range(170):   # moss patches
        x, y = rng.uniform(-6.0, 7.0), rng.uniform(-3.0, 9.0)
        r1, r2 = rng.uniform(0.15, 0.45), rng.uniform(0.12, 0.35)
        c = rng.choice(["0.36 0.46 0.25 1", "0.42 0.52 0.28 1", "0.33 0.42 0.24 1", "0.47 0.55 0.32 1"])
        g(f'<geom type="ellipsoid" size="{r1:.3f} {r2:.3f} 0.006" pos="{x:.3f} {y:.3f} 0.0" quat="{q2s(quat_yaw(rng.uniform(0, 3.14)))}" rgba="{c}" contype="0" conaffinity="0"/>')
    # pond and its rim
    g(f'<geom type="ellipsoid" size="0.75 0.48 0.004" pos="{POND[0]} {POND[1]} 0.004" material="water" contype="0" conaffinity="0"/>')
    for t in np.linspace(0, 2 * np.pi, 26, endpoint=False):
        x, y = POND[0] + 0.80 * np.cos(t), POND[1] + 0.53 * np.sin(t)
        s = rng.uniform(0.04, 0.08)
        g(f'<geom type="ellipsoid" size="{s:.3f} {s * 0.8:.3f} {s * 0.5:.3f}" pos="{x:.3f} {y:.3f} {s * 0.25:.3f}" quat="{q2s(quat_yaw(t))}" rgba="0.55 0.54 0.51 1" contype="0" conaffinity="0"/>')
    # tree, boulders, base stone
    g(f'<geom type="mesh" mesh="tree" material="bark" contype="0" conaffinity="0"/>')
    for (xy, s, yaw) in S["boulders"]:
        g(f'<geom type="ellipsoid" size="{s[0]} {s[1]} {s[2]}" pos="{xy[0]} {xy[1]} {s[2] * 0.85:.3f}" quat="{q2s(quat_yaw(yaw))}" rgba="0.52 0.51 0.48 1" contype="0" conaffinity="0"/>')
    bs, bT = BASE["semi"], S["base_T"]
    g(f'<geom type="ellipsoid" size="{bs[0]} {bs[1]} {bs[2]}" pos="{" ".join(map(str, bT[:3, 3]))}" quat="{q2s(quat_yaw(BASE["yaw"]))}" rgba="0.48 0.47 0.45 1" contype="0" conaffinity="0"/>')
    g(f'<geom type="mesh" mesh="dock" pos="{bT[0, 3]} {bT[1, 3]} {2 * bs[2] - 0.004}" quat="0 1 0 0" rgba="{teal}" contype="0" conaffinity="0"/>')
    # floor docks along the trail
    for name, (x, y) in D.items():
        g(f'<geom type="mesh" mesh="dock" pos="{x} {y} 0" quat="0 1 0 0" rgba="{teal}" contype="0" conaffinity="0"/>')
    # ferns: fans of fronds at the tree foot and by the pond
    for (fx, fy), k in [((2.05, 0.55), 9), ((1.1, 1.55), 8), ((-1.6, 0.35), 8), ((-0.05, 1.0), 7), ((2.1, 1.5), 9)]:
        for j in range(k):
            a = 2 * np.pi * j / k + rng.uniform(-0.2, 0.2)
            L, up = rng.uniform(0.30, 0.45), rng.uniform(0.18, 0.30)
            mid = (fx + 0.55 * L * np.cos(a), fy + 0.55 * L * np.sin(a), up * 0.85)
            tip = (fx + L * np.cos(a), fy + L * np.sin(a), up * 0.55)
            col = rng.choice(["0.30 0.50 0.22 1", "0.36 0.56 0.25 1", "0.26 0.44 0.20 1"])
            g(f'<geom type="capsule" fromto="{fx} {fy} 0.01 {mid[0]:.3f} {mid[1]:.3f} {mid[2]:.3f}" size="0.011" rgba="{col}" contype="0" conaffinity="0"/>')
            g(f'<geom type="capsule" fromto="{mid[0]:.3f} {mid[1]:.3f} {mid[2]:.3f} {tip[0]:.3f} {tip[1]:.3f} {tip[2]:.3f}" size="0.008" rgba="{col}" contype="0" conaffinity="0"/>')
    # fallen leaves, kept off the docks
    cols = ["0.80 0.55 0.20 1", "0.72 0.40 0.16 1", "0.62 0.66 0.26 1", "0.84 0.68 0.30 1", "0.55 0.30 0.14 1"]
    pts = np.array(list(D.values()))
    n_ok = 0
    while n_ok < 110:
        x, y = rng.uniform(-2.2, 2.9), rng.uniform(-1.5, 1.9)
        if np.min(np.hypot(pts[:, 0] - x, pts[:, 1] - y)) < 0.13 or np.hypot(x - POND[0], (y - POND[1]) * 1.5) < 0.85:
            continue
        if np.hypot(x - TREE[0], y - TREE[1]) < 0.5:
            continue
        s = rng.uniform(0.028, 0.045)
        g(f'<geom type="ellipsoid" size="{s:.3f} {s * 0.5:.3f} 0.0015" pos="{x:.3f} {y:.3f} 0.008" quat="{q2s(quat_yaw(rng.uniform(0, 6.28)))}" rgba="{rng.choice(cols)}" contype="0" conaffinity="0"/>')
        n_ok += 1
    # the leaf it studies
    g(f'<geom type="ellipsoid" size="0.075 0.036 0.002" pos="{LEAF[0]} {LEAF[1]} 0.010" quat="{q2s(quat_yaw(0.5))}" rgba="0.86 0.50 0.14 1" contype="0" conaffinity="0"/>')
    g(f'<geom type="capsule" fromto="{LEAF[0] - 0.07 * np.cos(0.5):.3f} {LEAF[1] - 0.07 * np.sin(0.5):.3f} 0.012 {LEAF[0] + 0.075 * np.cos(0.5):.3f} {LEAF[1] + 0.075 * np.sin(0.5):.3f} 0.012" size="0.0022" rgba="0.62 0.32 0.10 1" contype="0" conaffinity="0"/>')
    # movable stones, each with a dock on top
    for i, st in enumerate(STONES):
        a, b_, c = st["semi"]
        bodies.append(f'<body name="st{i}" mocap="true"><geom type="ellipsoid" size="{a} {b_} {c}" rgba="{STONE_RGBA[i]}" contype="0" conaffinity="0"/>'
                      f'<geom type="mesh" mesh="dock" pos="0 0 {c - 0.004}" quat="0 1 0 0" rgba="{teal}" contype="0" conaffinity="0"/></body>')
    xml = f"""<mujoco><compiler/><statistic extent="4" center="0.5 0.5 0.3"/><visual><global offwidth="{W}" offheight="{H}"/><quality shadowsize="4096"/>
      <headlight ambient="0.40 0.40 0.38" diffuse="0.38 0.38 0.36"/><map fogstart="1.1" fogend="3.2"/><rgba fog="0.91 0.925 0.89 1"/></visual>
      <asset><texture type="skybox" builtin="gradient" rgb1="0.96 0.96 0.93" rgb2="0.86 0.89 0.84" width="64" height="64"/>
      <texture name="mt" type="2d" builtin="flat" mark="random" random="0.25" rgb1="0.40 0.50 0.27" markrgb="0.31 0.41 0.22" width="512" height="512"/>
      <material name="moss" texture="mt" texrepeat="6 6" texuniform="true"/>
      <material name="bark" rgba="0.42 0.33 0.25 1"/>
      <material name="water" rgba="0.33 0.50 0.52 1" specular="0.6" shininess="0.8" reflectance="0.15"/>
      <material name="plain" rgba="0.55 0.54 0.51 1"/>{''.join(assets)}</asset>
      <worldbody><light directional="true" pos="1 -2 4" dir="-0.35 0.45 -1" diffuse="0.62 0.58 0.50" castshadow="true"/>
      {''.join(world)}{''.join(bodies)}</worldbody></mujoco>"""
    return mujoco.MjModel.from_xml_string(xml), names


SLOW = {"look": 3, "lean": 3}


def timeline(frames):
    out = []
    for i, fr in enumerate(frames):
        out.append(fr)
        nx = frames[i + 1] if i + 1 < len(frames) else fr
        if fr[4] in ("hold", "start", "done") or nx[3] != fr[3] or nx[4] != fr[4]:
            continue
        m = SLOW.get(fr[4], 2)
        for j in range(1, m):
            t = j / m
            out.append(fr[:2] + (fr[2] + t * wrapd(nx[2] - fr[2]),) + fr[3:8] + (fr[8] + t * (nx[8] - fr[8]),))
    return out


def overlay(idx, n, title_n, end_n, end_txt):
    img = Image.new("RGBA", (W, H), (0, 0, 0, 0)); d = ImageDraw.Draw(img)
    if idx < title_n:
        a = min(1, idx / 12, (title_n - idx) / 18)
        d.text((64, 70), "Tether in the Glade", font=font("SemiBold", 50), fill=INK + (int(255 * a),), anchor="lm")
        d.text((66, 122), "Concept simulation", font=font("Regular", 26), fill=INK2 + (int(255 * a),), anchor="lm")
    if idx >= n - end_n:
        a = min(1, (idx - (n - end_n)) / 15)
        d.rectangle((0, H - BAND, W, H), fill=(255, 255, 255, int(236 * a)))
        f = font("Medium", 30)
        lines, line = [], ""
        for w_ in end_txt.split():
            if d.textlength((line + " " + w_).strip(), font=f) > W - 112:
                lines.append(line); line = w_
            else:
                line = (line + " " + w_).strip()
        lines.append(line)
        y = H - BAND / 2 - (len(lines) - 1) * 19
        for ln in lines:
            d.text((56, y), ln, font=f, fill=INK + (int(255 * a),), anchor="lm"); y += 38
    else:
        d.text((W - 40, H - 28), "Concept simulation. Speed is not real.", font=font("Regular", 17), fill=INK2 + (200,), anchor="rs")
    return img


def main(preview=False):
    os.makedirs("plans/construction", exist_ok=True)
    cache = "plans/construction/glade_curious.pkl"
    S = scene()
    if os.path.exists(cache):
        frames, relaxed = pickle.load(open(cache, "rb"))
        P = CarryPlanner([S["floor"]])
    else:
        for seed in range(6):
            try:
                R = plan(seed); break
            except RuntimeError as e:
                print("  seed", seed, "failed:", e, flush=True)
        else:
            raise RuntimeError("planning failed for all seeds")
        frames, relaxed, P = R.frames, R.relaxed, R.P
        pickle.dump((frames, relaxed), open(cache, "wb"))
    # worst static torque over the whole clip, as a fraction of rated
    worst, wname = 0.0, None
    for fr in frames[::2]:
        stance, Tws, x6, k, ph, c = fr[:6]
        X = P.frames(stance, x6, Tws)
        t = P.carry_torques(stance, X, c) if c is not None else P.load(stance, X, x6)[0]
        for nm, v in t.items():
            r = v / SPEC[MOTOR[nm]]["rated"]
            if r > worst: worst, wname = r, (nm, k, ph)
    print(f"worst rated ratio over clip {worst:.2f} at {wname}; moves planned without the rated cap: {relaxed}")
    if worst <= 1:
        torque_txt = "Every move stays within the motors' rated torque."
    elif worst <= 1.15:
        torque_txt = f"Motors stay within rated torque, apart from a brief {round((worst - 1) * 100)}% over on one motor, well under peak."
    else:
        torque_txt = f"Motors reach up to {worst:.1f}x rated torque, under peak."
    end_txt = ("Concept simulation, not real speed. The stones are light props with a dock on top: the robot only grips docks. "
               f"{torque_txt} Tree 6 ft 10 in.")
    model, names = build_glade(P, S, os.path.abspath("mjtmp/glade"))
    arm = P.M[EE_A].arm
    tl = timeline(frames)
    title_n, end_n = int(3.5 * FPS), int(6.0 * FPS)
    tl = [tl[0]] * title_n + tl + [tl[-1]] * end_n
    # camera: drifts slowly and follows the robot, heavily smoothed
    mids = []
    for fr in tl:
        X = P.frames(fr[0], fr[2], fr[1])
        mids.append((X[EE_A][:3, 3] + X[EE_B][:3, 3]) / 2)
    mids = np.array(mids)
    ker = np.ones(151) / 151
    sm = np.stack([np.convolve(np.pad(mids[:, i], 75, mode="edge"), ker, mode="valid") for i in range(3)], 1)
    az = np.linspace(50, 76, len(tl))
    data = mujoco.MjData(model)
    rr = mujoco.Renderer(model, H, W)
    na = len(names)
    os.makedirs(OUT, exist_ok=True)
    out = f"{OUT}/glade_curious.mp4"
    proc = None
    if not preview:
        proc = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
                                 "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "19", "-movflags", "+faststart", out], stdin=subprocess.PIPE)
    sel = range(len(tl)) if not preview else np.linspace(title_n // 2, len(tl) - 1, 8).astype(int)
    sheet = []
    for idx in sel:
        stance, Tws, x6, k, phase, c, held, stat, spin = tl[idx]
        M = P.M[stance]
        q = M.qdict(x6[:5])
        q["SlipRing_1_spin" if M.swing == EE_A else "SlipRing_2_spin"] = spin
        X = arm.fk(q, P.base(stance, x6, Tws))
        for i, n in enumerate(names):
            qq = np.zeros(4); mujoco.mju_mat2Quat(qq, X[n][:3, :3].flatten())
            data.mocap_pos[i] = X[n][:3, 3]; data.mocap_quat[i] = qq
        Xs = P.frames(stance, x6, Tws)
        for i in range(len(STONES)):
            T = P.block_pose(stance, Xs, c) if i == held else stat[i]
            qq = np.zeros(4); mujoco.mju_mat2Quat(qq, np.ascontiguousarray(T[:3, :3]).flatten())
            data.mocap_pos[na + i] = T[:3, 3]; data.mocap_quat[na + i] = qq
        mujoco.mj_forward(model, data)
        cam = Cam(az[idx], -14, sm[idx] + (0.10, 0.20, 0.32), 2.4)
        rr.update_scene(data, cam.mj(model))
        rr.scene.flags[mujoco.mjtRndFlag.mjRND_FOG] = True
        img = Image.fromarray(rr.render()).convert("RGBA")
        img = Image.alpha_composite(img, overlay(idx, len(tl), title_n, end_n, end_txt))
        frame = np.asarray(img.convert("RGB"))
        if preview:
            sheet.append(Image.fromarray(frame).resize((W // 2, H // 2)))
        else:
            proc.stdin.write(frame.tobytes())
    if proc:
        proc.stdin.close(); proc.wait()
        print("wrote", out, len(tl), "frames", f"{len(tl)/FPS:.1f}s", flush=True)
    else:
        g = Image.new("RGB", (W, H * 2))
        for i, im in enumerate(sheet):
            g.paste(im, ((i % 2) * W // 2, (i // 2) * H // 2))
        g.save(f"{OUT}/glade_curious_preview.png"); print("preview written")


if __name__ == "__main__":
    main("--preview" in sys.argv)
