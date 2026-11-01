"""Concept clip: Tether as a construction robot. It walks across a grid of floor docks, then picks
dock-fitted blocks off a pile and stacks them into a cross-laid tower.

Usage: python construct.py [--preview]
Writes videos/construction/construction_concept.mp4 (or a PNG contact sheet with --preview).

Each block has a female dock on top and a matching pocket underneath, so blocks stack over the dock
below. While a block is carried, its weight is in the motor torques and its box is collision-checked
against the scene and the arm. The free gripper's spin motor turns the block on the way.
Block size and mass are assumptions for the concept, not a design."""
import os, sys, pickle, subprocess, numpy as np
from showcase import *          # also patches Planner.collides for multi-part obstacles

BLOCK = np.array([0.40, 0.20, 0.12])   # 16 x 8 x 4.7 in
BLOCK_KG = 2.5                         # assumed: plywood box with a dock on top
TILT_MAX = np.radians(50)              # keep a carried block within 50 deg of level
PITCH = 0.50                           # floor dock grid, 1 ft 8 in
WORK = np.array([1.5, 0.0])            # floor dock the robot works from
OUT = "videos/construction"
FOOTER_C = f"Concept simulation. Speed is not real.   Arm 31.6 lb (14.35 kg)   Grid squares 1 ft"


def Rz3(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def block_T(xy, z_bottom, yaw):
    T = np.eye(4); T[:3, :3] = Rz3(yaw); T[:3, 3] = (xy[0], xy[1], z_bottom + BLOCK[2] / 2)
    return T


def top_dock(Tb):
    return dock_pose(Tb[:3, 3] + (0, 0, BLOCK[2] / 2), DOWN)


def wrapd(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class CarryPlanner(Planner):
    """Planner whose free gripper may hold a block: carry = dict(th0, th1, yaw0, dyaw[, center, size, mass])."""
    carry = None
    next_carry = None   # block the empty gripper is about to pick: counted at the pick pose

    def block_pose(self, stance, X, carry=None):
        c = carry or self.carry
        M = self.M[stance]
        Ts = X[M.swing] @ M.dock[M.swing]
        p = Ts[:3, 3] - c.get("center", WORK_XYZ)
        th = np.arctan2(p[1], p[0])
        d0, d1 = abs(wrapd(th - c["th0"])), abs(wrapd(th - c["th1"]))
        u = d0 / max(d0 + d1, 1e-9)
        u = u * u * (3 - 2 * u)
        yaw = c["yaw0"] + u * c["dyaw"]
        t = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        R = Ts[:3, :3]
        psi = np.arctan2(t @ R[:, 1], t @ R[:, 0])
        Tb = np.eye(4)
        Tb[:3, :3] = R @ Rz3(psi)
        Tb[:3, 3] = Ts[:3, 3] + R[:, 2] * np.asarray(c.get("size", BLOCK))[2] / 2
        return Tb

    def collides(self, stance, X, swing_foot_free):
        if Planner.collides(self, stance, X, swing_foot_free):
            return True
        if self.carry is None:
            return False
        M = self.M[stance]
        Ts = X[M.swing] @ M.dock[M.swing]
        if Ts[:3, 2] @ np.array(DOWN) < np.cos(TILT_MAX):
            return True
        Tb = self.block_pose(stance, X)
        ob = fcl.CollisionObject(fcl.Box(*(np.asarray(self.carry.get("size", BLOCK)) - 0.004)), fcl.Transform(Tb[:3, :3], Tb[:3, 3]))
        for b in self.boxes:
            for o in getattr(b, "objs", [b.obj]):
                if fcl.collide(ob, o, fcl.CollisionRequest(), fcl.CollisionResult()):
                    return True
        for n in M.cl:            # placed by self_collision above
            if n in FOOT[M.swing]:
                continue
            if fcl.collide(ob, M.cl[n]["obj"], fcl.CollisionRequest(), fcl.CollisionResult()):
                return True
        return False

    def load(self, stance, X, x6):
        t, w = Planner.load(self, stance, X, x6)
        c = self.carry or self.next_carry
        if c is None:
            return t, w
        return self.carry_torques(stance, X, c), w

    def peak_ratio(self, stance, X, x6):
        t = self.carry_torques(stance, X) if self.carry is not None else Planner.load(self, stance, X, x6)[0]
        return self.M[stance].ratios(t)[1]

    def carry_torques(self, stance, X, carry=None, mass=None):
        c = carry or self.carry
        mass = c.get("mass", BLOCK_KG) if mass is None else mass
        M = self.M[stance]
        Mm = M.mass["datasheet_motors"]
        pc = self.block_pose(stance, X, carry)[:3, 3]
        com = {n: X[n][:3, :3] @ L["com"] + X[n][:3, 3] for n, L in M.arm.links.items()}
        t = {}
        for name, ji in M.jinfo.items():
            j = ji["j"]; Tj = X[j["parent"]] @ j["T"]
            a = Tj[:3, :3] @ (j["axis"] / np.linalg.norm(j["axis"])); p = Tj[:3, 3]
            mom = sum(Mm[n] * np.cross(com[n] - p, G) for n in ji["side"])
            if M.swing in ji["side"]:
                mom = mom + mass * np.cross(pc - p, G)
            t[name] = abs(a @ mom)
        return t


WORK_XYZ = np.array([WORK[0], WORK[1], 0.0])


def scene():
    xs, ys = np.arange(0, 2.51, PITCH), (-PITCH, 0.0, PITCH)
    grid = [(x, y) for y in ys for x in xs]
    docks = [dock_pose((x, y, 0), DOWN) for x, y in grid]
    gi = {g: i for i, g in enumerate(grid)}
    floor = floor_box(-1.0, 3.4, -1.3, 1.3)
    # pile: two stacks of two, long side along x
    pile = [block_T((1.27, -0.32), 0.0, 0.0), block_T((1.27, -0.32), BLOCK[2], 0.0),
            block_T((1.73, -0.32), 0.0, 0.0), block_T((1.73, -0.32), BLOCK[2], 0.0)]
    # tower: four layers, each turned 90 deg
    tower = [block_T((1.5, 0.34), i * BLOCK[2], (np.pi / 2) * (i % 2)) for i in range(4)]
    # (block index, tower slot): take the top of each stack first
    order = [(1, 0), (3, 1), (0, 2), (2, 3)]
    return dict(docks=docks, gi=gi, floor=floor, pile=pile, tower=tower, order=order)


def as_box(T):
    b = Box(T[:3, 3], BLOCK, T[:3, :3]); b.mat = "wood"
    return b


def plan(seed=0):
    S = scene()
    rng = np.random.default_rng(seed)
    docks, gi = S["docks"], S["gi"]
    blocks = [T.copy() for T in S["pile"]]
    P = CarryPlanner([S["floor"]] + [as_box(T) for T in blocks])
    # start: B in (0, 0), A in (0.5, 0)
    s, w = EE_B, EE_A
    Tws = docks[gi[(0.0, 0.0)]]
    tgt = docks[gi[(0.5, 0.0)]]
    sols = P.ik(s, Tws, tgt[:3, 3], tgt[:3, 2], rng.uniform(-np.pi, np.pi, (300, 6)))
    good = [(P.M[s].ratios(P.load(s, X, x)[0])[0], x) for x in sols for X in [P.frames(s, x, Tws)] if not P.collides(s, X, False)]
    x6 = min(good, key=lambda t: t[0])[1]
    stance = s
    at = {EE_B: docks[gi[(0.0, 0.0)]], EE_A: docks[gi[(0.5, 0.0)]]}
    frames = []

    def snap(phase, k, x, carry=None, held=None):
        stat = {i: T.copy() for i, T in enumerate(blocks) if i != held}
        frames.append((stance, Tws, x, k, phase, carry, held, stat))

    for _ in range(20):
        snap("start", -1, x6)
    k = 0
    # --- walk in
    for mover, g in [(EE_B, (1.0, 0.0)), (EE_A, (1.5, 0.0))]:
        new_st = EE_A if mover == EE_B else EE_B
        if new_st != stance:
            x6 = P.swap(stance, x6, Tws, new_st, at[new_st]); stance, Tws = new_st, at[new_st]
        lift, swing, down = P.step(stance, Tws, x6, at[mover], docks[gi[g]], rng)
        for ph, seg in (("lift out of dock", lift), ("swing", swing), ("lower into dock", down)):
            for x in seg:
                snap(ph, k, x)
        x6 = down[-1]; at[mover] = docks[gi[g]]
        for _ in range(10):
            snap("docked", k, x6)
        print(f"  walk {k}: {mover} -> {g}", flush=True); k += 1
    # stance is now A in the work dock; B is the tool end
    new_st = EE_A
    if stance != new_st:
        x6 = P.swap(stance, x6, Tws, new_st, at[new_st]); stance, Tws = new_st, at[new_st]
    worst = []
    for bi, slot in S["order"]:
        # --- reach for the block (empty gripper)
        P.carry = None
        P.boxes = [S["floor"]] + [as_box(T) for T in blocks]
        Tpick = top_dock(blocks[bi])
        Tgoal = S["tower"][slot]
        yaw0 = np.arctan2(blocks[bi][1, 0], blocks[bi][0, 0])
        yaw1 = np.arctan2(Tgoal[1, 0], Tgoal[0, 0])
        dyaw = (yaw1 - yaw0 + np.pi / 2) % np.pi - np.pi / 2       # a box looks the same turned 180 deg
        c = dict(th0=np.arctan2(*(blocks[bi][:2, 3] - WORK)[::-1]), th1=np.arctan2(*(Tgoal[:2, 3] - WORK)[::-1]),
                 yaw0=yaw0, dyaw=dyaw)
        P.next_carry, P.max_peak = c, None
        lift, swing, down = P.step(stance, Tws, x6, at[EE_B], Tpick, rng)
        P.next_carry = None
        for ph, seg in (("lift out of dock", lift), ("swing", swing), ("lower into dock", down)):
            for x in seg:
                snap(ph, k, x)
        x6 = down[-1]; at[EE_B] = Tpick
        for _ in range(8):
            snap("docked", k, x6)
        print(f"  pick {k}: block {bi}", flush=True); k += 1
        # --- carry it to its tower slot
        Tplace = top_dock(Tgoal)
        P.carry, P.max_peak = c, 0.26   # swing within rated torque on every motor while carrying
        P.boxes = [S["floor"]] + [as_box(T) for i, T in enumerate(blocks) if i != bi]
        lift, swing, down = P.step(stance, Tws, x6, Tpick, Tplace, rng)
        for ph, seg in (("lift block", lift), ("carry", swing), ("set down", down)):
            for x in seg:
                snap(ph, k, x, c, bi)
                X = P.frames(stance, x, Tws)
                worst.append(P.M[stance].ratios(P.carry_torques(stance, X, c))[0])
        x6 = down[-1]; at[EE_B] = Tplace
        blocks[bi] = P.block_pose(stance, P.frames(stance, x6, Tws), c)
        blocks[bi][:3, :3] = Tgoal[:3, :3]; blocks[bi][:3, 3] = Tgoal[:3, 3]
        P.carry, P.max_peak = None, None
        for _ in range(8):
            snap("docked", k, x6)
        print(f"  place {k}: block {bi} -> slot {slot}, worst rated ratio so far {max(worst):.2f}", flush=True); k += 1
    # --- back to the floor
    P.carry = None
    P.boxes = [S["floor"]] + [as_box(T) for T in blocks]
    home = docks[gi[(1.0, 0.0)]]
    lift, swing, down = P.step(stance, Tws, x6, at[EE_B], home, rng)
    for ph, seg in (("lift out of dock", lift), ("swing", swing), ("lower into dock", down)):
        for x in seg:
            snap(ph, k, x)
    x6 = down[-1]
    for _ in range(20):
        snap("done", k, x6)
    print("  done; carry worst rated ratio", max(worst), flush=True)
    return P, frames, S


def build(P, S, tmp):
    arm = P.M[EE_A].arm
    os.makedirs(tmp, exist_ok=True)

    def color(n):
        if "DM-J10010" in n: return "0.24 0.25 0.28 1"
        if "DM-J4340" in n: return "0.34 0.35 0.38 1"
        if "LiPo" in n: return "0.30 0.40 0.58 1"
        if "SlipRing" in n: return "0.55 0.56 0.60 1"
        if "EndEffector" in n: return "0.93 0.48 0.22 1"
        return "0.80 0.81 0.83 1"
    teal = f"{TEAL[0]/255:.3f} {TEAL[1]/255:.3f} {TEAL[2]/255:.3f} 1"
    assets, bodies, world = [], [], []
    names = []
    for i, (n, m) in enumerate(arm.link_mesh.items()):
        if m is None: continue
        fn = f"{tmp}/l{i}.stl"; m.export(fn)
        assets.append(f'<mesh name="m{i}" file="{fn}" inertia="shell"/>')
        bodies.append(f'<body name="b{i}" mocap="true"><geom type="mesh" mesh="m{i}" rgba="{color(n)}" contype="0" conaffinity="0"/></body>')
        names.append(n)
    dm = Arm(PKG + "/urdf/dock_female.urdf").load_meshes()["dock_female"]
    dm.export(f"{tmp}/dock.stl")
    assets.append(f'<mesh name="dock" file="{tmp}/dock.stl" inertia="shell"/>')
    for j, T in enumerate(S["docks"]):
        q = np.zeros(4); mujoco.mju_mat2Quat(q, T[:3, :3].flatten())
        world.append(f'<geom type="mesh" mesh="dock" pos="{" ".join(map(str, T[:3, 3]))}" quat="{" ".join(map(str, q))}" rgba="{teal}" contype="0" conaffinity="0"/>')
    b = S["floor"]
    world.append(f'<geom type="box" size="{" ".join(map(str, b.size / 2))}" pos="{" ".join(map(str, b.c))}" material="grid" contype="0" conaffinity="0"/>')
    hx, hy, hz = BLOCK / 2
    for i in range(len(S["pile"])):
        # dock on top (dock frame at the top face, +z into the block)
        bodies.append(f'<body name="blk{i}" mocap="true"><geom type="box" size="{hx} {hy} {hz}" material="wood" contype="0" conaffinity="0"/>'
                      f'<geom type="mesh" mesh="dock" pos="0 0 {hz}" quat="0 1 0 0" rgba="{teal}" contype="0" conaffinity="0"/>'
                      f'<geom type="box" size="{hx + 0.0005} {hy + 0.0005} 0.004" pos="0 0 {-hz + 0.004}" rgba="0.55 0.50 0.44 1" contype="0" conaffinity="0"/></body>')
    grid = 1 / (2 * 0.3048)
    xml = f"""<mujoco><compiler/><visual><global offwidth="{W}" offheight="{H}"/><quality shadowsize="4096"/>
      <headlight ambient="0.42 0.42 0.42" diffuse="0.45 0.45 0.45"/></visual>
      <asset><texture type="skybox" builtin="gradient" rgb1="0.985 0.985 0.98" rgb2="0.93 0.935 0.94" width="64" height="64"/>
      <texture name="g" type="2d" builtin="checker" rgb1="0.90 0.895 0.88" rgb2="0.85 0.845 0.83" width="256" height="256"/>
      <material name="grid" texture="g" texrepeat="{grid:.4f} {grid:.4f}" texuniform="true"/>
      <material name="wood" rgba="0.80 0.68 0.50 1"/>{''.join(assets)}</asset>
      <worldbody><light directional="true" pos="1 -2 4" dir="-0.3 0.5 -1" diffuse="0.55 0.55 0.55" castshadow="true"/>
      {''.join(world)}{''.join(bodies)}</worldbody></mujoco>"""
    return mujoco.MjModel.from_xml_string(xml), names


CAPTIONS = [
    ("start", "Concept: Tether as a construction robot. A grid of floor docks, 1 ft 8 in (0.50 m) apart, covers the work area."),
    ("start", f"Each block has a dock on top and a pocket underneath, so blocks stack over the dock below. Block {BLOCK_KG * 2.2046:.1f} lb ({BLOCK_KG} kg), assumed."),
    (0, "It walks across the grid to the dock next to the pile."),
    (2, "One end stays locked in the floor dock. The other locks into the top of a block, the same way it locks into a dock."),
    (3, "It lifts the block out, swings it over, and sets it down. The gripper's spin motor turns it on the way."),
    (5, "Each layer goes down turned 90 degrees to the one below."),
    (7, "{capacity}"),
    (10, "Done. The free end goes back to a floor dock, ready to walk to the next job."),
    ("end", "Open items: a dock built into the material, how blocks lock to each other, and payload tests on hardware."),
]


def timeline(frames):
    """2x slow motion, short holds, 4 s start captions and 4.5 s end caption."""
    out = []
    for i, fr in enumerate(frames):
        if fr[4] in ("start", "done"):
            continue
        out.append(fr)
        nx = frames[i + 1] if i + 1 < len(frames) else fr
        if fr[4] not in ("docked",) and nx[3] == fr[3] and nx[4] == fr[4]:
            dd = wrapd(nx[2] - fr[2])
            out.append(fr[:2] + (fr[2] + 0.5 * dd,) + fr[3:])
    caps = {t: s for t, s in CAPTIONS if isinstance(t, int)}
    st = [s for t, s in CAPTIONS if t == "start"]
    en = [s for t, s in CAPTIONS if t == "end"]
    tl = []
    for s in st:
        tl += [(frames[0], s)] * int(4.0 * FPS)
    cur = st[-1]
    for fr in out:
        if fr[3] in caps:
            cur = caps[fr[3]]
        tl.append((fr, cur))
    last = [f for f in frames if f[4] == "done"][-1]
    tl += [(last, cur)] * int(2.0 * FPS)
    for s in en:
        tl += [(last, s)] * int(4.5 * FPS)
    return tl


def main(preview=False):
    os.makedirs("plans/construction", exist_ok=True)
    cache = "plans/construction/construction_concept.pkl"
    if os.path.exists(cache):
        frames, S = pickle.load(open(cache, "rb")), scene()
        P = CarryPlanner([S["floor"]])
    else:
        for seed in range(6):
            try:
                P, frames, S = plan(seed); break
            except RuntimeError as e:
                print("  seed", seed, "failed:", e, flush=True)
        else:
            raise RuntimeError("planning failed for all seeds")
        pickle.dump(frames, open(cache, "wb"))
    # payload capacity at the poses actually used: max block mass within rated torque, worst carry frame
    caps_kg = []
    for fr in frames:
        if fr[5] is None:
            continue
        stance, Tws, x6, k, ph, c = fr[:6]
        X = P.frames(stance, x6, Tws)
        lo, hi = 0.0, 20.0
        for _ in range(30):
            m = (lo + hi) / 2
            if P.M[stance].ratios(P.carry_torques(stance, X, c, m))[0] <= 1: lo = m
            else: hi = m
        caps_kg.append(lo)
    worst_kg = min(caps_kg)
    carry_ratio = max(P.M[fr[0]].ratios(P.carry_torques(fr[0], P.frames(fr[0], fr[2], fr[1]), fr[5]))[0] for fr in frames if fr[5] is not None)
    print(f"block {BLOCK_KG} kg; worst-frame capacity within rated {worst_kg:.2f} kg; worst rated ratio while carrying {carry_ratio:.2f}")
    cap_txt = (f"Carrying, the motors stay within rated torque for blocks up to {np.floor(worst_kg * 2.2046):.0f} lb "
               f"({np.floor(worst_kg * 10) / 10:.1f} kg) along this path.") if carry_ratio <= 1 else \
              (f"With a {BLOCK_KG} kg block some motors go to {carry_ratio:.1f}x rated torque mid-swing, still under peak.")
    global CAPTIONS
    CAPTIONS = [(t, s.replace("{capacity}", cap_txt)) for t, s in CAPTIONS]
    model, names = build(P, S, os.path.abspath("mjtmp/construction"))
    tl = timeline(frames)
    # camera: fixed three-quarter view fitted to the arm, the grid and the blocks
    arm = P.M[EE_A].arm
    sub = {n: arm.link_mesh[n].vertices[::60] for n in names}
    pts = []
    for fr in frames[::6]:
        X = P.frames(fr[0], fr[2], fr[1])
        for n in names:
            pts += list(sub[n] @ X[n][:3, :3].T + X[n][:3, 3])
    pts = pts[::7] + [T[:3, 3] for T in S["docks"]] + [T[:3, 3] + (0, 0, 0.07) for T in S["tower"]]
    cam = fit(pts, 68, -30, False, margin=40)
    data = mujoco.MjData(model)
    rr = mujoco.Renderer(model, H, W)
    na = len(names)
    os.makedirs(OUT, exist_ok=True)
    out = f"{OUT}/construction_concept.mp4"
    proc = None
    if not preview:
        proc = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
                                 "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "19", "-movflags", "+faststart", out], stdin=subprocess.PIPE)
    sel = range(len(tl)) if not preview else np.linspace(0, len(tl) - 1, 8).astype(int)
    prev, fade, sheet = None, 0, []
    for idx in sel:
        (stance, Tws, x6, k, phase, c, held, stat), txt = tl[idx]
        if txt != prev:
            fade, prev = 0, txt
        fade = 9 if preview else min(fade + 1, 9)
        X = P.frames(stance, x6, Tws)
        for i, n in enumerate(names):
            q = np.zeros(4); mujoco.mju_mat2Quat(q, X[n][:3, :3].flatten())
            data.mocap_pos[i] = X[n][:3, 3]; data.mocap_quat[i] = q
        for i in range(len(S["pile"])):
            T = P.block_pose(stance, X, c) if i == held else stat[i]
            q = np.zeros(4); mujoco.mju_mat2Quat(q, np.ascontiguousarray(T[:3, :3]).flatten())
            data.mocap_pos[na + i] = T[:3, 3]; data.mocap_quat[na + i] = q
        mujoco.mj_forward(model, data)
        rr.update_scene(data, cam.mj(model))
        img = Image.fromarray(rr.render()).convert("RGBA")
        img = Image.alpha_composite(img, caption_layer("Concept: building with docks", "", txt, fade / 9, FOOTER_C))
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
        g.save(f"{OUT}/construction_concept_preview.png"); print("preview written")


if __name__ == "__main__":
    main("--preview" in sys.argv)
