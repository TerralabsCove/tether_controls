"""Render dock-to-dock test videos. Usage: MUJOCO_GL=osmesa python render.py <scenario> [--preview]"""
import os, sys, subprocess, numpy as np
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "osmesa")  # headless software rendering
import mujoco
from PIL import Image, ImageDraw, ImageFont
from plan import *

W3, H = 960, 720
PW = 420
FPS = 30
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONTB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
DOWN, UP = (0, 0, -1.0), (0, 0, 1.0)

JOINT_LABEL = [("SlipRing_1_spin", "A gripper spin"), ("Revolute_22", "A pitch"), ("Revolute_57", "A roll"),
               ("Revolute_44", "Middle pitch"), ("Revolute_73", "B roll"), ("Revolute_29", "B pitch"),
               ("SlipRing_2_spin", "B gripper spin")]
END = {EE_A: "A", EE_B: "B"}
ARM_KG = sum(L["mass"] for L in Arm(PKG + "/urdf/arm.urdf").links.values())


def floor(x0=-1.5, x1=3.5, y=3.0, top=0.0):
    return Box(((x0 + x1) / 2, 0, top - 0.05), (x1 - x0, y, 0.1))


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def scenarios():
    S = {}
    # 1/2 flat floor
    for key, sp, n, title in [("01_floor_rated_0.44m", 0.44, 3, "Flat floor, 0.44 m steps (within rated torque)"),
                              ("02_floor_near_max_0.80m", 0.80, 2, "Flat floor, 0.80 m steps (max is 0.84 m; lifting out of the dock needs the margin)")]:
        docks = [dock_pose((i * sp, 0, 0), DOWN) for i in range(n + 2)]
        steps = [((EE_B, EE_A)[k % 2], k + 2) for k in range(n)]
        S[key] = dict(title=title, boxes=[floor()], docks=docks, start=(EE_B, 0, EE_A, 1), steps=steps,
                      cam=dict(lookat=(sp * (n + 1) / 2, 0, 0.45), distance=1.7 + 0.45 * sp * n, azimuth=90, elevation=-10))
    # 3 box up and over
    docks = [dock_pose(p, n) for p, n in [((0, 0, 0), DOWN), ((0.4, 0, 0), DOWN), ((0.8, 0, 0.5), DOWN),
                                          ((1.2, 0, 0.5), DOWN), ((1.6, 0, 0), DOWN), ((2.0, 0, 0), DOWN)]]
    S["03_box_up_and_over_0.5m"] = dict(
        title="Up and over a 0.5 m box (0.4 m steps)", boxes=[floor(), Box((1.0, 0, 0.25), (0.8, 1.0, 0.5))],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3), (EE_B, 4), (EE_A, 5)],
        cam=dict(lookat=(1.0, 0, 0.55), distance=3.6, azimuth=90, elevation=-12))
    # 4 incline 30 deg
    a = np.radians(30); R = Ry(-a); u = R @ [1, 0, 0]; nout = R @ [0, 0, 1]
    docks = [dock_pose(s * u, -nout) for s in np.arange(4) * 0.5]
    S["04_incline_30deg_up"] = dict(
        title="30 deg incline, climbing straight up, 0.5 m steps", boxes=[Box(0.75 * u - 0.1 * nout, (4.0, 2.0, 0.2), R)],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3)],
        cam=dict(lookat=tuple(0.75 * u + 0.4 * nout), distance=3.0, azimuth=90, elevation=-8))
    # 5 floor -> wall -> climb
    X = (1, 0, 0)
    docks = [dock_pose(p, n) for p, n in [((-0.44, 0, 0), DOWN), ((0, 0, 0), DOWN), ((0.6, 0, 0.3), X),
                                          ((0.6, 0, 1.1), X), ((0.6, 0, 1.9), X)]]
    S["05_floor_to_wall_climb"] = dict(
        title="Floor to wall, then climbing 0.8 m steps", boxes=[floor(-1.5, 0.6), Box((0.8, 0, 1.25), (0.4, 2.0, 2.7))],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3), (EE_B, 4)],
        cam=dict(lookat=(0.15, 0, 1.0), distance=4.2, azimuth=70, elevation=-8))
    # 6 wall sideways
    docks = [dock_pose((0, y, 1.2), X) for y in (0, 0.25, 0.5, 0.75, 1.25)]
    S["06_wall_sideways"] = dict(
        title="Wall traverse sideways: 0.25 m steps, then a 0.5 m step", boxes=[floor(-2, 0), Box((0.2, 0.6, 1.25), (0.4, 4.0, 2.7))],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3), (EE_B, 4)],
        note={2: "0.5 m sideways: gripper-spin motor over its 27 Nm peak. Would fail on hardware."},
        cam=dict(lookat=(-0.3, 0.6, 1.2), distance=3.3, azimuth=-25, elevation=-10))
    # 7 crest a wall top
    M_ = (-1, 0, 0)
    docks = [dock_pose(p, n) for p, n in [((0, 0, 0.5), M_), ((0, 0, 1.3), M_), ((-0.2, 0, 1.5), DOWN), ((-0.6, 0, 1.5), DOWN)]]
    S["07_crest_wall_top"] = dict(
        title="Climbing over the top edge of a 1.5 m wall", boxes=[floor(0, 3), Box((-0.75, 0, 0.75), (1.5, 2.0, 1.5))],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3)], max_peak=1.0,
        cam=dict(lookat=(-0.1, 0, 1.2), distance=3.4, azimuth=90, elevation=-10))
    # 8 ceiling
    docks = [dock_pose((i * 0.44, 0, 1.9), UP) for i in range(5)]
    S["08_ceiling_0.44m"] = dict(
        title="Hanging from a ceiling, 0.44 m steps", boxes=[floor(), Box((0.9, 0, 2.0), (5.0, 3.0, 0.2))],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3), (EE_B, 4)],
        cam=dict(lookat=(0.9, 0, 1.4), distance=3.4, azimuth=90, elevation=6))
    return S


def plan_scenario(sc, seed=0):
    P = Planner(sc["boxes"], sc.get("max_peak"))
    rng = np.random.default_rng(seed)
    docks = sc["docks"]
    s, ds, w, dw = sc["start"]
    at = {s: ds, w: dw}
    Tws = docks[ds]
    sols = P.ik(s, Tws, docks[dw][:3, 3], docks[dw][:3, 2], rng.uniform(-np.pi, np.pi, (300, 6)))
    good = []
    for x in sols:
        X = P.frames(s, x, Tws)
        if not P.collides(s, X, False):
            good.append((P.M[s].ratios(P.load(s, X, x)[0])[0], x))   # lowest rated-torque ratio
    if not good:
        raise RuntimeError("no start pose")
    x6 = min(good, key=lambda t: t[0])[1]
    frames = [(s, Tws, x6, -1, "start")] * 20
    stance = s
    for k, (mover, tgt) in enumerate(sc["steps"]):
        new_st = EE_A if mover == EE_B else EE_B
        if new_st != stance:
            x6 = P.swap(stance, x6, Tws, new_st, docks[at[new_st]])
            stance, Tws = new_st, docks[at[new_st]]
        lift, swing, down = P.step(stance, Tws, x6, docks[at[mover]], docks[tgt], rng)
        frames += [(stance, Tws, x, k, "lift out of dock") for x in lift]
        frames += [(stance, Tws, x, k, "swing") for x in swing]
        frames += [(stance, Tws, x, k, "lower into dock") for x in down]
        x6 = down[-1]
        at[mover] = tgt
        frames += [(stance, Tws, x6, k, "docked")] * 12
        print(f"  step {k+1}: {END[mover]} -> dock {tgt}, {len(swing)} swing frames", flush=True)
    frames += [(stance, Tws, x6, len(sc["steps"]) - 1, "done")] * 20
    return P, frames


def build_model(P, sc, tmp):
    arm = P.M[EE_A].arm
    os.makedirs(tmp, exist_ok=True)
    def color(n):
        if "DM-J10010" in n: return "0.22 0.23 0.25 1"
        if "DM-J4340" in n: return "0.32 0.33 0.36 1"
        if "LiPo" in n: return "0.25 0.42 0.75 1"
        if "SlipRing" in n: return "0.55 0.56 0.60 1"
        if "EndEffector" in n: return "0.92 0.45 0.20 1"
        return "0.78 0.80 0.83 1"
    assets, bodies = [], []
    for i, (n, m) in enumerate(arm.link_mesh.items()):
        if m is None: continue
        fn = f"{tmp}/l{i}.stl"; m.export(fn)
        assets.append(f'<mesh name="m{i}" file="{fn}" inertia="shell"/>')
        bodies.append(f'<body name="b{i}" mocap="true"><geom type="mesh" mesh="m{i}" rgba="{color(n)}" contype="0" conaffinity="0"/></body>')
    dm = Arm(PKG + "/urdf/dock_female.urdf").load_meshes()["dock_female"]
    dm.export(f"{tmp}/dock.stl")
    assets.append(f'<mesh name="dock" file="{tmp}/dock.stl" inertia="shell"/>')
    world = []
    for j, T in enumerate(sc["docks"]):
        q = np.zeros(4); mujoco.mju_mat2Quat(q, T[:3, :3].flatten())
        world.append(f'<geom type="mesh" mesh="dock" pos="{" ".join(map(str, T[:3, 3]))}" quat="{" ".join(map(str, q))}" rgba="0.2 0.62 0.38 1" contype="0" conaffinity="0"/>')
    for b in sc["boxes"]:
        q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(b.R, float).flatten())
        world.append(f'<geom type="box" size="{" ".join(map(str, b.size / 2))}" pos="{" ".join(map(str, b.c))}" quat="{" ".join(map(str, q))}" material="grid" contype="0" conaffinity="0"/>')
    xml = f"""<mujoco><compiler/><visual><global offwidth="{W3}" offheight="{H}"/><quality shadowsize="4096"/>
      <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5"/></visual>
      <asset><texture type="skybox" builtin="gradient" rgb1="1 1 1" rgb2="0.86 0.88 0.92" width="64" height="64"/>
      <texture name="g" type="2d" builtin="checker" rgb1="0.86 0.85 0.82" rgb2="0.80 0.79 0.76" width="256" height="256"/>
      <material name="grid" texture="g" texrepeat="4 4" texuniform="true"/>{''.join(assets)}</asset>
      <worldbody><light directional="true" pos="1 -2 4" dir="-0.3 0.5 -1" diffuse="0.6 0.6 0.6" castshadow="true"/>
      {''.join(world)}{''.join(bodies)}</worldbody></mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    names = [n for n, m in arm.link_mesh.items() if m is not None]
    return model, names


def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default(size)


def panel(t, wrench, title, step, nsteps, phase, stance, note):
    img = Image.new("RGB", (PW, H), (252, 252, 251))
    d = ImageDraw.Draw(img)
    f, fb, fs = font(FONT, 15), font(FONTB, 17), font(FONT, 12)
    y = 18
    words, line = title.split(), ""
    for w_ in words:
        if d.textlength(line + " " + w_, font=fb) > PW - 36:
            d.text((18, y), line.strip(), fill=(11, 11, 11), font=fb); y += 24; line = ""
        line += " " + w_
    d.text((18, y), line.strip(), fill=(11, 11, 11), font=fb); y += 34
    d.text((18, y), f"Step {max(step, 0) + 1}/{nsteps}  -  {phase}" if step >= 0 else "Start pose", fill=(82, 81, 78), font=f); y += 22
    d.text((18, y), f"Anchored gripper: {END[stance]}-end", fill=(82, 81, 78), font=f); y += 34
    d.text((18, y), "Motor torque (static)", fill=(11, 11, 11), font=fb); y += 26
    for jn, lab in JOINT_LABEL:
        mt = MOTOR[jn]; sp = SPEC[mt]; v = t[jn]
        col = (42, 120, 214) if v <= sp["rated"] else ((235, 104, 52) if v <= sp["peak"] else (227, 73, 72))
        d.text((18, y), f"{lab} ({mt})", fill=(11, 11, 11), font=fs)
        d.text((PW - 18, y), f"{v:5.1f} Nm", fill=(11, 11, 11), font=fs, anchor="ra"); y += 17
        x0, x1 = 18, PW - 18
        d.rounded_rectangle((x0, y, x1, y + 10), 3, fill=(232, 231, 227))
        frac = min(v / sp["peak"], 1.0)
        if frac > 0.004:
            d.rounded_rectangle((x0, y, x0 + max(frac * (x1 - x0), 4), y + 10), 3, fill=col)
        xr = x0 + sp["rated"] / sp["peak"] * (x1 - x0)
        d.line((xr, y - 3, xr, y + 13), fill=(11, 11, 11), width=2)
        y += 24
    d.text((18, y), "bar end = peak, tick = rated", fill=(82, 81, 78), font=fs); y += 30
    fn, sh, bend, tw = wrench
    d.text((18, y), "Load on anchored gripper", fill=(11, 11, 11), font=fb); y += 26
    for txt in [(f"Pressing in: {fn:5.0f} N" if fn >= 1 else (f"PULLING OUT: {-fn:5.0f} N" if fn <= -1 else "Normal force: 0 N")), f"Shear: {sh:5.0f} N",
                f"Bending: {bend:5.1f} Nm", f"Twist: {tw:5.1f} Nm"]:
        d.text((18, y), txt, fill=(11, 11, 11), font=f); y += 21
    if note:
        y += 10
        words, line = note.split(), ""
        for w_ in words:
            if d.textlength(line + " " + w_, font=f) > PW - 36:
                d.text((18, y), line.strip(), fill=(227, 73, 72), font=f); y += 20; line = ""
            line += " " + w_
        d.text((18, y), line.strip(), fill=(227, 73, 72), font=f)
    d.text((18, H - 46), f"Quasi-static. Arm {ARM_KG:.2f} kg (corrected URDF: datasheet", fill=(82, 81, 78), font=fs)
    d.text((18, H - 30), "motors, weighed batteries, slip rings, servos).", fill=(82, 81, 78), font=fs)
    return img


def main(key, preview=False):
    sc = scenarios()[key]
    print(key, flush=True)
    import pickle
    cache = f"plans/{key}.pkl"
    os.makedirs("plans", exist_ok=True)
    if os.path.exists(cache):
        frames = pickle.load(open(cache, "rb")); P = Planner(sc["boxes"])
    else:
        P, frames = plan_scenario(sc)
        pickle.dump(frames, open(cache, "wb"))
    model, names = build_model(P, sc, os.path.abspath(f"mjtmp/{key}"))
    slow = []
    for a, b in zip(frames, frames[1:] + frames[-1:]):
        slow.append(a)
        if a[4] in ("lift out of dock", "swing", "lower into dock") and a[0] == b[0] and a[4] == b[4]:
            d = (b[2] - a[2] + np.pi) % (2 * np.pi) - np.pi
            slow.append((a[0], a[1], a[2] + 0.5 * d, a[3], a[4]))
    frames = slow
    data = mujoco.MjData(model)
    r = mujoco.Renderer(model, H, W3)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    c = sc["cam"]; cam.azimuth = c["azimuth"]; cam.elevation = c["elevation"]
    pts = []
    for stance, Tws, x6, k, phase in frames[::4]:
        X = P.frames(stance, x6, Tws); pts += [X[n][:3, 3] for n in names]
    pts += [T[:3, 3] for T in sc["docks"]]
    pts = np.array(pts)
    az, el = np.radians(c["azimuth"]), np.radians(c["elevation"])
    f = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    right = np.cross(f, [0, 0, 1.0]); right /= np.linalg.norm(right); up = np.cross(right, f)
    lo, hi = pts.min(0) - 0.08, pts.max(0) + 0.08
    cam.lookat[:] = (lo + hi) / 2
    corners = np.array(np.meshgrid(*zip(lo, hi))).reshape(3, -1).T - (lo + hi) / 2
    w, h = np.ptp(corners @ right), np.ptp(corners @ up)
    cam.distance = 0.95 * max(w / (W3 / H), h) / (2 * np.tan(np.radians(22.5))) + 0.2 * np.ptp(corners @ f)
    os.makedirs("videos", exist_ok=True)
    out = f"videos/{key}.mp4" if not preview else f"videos/{key}_preview.png"
    proc = None if preview else subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W3 + PW}x{H}", "-r", str(FPS),
         "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", "-movflags", "+faststart", out], stdin=subprocess.PIPE)
    sel = frames if not preview else [frames[len(frames) // 2]]
    if preview == "all":
        sel = frames[::max(1, len(frames) // 6)][:6]
    for stance, Tws, x6, k, phase in sel:
        X = P.frames(stance, x6, Tws)
        for i, n in enumerate(names):
            q = np.zeros(4); mujoco.mju_mat2Quat(q, X[n][:3, :3].flatten())
            data.mocap_pos[i] = X[n][:3, 3]; data.mocap_quat[i] = q
        mujoco.mj_forward(model, data)
        r.update_scene(data, cam)
        img3 = r.render()
        t, wr = P.load(stance, X, x6)
        note = sc.get("note", {}).get(k) if phase not in ("start",) else None
        pan = panel(t, wr, sc["title"], k, len(sc["steps"]), phase, stance, note)
        frame = np.concatenate([img3, np.asarray(pan)], 1)
        if preview:
            Image.fromarray(frame).save(out)
        else:
            proc.stdin.write(frame.tobytes())
    if proc:
        proc.stdin.close(); proc.wait()
    print("wrote", out, len(sel), "frames", flush=True)


if __name__ == "__main__":
    main(sys.argv[1], "--preview" in sys.argv)
