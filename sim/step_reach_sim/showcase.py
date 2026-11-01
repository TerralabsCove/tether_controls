"""Short explainer clips for set designers: what the arm can do, in feet and metres.

Usage: python showcase.py <clip> [--preview]   (clips: see CLIPS at the bottom)
Writes videos/showcase/<clip>.mp4 (or a PNG contact sheet with --preview).
Comfortable = within the motors' rated (continuous) torque, from the sweeps in this folder."""
import os, sys, pickle, subprocess, numpy as np
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "osmesa")
import mujoco, trimesh, fcl
from PIL import Image, ImageDraw, ImageFont
from plan import *
import render as R

W, H, FPS = 1600, 900, 30
TOP, BAND = 92, 128               # title strip and caption band heights (px)
INTER = "/usr/share/fonts/opentype/inter/Inter-{}.otf"
INK, INK2, TEAL, PAPER = (27, 36, 48), (90, 98, 112), (18, 128, 130), (250, 250, 248)
DOWN, UP = (0, 0, -1.0), (0, 0, 1.0)
ARM_KG = R.ARM_KG
ARM_LB = ARM_KG / 0.45359237


def font(w, s):
    return ImageFont.truetype(INTER.format(w), s)


def ftin(m, mode="round"):
    inch = m / 0.0254
    n = int(np.floor(inch + 1e-9)) if mode == "floor" else int(round(inch))
    f, i = divmod(n, 12)
    return (f"{f} ft {i} in" if f and i else f"{f} ft" if f else f"{i} in")


def both(m, mode="round"):
    return f"{ftin(m, mode)} ({m:.2f} m)"


# ---------------------------------------------------------------- scene pieces
class Solid:
    """Collision + render object from a trimesh (e.g. the tapered tree trunk)."""
    def __init__(self, mesh, color, parts):
        self.mesh, self.color = mesh, color
        self.objs = parts
        v = mesh.vertices
        self.aabb = (v.min(0), v.max(0))


class Multi:
    """Planner obstacle made of several fcl objects (duck-types Box for Planner.collides)."""
    def __init__(self, objs, aabb):
        self.obj = fcl.CollisionObject(fcl.Box(1e-3, 1e-3, 1e-3), fcl.Transform())  # unused placeholder
        self.objs, self.aabb = objs, aabb


def trunk_radius(z):
    zs = [0.0, 0.914, 1.524, 2.083]          # 0, 3 ft, 5 ft, 6 ft 10 in
    rs = [0.457, 0.381, 0.305, 0.28]         # 36, 30, 24 in diameter; top assumed slightly narrower
    return np.interp(z, zs, rs)


def trunk(center=(0, 0), height=2.083, n=72):
    zs = np.linspace(0, height, 40)
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    V, F = [], []
    for z in zs:
        r = trunk_radius(z)
        V += [(center[0] + r * np.cos(t), center[1] + r * np.sin(t), z) for t in th]
    for i in range(len(zs) - 1):
        for j in range(n):
            a, b = i * n + j, i * n + (j + 1) % n
            F += [(a, b, b + n), (a, b + n, a + n)]
    top = len(V); V.append((center[0], center[1], height))
    F += [((len(zs) - 1) * n + j, (len(zs) - 1) * n + (j + 1) % n, top) for j in range(n)]
    mesh = trimesh.Trimesh(np.array(V), np.array(F))
    objs = []
    for z0, z1 in zip(zs[:-1], zs[1:]):
        r = trunk_radius(z0)
        objs.append(fcl.CollisionObject(fcl.Cylinder(r, z1 - z0), fcl.Transform(np.eye(3), [center[0], center[1], (z0 + z1) / 2])))
    return mesh, objs


def collides_patch():
    """Let Planner.collides handle obstacles with several fcl objects."""
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
                for o in getattr(b, "objs", [b.obj]):
                    if fcl.collide(M.cl[n]["obj"], o, fcl.CollisionRequest(), fcl.CollisionResult()):
                        return True
        return False
    Planner.collides = collides


collides_patch()


def person_geoms(pos, yaw=0.0):
    """A 1.75 m (5 ft 9 in) mannequin from capsules, standing at pos (feet)."""
    x, y, z = pos
    c, s = np.cos(yaw), np.sin(yaw)
    P = lambda dx, dy, dz: (x + c * dx - s * dy, y + s * dx + c * dy, z + dz)
    cap = lambda a, b, r: f'<geom type="capsule" fromto="{" ".join(f"{v:.3f}" for v in a + b)}" size="{r}" material="person" contype="0" conaffinity="0"/>'
    g = [cap(P(0, -0.09, 0.06), P(0, -0.09, 0.86), 0.065), cap(P(0, 0.09, 0.06), P(0, 0.09, 0.86), 0.065),
         cap(P(0, 0, 0.95), P(0, 0, 1.38), 0.17),
         cap(P(0, -0.23, 1.40), P(0.02, -0.27, 0.82), 0.045), cap(P(0, 0.23, 1.40), P(0.02, 0.27, 0.82), 0.045),
         f'<geom type="sphere" pos="{" ".join(f"{v:.3f}" for v in P(0, 0, 1.635))}" size="0.105" material="person" contype="0" conaffinity="0"/>']
    return "".join(g)


def build_model(P, sc, tmp):
    arm = P.M[EE_A].arm
    os.makedirs(tmp, exist_ok=True)

    def color(n):
        if "DM-J10010" in n: return "0.24 0.25 0.28 1"
        if "DM-J4340" in n: return "0.34 0.35 0.38 1"
        if "LiPo" in n: return "0.30 0.40 0.58 1"
        if "SlipRing" in n: return "0.55 0.56 0.60 1"
        if "EndEffector" in n: return "0.93 0.48 0.22 1"
        return "0.80 0.81 0.83 1"
    assets, bodies, world = [], [], []
    for i, (n, m) in enumerate(arm.link_mesh.items()):
        if m is None: continue
        fn = f"{tmp}/l{i}.stl"; m.export(fn)
        assets.append(f'<mesh name="m{i}" file="{fn}" inertia="shell"/>')
        bodies.append(f'<body name="b{i}" mocap="true"><geom type="mesh" mesh="m{i}" rgba="{color(n)}" contype="0" conaffinity="0"/></body>')
    dm = Arm(PKG + "/urdf/dock_female.urdf").load_meshes()["dock_female"]
    dm.export(f"{tmp}/dock.stl")
    assets.append(f'<mesh name="dock" file="{tmp}/dock.stl" inertia="shell"/>')
    for j, T in enumerate(sc["docks"]):
        q = np.zeros(4); mujoco.mju_mat2Quat(q, T[:3, :3].flatten())
        world.append(f'<geom name="dock{j}" type="mesh" mesh="dock" pos="{" ".join(map(str, T[:3, 3]))}" quat="{" ".join(map(str, q))}" '
                     f'rgba="{TEAL[0]/255:.3f} {TEAL[1]/255:.3f} {TEAL[2]/255:.3f} 1" contype="0" conaffinity="0"/>')
    for k, b in enumerate(sc["boxes"]):
        if isinstance(b, Box):
            q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(b.R, float).flatten())
            mat = b.__dict__.get("mat", "grid")
            world.append(f'<geom type="box" size="{" ".join(map(str, b.size / 2))}" pos="{" ".join(map(str, b.c))}" quat="{" ".join(map(str, q))}" material="{mat}" contype="0" conaffinity="0"/>')
    for k, s in enumerate(sc.get("solids", [])):
        fn = f"{tmp}/solid{k}.stl"; s.mesh.export(fn)
        assets.append(f'<mesh name="solid{k}" file="{fn}" inertia="shell"/>')
        world.append(f'<geom type="mesh" mesh="solid{k}" rgba="{s.color}" contype="0" conaffinity="0"/>')
    if sc.get("person") is not None:
        world.append(person_geoms(*sc["person"]))
    for p in sc.get("dots", []):   # reach-map dots, placed behind the arm
        world.append(f'<geom type="sphere" size="0.0085" pos="{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}" rgba="{TEAL[0]/255:.3f} {TEAL[1]/255:.3f} {TEAL[2]/255:.3f} 0.75" contype="0" conaffinity="0"/>')
    grid = 1 / (2 * 0.3048)   # checker squares of 1 ft
    xml = f"""<mujoco><compiler/><visual><global offwidth="{W}" offheight="{H}"/><quality shadowsize="4096"/>
      <headlight ambient="0.42 0.42 0.42" diffuse="0.45 0.45 0.45"/></visual>
      <asset><texture type="skybox" builtin="gradient" rgb1="0.985 0.985 0.98" rgb2="0.93 0.935 0.94" width="64" height="64"/>
      <texture name="g" type="2d" builtin="checker" rgb1="0.90 0.895 0.88" rgb2="0.85 0.845 0.83" width="256" height="256"/>
      <material name="grid" texture="g" texrepeat="{grid:.4f} {grid:.4f}" texuniform="true"/>
      <material name="plain" rgba="0.66 0.655 0.64 1"/>
      <material name="wood" rgba="0.74 0.70 0.64 1"/>
      <material name="person" rgba="0.70 0.71 0.73 1"/>{''.join(assets)}</asset>
      <worldbody><light directional="true" pos="1 -2 4" dir="-0.3 0.5 -1" diffuse="0.55 0.55 0.55" castshadow="true"/>
      {''.join(world)}{''.join(bodies)}</worldbody></mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    names = [n for n, m in arm.link_mesh.items() if m is not None]
    return model, names


# ---------------------------------------------------------------- camera
class Cam:
    def __init__(self, az, el, lookat, dist=None, ortho_h=None):
        self.az, self.el, self.lookat, self.dist, self.ortho_h = az, el, np.asarray(lookat, float), dist, ortho_h
        a, e = np.radians(az), np.radians(el)
        self.f = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        self.r = np.cross(self.f, [0, 0, 1.0]); self.r /= np.linalg.norm(self.r)
        self.u = np.cross(self.r, self.f)

    def project(self, p):
        p = np.asarray(p, float)
        if self.ortho_h:
            s = H / self.ortho_h
            d = p - self.lookat
            return np.array([W / 2 + (d @ self.r) * s, H / 2 - (d @ self.u) * s])
        pos = self.lookat - self.dist * self.f
        d = p - pos
        z = d @ self.f
        foc = (H / 2) / np.tan(np.radians(22.5))
        return np.array([W / 2 + (d @ self.r) / z * foc, H / 2 - (d @ self.u) / z * foc])

    def mj(self, model):
        c = mujoco.MjvCamera(); c.type = mujoco.mjtCamera.mjCAMERA_FREE
        c.azimuth, c.elevation = self.az, self.el
        c.lookat[:] = self.lookat
        if self.ortho_h:   # orthographic free camera: fovy is the view height in metres
            dist = 8.0
            depth = getattr(self, "dmin", -4.0) + dist - self.lookat @ self.f
            c.lookat[:] = self.lookat + depth * self.f
            model.vis.global_.orthographic = 1; model.vis.global_.fovy = self.ortho_h; c.distance = dist
        else:
            model.vis.global_.orthographic = 0; model.vis.global_.fovy = 45; c.distance = self.dist
        return c


def fit(pts, az, el, ortho, margin=60):
    pts = np.asarray(pts)
    box = (margin, TOP + 10, W - margin, H - BAND - 16)
    tgt = np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
    cam = Cam(az, el, pts.mean(0))
    u, v = pts @ cam.r, pts @ cam.u
    if ortho:
        s = min((box[2] - box[0]) / max(np.ptp(u), 1e-3), (box[3] - box[1]) / max(np.ptp(v), 1e-3))
        cu, cv = (u.min() + u.max()) / 2, (v.min() + v.max()) / 2
        look = cu * cam.r + cv * cam.u + (pts @ cam.f).mean() * cam.f
        look = look - (tgt[0] - W / 2) / s * cam.r + (tgt[1] - H / 2) / s * cam.u
        c = Cam(az, el, look, ortho_h=H / s); c.dmin = (pts @ cam.f).min() - 3.5
        return c
    look = cam.lookat.copy()
    for _ in range(3):
        lo, hi = 0.5, 30.0
        for _ in range(40):
            d = (lo + hi) / 2
            px = np.array([Cam(az, el, look, d).project(p) for p in pts])
            span = px.max(0) - px.min(0)
            if span[0] <= box[2] - box[0] and span[1] <= box[3] - box[1]: hi = d
            else: lo = d
        c = Cam(az, el, look, hi)
        px = np.array([c.project(p) for p in pts])
        mid = (px.max(0) + px.min(0)) / 2
        foc = (H / 2) / np.tan(np.radians(22.5))
        depth = hi
        look = look + (mid[0] - tgt[0]) / foc * depth * cam.r - (mid[1] - tgt[1]) / foc * depth * cam.u
    return Cam(az, el, look, hi)


# ---------------------------------------------------------------- overlays
def pill(d, xy, text, f, fg=INK, bg=(255, 255, 255, 235), anchor="mm", pad=(10, 5)):
    x0, y0, x1, y1 = d.textbbox(xy, text, font=f, anchor=anchor)
    d.rounded_rectangle((x0 - pad[0], y0 - pad[1], x1 + pad[0], y1 + pad[1]), radius=6, fill=bg)
    d.text(xy, text, font=f, fill=fg, anchor=anchor)


def dim_line(d, cam, p1, p2, text, off, f, color=INK):
    """Dimension line between world points p1, p2, drawn offset by world vector off, with extension lines."""
    p1, p2, off = map(lambda v: np.asarray(v, float), (p1, p2, off))
    a, b = cam.project(p1), cam.project(p2)
    a2, b2 = cam.project(p1 + off), cam.project(p2 + off)
    for s, e in ((a, a2), (b, b2)):
        v = e - s; L = np.linalg.norm(v)
        if L > 1: d.line([tuple(s + v / L * 6), tuple(e + v / L * 8)], fill=color + (150,), width=1)
    d.line([tuple(a2), tuple(b2)], fill=color + (255,), width=2)
    v = b2 - a2; L = np.linalg.norm(v)
    if L > 1:
        n = np.array([-v[1], v[0]]) / L
        for q in (a2, b2):
            d.line([tuple(q - 7 * n - 7 * v / L), tuple(q + 7 * n + 7 * v / L)], fill=color + (255,), width=2)
    pill(d, tuple((a2 + b2) / 2), text, f)


def ruler(d, cam, origin, axis, length, f, ticks_ft=1, label_side=(0, -1)):
    """Ruler along a world axis from origin, ticks every foot, labels in ft and m."""
    o, ax = np.asarray(origin, float), np.asarray(axis, float)
    a, b = cam.project(o), cam.project(o + ax * length)
    d.line([tuple(a), tuple(b)], fill=INK2 + (200,), width=2)
    v = (b - a) / max(np.linalg.norm(b - a), 1); n = np.array([-v[1], v[0]])
    k = 0
    while k * 0.3048 <= length + 1e-6:
        p = cam.project(o + ax * k * 0.3048)
        d.line([tuple(p - 6 * n), tuple(p + 6 * n)], fill=INK2 + (220,), width=2)
        if k:
            q = p + np.array(label_side) * 18
            d.text(tuple(q), f"{k} ft", font=f, fill=INK2, anchor="mm")
        k += ticks_ft
    k = 1
    while k * 0.5 <= length + 1e-6:
        p = cam.project(o + ax * k * 0.5)
        d.line([tuple(p - 3 * n), tuple(p + 3 * n)], fill=INK2 + (160,), width=1)
        q = p - np.array(label_side) * 16
        d.text(tuple(q), f"{k * 0.5:.1f} m", font=f, fill=INK2 + (180,), anchor="mm")
        k += 1


def caption_layer(title, num, text, alpha, footer):
    img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.text((56, 44), num, font=font("SemiBold", 26), fill=TEAL, anchor="lm")
    d.text((56 + d.textlength(num, font=font("SemiBold", 26)) + 14, 44), title, font=font("SemiBold", 26), fill=INK, anchor="lm")
    d.rectangle((0, H - BAND, W, H), fill=(255, 255, 255, 236))
    d.line((0, H - BAND, W, H - BAND), fill=(220, 222, 226, 255), width=1)
    if text:
        f = font("Medium", 32)
        lines, line = [], ""
        for w_ in text.split():
            if d.textlength((line + " " + w_).strip(), font=f) > W - 112:
                lines.append(line); line = w_
            else:
                line = (line + " " + w_).strip()
        lines.append(line)
        y = H - BAND / 2 - 10 - (len(lines) - 1) * 20
        for ln in lines:
            d.text((56, y), ln, font=f, fill=INK + (int(255 * alpha),), anchor="lm"); y += 40
    d.text((W - 56, H - 18), footer, font=font("Regular", 17), fill=INK2, anchor="rs")
    return img


# ---------------------------------------------------------------- timeline
def retime(frames, sc):
    """Slow the motion 2x and set hold lengths so captions can be read."""
    hold = dict(start=1, docked=sc.get("hold_docked", 36), done=1)
    out, i = [], 0
    while i < len(frames):
        j = i
        while j + 1 < len(frames) and frames[j + 1][4] == frames[i][4] and frames[j + 1][3] == frames[i][3] and frames[i][4] in hold:
            j += 1
        fr = frames[i]
        if fr[4] in hold:
            n = hold[fr[4]]
            out += [fr] * n
        elif fr[4] == "return":
            if i % 3: out.append(fr)
        else:
            a, b = fr, frames[i + 1] if i + 1 < len(frames) else fr
            out.append(a)
            if a[4] == b[4] and a[3] == b[3] and not sc.get("reach_map"):
                dd = (b[2] - a[2] + np.pi) % (2 * np.pi) - np.pi
                out.append((a[0], a[1], a[2] + 0.5 * dd, a[3], a[4]))
        i = j + 1
    # caption timeline: start captions get 4 s each, end captions 4.5 s each
    st = [t for t in sc["captions"] if t[0] == "start"]
    en = [t for t in sc["captions"] if t[0] == "end"]
    first, last = out[0], out[-1]
    tl = []
    for _, txt in st:
        tl += [(first, txt)] * int(sc.get("start_s", 4.0) * FPS)
    caps = {t[0]: t[1] for t in sc["captions"] if isinstance(t[0], int)}
    cur = st[-1][1] if st else ""
    for fr in out:
        if fr[3] in caps and fr[4] not in ("start",):
            cur = caps[fr[3]]
        tl.append((fr, cur))
    for _, txt in en:
        tl += [(last, txt)] * int(sc.get("end_s", 4.5) * FPS)
    if not en:
        tl += [(last, cur)] * int(2.5 * FPS)
    return tl


# ---------------------------------------------------------------- clips
def floor_box(x0=-3, x1=4, y0=-2.5, y1=2.5, top=0.0, thick=0.1, mat="grid"):
    b = Box(((x0 + x1) / 2, (y0 + y1) / 2, top - thick / 2), (x1 - x0, y1 - y0, thick))
    b.mat = mat
    return b


def solid_box(c, size, R=np.eye(3)):
    b = Box(c, size, R); b.mat = "plain"
    return b


def pedestal():
    b = Box((-0.30, 0, -0.5), (0.70, 1.0, 1.0)); b.mat = "plain"
    return b


def ground(x0=-3, x1=4, top=0.0):
    """Side-view ground: a solid band, so it reads as ground in an elevation drawing."""
    return floor_box(x0, x1, -1.5, 1.5, top, 0.22, "plain")


FOOTER = "Simulation. Speed is not real.   Arm 31.6 lb (14.35 kg)   Grid squares 1 ft"


def clips():
    S = {}
    # 1 how it moves
    sp = 0.50
    docks = [dock_pose((i * sp, 0, 0), DOWN) for i in range(5)]
    S["01_how_it_moves"] = dict(
        num="01", title="How it moves", boxes=[floor_box()], docks=docks, start=(EE_B, 0, EE_A, 1),
        steps=[(EE_B, 2), (EE_A, 3), (EE_B, 4)], person=((-0.75, 0.9, 0), 0.0),
        cam=dict(az=70, el=-9),
        captions=[("start", "The robot only moves between docks. The teal plates are docks fixed to the set."),
                  ("start", "Both ends are the same. Each one is a gripper that locks into a dock."),
                  (0, "One gripper holds on. The other lifts out, swings over, and locks into the next dock."),
                  (1, "Then they swap roles. The robot walks end over end, like an inchworm."),
                  (2, f"While it steps, one dock holds the whole robot: {ARM_LB:.1f} lb ({ARM_KG:.2f} kg)."),
                  ("end", "So the set design comes down to where the docks go, and what they are fixed to.")],
        labels=[("start", 4, "dock")])
    # 2 longest comfortable step
    sp = 0.58
    docks = [dock_pose((i * sp, 0, 0), DOWN) for i in range(5)]
    dims = []
    for k in range(3):
        a, b = k + 1, k + 2
        dims.append(dict(after=k, until=k, p1=(a * sp, 0, 0), p2=(b * sp, 0, 0), off=(0, 0, -0.16), text=both(sp)))
    dims.append(dict(after=-1, until=-1, p1=(0, 0, 0), p2=(sp, 0, 0), off=(0, 0, -0.16), text=both(sp)))
    S["02_longest_comfortable_step"] = dict(
        num="02", title="Longest comfortable step", boxes=[ground(-1.5, 3.2)], docks=docks, start=(EE_B, 0, EE_A, 1),
        steps=[(EE_B, 2), (EE_A, 3), (EE_B, 4)], person=((-0.75, 0.6, 0), 0.0), ortho=True,
        cam=dict(az=90, el=0), dims=dims, rise=True, extra=[(2.65, 0, 0.85)],
        captions=[("start", f"Longest comfortable step on a flat surface: {both(0.58)} between dock centres."),
                  (0, "Comfortable means the motors are not pushed to their limit. Longer steps are possible, but with no margin left."),
                  (2, "Docks on a floor or platform can be anywhere up to this distance apart."),
                  ("end", "The absolute limit is 2 ft 7 in (0.80 m). Design to 1 ft 11 in."),
                  ("end", "During a step the arm rises about {rise} above the docks. Keep that space clear.")])
    # 3 reach map
    tg = [(0.58, 0.0), (0.62, 0.40), (0.40, 0.54), (0.44, -0.40), (0.44, -0.66)]
    docks = [dock_pose((0, 0, 0), DOWN), dock_pose((0.30, 0, 0.0), DOWN)] + [dock_pose((r, 0, h), DOWN) for r, h in tg]
    S["03_where_the_next_dock_can_go"] = dict(
        num="03", title="Where the next dock can go", ortho=True,
        boxes=[pedestal(), ground(-2, 1.4, top=-1.0)],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_A, 2 + i) for i in range(len(tg))],
        dock_show={i + 2: i for i in range(len(tg))}, dock_hide={1: -1, **{i + 2: i + 1 for i in range(len(tg))}}, hold_docked=75,
        person=((-0.95, 0.6, -1.0), 0.0), cam=dict(az=90, el=0), reach_map=True, extra=[(0.92, 0, 0.75), (0.92, 0, -1.0)],
        dims=[dict(after=i, until=i, p1=(0, 0, 0), p2=(r, 0, 0) if True else None, off=(0, 0, -0.10 if h >= 0 else 0.10), text=both(r)) for i, (r, h) in enumerate(tg)]
             + [dict(after=i, until=i, p1=(r, 0, 0), p2=(r, 0, h), off=(0.14, 0, 0), text=both(abs(h))) for i, (r, h) in enumerate(tg) if h],
        captions=[("start", "The robot holds the dock on the left. Where can the next dock go?"),
                  ("start", "Every teal dot is a comfortable spot for the centre of the next dock. The dashed line is the absolute limit."),
                  (0, f"Level: up to {both(0.58)} away."),
                  (1, f"Up {both(0.40)}: up to about {both(0.62)} away."),
                  (2, f"Highest: {both(0.54)} up, with the dock no more than {both(0.44)} forward."),
                  (3, f"Down {both(0.40)}: up to about {both(0.44)} away. Stepping down is harder than stepping up."),
                  (4, f"Lowest: {both(0.66)} down, {both(0.44)} away."),
                  ("end", "This shape works the same on the floor, a platform, or a ledge. Only the dock positions matter.")])
    # 4 steps up and down
    docks = [dock_pose(p, DOWN) for p in [(0, 0, 0), (0.45, 0, 0), (0.95, 0, 0.40), (1.45, 0, 0.80), (1.89, 0, 0.40)]]
    S["04_steps_up_and_down"] = dict(
        num="04", title="Steps up and down", ortho=True,
        boxes=[ground(-1.5, 3.0), solid_box((1.55, 0, 0.20), (1.30, 1.0, 0.40)), solid_box((1.45, 0, 0.60), (0.45, 1.0, 0.40))],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3), (EE_B, 4)],
        person=((-0.75, 0.6, 0), 0.0), cam=dict(az=90, el=0),
        dims=[dict(after=0, until=0, p1=(0.45, 0, 0), p2=(0.95, 0, 0), off=(0, 0, -0.16), text=both(0.50)),
              dict(after=0, until=0, p1=(0.95, 0, 0), p2=(0.95, 0, 0.40), off=(0.30, 0, 0), text=both(0.40)),
              dict(after=2, until=2, p1=(1.45, 0, 0.80), p2=(1.89, 0, 0.80), off=(0, 0, 0.30), text=both(0.44)),
              dict(after=2, until=2, p1=(1.89, 0, 0.80), p2=(1.89, 0, 0.40), off=(0.25, 0, 0), text=both(0.40))],
        captions=[("start", "Steps up and down, for example onto rocks or platforms."),
                  (0, f"Up {both(0.40)} and {both(0.50)} forward: comfortable."),
                  (1, "Same again. Each platform needs its own dock."),
                  (2, f"Down {both(0.40)}: keep it to about {both(0.44)} forward. Down is harder than up."),
                  ("end", f"Biggest step up: {both(0.54)}. Biggest step down: {both(0.66)}. Both with the docks up to {ftin(0.44)} apart.")])
    # 5 slope
    a = np.radians(30); Rm = R.Ry(-a); u = Rm @ [1, 0, 0]; nout = Rm @ [0, 0, 1]
    sp = 0.60
    docks = [dock_pose(s * u, -nout) for s in np.arange(5) * sp]
    S["05_up_a_slope"] = dict(
        num="05", title="Up a 30° slope", boxes=[Box(1.2 * u - 0.1 * nout + (0, 0, 0), (4.4, 2.4, 0.2), Rm), floor_box(-3, 0.3)],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3), (EE_B, 4)],
        person=((-0.9, 0.9, 0), 0.0), cam=dict(az=72, el=-10),
        dims=[dict(after=k, until=k, p1=docks[k + 1][:3, 3], p2=docks[k + 2][:3, 3], off=-0.16 * nout, text=both(sp)) for k in range(3)],
        captions=[("start", "Up a 30° slope, for example along a large root or a ramp."),
                  (0, f"Straight up or down the slope: steps up to {both(0.70, 'floor')}."),
                  (2, f"Across the slope is much harder: keep sideways steps under {both(0.26, 'floor')}."),
                  ("end", "On a slope, line the docks up along the direction of climb.")])
    # 6 vertical
    X = (1, 0, 0)
    zs = [0.35, 1.05, 1.75, 2.45]
    docks = [dock_pose((0, 0, z), X) for z in zs]
    S["06_straight_up_a_vertical_surface"] = dict(
        num="06", title="Straight up a vertical surface", boxes=[floor_box(-3, 0), solid_box((0.2, 0, 1.6), (0.4, 3.0, 3.2))],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3)],
        person=((-1.1, 0.75, 0), 0.0), cam=dict(az=58, el=-6),
        dims=[dict(after=k, until=k, p1=docks[k + 1][:3, 3], p2=docks[k + 2][:3, 3], off=(0, -0.28, 0), text=both(0.70)) for k in range(2)]
             + [dict(after=-1, until=-1, p1=docks[0][:3, 3], p2=docks[1][:3, 3], off=(0, -0.28, 0), text=both(0.70))],
        captions=[("start", "Straight up a vertical surface, for example a trunk or the back wall."),
                  (0, f"Docks in a vertical line, up to {both(0.80)} apart. This is easy for the motors."),
                  (1, f"The holding dock now carries the whole {ARM_LB:.0f} lb sideways, and the arm's weight tries to pry it off the wall."),
                  ("end", f"Sideways on a vertical surface is the weak spot: keep sideways offsets under {both(0.14, 'floor')}.")])
    # 7 floor -> wall -> over the top
    M_ = (1, 0, 0)
    docks = [dock_pose((-1.05, 0, 0), DOWN), dock_pose((-0.55, 0, 0), DOWN), dock_pose((0, 0, 0.35), M_),
             dock_pose((0, 0, 1.10), M_), dock_pose((0.20, 0, 1.30), DOWN), dock_pose((0.60, 0, 1.30), DOWN)]
    S["07_floor_to_wall_and_over_the_top"] = dict(
        num="07", title="Onto a wall and over the top", boxes=[floor_box(-3, 0.0), solid_box((0.6, 0, 0.65), (1.2, 2.4, 1.30))],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3), (EE_B, 4), (EE_A, 5)], max_peak=1.0,
        person=((-1.6, 0.9, 0), 0.0), cam=dict(az=68, el=-8),
        dims=[dict(after=0, until=0, p1=(-0.55, 0, 0), p2=(0, 0, 0), off=(0, -0.3, 0), text=both(0.55)),
              dict(after=0, until=0, p1=(0, 0, 0), p2=(0, 0, 0.35), off=(0, -0.3, 0), text=both(0.35)),
              dict(after=2, until=3, p1=(0, 0, 1.10), p2=(0, 0, 1.30), off=(-0.25, -0.2, 0), text=both(0.20)),
              dict(after=2, until=3, p1=(0, 0, 1.30), p2=(0.20, 0, 1.30), off=(0, -0.2, 0.30), text=both(0.20))],
        captions=[("start", "From the floor onto a vertical face, then over the top edge."),
                  (0, f"First wall dock about {ftin(0.35)} up. The last floor dock {ftin(0.40)} to {ftin(0.84, 'floor')} from the wall."),
                  (1, "Then straight up the face."),
                  (2, f"Going over an edge is the hardest move. Docks within about {ftin(0.20)} of the edge, on both sides."),
                  (3, "This move also puts the biggest prying load on a dock, about 70 lbf·ft (95 Nm)."),
                  ("end", f"The wall here is {both(1.30)} tall. The same rules work for any height.")])
    # 8 tree concept
    mesh, objs = trunk()
    tree = Multi(objs, (mesh.vertices.min(0) - 0.01, mesh.vertices.max(0) + 0.01))
    tsolid = Solid(mesh, "0.74 0.70 0.64 1", objs)
    rise, chord = 0.30, 0.46
    docks, shelves, th, z = [], [], np.radians(-100), 0.0
    for i in range(8):
        zz = 0.0 if i < 2 else (i - 1) * rise
        rho = trunk_radius(zz) + 0.21
        p = (rho * np.cos(th), rho * np.sin(th), zz)
        docks.append(dock_pose(p, DOWN, xhint=(-np.sin(th), np.cos(th), 0)))
        if zz > 0:
            Rz_ = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
            depth = 0.36
            c = np.array([(trunk_radius(zz) + depth / 2 - 0.04) * np.cos(th), (trunk_radius(zz) + depth / 2 - 0.04) * np.sin(th), zz - 0.03])
            b = Box(c, (depth, 0.30, 0.06), Rz_); b.mat = "wood"; shelves.append(b)
        rho_next = trunk_radius(zz + rise) + 0.21
        th += 2 * np.arcsin(min(0.99, chord / (rho + rho_next)))
    S["08_concept_tree_spiral"] = dict(
        num="08", title="Concept: platforms spiralling up the tree", boxes=[floor_box(-3, 3, -3, 3), tree] + shelves,
        solids=[tsolid], docks=docks, start=(EE_B, 0, EE_A, 1), steps=[((EE_B, EE_A)[k % 2], k + 2) for k in range(6)],
        person=None, cam=dict(el=-16, follow=True),
        extra=[(trunk_radius(z) * np.cos(t), trunk_radius(z) * np.sin(t), z) for z in (0, 1.0, 2.083) for t in np.linspace(0, 2 * np.pi, 12)]
              + [(0.95 * np.cos(t), 0.95 * np.sin(t), -0.12) for t in np.linspace(0, 2 * np.pi, 12)],
        dims=[dict(after=-1, until=99, p1=(0, 0, 0), p2=(0, 0, 2.083), off=(0, 0, 0), text="6 ft 10 in (2.08 m)", tree=True)],
        captions=[("start", "Concept only. One way the robot could live on the 6 ft 10 in tree, not a decision."),
                  ("start", "Small platforms spiral up the trunk. Each one has a dock on top."),
                  (0, f"Each platform is {both(rise)} higher and about {ftin(chord)} around from the last."),
                  (2, "The robot stays on top of the platforms, which is easier on the gripper than hanging off the bark."),
                  (4, "Each platform must bolt into the tree's wooden frame, not the plaster shell. It carries the whole robot."),
                  ("end", f"Per platform: {ARM_LB:.0f} lbf down plus about 42 lbf·ft (57 Nm) of prying, before any safety factor.")])
    # 9 hanging under
    hz = 2.25
    sp = 0.55
    docks = [dock_pose((i * sp, 0, hz), UP) for i in range(5)]
    S["09_hanging_underneath"] = dict(
        num="09", title="Hanging underneath (needs testing first)", boxes=[floor_box(), solid_box((1.1, 0, hz + 0.1), (4.6, 0.6, 0.2))],
        docks=docks, start=(EE_B, 0, EE_A, 1), steps=[(EE_B, 2), (EE_A, 3), (EE_B, 4)],
        person=((-0.9, 0.7, 0), 0.0), cam=dict(az=75, el=4), drop=hz, extra=[(-0.25, 0, hz + 0.25), (2.6, 0, hz + 0.25)],
        dims=[dict(after=-1, until=99, p1=(-0.25, 0, 0), p2=(-0.25, 0, hz), off=(-0.15, 0, 0), text=both(hz))]
             + [dict(after=k, until=k, p1=docks[k + 1][:3, 3], p2=docks[k + 2][:3, 3], off=(0, 0, 0.18), text=both(sp)) for k in range(3)],
        captions=[("start", "Hanging under a horizontal surface, for example a thick branch."),
                  (0, f"The motors can do it: comfortable steps up to {both(0.58)}."),
                  (1, f"But the dock is pulled straight down by the whole robot, {ARM_LB:.0f} lbf (141 N), and must not let go."),
                  (2, "The arm hangs down to about {drop} below the surface. Keep visitors clear underneath."),
                  ("end", "We have not tested the gripper for this yet. Treat it as a maybe until we do.")])
    return S


# ---------------------------------------------------------------- render
def reach_masks():
    from plot import load
    d = load(EE_B)
    RS, HS = d["RS"], d["HS"]
    ku = np.zeros_like(d["selfok"]); ku[:-6] = d["selfok"][6:]   # approach pose 12 cm above the dock must be reachable too
    comfy = (d["feas"] & (d["rated"][:, 1] <= 1)).any(0) & ku
    hard = (d["feas"][0] | d["feas"][1]) & ku
    return RS, HS, comfy, hard


def arm_points(P, frames, names, step=3):
    pts = []
    for stance, Tws, x6, k, ph in frames[::step]:
        X = P.frames(stance, x6, Tws)
        pts += [X[n][:3, 3] for n in names]
    return np.array(pts)


def max_rise(P, frames, names, ref_z=0.0):
    arm = P.M[EE_A].arm
    sub = {n: arm.link_mesh[n].vertices[::40] for n in names}
    top = -1e9
    for stance, Tws, x6, k, ph in frames[::2]:
        X = P.frames(stance, x6, Tws)
        for n in names:
            v = sub[n] @ X[n][:3, :3].T + X[n][:3, 3]
            top = max(top, v[:, 2].max())
    return top - ref_z


def plan_reach(sc):
    """Reach map: plan home -> target for each target separately, then play it forward and back."""
    docks, frames = sc["docks"], []
    for i in range(len(docks) - 2):
        sub = dict(sc, steps=[(EE_A, 2 + i)])
        for seed in range(8):
            try:
                P, fr = R.plan_scenario(sub, seed)
                break
            except RuntimeError as e:
                print("  target", i, "seed", seed, "failed:", e, flush=True)
        else:
            raise RuntimeError(f"target {i} failed")
        if not frames:
            frames += [f for f in fr if f[4] == "start"][:1]
        move = [(st, T, x, i, ph) for st, T, x, k, ph in fr if ph in ("lift out of dock", "swing", "lower into dock")]
        X = P.frames(move[-1][0], move[-1][2], move[-1][1])
        t = P.M[EE_B].ratios(P.load(EE_B, X, move[-1][2])[0])[0]
        print(f"  target {i}: {len(move)} frames, rated ratio at target {t:.2f}", flush=True)
        frames += move + [(move[-1][0], move[-1][1], move[-1][2], i, "docked")]
        frames += [(st, T, x, i, "return") for st, T, x, k, ph in move[::-1]]
    frames += [frames[-1][:4] + ("done",)]
    return P, frames


def main(key, preview=False):
    sc = clips()[key]
    os.makedirs("plans/showcase", exist_ok=True)
    cache = f"plans/showcase/{key}.pkl"
    rend = dict(sc); rend.setdefault("captions", [])
    if os.path.exists(cache):
        frames = pickle.load(open(cache, "rb")); P = Planner(sc["boxes"], sc.get("max_peak"))
    else:
        if sc.get("reach_map"):
            P, frames = plan_reach(sc)
        else:
            for seed in range(sc.get("seeds", 6)):
                try:
                    P, frames = R.plan_scenario(sc, seed)
                    break
                except RuntimeError as e:
                    print("  seed", seed, "failed:", e, flush=True)
            else:
                raise RuntimeError("planning failed for all seeds")
        pickle.dump(frames, open(cache, "wb"))
    # rated check at each docked pose
    for fr in frames:
        if fr[4] == "docked" or fr[4] == "start":
            pass
    if sc.get("reach_map"):
        RS_, HS_, comfy_, hard_ = reach_masks()
        sc["dots"] = [(r, 0.30, h) for i, h in enumerate(HS_) for j, r in enumerate(RS_) if comfy_[i, j] and i % 2 == 0 and j % 2 == 0]
    model, names = build_model(P, sc, os.path.abspath(f"mjtmp/showcase_{key}"))
    if sc.get("rise"):
        rise = max_rise(P, frames, names, 0.0)
        sc.setdefault("dims", []).append(dict(after=0, until=99, done_only=True, p1=(2.62, 0, 0), p2=(2.62, 0, rise), off=(0, 0, 0),
                                              text=f"arm rises {ftin(np.ceil(rise / 0.0254) * 0.0254)}"))
        sc["captions"] = [(t, s.replace("{rise}", ftin(np.ceil(rise / 0.0254) * 0.0254) + f" ({rise:.2f} m)")) for t, s in sc["captions"]]
        print("max rise", rise)
    if sc.get("drop"):
        arm = P.M[EE_A].arm
        sub = {n: arm.link_mesh[n].vertices[::40] for n in names}
        zmin = min((sub[n] @ X[n][:3, :3].T + X[n][:3, 3])[:, 2].min()
                   for st, T, x6, k, ph in frames[::2] for X in [P.frames(st, x6, T)] for n in names)
        drop = sc["drop"] - zmin
        sc["captions"] = [(t, c_.replace("{drop}", f"{ftin(np.ceil(drop / 0.0254) * 0.0254)} ({drop:.2f} m)")) for t, c_ in sc["captions"]]
        print("hangs down", drop)
    tl = retime(frames, sc)
    # camera
    pts = list(arm_points(P, frames, names)) + [T[:3, 3] for T in sc["docks"]] + [np.array(p) for p in sc.get("extra", [])]
    if sc.get("person"):
        (px, py, pz), _ = sc["person"]
        pts += [np.array([px, py, pz]), np.array([px, py, pz + 1.95])]
    for dm in sc.get("dims", []):
        pts += [np.asarray(dm["p1"]) + np.asarray(dm["off"]), np.asarray(dm["p2"]) + np.asarray(dm["off"])]
    c = sc["cam"]
    if c.get("follow"):
        th = []
        for (stance, Tws, x6, k, phase), _ in tl:
            X = P.frames(stance, x6, Tws)
            m = (X[EE_A][:3, 3] + X[EE_B][:3, 3]) / 2
            th.append(np.arctan2(m[1], m[0]))
        th = np.unwrap(np.array(th))
        ker = np.ones(91) / 91
        th = np.convolve(np.pad(th, 45, mode="edge"), ker, mode="valid")
        follow_az = np.degrees(th) + 180 - 25
        fits = [fit(pts, a_, c["el"], False) for a_ in np.linspace(follow_az.min(), follow_az.max(), 6)]
        dist = max(cc.dist for cc in fits); look = np.array([0, 0, np.mean([cc.lookat[2] for cc in fits])])
    if c.get("orbit"):
        cams_fit = [fit(pts, c["az"] + a, c["el"], False) for a in np.linspace(0, c["orbit"], 5)]
        dist = max(cc.dist for cc in cams_fit); look = np.mean([cc.lookat for cc in cams_fit], 0)
    elif not c.get("follow"):
        cam0 = fit(pts, c["az"], c["el"], sc.get("ortho", False))
    data = mujoco.MjData(model)
    rr = mujoco.Renderer(model, H, W)
    fsm, fdim, flab = font("Medium", 17), font("SemiBold", 22), font("Medium", 18)
    gid = {j: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"dock{j}") for j in range(len(sc["docks"]))}
    rgba0 = {j: model.geom_rgba[g].copy() for j, g in gid.items()}
    if sc.get("reach_map"):
        RS, HS, comfy, hard = reach_masks()
        env_r = []
        for i, h in enumerate(HS):
            js = np.where(hard[i])[0]
            env_r.append((RS[js].max(), h) if len(js) else None)
    os.makedirs("videos/showcase", exist_ok=True)
    out = f"videos/showcase/{key}.mp4"
    proc = None
    if not preview:
        proc = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
                                 "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "19", "-movflags", "+faststart", out], stdin=subprocess.PIPE)
    sel = range(len(tl)) if not preview else np.linspace(0, len(tl) - 1, 8).astype(int)
    prev_txt, fade = None, 0
    sheet = []
    for idx in sel:
        (stance, Tws, x6, k, phase), txt = tl[idx]
        if txt != prev_txt:
            fade = 0; prev_txt = txt
        fade = 9 if preview else min(fade + 1, 9)
        if c.get("follow"):
            cam = Cam(follow_az[idx], c["el"], look, dist)
        elif c.get("orbit"):
            cam = Cam(c["az"] + c["orbit"] * idx / max(len(tl) - 1, 1), c["el"], look, dist)
        else:
            cam = cam0
        for j, g in gid.items():
            show = sc.get("dock_show", {}).get(j, -2) <= k if j in sc.get("dock_show", {}) else True
            if j in sc.get("dock_hide", {}) and k >= sc["dock_hide"][j]:
                show = False
            model.geom_rgba[g] = rgba0[j] if show else [0, 0, 0, 0]
        X = P.frames(stance, x6, Tws)
        for i, n in enumerate(names):
            q = np.zeros(4); mujoco.mju_mat2Quat(q, X[n][:3, :3].flatten())
            data.mocap_pos[i] = X[n][:3, 3]; data.mocap_quat[i] = q
        mujoco.mj_forward(model, data)
        rr.update_scene(data, cam.mj(model))
        img = Image.fromarray(rr.render()).convert("RGBA")
        ov = Image.new("RGBA", (W, H), (0, 0, 0, 0)); d = ImageDraw.Draw(ov)
        if sc.get("reach_map"):
            pl = [cam.project((r, 0, h)) for e in env_r if e for r, h in [e]]
            for a, b in zip(pl[::2], pl[1::2]):
                d.line([tuple(a), tuple(b)], fill=INK2 + (200,), width=2)
            for z_ in (0.5, -0.5):
                pass
        for dm in sc.get("dims", []):
            if dm.get("done_only") and phase != "done":
                continue
            if dm.get("tree"):
                side = -cam.r * (trunk_radius(0) + 0.35)
                dim_line(d, cam, side, side + (0, 0, 2.083), dm["text"], (0, 0, 0), fdim)
                continue
            if dm["after"] <= k <= dm["until"] and (phase in ("docked", "done", "start") or dm.get("tree")):
                if dm["after"] == -1 and k >= 0 and dm["until"] == -1:
                    continue
                dim_line(d, cam, dm["p1"], dm["p2"], dm["text"], dm["off"], fdim)
        if sc.get("person"):
            (px, py, pz), _ = sc["person"]
            p = cam.project((px, py, pz + 1.86))
            d.text(tuple(p), "5 ft 9 in person", font=flab, fill=INK2, anchor="ms")
        for when, j, lab in sc.get("labels", []):
            if when == "start" and k < 0:
                p = cam.project(sc["docks"][j][:3, 3])
                d.line([tuple(p), tuple(p + (0, -70))], fill=INK + (255,), width=2)
                pill(d, tuple(p + (0, -84)), lab, fdim)
        img = Image.alpha_composite(img, ov)
        img = Image.alpha_composite(img, caption_layer(sc["title"], sc["num"], txt, fade / 9, FOOTER))
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
        g.save(f"videos/showcase/{key}_preview.png"); print("preview", key)


if __name__ == "__main__":
    main(sys.argv[1], "--preview" in sys.argv)
