"""Dimensioned side view of the Tether arm, laid out straight (the URDF zero pose).

Usage: python dimensions.py
Writes videos/showcase/11_dimensions.png and prints the numbers it drew.
Module split: the two roll joints are the module boundaries (middle module = middle pitch motor and
batteries; each end module = roll, end pitch, gripper spin and gripper). Lengths are along the arm axis;
cross-sections are the largest extent of each module's meshes."""
import os, sys, numpy as np
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "osmesa")
import mujoco
from PIL import Image, ImageDraw, ImageFont
from model import Arm, T_of
from step_reach import DOCKS, EE_A, EE_B

W, H = 2400, 1010
PAPER, INK, INK2, MUTED, TEAL = (247, 248, 247), (27, 36, 48), (79, 88, 102), (125, 133, 146), (18, 128, 127)
INTER = "/usr/share/fonts/opentype/inter/Inter-{}.otf"
f = lambda w, s: ImageFont.truetype(INTER.format(w), s)


def ftin(m):
    n = int(round(m / 0.0254)); a, b = divmod(n, 12)
    return f"{a} ft {b} in" if a and b else f"{a} ft" if a else f"{b} in"


def both(m):
    return f"{ftin(m)} ({m:.2f} m)"


arm = Arm(); meshes = arm.load_meshes()
# lay the arm along world +x (arm axis = local z), local x up, local y away from the viewer
R = np.column_stack([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])    # columns: images of local x, y, z
assert abs(np.linalg.det(R) - 1) < 1e-9
Tb = np.eye(4); Tb[:3, :3] = R
X = arm.fk({}, Tb)
J = {j["name"]: j for j in arm.joints}
jpos = {n: (X[J[n]["parent"]] @ J[n]["T"])[:3, 3] for n in ("Revolute_44", "Revolute_73", "Revolute_57", "Revolute_29", "Revolute_22")}
dock = {e: (X[e] @ T_of(np.array(DOCKS[e][0]), np.array(DOCKS[e][1])))[:3, 3] for e in (EE_A, EE_B)}

kids = {}
for j in arm.joints:
    kids.setdefault(j["parent"], []).append(j["child"])


def sub(n):
    out, st = set(), [n]
    while st:
        c = st.pop(); out.add(c); st.extend(kids.get(c, []))
    return out


groups = {"end_hi": sub(J["Revolute_57"]["child"]), "end_lo": sub(J["Revolute_73"]["child"])}
groups["middle"] = set(arm.links) - groups["end_hi"] - groups["end_lo"]
V = {k: np.vstack([meshes[n].vertices @ X[n][:3, :3].T + X[n][:3, 3] for n in g if meshes.get(n) is not None]) for k, g in groups.items()}
mass = {k: sum(arm.links[n]["mass"] for n in g) for k, g in groups.items()}
allV = np.vstack(list(V.values()))
x_lo, x_hi = allV[:, 0].min(), allV[:, 0].max()
# axial positions (world x) of the features
xb = {"tip_lo": x_lo, "tip_hi": x_hi,
      "roll_lo": jpos["Revolute_73"][0], "roll_hi": jpos["Revolute_57"][0],
      "pitch_lo": jpos["Revolute_29"][0], "pitch_mid": jpos["Revolute_44"][0], "pitch_hi": jpos["Revolute_22"][0],
      "dock_lo": min(d[0] for d in dock.values()), "dock_hi": max(d[0] for d in dock.values())}
sec = {k: (np.ptp(v[:, 2]), np.ptp(v[:, 1])) for k, v in V.items()}          # (height in view, depth)
grip = np.vstack([meshes[e].vertices @ X[e][:3, :3].T + X[e][:3, 3] for e in (EE_A, EE_B)])

# ---------------------------------------------------------------- render (orthographic side view)
s = 0.80 * W / (x_hi - x_lo)              # px per metre
cx_w = (x_lo + x_hi) / 2
zc = (allV[:, 2].min() + allV[:, 2].max()) / 2
CY = 470                                  # image row of the arm's centre
geoms = []
assets = []
os.makedirs("mjtmp/dims", exist_ok=True)


def color(n):
    if "DM-J10010" in n: return "0.24 0.25 0.28 1"
    if "DM-J4340" in n: return "0.34 0.35 0.38 1"
    if "LiPo" in n: return "0.30 0.40 0.58 1"
    if "SlipRing" in n: return "0.55 0.56 0.60 1"
    if "EndEffector" in n: return "0.93 0.48 0.22 1"
    return "0.82 0.83 0.85 1"


for i, (n, m) in enumerate(meshes.items()):
    if m is None: continue
    fn = os.path.abspath(f"mjtmp/dims/l{i}.stl"); m.export(fn)
    assets.append(f'<mesh name="m{i}" file="{fn}" inertia="shell"/>')
    q = np.zeros(4); mujoco.mju_mat2Quat(q, X[n][:3, :3].flatten())
    geoms.append(f'<geom type="mesh" mesh="m{i}" pos="{" ".join(map(str, X[n][:3, 3]))}" quat="{" ".join(map(str, q))}" rgba="{color(n)}" contype="0" conaffinity="0"/>')
p = [c / 255 for c in PAPER]
xml = f"""<mujoco><visual><global offwidth="{W}" offheight="{H}"/><headlight ambient="0.48 0.48 0.48" diffuse="0.42 0.42 0.42"/></visual>
  <asset><texture type="skybox" builtin="gradient" rgb1="{p[0]} {p[1]} {p[2]}" rgb2="{p[0]} {p[1]} {p[2]}" width="16" height="16"/>{''.join(assets)}</asset>
  <worldbody><light directional="true" dir="0.2 0.6 -0.8" diffuse="0.35 0.35 0.35"/>{''.join(geoms)}</worldbody></mujoco>"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model); mujoco.mj_forward(model, data)
model.vis.global_.orthographic = 1
model.vis.global_.fovy = H / s                                   # view height in metres
cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.azimuth, cam.elevation, cam.distance = 90, 0, 6.0
# lookat so that (cx_w, zc) lands at (W/2, CY)
cam.lookat[:] = (cx_w, -3.0, zc + (CY - H / 2) / s)
rr = mujoco.Renderer(model, H, W)
rr.update_scene(data, cam)
img = Image.fromarray(rr.render()).convert("RGB")
d = ImageDraw.Draw(img)
u = lambda xw: W / 2 + (xw - cx_w) * s
v = lambda zw: CY - (zw - zc) * s
arm_top, arm_bot = v(allV[:, 2].max()), v(allV[:, 2].min())


# ---------------------------------------------------------------- dimension lines
def hdim(x1, x2, y, text, sub=None, ext_from=None, size=30, color=INK, ticks=True):
    a, b = u(x1), u(x2)
    for xx in (a, b):
        if ext_from is not None:
            d.line([(xx, ext_from), (xx, y + (8 if y > ext_from else -8))], fill=MUTED, width=2)
    d.line([(a, y), (b, y)], fill=color, width=3)
    if ticks:
        for xx in (a, b):
            d.line([(xx - 9, y + 9), (xx + 9, y - 9)], fill=color, width=4)
    ft = f("SemiBold", size)
    tw = d.textlength(text, font=ft)
    mid = (a + b) / 2
    if tw + 24 < b - a:
        d.rectangle([mid - tw / 2 - 10, y - size * 0.7, mid + tw / 2 + 10, y + size * 0.7], fill=PAPER)
        d.text((mid, y), text, font=ft, fill=color, anchor="mm")
    else:
        d.text((mid, y - 18), text, font=ft, fill=color, anchor="ms")
    if sub:
        d.text((mid, y - size * 0.75 - 8), sub, font=f("Medium", 24), fill=INK2, anchor="ms")


def vdim(z1, z2, x, text, size=30):
    a, b = v(z1), v(z2)
    d.line([(x, a), (x, b)], fill=INK, width=3)
    for yy in (a, b):
        d.line([(x - 9, yy + 9), (x + 9, yy - 9)], fill=INK, width=4)
    d.text((x + 22, (a + b) / 2), text, font=f("SemiBold", size), fill=INK, anchor="lm")


# the two sides differ by a few mm in the CAD export; label each symmetric pair with its mean
L_end = ((xb["roll_lo"] - xb["tip_lo"]) + (xb["tip_hi"] - xb["roll_hi"])) / 2
B_end = ((xb["pitch_lo"] - xb["dock_lo"]) + (xb["dock_hi"] - xb["pitch_hi"])) / 2
B_mid = ((xb["pitch_mid"] - xb["pitch_lo"]) + (xb["pitch_hi"] - xb["pitch_mid"])) / 2
m_end = (mass["end_lo"] + mass["end_hi"]) / 2
# above: modules, then overall
y_mod, y_all = arm_top - 120, arm_top - 260
hdim(xb["tip_lo"], xb["roll_lo"], y_mod, both(L_end), ext_from=arm_top - 10,
     sub=f"End module · 3 motors · {m_end / 0.4536:.1f} lb ({m_end:.1f} kg)")
hdim(xb["roll_lo"], xb["roll_hi"], y_mod, both(xb["roll_hi"] - xb["roll_lo"]), ext_from=arm_top - 10, size=26)
d.text((u((xb["roll_lo"] + xb["roll_hi"]) / 2), y_mod - 58), f"Middle · 1 motor · {mass['middle'] / 0.4536:.1f} lb ({mass['middle']:.1f} kg)",
       font=f("Medium", 24), fill=INK2, anchor="ms")
hdim(xb["roll_hi"], xb["tip_hi"], y_mod, both(L_end), ext_from=arm_top - 10,
     sub=f"End module · 3 motors · {m_end / 0.4536:.1f} lb ({m_end:.1f} kg)")
hdim(xb["tip_lo"], xb["tip_hi"], y_all, f"Overall, gripper tip to gripper tip: {both(xb['tip_hi'] - xb['tip_lo'])}", ext_from=y_mod - 70, size=32)

# below: where it bends, then dock to dock
y_j, y_dd = arm_bot + 130, arm_bot + 260
pts = [("dock_lo", "Dock face"), ("pitch_lo", "Bend"), ("pitch_mid", "Bend"), ("pitch_hi", "Bend"), ("dock_hi", "Dock face")]
for ((k1, _), (k2, _)), L in zip(zip(pts[:-1], pts[1:]), (B_end, B_mid, B_mid, B_end)):
    hdim(xb[k1], xb[k2], y_j, both(L), ext_from=None, size=27, color=TEAL)
for k, lab in pts:
    xx = u(xb[k])
    d.line([(xx, CY), (xx, y_j - 10)], fill=TEAL, width=2)
    d.ellipse([xx - 7, CY - 7, xx + 7, CY + 7], fill=TEAL)
    d.text((xx, y_j + 44), lab, font=f("Medium", 24), fill=TEAL, anchor="mm")
hdim(xb["dock_lo"], xb["dock_hi"], y_dd, f"Dock face to dock face: {both(xb['dock_hi'] - xb['dock_lo'])}", size=32)

# right: cross-section of the thickest part (middle module)
vm = V["middle"]
vdim(vm[:, 2].min(), vm[:, 2].max(), u(xb["tip_hi"]) + 40, f"{ftin(np.ptp(vm[:, 2]))}")
d.text((60, H - 46), "Laid out straight, side view. From the CAD model (URDF v0.2.0, 14.35 kg total); not measured on hardware.",
       font=f("Regular", 24), fill=MUTED, anchor="ls")
os.makedirs("videos/showcase", exist_ok=True)
img.save("videos/showcase/11_dimensions.png")

print("overall tip-tip", round(xb["tip_hi"] - xb["tip_lo"], 3), "dock-dock", round(xb["dock_hi"] - xb["dock_lo"], 3))
print("modules", round(xb["roll_lo"] - xb["tip_lo"], 3), round(xb["roll_hi"] - xb["roll_lo"], 3), round(xb["tip_hi"] - xb["roll_hi"], 3))
print("bends", [round(xb[b] - xb[a], 3) for (a, _), (b, _) in zip(pts[:-1], pts[1:])])
print("cross-sections (in view, depth):", {k: tuple(round(x, 3) for x in v_) for k, v_ in sec.items()})
print("gripper", round(np.ptp(grip[:, 2]), 3), round(np.ptp(grip[:, 1]), 3), "masses", {k: round(m, 2) for k, m in mass.items()})
