"""Two one-page summaries for set designers (PNG, 1920x1080): where docks can go, and what docks must hold.
Comfortable limits include the 12 cm lift-out. Load numbers are from the showcase clips (static, swing gripper unsupported)."""
import os, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from PIL import Image, ImageDraw, ImageFont
from plot import load
from step_reach import EE_B

W, H = 1920, 1080
INK, INK2, MUTED, TEAL, PAPER, RULE = "#1B2430", "#5A6270", "#8A919C", "#128082", "#FAFAF8", "#DCDDE0"
INTER = "/usr/share/fonts/opentype/inter/Inter-{}.otf"
for w in ("Regular", "Medium", "SemiBold"):
    font_manager.fontManager.addfont(INTER.format(w))
plt.rcParams.update({"font.family": "Inter", "text.color": INK, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2})
FT = 0.3048
os.makedirs("videos/showcase", exist_ok=True)


def f(w, s):
    return ImageFont.truetype(INTER.format(w), s)


def header(d, title, sub):
    d.text((80, 64), title, font=f("SemiBold", 46), fill=INK)
    d.text((80, 130), sub, font=f("Regular", 24), fill=INK2)
    d.line((80, 178, W - 80, 178), fill=RULE, width=2)
    d.text((80, H - 44), "Tether arm, 31.6 lb (14.35 kg). Simulation from the CAD model and motor datasheets, not yet measured on hardware.",
           font=f("Regular", 18), fill=MUTED)


def rows(d, x, y, w, items, label_w=330, size=25, gap=20):
    for lab, txt in items:
        d.text((x, y), lab, font=f("SemiBold", size), fill=INK)
        words, line, yy = txt.split(), "", y
        for w_ in words:
            if d.textlength((line + " " + w_).strip(), font=f("Regular", size)) > w - label_w:
                d.text((x + label_w, yy), line, font=f("Regular", size), fill=INK); yy += size + 10; line = w_
            else:
                line = (line + " " + w_).strip()
        d.text((x + label_w, yy), line, font=f("Regular", size), fill=INK); yy += size + 10
        y = yy + gap
        d.line((x, y - gap / 2, x + w, y - gap / 2), fill=RULE, width=1)
    return y


# ------------------------------------------------------------------ sheet 1: where docks can go
d_ = load(EE_B)
RS, HS = d_["RS"], d_["HS"]
ku = np.zeros_like(d_["selfok"]); ku[:-6] = d_["selfok"][6:]
comfy = (d_["feas"] & (d_["rated"][:, 1] <= 1)).any(0) & ku
hard = (d_["feas"][0] | d_["feas"][1]) & ku

fig, ax = plt.subplots(figsize=(9.6, 8.3), dpi=100, facecolor=PAPER)
ax.set_facecolor(PAPER)
ii, jj = np.where(comfy)
ax.scatter(RS[jj] / FT, HS[ii] / FT, s=14, color=TEAL, linewidths=0, label="Comfortable spot for the next dock")
env = [(RS[np.where(hard[i])[0]].max() / FT, HS[i] / FT) for i in range(len(HS)) if hard[i].any()]
ex, ey = zip(*env)
ax.plot(ex, ey, "--", color=INK2, lw=1.6, label="Absolute limit (no margin)")
ax.plot([0], [0], "s", ms=13, color=INK, zorder=5)
ax.annotate("Holding\ndock", (0, 0), (-0.2, 0), color=INK, fontsize=13, fontweight="semibold", ha="right", va="center")
for (r, h), txt, xy in [((0.58, 0), "Level: up to 1 ft 11 in", (2.95, 0.75)), ((0.44, 0.54), "Up: 1 ft 9 in at most", (2.25, 2.05)),
                         ((0.44, -0.66), "Down: 2 ft 2 in at most", (2.25, -2.4))]:
    ax.annotate(txt, (r / FT, h / FT), xy, fontsize=13, color=INK, va="center",
                arrowprops=dict(arrowstyle="-", color=INK2, lw=1, shrinkA=2, shrinkB=3))
ax.axhline(0, color=RULE, lw=1, zorder=0)
ax.set_xlim(-0.9, 4.4); ax.set_ylim(-2.6, 2.4); ax.set_aspect("equal")
ax.set_xticks(range(0, 4)); ax.set_xticklabels([f"{k} ft" for k in range(4)])
ax.set_yticks(range(-2, 3)); ax.set_yticklabels([f"{k:+d} ft" if k else "0" for k in range(-2, 3)])
ax.grid(color=RULE, lw=0.8); ax.set_axisbelow(True)
for s in ax.spines.values(): s.set_visible(False)
ax.tick_params(length=0, labelsize=13)
ax.set_xlabel("Distance forward to the next dock centre", fontsize=14, labelpad=10)
ax.set_ylabel("Next dock higher (+) or lower (−)", fontsize=14, labelpad=10)
sec = ax.secondary_xaxis("top", functions=(lambda x: x * FT, lambda m: m / FT))
sec.set_xticks([0, 0.25, 0.5, 0.75]); sec.set_xticklabels(["0", "0.25 m", "0.5 m", "0.75 m"]); sec.tick_params(length=0, labelsize=11, colors=MUTED)
for s in sec.spines.values(): s.set_visible(False)
ax.legend(loc="lower left", bbox_to_anchor=(0.0, 1.07), ncol=2, fontsize=12.5, frameon=False, labelcolor=INK)
fig.tight_layout(); fig.savefig("videos/showcase/_reach_chart.png", facecolor=PAPER); plt.close(fig)

img = Image.new("RGB", (W, H), PAPER); d = ImageDraw.Draw(img)
header(d, "Where the docks can go", "Comfortable limits, measured dock centre to dock centre. Side view: the robot holds the dark square and reaches for the next dock.")
chart = Image.open("videos/showcase/_reach_chart.png")
chart = chart.resize((int(chart.width * 0.98), int(chart.height * 0.98)))
img.paste(chart, (40, 200))
rows(d, 1040, 222, 800, [
    ("Flat floor or platform", "Up to 1 ft 11 in (0.58 m) apart. Absolute limit 2 ft 7 in (0.80 m)."),
    ("Step up", "Up to 1 ft 9 in (0.54 m) higher, with the docks up to 1 ft 5 in (0.44 m) apart. A 1 ft 4 in (0.40 m) rise allows 2 ft (0.62 m)."),
    ("Step down", "Up to 2 ft 2 in (0.66 m) lower, with the docks up to 1 ft 5 in (0.44 m) apart."),
    ("30° slope", "Up or down the slope: up to 2 ft 3 in (0.70 m). Across it: under 10 in (0.26 m)."),
    ("Vertical surface", "In a vertical line, up to 2 ft 7 in (0.80 m) apart. Sideways: under 5 in (0.14 m)."),
    ("Floor to wall", "First wall dock about 1 ft (0.30 m) up. Last floor dock 1 ft 4 in to 2 ft 9 in (0.40 to 0.84 m) from the wall."),
    ("Over a top edge", "Both docks within about 8 in (0.20 m) of the edge."),
    ("Hanging underneath", "Up to 1 ft 11 in (0.58 m) apart. Not tested yet."),
], label_w=300, size=23, gap=18)
img.save("videos/showcase/10a_where_docks_can_go.png")

# ------------------------------------------------------------------ sheet 2: what docks must hold
img = Image.new("RGB", (W, H), PAPER); d = ImageDraw.Draw(img)
header(d, "What each dock must hold", "While one end swings, a single dock carries the whole robot. Static loads, before any safety factor.")
cols = [80, 520, 1010, 1440]
heads = ["Where the dock is", "Force", "Prying (bending)", "Twist"]
y = 220
for x, t in zip(cols, heads):
    d.text((x, y), t, font=f("SemiBold", 24), fill=INK2)
y += 50; d.line((80, y - 10, W - 80, y - 10), fill=INK2, width=2)
data = [
    ("Floor or platform", "32 lbf (141 N) pressing in", "up to 33 lbf·ft (45 Nm)", "none"),
    ("30° slope", "28 lbf in, 16 lbf along (122 N, 70 N)", "up to 53 lbf·ft (72 Nm)", "up to 20 lbf·ft (27 Nm)"),
    ("Vertical surface", "32 lbf (141 N) along the surface", "up to 75 lbf·ft (102 Nm)", "up to 20 lbf·ft (27 Nm)"),
    ("Over a top edge", "32 lbf (141 N)", "up to 70 lbf·ft (95 Nm)", "up to 20 lbf·ft (27 Nm)"),
    ("Hanging underneath", "32 lbf (141 N) pulling straight out", "up to 30 lbf·ft (40 Nm)", "none"),
]
for row in data:
    for x, t in zip(cols, row):
        d.text((x, y), t, font=f("SemiBold" if x == cols[0] else "Regular", 25), fill=INK)
    y += 62; d.line((80, y - 18, W - 80, y - 18), fill=RULE, width=1)
y += 24
d.text((80, y), "What this means for the set", font=f("SemiBold", 30), fill=INK); y += 58
rows(d, 80, y, 1760, [
    ("Fix docks to structure", "Bolt every dock into the frame (for the tree, the wooden stand), not into plaster, foam or chicken wire. The fixing carries the same loads as the dock."),
    ("Twist", "On anything but a flat surface, the gripper can be twisted in its dock up to 20 lbf·ft (27 Nm), the limit of the motor that turns it. The dock lock should hold more."),
    ("Safety factor", "Proposed: design each dock and its fixing for at least twice the numbers above. The sim leaves out swing dynamics and wiring weight."),
    ("Clear space", "On a flat step the arm rises up to 2 ft 8 in (0.80 m) above the docks. Hanging, it drops about the same below. Keep visitors clear of that space."),
], label_w=360, size=24, gap=16)
img.save("videos/showcase/10b_what_docks_must_hold.png")
print("sheets written")
