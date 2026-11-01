"""Concept clip: Tether builds a short wall. Floor docks run along the wall line, one block-length apart.
From each work dock the robot reaches a staged pile and two wall columns, builds that section two courses
high, then walks to the next work dock and builds the next section.

Usage: python wall.py [--preview]
Writes videos/construction/wall_concept.mp4 (or a PNG contact sheet with --preview).
Uses the carry planner in construct.py (block mass in the torques, block box in the collision checks,
every motor within rated torque while carrying). Block size and mass are assumptions."""
import os, sys, pickle, subprocess, numpy as np
from construct import *

PITCH_X, ROW_Y = 0.40, 0.60            # floor docks: one block length along the wall, rows 0.6 m apart
WALL_Y, PILE_Y = 0.34, -0.36           # wall line and pile line, either side of the dock row
SECTIONS = [0.8, 1.6]                  # work docks; each builds the two columns either side of it
COURSES = 2
SPEEDUP_FROM = None                    # set in plan(): step index where the time-lapse starts


def scene():
    xs = np.round(np.arange(0, 2.41, PITCH_X), 3)
    grid = [(x, y) for y in (-ROW_Y, 0.0, ROW_Y) for x in xs]
    docks = [dock_pose((x, y, 0), DOWN) for x, y in grid]
    gi = {g: i for i, g in enumerate(grid)}
    floor = floor_box(-0.8, 3.2, -1.3, 1.3)
    pile, slots, order = [], [], []
    for s, xd in enumerate(SECTIONS):
        base = len(pile)
        pile += [block_T((xd, PILE_Y), i * BLOCK[2], 0.0) for i in range(4)]
        sl = {(c, h): block_T((xd + dx, WALL_Y), h * BLOCK[2], 0.0) for c, dx in enumerate((-0.2, 0.2)) for h in range(COURSES)}
        # top of the pile first; bottom course left then right, then the next course
        seq = [(c, h) for h in range(COURSES) for c in range(2)]
        for n, key in enumerate(seq):
            order.append((s, base + 3 - n, len(slots)))
            slots.append(sl[key])
    return dict(docks=docks, gi=gi, floor=floor, pile=pile, slots=slots, order=order)


def plan(seed=0):
    global SPEEDUP_FROM
    S = scene()
    rng = np.random.default_rng(seed)
    docks, gi = S["docks"], S["gi"]
    blocks = [T.copy() for T in S["pile"]]
    P = CarryPlanner([S["floor"]] + [as_box(T) for T in blocks])
    D = lambda x: docks[gi[(round(x, 3), 0.0)]]
    s = EE_B
    st = dict(stance=EE_B, Tws=D(0.0), at={EE_B: D(0.0), EE_A: D(0.4)}, k=0)
    tgt = D(0.4)
    sols = P.ik(s, st["Tws"], tgt[:3, 3], tgt[:3, 2], rng.uniform(-np.pi, np.pi, (300, 6)))
    good = [(P.M[s].ratios(P.load(s, X, x)[0])[0], x) for x in sols for X in [P.frames(s, x, st["Tws"])] if not P.collides(s, X, False)]
    st["x6"] = min(good, key=lambda t: t[0])[1]
    frames = []
    worst = []

    def snap(phase, x, carry=None, held=None, n=1):
        stat = {i: T.copy() for i, T in enumerate(blocks) if i != held}
        for _ in range(n):
            frames.append((st["stance"], st["Tws"], x, st["k"], phase, carry, held, stat))

    def move(mover, Ttgt, carry=None, held=None, names=("lift out of dock", "swing", "lower into dock")):
        new_st = EE_A if mover == EE_B else EE_B
        if new_st != st["stance"]:
            st["x6"] = P.swap(st["stance"], st["x6"], st["Tws"], new_st, st["at"][new_st])
            st["stance"], st["Tws"] = new_st, st["at"][new_st]
        lift, swing, down = P.step(st["stance"], st["Tws"], st["x6"], st["at"][mover], Ttgt, rng)
        for ph, seg in zip(names, (lift, swing, down)):
            for x in seg:
                snap(ph, x, carry, held)
                if carry is not None:
                    worst.append(P.M[st["stance"]].ratios(P.carry_torques(st["stance"], P.frames(st["stance"], x, st["Tws"]), carry))[0])
        st["x6"] = down[-1]; st["at"][mover] = Ttgt

    snap("start", st["x6"], n=20)
    # walk to the first work dock: B 0 -> 0.8 over A
    move(EE_B, D(0.8)); snap("docked", st["x6"], n=10); print("  walk in", flush=True); st["k"] += 1
    tool = EE_A
    for s_i, xd in enumerate(SECTIONS):
        if s_i == 1:
            SPEEDUP_FROM = st["k"]
        center = np.array([xd, 0.0, 0.0])
        for sec, bi, sl in [o for o in S["order"] if o[0] == s_i]:
            Tgoal = S["slots"][sl]
            Tpick = top_dock(blocks[bi])
            c = dict(center=center, th0=np.arctan2(blocks[bi][1, 3], blocks[bi][0, 3] - xd),
                     th1=np.arctan2(Tgoal[1, 3], Tgoal[0, 3] - xd), yaw0=0.0, dyaw=0.0)
            P.carry, P.next_carry, P.max_peak = None, c, None
            P.boxes = [S["floor"]] + [as_box(T) for T in blocks]
            move(tool, Tpick)
            P.next_carry = None
            snap("docked", st["x6"], n=8); print(f"  pick {st['k']}: block {bi}", flush=True); st["k"] += 1
            P.carry, P.max_peak = c, 0.26        # carry within rated torque on every motor
            P.boxes = [S["floor"]] + [as_box(T) for i, T in enumerate(blocks) if i != bi]
            move(tool, top_dock(Tgoal), c, bi, ("lift block", "carry", "set down"))
            blocks[bi] = Tgoal.copy()
            P.carry, P.max_peak = None, None
            snap("docked", st["x6"], n=8)
            print(f"  place {st['k']}: block {bi} -> slot {sl}, worst rated ratio so far {max(worst):.2f}", flush=True); st["k"] += 1
        # park the free end in the next floor dock along the row
        P.boxes = [S["floor"]] + [as_box(T) for T in blocks]
        move(tool, D(xd + 0.4)); snap("docked", st["x6"], n=10); print(f"  park {st['k']}", flush=True); st["k"] += 1
        if s_i + 1 < len(SECTIONS):
            # step the other end to the next work dock; the parked end becomes the tool again
            other = EE_B if tool == EE_A else EE_A
            move(other, D(SECTIONS[s_i + 1])); snap("docked", st["x6"], n=10); print(f"  walk {st['k']}", flush=True); st["k"] += 1
    snap("done", st["x6"], n=20)
    print("  done; carry worst rated ratio", max(worst), "speed-up from step", SPEEDUP_FROM, flush=True)
    return P, frames, S


def captions(cap_txt, k2):
    wall_len = len(SECTIONS) * 0.8
    return [
        ("start", "Concept: Tether builds a wall. Floor docks run along the wall line, 1 ft 4 in (0.40 m) apart: one block length."),
        ("start", f"Material is staged beside each work position. Blocks {BLOCK_KG * 2.2046:.1f} lb ({BLOCK_KG} kg), assumed, each with a dock on top."),
        (0, "It walks to the first work position."),
        (1, "From one floor dock it reaches the pile and two wall columns."),
        (2, "Bottom course first. Each block's pocket sits over the dock of the block below."),
        (5, "Then the second course on top."),
        (9, "Section done: 2 ft 7 in (0.80 m) of wall from one stop. It parks the free end in the next floor dock."),
        (10, "It steps along the row to the next work position, next to the next pile."),
        (k2, "Second section, shown 3x faster."),
        (k2 + 8, f"{ftin(wall_len)} ({wall_len:.1f} m) of wall, two courses high. Every stop adds another 2 ft 7 in."),
        ("end", cap_txt),
        ("end", "Open items: a dock built into the material, how blocks lock to each other, and payload tests on hardware."),
    ]


def timeline(frames, caps, k2):
    out = []
    for i, fr in enumerate(frames):
        if fr[4] in ("start", "done"):
            continue
        fast = fr[3] >= k2
        if fast and (i % 3):            # time-lapse: every third frame, no in-betweens
            continue
        out.append(fr)
        nx = frames[i + 1] if i + 1 < len(frames) else fr
        if not fast and fr[4] != "docked" and nx[3] == fr[3] and nx[4] == fr[4]:
            out.append(fr[:2] + (fr[2] + 0.5 * wrapd(nx[2] - fr[2]),) + fr[3:])
    byk = {t: s for t, s in caps if isinstance(t, int)}
    st = [s for t, s in caps if t == "start"]
    en = [s for t, s in caps if t == "end"]
    tl = []
    for s in st:
        tl += [(frames[0], s)] * int(4.0 * FPS)
    cur = st[-1]
    for fr in out:
        if fr[3] in byk:
            cur = byk[fr[3]]
        tl.append((fr, cur))
    last = [f for f in frames if f[4] == "done"][-1]
    tl += [(last, cur)] * int(2.5 * FPS)
    for s in en:
        tl += [(last, s)] * int(4.5 * FPS)
    return tl


def main(preview=False):
    os.makedirs("plans/construction", exist_ok=True)
    cache = "plans/construction/wall_concept.pkl"
    if os.path.exists(cache):
        frames, k2 = pickle.load(open(cache, "rb")); S = scene()
        P = CarryPlanner([S["floor"]])
    else:
        for seed in range(6):
            try:
                P, frames, S = plan(seed); break
            except RuntimeError as e:
                print("  seed", seed, "failed:", e, flush=True)
        else:
            raise RuntimeError("planning failed for all seeds")
        k2 = SPEEDUP_FROM
        pickle.dump((frames, k2), open(cache, "wb"))
    carried = [fr for fr in frames if fr[5] is not None]
    caps_kg = []
    for fr in carried:
        X = P.frames(fr[0], fr[2], fr[1])
        lo, hi = 0.0, 20.0
        for _ in range(30):
            m = (lo + hi) / 2
            if P.M[fr[0]].ratios(P.carry_torques(fr[0], X, fr[5], m))[0] <= 1: lo = m
            else: hi = m
        caps_kg.append(lo)
    worst_kg = min(caps_kg)
    ratio = max(P.M[fr[0]].ratios(P.carry_torques(fr[0], P.frames(fr[0], fr[2], fr[1]), fr[5]))[0] for fr in carried)
    print(f"block {BLOCK_KG} kg; worst-frame capacity within rated {worst_kg:.2f} kg; worst rated ratio while carrying {ratio:.2f}")
    cap_txt = (f"Carrying, every motor stays within rated torque. Along this path that holds for blocks up to "
               f"{np.floor(worst_kg * 2.2046 * 2) / 2:g} lb ({np.floor(worst_kg * 10) / 10:.1f} kg).") if ratio <= 1 else \
              (f"With a {BLOCK_KG} kg block some motors reach {ratio:.1f}x rated torque, still under peak.")
    caps = captions(cap_txt, k2)
    model, names = build(P, S, os.path.abspath("mjtmp/wall"))
    tl = timeline(frames, caps, k2)
    arm = P.M[EE_A].arm
    sub = {n: arm.link_mesh[n].vertices[::60] for n in names}
    pts = []
    for fr in frames[::8]:
        X = P.frames(fr[0], fr[2], fr[1])
        for n in names:
            pts += list(sub[n] @ X[n][:3, :3].T + X[n][:3, 3])
    pts = pts[::7] + [T[:3, 3] for T in S["docks"] if abs(T[1, 3]) < 0.7] + [T[:3, 3] + (0, 0, 0.07) for T in S["slots"]]
    cam = fit(pts, 72, -28, False, margin=40)
    data = mujoco.MjData(model)
    rr = mujoco.Renderer(model, H, W)
    na = len(names)
    os.makedirs(OUT, exist_ok=True)
    out = f"{OUT}/wall_concept.mp4"
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
        img = Image.alpha_composite(img, caption_layer("Concept: building a wall", "", txt, fade / 9, FOOTER_C))
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
        g.save(f"{OUT}/wall_concept_preview.png"); print("preview written")


if __name__ == "__main__":
    main("--preview" in sys.argv)
