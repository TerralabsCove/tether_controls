"""Rewrite the <inertial> blocks of terra_arm_description/urdf/arm.urdf with measured part masses.

The Fusion export assigned default materials, so several parts came out as solid steel. This sets:
  * whole parts with a known mass (PART_KG): mass replaced, centre of mass kept, inertia scaled with mass
  * slip-ring and gripper links (MESH_LINKS): rebuilt part by part from MESH_MAT (a material density or a
    fixed mass per mesh, uniform density within each mesh); unlisted meshes in those links are aluminium
  * every other link: unchanged
Usage: python apply_measured_masses.py [--check]   (--check prints the result without writing)
"""
import os, re, sys
import numpy as np
import trimesh
from model import Arm, URDF

OZ, LB = 0.0283495, 0.45359237
AL, TPU, STEEL = 2.70, 1.21, 7.85   # g/cm3
# Masses as of 2026-10-31: motors from their user manuals, the rest weighed by the team.
PART_KG = {"DM-J10010L-2EC_Motor": 1.485,   # DM-J10010-2EC manual (URDF name says L; the motor is the plain 10010)
           "DM-J4340P-2EC_Motor": 0.375,    # DM-J4340P-2EC manual
           "LiPo_3200mAh_batt": 0.218,      # weighed
           "LiPo_5000mAh_Batt": 1.52 * LB}  # weighed, 1.52 lb
MESH_MAT = {"SlipRingBase": ("kg", 10.5 * OZ),         # the slip ring itself, 10.5 oz
            "BasePlate": AL, "Pin": AL, "PinBase": AL, "slipRingRotator": AL,
            "EE3_BLS3335_Servo": ("kg", 3.52 * OZ),     # 3.52 oz each
            "EE3_ClawJawA": TPU, "EE3_ClawJawB": TPU, "finray_finger": TPU,   # printed TPU, taken as solid
            "EE3_LockPin": STEEL, "EE3_PegDowel": STEEL}
MESH_LINKS = ("SlipRing_", "EndEffectorAssembly_")


def part_inertial(L, mat_of):
    """Mass, COM and inertia (about the COM, link axes) of a link built from its collision meshes."""
    parts = []
    for fn, sc, Tg in L["geoms"]:
        m = trimesh.load(fn, force="mesh"); m.apply_scale(sc); m.apply_transform(Tg)
        mat = mat_of(os.path.basename(fn)[:-4])
        kg = mat[1] if isinstance(mat, tuple) else abs(m.volume) * 1e6 * mat / 1000
        I = m.moment_inertia * (kg / m.mass)        # trimesh: unit density, about the mesh COM
        parts.append((kg, m.center_mass, I))
    M = sum(k for k, _, _ in parts)
    c = sum(k * p for k, p, _ in parts) / M
    I = np.zeros((3, 3))
    for k, p, Ip in parts:
        d = p - c
        I += Ip + k * (d @ d * np.eye(3) - np.outer(d, d))
    return M, c, I


def new_inertials(arm):
    out = {}
    for n, L in arm.links.items():
        if n.startswith(MESH_LINKS):
            out[n] = part_inertial(L, lambda k: MESH_MAT.get(k, AL))
            continue
        for pre, kg in PART_KG.items():
            if n.startswith(pre):
                out[n] = (kg, L["com"], L["I"] * kg / L["mass"])
    return out


def fmt(v):
    return f"{v:.9g}"


def main(check):
    arm = Arm()
    text = open(URDF).read()
    # read the current inertia tensors
    for n, L in arm.links.items():
        blk = re.search(rf'<link name="{re.escape(n)}">\s*<inertial>(.*?)</inertial>', text, re.S).group(1)
        a = dict(re.findall(r'(i[xyz]{2})="([^"]+)"', blk))
        a = {k: float(v) for k, v in a.items()}
        L["I"] = np.array([[a["ixx"], a["ixy"], a["ixz"]], [a["ixy"], a["iyy"], a["iyz"]], [a["ixz"], a["iyz"], a["izz"]]])

    # method check: an aluminium bracket rebuilt from its mesh should match the CAD export
    b = "BracketLarge_2"
    M, c, I = part_inertial(arm.links[b], lambda k: AL)
    print(f"check {b}: mass {M:.4f} vs CAD {arm.links[b]['mass']:.4f} kg; "
          f"max inertia error {np.abs(I - arm.links[b]['I']).max() / np.abs(arm.links[b]['I']).max():.1%}")

    new = new_inertials(arm)
    for n, (M, c, I) in new.items():
        ev = np.linalg.eigvalsh(I)
        assert ev.min() > 0 and ev[2] <= ev[0] + ev[1] + 1e-12, f"bad inertia on {n}"
        print(f"  {n:28s} {arm.links[n]['mass']:7.3f} -> {M:6.3f} kg")
        new_blk = (f'<inertial>\n      <origin xyz="{fmt(c[0])} {fmt(c[1])} {fmt(c[2])}" rpy="0 0 0"/>\n'
                   f'      <mass value="{fmt(M)}"/>\n'
                   f'      <inertia ixx="{fmt(I[0,0])}" ixy="{fmt(I[0,1])}" ixz="{fmt(I[0,2])}" '
                   f'iyy="{fmt(I[1,1])}" iyz="{fmt(I[1,2])}" izz="{fmt(I[2,2])}"/>\n    </inertial>')
        text, k = re.subn(rf'(<link name="{re.escape(n)}">\s*)<inertial>.*?</inertial>',
                          lambda mm: mm.group(1) + new_blk, text, count=1, flags=re.S)
        assert k == 1, n
    total_old = sum(L["mass"] for L in arm.links.values())
    total = total_old + sum(M - arm.links[n]["mass"] for n, (M, _, _) in new.items())
    print(f"total {total_old:.2f} kg ({total_old / LB:.1f} lb) -> {total:.2f} kg ({total / LB:.1f} lb)")
    if not check:
        open(URDF, "w").write(text)
        print("wrote", URDF)


if __name__ == "__main__":
    main("--check" in sys.argv)
