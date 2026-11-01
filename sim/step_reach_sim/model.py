"""Load terra_arm_description/urdf/arm.urdf: FK, meshes, masses."""
import os, xml.etree.ElementTree as ET
import numpy as np
import trimesh

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../description/terra_arm_description"))
URDF = os.path.join(PKG, "urdf/arm.urdf")

def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])

def T_of(xyz, rpy):
    T = np.eye(4); T[:3, :3] = rpy2R(*rpy); T[:3, 3] = xyz; return T

def axis_angle(axis, q):
    a = np.asarray(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4); T[:3, :3] = np.eye(3) + np.sin(q)*K + (1-np.cos(q))*K@K; return T

def _vec(s, d="0 0 0"):
    return np.array([float(v) for v in (s or d).split()])

class Arm:
    def __init__(self, urdf=URDF):
        root = ET.parse(urdf).getroot()
        self.links = {}
        for L in root.findall("link"):
            n = L.get("name")
            inert = L.find("inertial")
            m = float(inert.find("mass").get("value")) if inert is not None else 0.0
            com = _vec(inert.find("origin").get("xyz")) if inert is not None else np.zeros(3)
            geoms = []
            for C in L.findall("collision"):
                o = C.find("origin")
                mesh = C.find("geometry/mesh")
                fn = mesh.get("filename").replace("package://terra_arm_description", PKG)
                sc = _vec(mesh.get("scale"), "1 1 1")
                Tg = T_of(_vec(o.get("xyz")), _vec(o.get("rpy"))) if o is not None else np.eye(4)
                geoms.append((fn, sc, Tg))
            self.links[n] = dict(mass=m, com=com, geoms=geoms)
        self.joints = []
        for J in root.findall("joint"):
            o = J.find("origin")
            ax = J.find("axis")
            self.joints.append(dict(name=J.get("name"), type=J.get("type"),
                parent=J.find("parent").get("link"), child=J.find("child").get("link"),
                T=T_of(_vec(o.get("xyz")), _vec(o.get("rpy"))) if o is not None else np.eye(4),
                axis=_vec(ax.get("xyz")) if ax is not None else None))
        children = {j["child"] for j in self.joints}
        self.root = [n for n in self.links if n not in children][0]
        # topological order
        order, frontier = [], [self.root]
        while frontier:
            p = frontier.pop(0)
            for j in self.joints:
                if j["parent"] == p:
                    order.append(j); frontier.append(j["child"])
        self.jorder = order
        self.active = [j["name"] for j in self.joints if j["type"] != "fixed"]

    def fk(self, q, T_root=np.eye(4)):
        """q: dict joint name -> angle. Returns dict link -> 4x4 world transform."""
        X = {self.root: T_root}
        for j in self.jorder:
            T = X[j["parent"]] @ j["T"]
            if j["type"] != "fixed":
                T = T @ axis_angle(j["axis"], q.get(j["name"], 0.0))
            X[j["child"]] = T
        return X

    def load_meshes(self):
        cache = {}
        self.link_mesh = {}
        for n, L in self.links.items():
            parts = []
            for fn, sc, Tg in L["geoms"]:
                if fn not in cache:
                    cache[fn] = trimesh.load(fn, force="mesh")
                m = cache[fn].copy(); m.apply_scale(sc); m.apply_transform(Tg)
                parts.append(m)
            self.link_mesh[n] = trimesh.util.concatenate(parts) if parts else None
        return self.link_mesh
