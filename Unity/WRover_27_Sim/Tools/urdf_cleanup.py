#!/usr/bin/env python3
"""
Clean the Onshape URDF export of the simplified WRover model for Unity 2022.3
(URDF Importer / ArticulationBody).

Steps, in order:
  1. Remove the ball-joint rod chains (the differential linkage). The differential
     is reproduced in Unity by RockerDifferential.cs instead.
  2. Fix wrong masses (MASS_OVERRIDES). Inertia is scaled with the mass, so the
     shape and centre of mass are kept.
  3. Add collision shapes: a box for the chassis, a cylinder per wheel, and an
     inertia-equivalent box for each arm link.
  4. Merge every fixed joint into its parent: masses summed, centre of mass
     combined, inertia combined (rotated into one frame + parallel-axis theorem),
     visuals and collisions re-posed into the merged frame.
  5. Rename links and joints (front/back and left/right worked out from geometry).
  6. Give every reused mesh a unique file per use. Unity's URDF Importer breaks
     meshes that are referenced more than once (the copies end up as
     "Missing (Mesh)"), so e.g. the 4 wheels each get their own Tread copy.

Joint limits are left exactly as exported.

Usage:  python urdf_cleanup.py <input.urdf> <output.urdf> [meshes_dir]
        meshes_dir = the folder with the .stl files (e.g.
        ../Assets/Robots/wrover/wrover_27_simple/meshes). If given, the unique
        copies are written there. Without it, a Windows batch file that makes
        the same copies (make_mesh_copies.bat) is written next to the output URDF.
Needs:  numpy
Re-run it on every new Onshape export.
"""
import os
import re
import shutil
import sys
from collections import Counter
import xml.etree.ElementTree as ET

import numpy as np

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------

# Direction (in the chassis frame) that points to the rover's FRONT.
# The arm column sits on the -y side of the chassis, assumed to be the front.
# If your front is the other way, change this to (0, 1, 0): left/right and
# front/back names flip accordingly.
FORWARD = np.array([0.0, -1.0, 0.0])
UP = np.array([0.0, 0.0, 1.0])

# Mass fixes. Keys are regexes matched against ORIGINAL link names.
# value: (new mass in kg, reason). Inertia is scaled by new/old mass.
MASS_OVERRIDES = {
    r"^chassis$": (
        10.0,
        "Placeholder box modeled at 1 kg/m^3 (35 g). ESTIMATE for frame + "
        "electronics + batteries; replace with the real value.",
    ),
    r"^swerve_motor(_\d+)?$": (
        0.50,
        "Placeholder cylinder modeled at ~1 kg/m^3 (0.33 g). ESTIMATE for a "
        "steering motor + gearbox; replace with the real value.",
    ),
    r"^swerve_arm(_\d+)?$": (
        0.62,
        "Modeled at 1 kg/m^3 (0.23 g). Rescaled to aluminium (x2700).",
    ),
    r"^rocker_bracket(_\d+)?$": (
        0.024,
        "Modeled at 1 kg/m^3 (0.009 g). Rescaled to aluminium (x2700).",
    ),
}

# Rod chains to delete: a link matching this regex whose parent does NOT match
# starts a chain; it is removed with everything below it.
ROD_RE = re.compile(r"^(60645k211_ball_joint_linkage(_\d+)?|ball_\d(_\d+)?)$")

# Wheel collision. Derived from the exported inertia: hubcap disc radius 54.5 mm,
# tread ring  -> outer radius ~0.110 m, width ~0.100 m. Cylinder axis = tread +y.
WHEEL_LINK_RE = re.compile(r"^tread(_\d+)?$")
WHEEL_RADIUS = 0.110
WHEEL_WIDTH = 0.100

# Links that get an inertia-equivalent box collider (box of uniform density with
# the same mass distribution). The chassis box comes out 0.414 x 0.414 x 0.203 m.
BOX_COLLISION_LINKS = [
    "chassis",
    "vertical_c_beam",
    "vertical_plate",
    "supinator",
    "differential",
    "manipulator",
    "claw",
]

# Original movable joint -> (new joint name, new child link name).
# Rocker / steering / wheel names get the side and end filled in automatically.
ARM_NAMES = {
    "slider_1": ("arm_slide_joint", "arm_column"),        # arm base slides sideways
    "slider_3": ("arm_lift_joint", "arm_carriage"),       # carriage lifts on the column
    "revolute_13": ("arm_roll_joint", "arm_supinator"),   # supinator twists about its length
    "revolute_11": ("wrist_pitch_joint", "wrist_differential"),
    "revolute_12": ("wrist_roll_joint", "manipulator"),
    "slider_2": ("claw_joint", "claw"),
}
BASE_LINK_NAME = "base_link"

# Onshape exports the swerve modules at whatever angle they had in the assembly.
# When True, each steering joint is re-zeroed so that 0 = wheel rolling straight
# ahead (axle perpendicular to FORWARD). Only the default pose changes.
ZERO_STEERING = True

# ----------------------------------------------------------------------------
# Small rigid-transform helpers (URDF rpy = fixed-axis X, then Y, then Z)
# ----------------------------------------------------------------------------


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def R_to_rpy(R):
    sp = -R[2, 0]
    if abs(sp) < 1 - 1e-9:
        p = np.arcsin(sp)
        r = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        p = np.pi / 2 * np.sign(sp)
        r = 0.0
        y = np.arctan2(-R[0, 1], R[1, 1])
    return r, p, y


def origin_T(el):
    """4x4 transform of an <origin> child of el (identity if absent)."""
    T = np.eye(4)
    o = el.find("origin") if el is not None else None
    if o is not None:
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
        T[:3, :3] = rpy_to_R(*rpy)
        T[:3, 3] = xyz
    return T


def axis_angle_T(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
    return T


def fmt(v):
    return " ".join(f"{x:.9g}" if abs(x) > 1e-12 else "0" for x in v)


def set_origin(el, T):
    o = el.find("origin")
    if o is None:
        o = ET.SubElement(el, "origin")
        el.remove(o)
        el.insert(0, o)
    o.set("xyz", fmt(T[:3, 3]))
    o.set("rpy", fmt(R_to_rpy(T[:3, :3])))


def inertial_data(link):
    """(mass, com in link frame, 3x3 inertia about com in link frame) or None."""
    i = link.find("inertial")
    if i is None:
        return None
    m = float(i.find("mass").get("value"))
    T = origin_T(i)
    a = {k: float(v) for k, v in i.find("inertia").attrib.items()}
    I = np.array([
        [a["ixx"], a["ixy"], a["ixz"]],
        [a["ixy"], a["iyy"], a["iyz"]],
        [a["ixz"], a["iyz"], a["izz"]],
    ])
    R = T[:3, :3]
    return m, T[:3, 3].copy(), R @ I @ R.T


def write_inertial(link, m, com, I):
    old = link.find("inertial")
    if old is not None:
        link.remove(old)
    i = ET.Element("inertial")
    ET.SubElement(i, "mass", value=f"{m:.9g}")
    ET.SubElement(i, "origin", xyz=fmt(com), rpy="0 0 0")
    I = (I + I.T) / 2
    ET.SubElement(i, "inertia", ixx=f"{I[0,0]:.9g}", ixy=f"{I[0,1]:.9g}", ixz=f"{I[0,2]:.9g}",
                  iyy=f"{I[1,1]:.9g}", iyz=f"{I[1,2]:.9g}", izz=f"{I[2,2]:.9g}")
    link.insert(0, i)


def equivalent_box(m, com, I):
    """Box of uniform density with the same mass, COM and inertia -> (size, T)."""
    w, V = np.linalg.eigh(I)
    if np.linalg.det(V) < 0:
        V[:, 2] *= -1
    dims = []
    for k in range(3):
        j, l = [n for n in range(3) if n != k]
        dims.append(np.sqrt(max(6.0 * (w[j] + w[l] - w[k]) / m, 1e-4)))
    T = np.eye(4)
    T[:3, :3] = V
    T[:3, 3] = com
    return dims, T


# ----------------------------------------------------------------------------
# Main processing
# ----------------------------------------------------------------------------


def main(src, dst):
    tree = ET.parse(src)
    robot = tree.getroot()
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = robot.findall("joint")
    report = []

    def parent_of(link_name):
        for j in robot.findall("joint"):
            if j.find("child").get("link") == link_name:
                return j
        return None

    def kids(link_name):
        return [j for j in robot.findall("joint") if j.find("parent").get("link") == link_name]

    # 1. Remove rod chains -----------------------------------------------------
    removed_links, removed_mass = [], 0.0
    starts = [n for n in links if ROD_RE.match(n)
              and not ROD_RE.match(parent_of(n).find("parent").get("link"))]
    for s in starts:
        stack = [s]
        while stack:
            n = stack.pop()
            for j in kids(n):
                stack.append(j.find("child").get("link"))
                robot.remove(j)
            pj = parent_of(n)
            if pj is not None:
                robot.remove(pj)
            d = inertial_data(links[n])
            removed_mass += d[0] if d else 0.0
            robot.remove(links.pop(n))
            removed_links.append(n)
    leftover = [j.get("name") for j in robot.findall("joint") if j.get("name").startswith("ball")]
    assert not leftover, f"ball joints left after rod removal: {leftover}"
    report.append(f"Removed {len(removed_links)} rod/ball links ({len(starts)} chains, "
                  f"{removed_mass*1000:.1f} g total).")

    # 2. Mass overrides --------------------------------------------------------
    report.append("Mass changes:")
    for pattern, (new_m, why) in MASS_OVERRIDES.items():
        hits = [n for n in links if re.match(pattern, n)]
        for n in hits:
            m, com, I = inertial_data(links[n])
            write_inertial(links[n], new_m, com, I * (new_m / m))
            report.append(f"  {n:18s} {m*1000:10.3f} g -> {new_m*1000:7.0f} g   {why}")

    # 3. Collisions (added on original links, re-posed by the merge) -----------
    for n, l in links.items():
        for c in l.findall("collision"):
            l.remove(c)
    for n, l in links.items():
        if WHEEL_LINK_RE.match(n):
            c = ET.SubElement(l, "collision", name=f"{n}_cyl")
            T = np.eye(4)
            T[:3, :3] = rpy_to_R(np.pi / 2, 0, 0)  # cylinder z-axis -> link y-axis
            set_origin(c, T)
            g = ET.SubElement(c, "geometry")
            ET.SubElement(g, "cylinder", radius=f"{WHEEL_RADIUS}", length=f"{WHEEL_WIDTH}")
    for n in BOX_COLLISION_LINKS:
        m, com, I = inertial_data(links[n])
        size, T = equivalent_box(m, com, I)
        c = ET.SubElement(links[n], "collision", name=f"{n}_box")
        set_origin(c, T)
        g = ET.SubElement(c, "geometry")
        ET.SubElement(g, "box", size=fmt(size))
    report.append(f"Collisions: wheel cylinders r={WHEEL_RADIUS} m w={WHEEL_WIDTH} m; "
                  f"equivalent boxes on {', '.join(BOX_COLLISION_LINKS)}.")

    # 4. Merge fixed joints ----------------------------------------------------
    root = [n for n in links if parent_of(n) is None]
    assert len(root) == 1, root
    root = root[0]

    # body -> list of (original link, T_body_link)
    groups = {}

    def walk(n, body, T):
        groups.setdefault(body, []).append((n, T))
        for j in kids(n):
            c = j.find("child").get("link")
            if j.get("type") == "fixed":
                walk(c, body, T @ origin_T(j))
            else:
                walk(c, c, np.eye(4))

    walk(root, root, np.eye(4))
    pose_in_body = {n: (b, T) for b, members in groups.items() for n, T in members}

    for body, members in groups.items():
        bl = links[body]
        M, mc, Is = 0.0, np.zeros(3), []
        for n, T in members:
            d = inertial_data(links[n])
            if d is None:
                continue
            m, com, I = d
            R = T[:3, :3]
            c = R @ com + T[:3, 3]
            M += m
            mc += m * c
            Is.append((m, c, R @ I @ R.T))
        if M > 0:
            C = mc / M
            Itot = np.zeros((3, 3))
            for m, c, I in Is:
                d = c - C
                Itot += I + m * (d @ d * np.eye(3) - np.outer(d, d))
            write_inertial(bl, M, C, Itot)
        for n, T in members:
            if n == body:
                continue
            for tag in ("visual", "collision"):
                for el in links[n].findall(tag):
                    set_origin(el, T @ origin_T(el))
                    bl.append(el)

    # Re-parent movable joints onto their body, drop fixed joints + merged links
    for j in list(robot.findall("joint")):
        if j.get("type") == "fixed":
            robot.remove(j)
            continue
        p = j.find("parent").get("link")
        body, T = pose_in_body[p]
        set_origin(j, T @ origin_T(j))
        j.find("parent").set("link", body)
    for n in list(links):
        if n not in groups:
            robot.remove(links.pop(n))
    report.append(f"Merged fixed joints -> {len(links)} links.")

    # 5. Rename ----------------------------------------------------------------
    # Forward kinematics at zero pose to find where each module sits.
    def world_T(link_name):
        T = np.eye(4)
        n = link_name
        while True:
            j = parent_of(n)
            if j is None:
                return T
            T = origin_T(j) @ T
            n = j.find("parent").get("link")

    left = np.cross(UP, FORWARD)
    jmap, lmap = {}, {root: BASE_LINK_NAME}
    rockers = [j for j in robot.findall("joint") if j.find("parent").get("link") == root
               and j.get("type") in ("continuous", "revolute")]
    assert len(rockers) == 2, [j.get("name") for j in rockers]
    rocker_axes = {}
    for j in rockers:
        child = j.find("child").get("link")
        side = "left" if world_T(child)[:3, 3] @ left > 0 else "right"
        jmap[j.get("name")] = f"rocker_{side}_joint"
        lmap[child] = f"rocker_{side}"
        ax = np.array([float(v) for v in j.find("axis").get("xyz").split()])
        rocker_axes[side] = world_T(child)[:3, :3] @ ax
    coupling = 1 if rocker_axes["left"] @ rocker_axes["right"] < 0 else -1

    for j in robot.findall("joint"):
        if j.get("type") != "continuous" or j.get("name") in jmap:
            continue
        child = j.find("child").get("link")
        grand = kids(child)
        if grand and grand[0].get("type") == "continuous":  # steering: its child spins the wheel
            pos = world_T(child)[:3, 3]
            tag = ("f" if pos @ FORWARD > 0 else "r") + ("l" if pos @ left > 0 else "r")
            jmap[j.get("name")] = f"steer_{tag}_joint"
            lmap[child] = f"steer_{tag}"
            wj = grand[0]
            jmap[wj.get("name")] = f"wheel_{tag}_joint"
            lmap[wj.find("child").get("link")] = f"wheel_{tag}"
    for old, (jn, ln) in ARM_NAMES.items():
        j = [x for x in robot.findall("joint") if x.get("name") == old]
        assert j, f"arm joint {old} not found"
        jmap[old] = jn
        lmap[j[0].find("child").get("link")] = ln

    # Re-zero steering: rotate each steering joint's origin about its own axis so
    # the wheel axle is perpendicular to FORWARD at joint position 0.
    steer_offsets = {}
    if ZERO_STEERING:
        for old, new in jmap.items():
            if not new.startswith("steer_"):
                continue
            j = [x for x in robot.findall("joint") if x.get("name") == old][0]
            child = j.find("child").get("link")
            wj = kids(child)[0]
            a = np.array([float(v) for v in j.find("axis").get("xyz").split()])
            w = origin_T(wj)[:3, :3] @ np.array([float(v) for v in wj.find("axis").get("xyz").split()])
            f = world_T(child)[:3, :3].T @ FORWARD
            phis = np.linspace(-np.pi, np.pi, 720001)
            dots = np.array([abs((axis_angle_T(a, p)[:3, :3] @ w) @ f) for p in phis[::400]])
            p0 = phis[::400][np.argmin(np.where(np.abs(phis[::400]) <= np.pi / 2, dots, np.inf))]
            fine = np.linspace(p0 - 0.01, p0 + 0.01, 20001)
            phi = fine[np.argmin([abs((axis_angle_T(a, p)[:3, :3] @ w) @ f) for p in fine])]
            set_origin(j, origin_T(j) @ axis_angle_T(a, phi))
            steer_offsets[new] = np.degrees(phi)

    unnamed = [j.get("name") for j in robot.findall("joint") if j.get("name") not in jmap]
    assert not unnamed, f"joints without a new name: {unnamed}"
    for j in robot.findall("joint"):
        j.set("name", jmap[j.get("name")])
        j.find("parent").set("link", lmap[j.find("parent").get("link")])
        j.find("child").set("link", lmap[j.find("child").get("link")])
    for n, l in links.items():
        l.set("name", lmap[n])

    # Order: links then joints, base first
    link_els, joint_els = robot.findall("link"), robot.findall("joint")
    for e in link_els + joint_els:
        robot.remove(e)
    for e in sorted(link_els, key=lambda e: (e.get("name") != BASE_LINK_NAME, e.get("name"))):
        robot.append(e)
    for e in sorted(joint_els, key=lambda e: e.get("name")):
        robot.append(e)
    # 6. Unique mesh file per reuse ----------------------------------------------
    meshes = [(l.get("name"), m) for l in robot.findall("link")
              for m in l.iter("mesh")]
    uses = Counter(m.get("filename") for _, m in meshes)
    copies, seen = [], Counter()
    for link_name, m in meshes:
        fn = m.get("filename")
        if uses[fn] < 2:
            continue
        d, base = fn.rsplit("/", 1)
        stem, ext = os.path.splitext(base)
        key = (fn, link_name)
        new_base = f"{stem}__{link_name}_{seen[key]}{ext}"
        seen[key] += 1
        m.set("filename", f"{d}/{new_base}")
        copies.append((base, new_base))
    copies = sorted(set(copies))
    if len(sys.argv) <= 3:
        bat = os.path.join(os.path.dirname(os.path.abspath(dst)), "make_mesh_copies.bat")
        with open(bat, "w", newline="\r\n") as f:
            f.write("@echo off\nrem Run this inside the meshes folder (or double-click it there).\n")
            f.write('cd /d "%~dp0"\n')
            for a, b in copies:
                f.write(f'copy /Y "{a}" "{b}" >nul\n')
            f.write(f"echo Made {len(copies)} mesh copies.\npause\n")
    else:
        mdir = sys.argv[3]
        for a, b in copies:
            shutil.copyfile(os.path.join(mdir, a), os.path.join(mdir, b))
    report.append(f"Reused meshes: {sum(1 for v in uses.values() if v > 1)} files were used more "
                  f"than once -> {len(copies)} unique copies "
                  f"({'written to ' + sys.argv[3] if len(sys.argv) > 3 else 'run make_mesh_copies.bat in the meshes folder'}).")

    robot.insert(0, ET.Comment(
        " Cleaned for Unity by urdf_cleanup.py: rod chains removed (differential is done in "
        "RockerDifferential.cs), masses fixed, collisions added, fixed joints merged, renamed. "))

    ET.indent(tree, space="    ")
    tree.write(dst, encoding="utf-8", xml_declaration=True)

    # Report -------------------------------------------------------------------
    report.append("Renamed joints:")
    for old, new in sorted(jmap.items(), key=lambda kv: kv[1]):
        report.append(f"  {old:14s} -> {new}")
    total = sum(float(l.find("inertial/mass").get("value")) for l in links.values()
                if l.find("inertial") is not None)
    movable = [j for j in robot.findall("joint")]
    report.append(f"Result: {len(links)} links, {len(movable)} joints (all movable), "
                  f"total mass {total:.2f} kg.")
    if steer_offsets:
        report.append("Steering re-zeroed (rotation applied so 0 = straight ahead): " + ", ".join(
            f"{k.replace('_joint', '')} {v:+.1f} deg" for k, v in sorted(steer_offsets.items())))
    report.append(f"Rocker joint axes point {'opposite' if coupling == 1 else 'the same'} "
                  f"directions -> use coupling = {coupling:+d} in RockerDifferential "
                  f"(rocker_left angle = {coupling:+d} x rocker_right angle in joint coordinates).")
    print("\n".join(report))


if __name__ == "__main__":
    if len(sys.argv) not in (3, 4):
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
