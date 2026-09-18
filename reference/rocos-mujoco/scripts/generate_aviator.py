#!/usr/bin/env python3
"""Generate the AVIATOR MJCF, binary mesh assets and 14-drive YAML from URDF.

Run from any working directory. The original URDF and meshes are never modified.
Requires numpy and scipy to compute the collision convex hulls (generation only).
"""

import copy
import math
import json
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull


PROJECT = Path(__file__).resolve().parents[1]
REPOSITORY = PROJECT.parents[1]
PASSIVE = {"roll_input_joint", "pitch_input_joint"}


def numbers(values):
    return " ".join(format(v, ".12g") for v in values)


def pose(element):
    origin = element.find("origin")
    if origin is None:
        return {"pos": "0 0 0"}
    r, p, y = (float(v) / 2 for v in origin.get("rpy", "0 0 0").split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF fixed-axis RPY = Rz(yaw) Ry(pitch) Rx(roll).
    return {"pos": origin.get("xyz", "0 0 0"), "quat": numbers([
        cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])}


def quat_to_rpy(w, x, y, z):
    # pose() 的逆:URDF 固定轴 RPY = Rz(yaw) Ry(pitch) Rx(roll)。
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_from_z_to(direction):
    # 把 +Z 转到单位向量 direction 的单位四元数(wxyz)。
    x, y, z = direction
    if z > 1 - 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    if z < -1 + 1e-12:
        return (0.0, 1.0, 0.0, 0.0)  # 绕 X 转 180°
    axis = (-y, x, 0.0)
    norm = math.hypot(axis[0], axis[1])
    axis = (axis[0] / norm, axis[1] / norm, 0.0)
    half = math.acos(z) / 2
    s = math.sin(half)
    return (math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s)


def binary_stl(source, destination):
    """Copy binary STL or convert ASCII STL without changing triangle geometry."""
    with source.open("rb") as stream:
        header = stream.read(84)
    if len(header) == 84 and source.stat().st_size == 84 + 50 * struct.unpack("<I", header[80:])[0]:
        shutil.copyfile(source, destination)
        return
    triangles = []
    normal, vertices = None, []
    for line in source.read_text().splitlines():
        fields = line.split()
        if fields[:2] == ["facet", "normal"]:
            normal, vertices = list(map(float, fields[2:])), []
        elif fields[:1] == ["vertex"]:
            vertices.extend(map(float, fields[1:]))
        elif fields[:1] == ["endfacet"]:
            if normal is None or len(vertices) != 9:
                raise ValueError(f"Malformed STL facet in {source}")
            triangles.append(struct.pack("<12fH", *normal, *vertices, 0))
    if not triangles:
        raise ValueError(f"No STL triangles in {source}")
    with destination.open("wb") as stream:
        stream.write(b"AVIATOR: converted from source ASCII STL".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(triangles)))
        stream.writelines(triangles)


def write_convex_hull(source, scale, destination):
    """Write the scaled convex hull of a binary STL as a new binary STL.

    MuJoCo uses each collision mesh's convex hull (qhull on the raw vertices);
    replicate that here so Pinocchio sees the same convex polytope instead of
    a concave mesh mislabelled <convex> (which breaks GJK's support walk).
    """
    raw = source.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    vertices = np.array([struct.unpack_from("<9f", raw, 84 + 50 * i + 12) for i in range(count)],
                        dtype=np.float64).reshape(-1, 3)
    points = np.unique(np.round(vertices, 6), axis=0) * np.asarray(scale, dtype=np.float64)
    hull = ConvexHull(points)
    center = points.mean(axis=0)
    triangles = []
    for tri in hull.points[hull.simplices]:
        p0, p1, p2 = tri
        normal = np.cross(p1 - p0, p2 - p0)
        length = float(np.linalg.norm(normal))
        normal = normal / length if length > 1e-12 else np.zeros(3)
        if float(normal @ (p0 - center)) < 0.0:  # inward normal -> flip winding
            normal, p1, p2 = -normal, p2, p1
        triangles.append(struct.pack("<12fH", *normal, *p0, *p1, *p2, 0))
    destination.write_bytes(b"AVIATOR convex hull".ljust(80, b"\0") +
                            struct.pack("<I", len(triangles)) + b"".join(triangles))


def collision_parts(path):
    """Split disconnected CAD components before MuJoCo constructs convex hulls.

    In particular, a single aircraft hull would fill the empty cockpit between
    the rear mounting bracket and front panel. Triangle coordinates are kept.
    """
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    parents, vertices, faces = [], {}, []

    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i

    for index in range(count):
        xyz = struct.unpack_from("<9f", raw, 84 + 50 * index + 12)
        face = []
        for i in range(0, 9, 3):
            key = tuple(round(x, 6) for x in xyz[i:i + 3])
            if key not in vertices:
                vertices[key] = len(parents)
                parents.append(len(parents))
            face.append(vertices[key])
        parents[root(face[1])] = root(face[0])
        parents[root(face[2])] = root(face[0])
        faces.append(face[0])
    groups = {}
    for index, vertex in enumerate(faces):
        groups.setdefault(root(vertex), []).append(raw[84 + 50 * index:134 + 50 * index])
    files = []
    for index, triangles in enumerate(groups.values()):
        dest = path.with_name(f"{path.stem}_collision_{index}.stl")
        dest.write_bytes(b"AVIATOR CAD connected component".ljust(80, b"\0") +
                         struct.pack("<I", len(triangles)) + b"".join(triangles))
        files.append(dest.name)
    return files


def generate():
    urdf = ET.parse(REPOSITORY / "urdf/aviator.urdf").getroot()
    links = {link.get("name"): link for link in urdf.findall("link")}
    joints = {joint.get("name"): joint for joint in urdf.findall("joint")}
    control_dir = REPOSITORY / "AviatorRobot/config"
    grasp = json.loads((control_dir / "grasp.json").read_text())
    posture = json.loads((control_dir / "posture.json").read_text())
    children = {}
    for joint in joints.values():
        children.setdefault(joint.find("parent").get("link"), []).append(joint)
    model = ET.Element("mujoco", model="aviator")
    model.append(ET.Comment("Generated by scripts/generate_aviator.py from urdf/aviator.urdf."))
    ET.SubElement(model, "compiler", angle="radian", meshdir="aviator_meshes", autolimits="true", inertiafromgeom="false")
    ET.SubElement(model, "option", timestep="0.001", gravity="0 0 -9.81", integrator="implicitfast", cone="elliptic")
    defaults = ET.SubElement(model, "default")
    visual = ET.SubElement(defaults, "default", {"class": "visual"})
    ET.SubElement(visual, "geom", type="mesh", contype="0", conaffinity="0", group="2", density="0")
    collision = ET.SubElement(defaults, "default", {"class": "collision"})
    ET.SubElement(collision, "geom", type="mesh", group="3", rgba="0 0 0 0", friction="0.8 0.005 0.0001",
                  density="0", contype="1", conaffinity="7")
    # Reuse the existing scene's lighting, sky and ground appearance.
    scene = ET.parse(PROJECT / "model/scene.xml").getroot()
    ET.SubElement(model, "statistic", center="-0.4 0 0.1", extent="1.5")
    model.append(copy.deepcopy(scene.find("visual")))
    custom = ET.SubElement(model, "custom")
    ET.SubElement(custom, "numeric", name="viewer_camera", data="-0.4 0 0.1 2.5 130 -20")
    asset = copy.deepcopy(scene.find("asset"))
    model.append(asset)
    materials = {m.get("name"): m.find("color").get("rgba") for m in urdf.findall("material")}
    materials.setdefault("white", "1 1 1 1")  # Referenced but not defined in the URDF.
    for name, rgba in materials.items():
        ET.SubElement(asset, "material", name=name, rgba=rgba)
    meshdir = PROJECT / "model/aviator_meshes"
    meshdir.mkdir(parents=True, exist_ok=True)
    meshes = {}
    parts = {}
    for mesh in urdf.findall(".//mesh"):
        filename = Path(mesh.get("filename")).name
        scale = numbers(map(float, mesh.get("scale", "1 1 1").split()))
        key = (filename, scale)
        if key not in meshes:
            meshes[key] = Path(filename).stem
            binary_stl(REPOSITORY / "meshes" / filename, meshdir / filename)
            ET.SubElement(asset, "mesh", name=meshes[key], file=filename, scale=scale)
            if filename in ("aircraft.STL", "steering_wheel.STL"):
                parts[meshes[key]] = collision_parts(meshdir / filename)
                for component in parts[meshes[key]]:
                    ET.SubElement(asset, "mesh", name=Path(component).stem, file=component, scale=scale)
    world = copy.deepcopy(scene.find("worldbody"))
    model.append(world)
    # Keep the URDF world origin; lower the decorative floor below the cockpit.
    world.find("geom[@name='floor']").set("pos", "0 0 -0.3")
    world.find("geom[@name='floor']").set("conaffinity", "7")

    def add_joint(body, joint):
        limit = joint.find("limit")
        attributes = dict(name=joint.get("name"), type="slide" if joint.get("type") == "prismatic" else "hinge",
                          axis=joint.find("axis").get("xyz"), range=f"{limit.get('lower')} {limit.get('upper')}")
        if joint.get("name") in PASSIVE:
            # Simulation damping only; no actuator, spring or return-to-center.
            attributes["damping"] = "0.1" if joint.get("type") == "revolute" else "1"
        ET.SubElement(body, "joint", attributes)

    def add_link(parent, name, incoming=None):
        link = links[name]
        body = ET.SubElement(parent, "body", name=name, **(pose(incoming) if incoming is not None else {}))
        if incoming is not None and incoming.get("type") != "fixed":
            add_joint(body, incoming)
        if name == "dummy_link":
            # These two coaxial joints have coincident frames. Put both on the
            # wheel body, preserving joint order and avoiding a massless body.
            pitch = joints["pitch_input_joint"]
            assert pose(pitch) == {"pos": "0 0 0", "quat": "1 0 0 0"}
            assert len(children[name]) == 1
            name, link = "steering_wheel", links["steering_wheel"]
            body.set("name", name)
            add_joint(body, pitch)
        inertial = link.find("inertial")
        if inertial is not None and float(inertial.find("mass").get("value")) > 0:
            inertia = inertial.find("inertia")
            # All AVIATOR inertia frames have zero RPY; fullinertia preserves
            # the off-diagonal entries without re-estimating from mesh volume.
            assert inertial.find("origin").get("rpy", "0 0 0") == "0 0 0"
            ET.SubElement(body, "inertial", pos=inertial.find("origin").get("xyz"),
                          mass=inertial.find("mass").get("value"),
                          fullinertia=" ".join(inertia.get(k) for k in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]))
        for kind in ["visual", "collision"]:
            for index, geometry in enumerate(link.findall(kind)):
                mesh = geometry.find("geometry/mesh")
                key = (Path(mesh.get("filename")).name, numbers(map(float, mesh.get("scale", "1 1 1").split())))
                attributes = {"name": f"{name}_{kind}_{index}", "class": kind, "mesh": meshes[key], **pose(geometry)}
                if kind == "visual":
                    attributes["material"] = geometry.find("material").get("name")
                if kind == "collision" and meshes[key] in parts:
                    for component in parts[meshes[key]]:
                        ET.SubElement(body, "geom", {**attributes, "name": Path(component).stem,
                                                     "mesh": Path(component).stem})
                else:
                    ET.SubElement(body, "geom", attributes)
        for joint in children.get(name, []):
            add_link(body, joint.find("child").get("link"), joint)

    add_link(world, "aircraft")
    tool = grasp["tool"]
    radius, length, mass = (float(tool[k]) for k in ("radius", "length", "mass"))
    assert radius > 0 and length > 0 and mass > 0
    stem_length = math.sqrt(sum(v * v for v in tool["position"]))
    direction = [v / stem_length for v in tool["position"]]
    w, x, y, z = tool["quaternion"]
    cylinder_axis = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]
    axial = abs(sum(a * b for a, b in zip(direction, cylinder_axis)))
    support = length / 2 * axial + radius * math.sqrt(max(0, 1 - axial * axial))
    mount_end = stem_length - support - tool["mount_radius"]
    assert mount_end > 0, "Cylinder intersects the flange: increase tool offset"
    for side, suffix in [("left", "L"), ("right", "R")]:
        flange = world.find(f".//body[@name='AR5-5_07{suffix}-W4C4A2_flan_link']")
        proxy = ET.SubElement(flange, "body", name=f"{side}_gripper",
                              pos=numbers(tool["position"]), quat=numbers(tool["quaternion"]))
        transverse = mass * (3 * radius * radius + length * length) / 12
        ET.SubElement(proxy, "inertial", pos="0 0 0", mass=numbers([mass]),
                      diaginertia=numbers([transverse, transverse, mass * radius * radius / 2]))
        # Bits 2/4 identify the left/right grasp proxies. Only the matching
        # handle shell/buttons omit that bit; the rest of the scene accepts both.
        bit, other = (2, 4) if side == "left" else (4, 2)
        ET.SubElement(proxy, "geom", name=f"{side}_grasp_cylinder", type="cylinder",
                      size=numbers([radius, length / 2]), rgba="0.1 0.8 0.9 0.55",
                      contype=str(bit), conaffinity=str(other), group="0", mass="0")
        ET.SubElement(proxy, "site", name=f"{side}_tcp", size="0.005", rgba="1 0.3 0.1 1")
        ET.SubElement(flange, "geom", name=f"{side}_tool_mount", type="capsule",
                      fromto="0 0 0 " + numbers(mount_end * v for v in direction), size=str(tool["mount_radius"]),
                      rgba="0.45 0.45 0.45 1", contype="1", conaffinity="7", group="0", mass="0")
        for name in grasp["handle_geometry"][side]["overlap_geoms"]:
            geom = world.find(f".//geom[@name='{name}']")
            assert geom is not None, f"Missing handle collision geometry: {name}"
            geom.set("contype", "1")
            geom.set("conaffinity", str(7 & ~bit))
        wheel = world.find(".//body[@name='steering_wheel']")
        ET.SubElement(wheel, "site", name=f"{side}_handle", pos=numbers(grasp[side]["position"]),
                      quat=numbers(grasp[side]["quaternion"]), size="0.008", rgba="0.2 1 0.2 1")
    equality = ET.SubElement(model, "equality")
    for side in ["left", "right"]:
        ET.SubElement(equality, "weld", name=f"{side}_grasp", site1=f"{side}_tcp", site2=f"{side}_handle",
                      active="false", solref="0.005 1", solimp="0.95 0.99 0.001")
    # CAD meshes overlap at fitted interfaces; their convex collision hulls
    # must not push apart the wheel bearing, shoulder bearing or wrist housing.
    # Keep all other collision pairs, including arm/wheel contact, enabled.
    contact = ET.SubElement(model, "contact")
    ET.SubElement(contact, "exclude", body1="aircraft", body2="steering_wheel")
    for side in "LR":
        prefix = f"AR5-5_07{side}-W4C4A2_"
        ET.SubElement(contact, "exclude", body1=prefix + "base", body2=prefix + "link1")
        ET.SubElement(contact, "exclude", body1=prefix + "link5", body2=prefix + "link7")
    # A keyframe changes the initial configuration without changing joint zero
    # references or the URDF/MJCF kinematic correspondence.
    home = {}
    for side, suffix in [("left", "L"), ("right", "R")]:
        values = posture[f"{side}_home_deg"]
        assert len(values) == 7 and 85 <= values[1] <= 95
        for axis, degrees in enumerate(values, 1):
            name = f"AR5-5_07{suffix}-W4C4A2_joint_{axis}"
            value = math.radians(degrees)
            limit = joints[name].find("limit")
            assert math.isfinite(value) and float(limit.get("lower")) <= value <= float(limit.get("upper"))
            home[name] = value
    keyframes = ET.SubElement(model, "keyframe")
    ET.SubElement(keyframes, "key", name="aviator_home",
                  qpos=numbers(home.get(j.get("name"), 0) for j in world.findall(".//joint")))
    # The existing simulator applies joint forces to configured drivers. The
    # passive wheel joints deliberately have neither actuators nor YAML drivers.
    ET.indent(model, space="  ")
    ET.ElementTree(model).write(PROJECT / "model/aviator.xml", encoding="utf-8", xml_declaration=True)
    hardware = ["# Generated by scripts/generate_aviator.py from urdf/aviator.urdf.",
                "# Left arm: slaves 0..6; right arm: 7..13. Wheel joints are passive.",
                "# lower/upper/vel/effort are URDF values (rad, rad/s, N*m).",
                "# acc, jerk and encoder/torque transforms are simulation defaults,",
                "# not measured hardware parameters. All encoder zero offsets are 0.",
                "# MJCF enforces position ranges. This simulator does not enforce",
                "# YAML velocity/effort limits; clients must respect those limits.", "hardware:"]
    for side in "LR":
        for axis in range(1, 8):
            name = f"AR5-5_07{side}-W4C4A2_joint_{axis}"
            limit = joints[name].find("limit")
            slave = (0 if side == "L" else 7) + axis - 1
            hw = ET.SubElement(joints[name], "hardware", id=str(slave), type="driver")
            ET.SubElement(hw, "limit", lower=limit.get("lower"), upper=limit.get("upper"),
                          vel=limit.get("velocity"), acc="10", jerk="150", effort=limit.get("effort"))
            ET.SubElement(hw, "transform", ratio="1", offset_pos_cnt="0", cnt_per_unit="156455.678",
                          torque_per_unit="1", user_unit_name="rad")
            for section, fields in {
                "inputs": {"status_word": "Status word", "position_actual_value": "Position actual value",
                           "velocity_actual_value": "Velocity actual value", "torque_actual_value": "Torque actual value",
                           "load_torque_value": "Analog Input 1", "secondary_position_value": "Auxiliary position actual value"},
                "outputs": {"control_word": "Control word", "mode_of_operation": "Mode of operation",
                            "target_position": "Target Position", "target_velocity": "Target Velocity", "target_torque": "Target Torque"}
            }.items():
                node = ET.SubElement(hw, section)
                for key, value in fields.items():
                    ET.SubElement(node, key).text = value
            hardware.append(f"""  - id: {slave}
    type: driver
    joint_name: {name}
    torque_source: load_torque
    limit:
      lower: {limit.get('lower')}
      upper: {limit.get('upper')}
      vel: {limit.get('velocity')}
      acc: 10.0
      jerk: 150.0
      effort: {limit.get('effort')}
    transform:
      ratio: 1.0
      offset_pos_cnt: 0
      cnt_per_unit: 156455.678
      torque_per_unit: 1.0
      user_unit_name: rad
    inputs:
      status_word: "Status word"
      position_actual_value: "Position actual value"
      velocity_actual_value: "Velocity actual value"
      torque_actual_value: "Torque actual value"
      load_torque_value: "Analog Input 1"
      secondary_position_value: "Auxiliary position actual value"
    outputs:
      control_word: "Control word"
      mode_of_operation: "Mode of operation"
      target_position: "Target Position"
      target_velocity: "Target Velocity"
      target_torque: "Target Torque"
""")
    hardware.append("aviator_grasp:\n  enabled: true\n  position_tolerance: 0.003\n  rotation_tolerance: 0.035\n  speed_tolerance: 0.02\n  stable_time: 0.1\n")
    (PROJECT / "config/hardware_aviator_config.yaml").write_text("\n".join(hardware))
    if urdf.find("material[@name='white']") is None:
        ET.SubElement(ET.SubElement(urdf, "material", name="white"), "color", rgba="1 1 1 1")
    ET.indent(urdf, space="  ")
    ET.ElementTree(urdf).write(control_dir / "aviator_control.urdf", encoding="utf-8", xml_declaration=True)
    generate_collision_assets(links, joints, grasp, meshes, parts)
    print(f"Generated aviator.xml, {len(meshes)} mesh assets, and 14-drive YAML")


def generate_collision_assets(links, joints, grasp, meshes, parts):
    """导出 Pinocchio 碰撞模型:分片凸网格 + 圆柱/胶囊工具 + SRDF 排除对。

    只保留 <collision>,网格路径用 package://aviator_meshes 并由碰撞后端定位;
    所有网格标记 <convex>(与 MuJoCo 凸包语义一致),安装柱标记 <capsule>。
    """
    robot = ET.Element("robot", name="aviator_collision")
    meshdir = PROJECT / "model/aviator_meshes"

    def add_collision(link_el, name, origin_xyz, origin_rpy, geometry):
        collision = ET.SubElement(link_el, "collision", name=name)
        ET.SubElement(collision, "origin", xyz=origin_xyz, rpy=origin_rpy)
        collision.append(geometry)

    def mesh_geometry(filename, scale):
        geometry = ET.Element("geometry")
        ET.SubElement(geometry, "mesh", filename="package://aviator_meshes/" + filename, scale=scale)
        return geometry

    convex_names = {}
    capsule_names = {}

    for link in links.values():
        name = link.get("name")
        link_el = ET.SubElement(robot, "link", name=name)
        convex_names[name] = []
        capsule_names[name] = []
        inertial = link.find("inertial")
        if inertial is not None:
            link_el.append(copy.deepcopy(inertial))
        for collision in link.findall("collision"):
            origin = collision.find("origin")
            xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            filename = Path(mesh.get("filename")).name
            scale = numbers(map(float, mesh.get("scale", "1 1 1").split()))
            scale_values = tuple(map(float, scale.split()))
            stem = meshes.get((filename, scale))
            if stem is None:
                raise ValueError(f"Collision mesh not registered: {filename}")
            components = parts.get(stem)
            if components:
                for component in components:
                    hull_name = Path(component).stem + "_ch.stl"
                    write_convex_hull(meshdir / component, scale_values, meshdir / hull_name)
                    add_collision(link_el, Path(component).stem, xyz, rpy,
                                  mesh_geometry(hull_name, "1 1 1"))
                    convex_names[name].append(Path(component).stem)
            else:
                hull_name = Path(filename).stem + "_ch.stl"
                write_convex_hull(meshdir / filename, scale_values, meshdir / hull_name)
                add_collision(link_el, Path(filename).stem, xyz, rpy,
                              mesh_geometry(hull_name, "1 1 1"))
                convex_names[name].append(Path(filename).stem)

    # 圆柱工具 + 安装柱(胶囊),几何量与 MJCF 生成保持一致。
    tool = grasp["tool"]
    radius, length, mass = (float(tool[k]) for k in ("radius", "length", "mass"))
    mount_radius = float(tool["mount_radius"])
    stem_length = math.sqrt(sum(v * v for v in tool["position"]))
    direction = [v / stem_length for v in tool["position"]]
    w, x, y, z = tool["quaternion"]
    cylinder_axis = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]
    axial = abs(sum(a * b for a, b in zip(direction, cylinder_axis)))
    support = length / 2 * axial + radius * math.sqrt(max(0, 1 - axial * axial))
    mount_end = stem_length - support - mount_radius
    assert mount_end > 0, "Cylinder intersects the flange: increase tool offset"
    mount_xyz = numbers(mount_end / 2 * v for v in direction)
    mount_rpy = numbers(quat_to_rpy(*quat_from_z_to(direction)))
    cylinder_xyz = numbers(tool["position"])
    cylinder_rpy = numbers(quat_to_rpy(w, x, y, z))
    for side, suffix in [("left", "L"), ("right", "R")]:
        link_el = robot.find(f".//link[@name='AR5-5_07{suffix}-W4C4A2_flan_link']")
        assert link_el is not None, "Missing flange link"
        mount = ET.Element("geometry")
        ET.SubElement(mount, "cylinder", radius=numbers([mount_radius]), length=numbers([mount_end]))
        add_collision(link_el, f"{side}_tool_mount", mount_xyz, mount_rpy, mount)
        capsule_names[link_el.get("name")].append(f"{side}_tool_mount")
        cylinder = ET.Element("geometry")
        ET.SubElement(cylinder, "cylinder", radius=numbers([radius]), length=numbers([length]))
        add_collision(link_el, f"{side}_grasp_cylinder", cylinder_xyz, cylinder_rpy, cylinder)

    # 标记凸网格与胶囊,供 Pinocchio URDF 解析器构建凸包/胶囊几何。
    for link_el in robot.findall("link"):
        name = link_el.get("name")
        if not convex_names.get(name) and not capsule_names.get(name):
            continue
        cc = ET.SubElement(link_el, "collision_checking")
        for geom_name in convex_names.get(name, []):
            ET.SubElement(cc, "convex", name=geom_name)
        for geom_name in capsule_names.get(name, []):
            ET.SubElement(cc, "capsule", name=geom_name)

    # dummy_link 是 roll/pitch 之间的运动体,补一个微量惯量避免零惯量运动链。
    dummy = robot.find(".//link[@name='dummy_link']")
    inertial = ET.SubElement(dummy, "inertial")
    ET.SubElement(inertial, "origin", rpy="0 0 0", xyz="0 0 0")
    ET.SubElement(inertial, "mass", value="1e-6")
    ET.SubElement(inertial, "inertia", ixx="1e-9", ixy="0", ixz="0", iyy="1e-9", iyz="0", izz="1e-9")

    # 原样复制全部关节(parent/child/origin/axis/limit)。
    for joint in joints.values():
        robot.append(copy.deepcopy(joint))

    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(PROJECT / "model/aviator_collision.urdf", encoding="utf-8", xml_declaration=True)

    # SRDF:镜像 MuJoCo 默认的 mjDSBL_FILTERPARENT —— 直接父子 link 之间不检测碰撞,
    # 即使凸包在贴合面重叠;再叠加非相邻的结构性排除(轮盘轴承/腕部壳体)。
    # 用 link 名枚举父子对(而非 Pinocchio 的 parentJoint),因为 Pinocchio 会把 fixed
    # joint 合并进父体,导致 aircraft/base/link1 的父子关系被压平,parentJoint 无法区分。
    srdf = ET.Element("robot", name="aviator_collision")
    has_collision = lambda name: bool(convex_names.get(name)) or bool(capsule_names.get(name))
    for joint in joints.values():
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        if has_collision(parent) and has_collision(child):
            ET.SubElement(srdf, "disable_collisions", link1=parent, link2=child, reason="parent-child")
    for a, b, reason in [
        ("aircraft", "steering_wheel", "wheel bearing"),
        ("AR5-5_07L-W4C4A2_link5", "AR5-5_07L-W4C4A2_link7", "wrist housing"),
        ("AR5-5_07R-W4C4A2_link5", "AR5-5_07R-W4C4A2_link7", "wrist housing"),
    ]:
        ET.SubElement(srdf, "disable_collisions", link1=a, link2=b, reason=reason)
    ET.indent(srdf, space="  ")
    ET.ElementTree(srdf).write(PROJECT / "model/aviator_collision.srdf", encoding="utf-8", xml_declaration=True)
    print("Generated aviator_collision.urdf + aviator_collision.srdf")


if __name__ == "__main__":
    generate()
