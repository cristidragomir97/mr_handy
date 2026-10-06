"""Convert the expanded, imported URDF into a derived MuJoCo scene.

URDF stays authoritative for topology, datums, geometry, mass and joint limits.
No chassis or arm meshes are copied. Absolute resolved assets reference their
owning packages. Primitive URDF collisions stay separate from visual meshes.
Self contact is disabled for initial controller commissioning because the CAD
mechanisms and coarse collisions overlap by design. Robot/environment contact
remains enabled; this is not yet a grasp/contact fidelity model.
"""

import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def values(text):
    return [float(x) for x in text.split()]


def fmt(numbers):
    return " ".join(f"{x:.12g}" for x in numbers)


def quat(rpy):
    r, p, y = [v / 2 for v in rpy]
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def origin(element):
    o = element.find("origin")
    return {
        "pos": o.attrib.get("xyz", "0 0 0") if o is not None else "0 0 0",
        "quat": fmt(quat(values(o.attrib.get("rpy", "0 0 0")))) if o is not None else "1 0 0 0",
    }


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def resolve_mesh(filename, urdf_dir):
    if filename.startswith("package://"):
        from ament_index_python.packages import get_package_share_directory

        package, path = filename[10:].split("/", 1)
        return Path(get_package_share_directory(package)) / path
    if filename.startswith("file://"):
        filename = filename[7:]
    return (urdf_dir / filename).resolve()


def build_scene(urdf, destination, *, urdf_dir=Path("."), floating=True):
    """Write a deterministic scene from the exact robot description used by ROS."""
    robot = ET.fromstring(urdf)
    links = {l.attrib["name"]: l for l in robot.findall("link")}
    joints = {j.attrib["name"]: j for j in robot.findall("joint")}
    controlled = {
        j.attrib["name"]: j for s in robot.findall("ros2_control") for j in s.findall("joint")
    }
    movable = {n for n, j in joints.items() if j.attrib["type"] != "fixed"}
    if movable != set(controlled):
        raise ValueError(
            f"Joint/control mismatch: missing={movable - set(controlled)}, extra={set(controlled) - movable}"
        )
    children = {}
    for j in joints.values():
        children.setdefault(j.find("parent").attrib["link"], []).append(j)
    child_names = {j.find("child").attrib["link"] for j in joints.values()}
    roots = set(links) - child_names
    if len(roots) != 1:
        raise ValueError(f"Expected one URDF root, got {roots}")
    scene = ET.Element("mujoco", model="handy101")
    scene.append(
        ET.Comment(
            "Derived from imported URDF; source meshes remain in base101/mod101/handy101_description. Simulation gains are provisional."
        )
    )
    ET.SubElement(scene, "compiler", angle="radian", inertiafromgeom="false", fusestatic="false")
    ET.SubElement(scene, "option", timestep="0.002", integrator="implicitfast", gravity="0 0 -9.81")
    ET.SubElement(scene, "visual").append(ET.Element("global", offwidth="1600", offheight="1200"))
    asset = ET.SubElement(scene, "asset")
    mesh_assets = {}
    materials = {
        m.attrib["name"]: m.find("color").attrib["rgba"]
        for m in robot.findall("material")
        if m.find("color") is not None
    }
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "light", pos="0 -2 3", dir="0 0 -1", directional="true")
    ET.SubElement(
        world,
        "geom",
        name="floor",
        type="plane",
        size="10 10 .05",
        rgba=".25 .28 .32 1",
        contype="2",
        conaffinity="1",
        friction="1 .005 .0001",
    )
    actuator = ET.SubElement(scene, "actuator")
    equality = ET.SubElement(scene, "equality")

    def geometry(body, element, name, visual):
        g = next(iter(element.find("geometry")))
        attrs = {
            "name": name,
            **origin(element),
            "group": "2" if visual else "3",
            "contype": "0" if visual else "1",
            "conaffinity": "0" if visual else "2",
        }
        if visual:
            mat = element.find("material")
            rgba = ".4 .4 .4 1"
            if mat is not None:
                color = mat.find("color")
                rgba = (
                    color.attrib["rgba"]
                    if color is not None
                    else materials.get(mat.attrib.get("name"), rgba)
                )
            attrs["rgba"] = rgba
        else:
            attrs.update(rgba="0 0 0 0", friction="1 .005 .0001", condim="4")
        if g.tag == "mesh":
            path = resolve_mesh(g.attrib["filename"], urdf_dir)
            if not path.is_file():
                raise FileNotFoundError(path)
            key = (str(path), g.attrib.get("scale", "1 1 1"))
            if key not in mesh_assets:
                mesh_name = "mesh_" + hashlib.sha256("|".join(key).encode()).hexdigest()[:16]
                ET.SubElement(asset, "mesh", name=mesh_name, file=str(path), scale=key[1])
                mesh_assets[key] = mesh_name
            attrs.update(type="mesh", mesh=mesh_assets[key])
        elif g.tag == "box":
            attrs.update(type="box", size=fmt([v / 2 for v in values(g.attrib["size"])]))
        elif g.tag == "cylinder":
            attrs.update(
                type="cylinder",
                size=fmt([float(g.attrib["radius"]), float(g.attrib["length"]) / 2]),
            )
        elif g.tag == "sphere":
            attrs.update(type="sphere", size=g.attrib["radius"])
        else:
            raise ValueError("Unsupported geometry " + g.tag)
        ET.SubElement(body, "geom", **attrs)

    def body_for(parent, name, joint=None):
        attrs = {"name": name, **(origin(joint) if joint is not None else {"pos": "0 0 .002"})}
        body = ET.SubElement(parent, "body", **attrs)
        if joint is None and floating:
            ET.SubElement(body, "freejoint", name="floating_base_joint")
        if joint is not None and joint.attrib["type"] != "fixed":
            jname = joint.attrib["name"]
            limit = joint.find("limit")
            dynamics = joint.find("dynamics")
            attrs = {
                "name": jname,
                "type": "slide" if joint.attrib["type"] == "prismatic" else "hinge",
                "axis": joint.find("axis").attrib["xyz"],
            }
            if dynamics is not None:
                attrs["damping"] = dynamics.attrib.get("damping", "0")
            if joint.attrib["type"] != "continuous":
                attrs.update(
                    limited="true", range=limit.attrib["lower"] + " " + limit.attrib["upper"]
                )
            mimic = joint.find("mimic")
            if mimic is not None or jname.endswith("_6"):
                attrs["damping"] = "0.002"
                # Reflected drive inertia regularizes the source frame-only coupler.
                attrs["armature"] = "0.0001"
            ET.SubElement(body, "joint", **attrs)
            if mimic is not None:
                ET.SubElement(
                    equality,
                    "joint",
                    name=jname + "_mimic",
                    joint1=jname,
                    joint2=mimic.attrib["joint"],
                    polycoef=fmt(
                        [
                            float(mimic.attrib.get("offset", "0")),
                            float(mimic.attrib.get("multiplier", "1")),
                            0,
                            0,
                            0,
                        ]
                    ),
                    solref="0.02 1",
                    solimp="0.999 0.999 0.001",
                )
            command_interface = controlled[jname].find("command_interface")
            command = command_interface.attrib["name"] if command_interface is not None else None
            if command == "velocity":
                # Drive speed and radius settings remain owned by base101_control.
                ET.SubElement(
                    actuator,
                    "velocity",
                    name=jname,
                    joint=jname,
                    kv="5",
                    forcelimited="true",
                    forcerange="-5 5",
                )
            elif command == "position":
                lift = joint.attrib["type"] == "prismatic"
                head = jname == "head_camera_tilt_joint"
                force = float(limit.attrib["effort"])
                kp = 20000 if lift else 100 if head else 5 if jname.endswith("_6") else 200
                ET.SubElement(
                    actuator,
                    "position",
                    name=jname,
                    joint=jname,
                    kp=str(kp),
                    dampratio="1",
                    ctrllimited="true",
                    ctrlrange=limit.attrib["lower"] + " " + limit.attrib["upper"],
                    forcelimited="true",
                    forcerange=fmt([-force, force]),
                )
            elif mimic is None:
                raise ValueError("Unsupported command " + str(command))
        link = links[name]
        inertial = link.find("inertial")
        if inertial is not None:
            attrs = origin(inertial)
            attrs.pop("quat")
            i = inertial.find("inertia").attrib
            matrix = [
                [float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
                [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
                [float(i["ixz"]), float(i["iyz"]), float(i["izz"])],
            ]
            o = inertial.find("origin")
            R = rotation(values(o.attrib.get("rpy", "0 0 0")) if o is not None else [0, 0, 0])
            tensor = [
                [
                    sum(R[a][k] * matrix[k][l] * R[b][l] for k in range(3) for l in range(3))
                    for b in range(3)
                ]
                for a in range(3)
            ]
            ET.SubElement(
                body,
                "inertial",
                **attrs,
                mass=inertial.find("mass").attrib["value"],
                fullinertia=fmt(
                    [
                        tensor[0][0],
                        tensor[1][1],
                        tensor[2][2],
                        tensor[0][1],
                        tensor[0][2],
                        tensor[1][2],
                    ]
                ),
            )
        for kind, visual in [("visual", True), ("collision", False)]:
            for index, element in enumerate(link.findall(kind)):
                geometry(body, element, f"{name}_{kind}_{index}", visual)
        for child in children.get(name, []):
            body_for(body, child.find("child").attrib["link"], child)

    body_for(world, roots.pop())
    # These sensors are evaluated in the same live mjData as the wheel/arm physics.
    sensors = ET.SubElement(scene, "sensor")
    bodies = {b.get("name"): b for b in world.iter("body")}
    by_child = {j.find("child").get("link"): j for j in joints.values()}

    def multiply_quat(a, b):
        aw, ax, ay, az = a; bw, bx, by, bz = b
        return [aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
                aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw]

    def frame_mount(frame, stop=None):
        """Resolve fixed URDF chains onto their nearest moving/root body.

        MuJoCo fixed-body fusion preserves geoms/inertials but loses camera
        local transforms. Attach sensors directly to a body that cannot fuse.
        """
        pos = [0., 0., 0.]; orientation = [1., 0., 0., 0.]
        while frame != stop and frame in by_child and by_child[frame].get("type") == "fixed":
            joint = by_child[frame]; o = joint.find("origin")
            xyz = values(o.get("xyz", "0 0 0")) if o is not None else [0., 0., 0.]
            rpy = values(o.get("rpy", "0 0 0")) if o is not None else [0., 0., 0.]
            R = rotation(rpy)
            pos = [xyz[i] + sum(R[i][k]*pos[k] for k in range(3)) for i in range(3)]
            orientation = multiply_quat(quat(rpy), orientation)
            frame = joint.find("parent").get("link")
        return bodies[frame], pos, orientation

    imu_body, imu_pos, imu_quat = frame_mount("imu_link")
    ET.SubElement(imu_body, "site", name="base_imu_site", pos=fmt(imu_pos), quat=fmt(imu_quat), size="0.001", rgba="0 0 0 0")
    ET.SubElement(sensors, "framequat", name="base_imu_quat", objtype="site", objname="base_imu_site")
    ET.SubElement(sensors, "gyro", name="base_imu_gyro", site="base_imu_site")
    ET.SubElement(sensors, "accelerometer", name="base_imu_accel", site="base_imu_site")
    # Rangefinders exclude only their owning body's geometry. Keep rays on the
    # lidar housing body so its rotor shell cannot occlude every measurement.
    lidar_housing = by_child["lidar_frame"].find("parent").get("link")
    lidar_body, lidar_pos, lidar_quat = frame_mount("lidar_frame", stop=lidar_housing)
    for i in range(360):
        angle = -math.pi + i * 2 * math.pi / 359
        site = "base_lidar-" + str(i)
        # Native rangefinders cast along local +Z; rotate it into the lidar XY plane.
        ET.SubElement(lidar_body, "site", name=site, pos=fmt(lidar_pos), size="0.001",
                      quat=fmt(multiply_quat(lidar_quat, quat([0, math.pi/2, angle]))), rgba="0 0 0 0")
        ET.SubElement(sensors, "rangefinder", name=site, site=site, cutoff="12")
    camera_frames = [
        ("base_camera", "camera_optical_frame", 87),
        ("head_camera", "head_camera_optical_frame", 87),
        ("left_arm_camera", "left_arm_camera_tool_optical_frame", 69),
        ("left_arm_gopro", "left_arm_camera_tool_gopro_optical_frame", 120),
        ("right_wrist_camera", "right_arm_wrist_camera_optical_frame", 69),
    ]
    for camera, frame, hfov in camera_frames:
        if frame in bodies:
            fovy = 2 * math.atan(math.tan(math.radians(hfov)/2) * 480/640)
            # MuJoCo looks along -Z with +Y up; ROS optical has +Z forward/+Y down.
            camera_body, camera_pos, camera_quat = frame_mount(frame)
            ET.SubElement(camera_body, "camera", name=camera, mode="fixed", pos=fmt(camera_pos),
                          quat=fmt(multiply_quat(camera_quat, [0, 1, 0, 0])),
                          resolution="640 480", fovy=str(math.degrees(fovy)))
    ET.indent(scene, space="  ")
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(scene).write(destination, encoding="unicode")
    return destination
