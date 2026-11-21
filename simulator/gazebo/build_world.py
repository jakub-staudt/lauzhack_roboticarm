"""Generate a Gazebo (gz sim) SDF world of the gaze station from the compiled MuJoCo model.

The arm's links, joints, limits, inertias, collision shapes and meshes come from the same MuJoCo model the
`simulator` package uses, so Gazebo and MuJoCo describe the identical SO-101 and table. Run:

    robotic_arm/.venv/bin/python -m simulator.gazebo.build_world        # writes simulator/gazebo/station.sdf
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from xml.sax.saxutils import quoteattr

import mujoco
import numpy as np

from simulator.run import build_model, load_station
from soarm import ROBOT_DIR

HERE = Path(__file__).resolve().parent
OUT = HERE / "station.sdf"
MODEL = "so101"
WORLD = "gaze_station"
# P, I and D gains of the joint position controllers; the servo's torque limit is 2.94 N m.
KP, KI, KD, EFFORT = 40.0, 8.0, 1.0, 2.94


def quat_to_mat(q) -> np.ndarray:
    r = np.zeros(9)
    mujoco.mju_quat2Mat(r, np.asarray(q, float))
    return r.reshape(3, 3)


def pose_str(p, R) -> str:
    """SDF pose 'x y z roll pitch yaw' from a position and a rotation matrix."""
    sy = math.hypot(R[0, 0], R[1, 0])
    if sy > 1e-9:
        roll, pitch, yaw = math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0])
    else:
        roll, pitch, yaw = math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0
    return " ".join(f"{v:.8g}" for v in (*p, roll, pitch, yaw))


def rgba(m: mujoco.MjModel, g: int) -> str:
    mat = m.geom_matid[g]
    return " ".join(f"{v:.4g}" for v in (m.mat_rgba[mat] if mat >= 0 else m.geom_rgba[g]))


def material(color: str) -> str:
    return f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse><specular>0.1 0.1 0.1 1</specular></material>"


def relpath(path: Path) -> str:
    return os.path.relpath(path, HERE)


def geom_shape(m: mujoco.MjModel, g: int) -> tuple[str, np.ndarray, np.ndarray]:
    """(sdf <geometry> body, position, rotation) of a MuJoCo geom in its body's frame, using raw STL poses for meshes."""
    size, kind = m.geom_size[g], m.geom_type[g]
    p, R = m.geom_pos[g].copy(), quat_to_mat(m.geom_quat[g])
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return f"<box><size>{2*size[0]:.6g} {2*size[1]:.6g} {2*size[2]:.6g}</size></box>", p, R
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return f"<sphere><radius>{size[0]:.6g}</radius></sphere>", p, R
    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:  # MuJoCo and SDF both put the capsule axis along z
        return f"<capsule><radius>{size[0]:.6g}</radius><length>{2*size[1]:.6g}</length></capsule>", p, R
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        d = m.geom_dataid[g]
        stl = ROBOT_DIR / "assets" / f"{m.mesh(d).name.split('/')[-1]}.stl"
        Rm = quat_to_mat(m.mesh_quat[d])  # MuJoCo recentred the mesh: compiled = Rm^T (raw - mesh_pos)
        R_raw = R @ Rm.T
        p_raw = p - R_raw @ m.mesh_pos[d]
        return f"<mesh><uri>{relpath(stl)}</uri></mesh>", p_raw, R_raw
    raise ValueError(f"unsupported geom type {kind} ({m.geom(g).name})")


def link_xml(m: mujoco.MjModel, d: mujoco.MjData, b: int, name: str) -> str:
    out = [f'<link name="{name}">', f"<pose>{pose_str(d.xpos[b], d.xmat[b].reshape(3, 3))}</pose>"]
    ipos, iR = m.body_ipos[b], quat_to_mat(m.body_iquat[b])
    ixx, iyy, izz = m.body_inertia[b]
    out.append(f"<inertial><pose>{pose_str(ipos, iR)}</pose><mass>{m.body_mass[b]:.8g}</mass><inertia>"
               f"<ixx>{ixx:.8g}</ixx><iyy>{iyy:.8g}</iyy><izz>{izz:.8g}</izz>"
               "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")
    for g in range(m.body_geomadr[b], m.body_geomadr[b] + m.body_geomnum[b]):
        geometry, p, R = geom_shape(m, g)
        gname = f"g{g}"
        if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:  # visual only
            if m.geom_group[g] == 4:
                continue
            out.append(f'<visual name="{gname}"><pose>{pose_str(p, R)}</pose><geometry>{geometry}</geometry>'
                       f"{material(rgba(m, g))}</visual>")
        else:  # collision shape; the gripper pads keep MuJoCo's high friction
            mu = float(m.geom_friction[g][0])
            out.append(f'<collision name="{gname}"><pose>{pose_str(p, R)}</pose><geometry>{geometry}</geometry>'
                       f"<surface><friction><ode><mu>{mu:.4g}</mu><mu2>{mu:.4g}</mu2></ode></friction>"
                       "<contact><ode><kp>100000</kp><kd>10</kd></ode></contact></surface></collision>")
    out.append("</link>")
    return "\n".join(out)


def camera_xml(m: mujoco.MjModel, d: mujoco.MjData, c: int) -> tuple[str, str]:
    """(link name or '' for world, sdf camera sensor); MuJoCo cameras look along -z with +y up, Gazebo along +x with +z up."""
    cam = m.camera(c)
    to_gz = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]], float)  # gz axes (x fwd, y left, z up) in the mujoco camera frame
    R = d.cam_xmat[c].reshape(3, 3) @ to_gz
    res = m.cam_resolution[c] if m.cam_resolution[c][0] > 0 else (640, 480)
    w, h = int(res[0]), int(res[1])
    if w > 960:  # keep the wrist camera cheap to render; the field of view is unchanged
        w, h = w // 3, h // 3
    hfov = 2 * math.atan(math.tan(math.radians(m.cam_fovy[c]) / 2) * w / h)
    name = cam.name.split("/")[-1]
    xml = (f'<sensor name="{name}" type="camera"><pose>{pose_str(d.cam_xpos[c], R)}</pose><update_rate>20</update_rate>'
           f'<always_on>true</always_on><topic>{name}/image</topic><camera><horizontal_fov>{hfov:.6g}</horizontal_fov>'
           f"<image><width>{w}</width><height>{h}</height></image><clip><near>0.02</near><far>5</far></clip></camera></sensor>")
    return name, xml


def build() -> str:
    station = load_station()
    m = build_model(station)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)  # qpos = 0 reference pose; Gazebo link poses are absolute at that pose

    robot_bodies = [b for b in range(m.nbody) if m.body(b).name.startswith("robot/")]
    names = {b: m.body(b).name.split("/")[-1] + "_link" for b in robot_bodies}  # joints and links share one SDF namespace
    arm = []
    for b in robot_bodies:
        arm.append(link_xml(m, d, b, names[b]))
    arm.append(f'<joint name="world_fixed" type="fixed"><parent>world</parent><child>{names[robot_bodies[0]]}</child></joint>')
    for b in robot_bodies[1:]:  # bodies MuJoCo welds to their parent (no joint), e.g. the wrist camera mount
        if m.body_jntnum[b] == 0:
            arm.append(f'<joint name="{names[b]}_fixed" type="fixed"><parent>{names[int(m.body_parentid[b])]}</parent>'
                       f"<child>{names[b]}</child></joint>")
    controllers = []
    for j in range(m.njnt):
        jn = m.joint(j)
        if not jn.name.startswith("robot/"):
            continue
        child = int(m.jnt_bodyid[j])
        axis = m.jnt_axis[j]
        lo, hi = m.jnt_range[j]
        short = jn.name.split("/")[-1]
        arm.append(f'<joint name="{short}" type="revolute"><parent>{names[int(m.body_parentid[child])]}</parent>'
                   f"<child>{names[child]}</child><axis><xyz>{axis[0]:.6g} {axis[1]:.6g} {axis[2]:.6g}</xyz>"
                   f"<limit><lower>{lo:.8g}</lower><upper>{hi:.8g}</upper><effort>{EFFORT}</effort><velocity>6</velocity></limit>"
                   f"<dynamics><damping>{m.dof_damping[m.jnt_dofadr[j]]:.4g}</damping>"
                   f"<friction>{m.dof_frictionloss[m.jnt_dofadr[j]]:.4g}</friction></dynamics></axis></joint>")
        controllers.append(
            '<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">'
            f"<joint_name>{short}</joint_name><topic>/{MODEL}/{short}/cmd_pos</topic><p_gain>{KP}</p_gain><i_gain>{KI}</i_gain><i_max>1</i_max><i_min>-1</i_min>"
            f"<d_gain>{KD}</d_gain><cmd_max>{EFFORT}</cmd_max><cmd_min>-{EFFORT}</cmd_min></plugin>")
    state = (f'<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">'
             f"<topic>/{MODEL}/joint_states</topic></plugin>")

    cams = {"": []}
    for c in range(m.ncam):
        if m.camera(c).name not in ("overhead", "robot/wrist_cam"):
            continue  # the stock scene's two side cameras are not part of the station
        owner = m.body(m.cam_bodyid[c]).name
        name, xml = camera_xml(m, d, c)
        cams.setdefault(names.get(m.cam_bodyid[c], ""), []).append(xml)
    for owner, sensors in cams.items():
        if owner and owner in names.values():  # camera rides on a robot link (wrist cam)
            for i, s in enumerate(arm):
                if s.startswith(f'<link name="{owner}">'):
                    arm[i] = s.replace("</link>", "\n".join(sensors) + "\n</link>")

    def static_box(name, pos, size, color, collide=False):
        c = (f'<collision name="c"><geometry><box><size>{size}</size></box></geometry></collision>') if collide else ""
        return (f'<model name="{name}"><static>true</static><pose>{pos} 0 0 0</pose><link name="l">{c}'
                f'<visual name="v"><geometry><box><size>{size}</size></box></geometry>{material(color)}</visual></link></model>')

    world = [static_box("table", "0 0 0.375", "0.9 0.6 0.75", "0.55 0.55 0.57 1", collide=True)]
    for g in range(m.ngeom):
        name = m.geom(g).name
        if name.startswith("gaze/"):
            s = m.geom_size[g]
            world.append(static_box(name.replace("/", "_"), " ".join(f"{v:.6g}" for v in m.geom_pos[g]),
                                    " ".join(f"{2*v:.6g}" if v > 1e-4 else "0.0004" for v in s), rgba(m, g)))
    for color in ("red", "blue", "yellow"):
        b = m.body(f"{color}_block").id
        g = int(m.body_geomadr[b])
        mass, side = float(m.body_mass[b]), 2 * float(m.geom_size[g][0])
        inertia = mass * side * side / 6
        world.append(
            f'<model name="{color}_block"><pose>{pose_str(d.xpos[b], np.eye(3))}</pose><link name="l">'
            f"<inertial><mass>{mass}</mass><inertia><ixx>{inertia:.6g}</ixx><iyy>{inertia:.6g}</iyy><izz>{inertia:.6g}</izz>"
            f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
            f'<collision name="c"><geometry><box><size>{side} {side} {side}</size></box></geometry><surface><friction>'
            f"<ode><mu>1</mu><mu2>1</mu2></ode></friction><contact><ode><kp>100000</kp><kd>10</kd></ode></contact></surface></collision>"
            f'<visual name="v"><geometry><box><size>{side} {side} {side}</size></box></geometry>{material(rgba(m, g))}</visual>'
            "</link></model>")

    overhead = "\n".join(cams[""])
    world.append(f'<model name="overhead_camera"><static>true</static><link name="l">{overhead}</link></model>')

    return f"""<?xml version="1.0"?>
<!-- Generated by simulator/gazebo/build_world.py from the MuJoCo gaze station. Do not edit; regenerate instead. -->
<sdf version="1.9">
<world name="{WORLD}">
  <physics name="fast" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <gravity>0 0 -9.81</gravity>
  <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.55 0.6 0.68 1</background><shadows>true</shadows></scene>
  <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 3 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.2 -1</direction></light>
  <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>{material("0.35 0.36 0.4 1")}</visual></link></model>
  {chr(10).join(world)}
  <model name="{MODEL}">
    {chr(10).join(arm)}
    {chr(10).join(controllers)}
    {state}
  </model>
</world>
</sdf>
"""


def main() -> None:
    OUT.write_text(build())
    print(f"wrote {OUT} ({OUT.stat().st_size // 1024} kB)")


if __name__ == "__main__":
    main()
