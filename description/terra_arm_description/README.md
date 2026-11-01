# terra_arm_description

URDF of the Terra 7-DOF tether arm with slip rings and EndEffectorAssembly_v3,
generated from Fusion document `MainArm/ArmAssem-copy` (v3).

- `urdf/arm.urdf` — 38 links, 7 continuous joints (5 arm motors + 2 slip-ring spins), base `DM-J10010L-2EC_Motor_1`.
  Each gripper link (`EndEffectorAssembly_v3_1/_2`) is the **male** side only.
- `urdf/dock_female.urdf` — the **female** dock (`FemaleAssembly`: `EE3_DockWall` + `EE3_DockBasePlate`)
  as a separate one-link model, to place as an obstacle/docking target. Its frame is the
  `EE3_DockBasePlate` part frame; the header comment gives its docked pose relative to each gripper link.
- `meshes/` — binary STL in millimetres (URDF applies `scale="0.001"`)
- Mesh paths use `package://terra_arm_description/meshes/...`

ROS 2: put this folder in your workspace `src/`, `colcon build`, source, then
`ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix terra_arm_description)/share/terra_arm_description/urdf/arm.urdf`

Masses: corrected on 2026-10-31 from datasheets and weighed parts (total 14.35 kg, was 27.17 kg from
Fusion defaults); see the header comment in `urdf/arm.urdf` and `sim/step_reach_sim/apply_measured_masses.py`.
Wiring and electronics are not modelled.

Caveats: joint effort/velocity
limits are placeholders; each gripper is a single rigid link (internal servos/jaws not articulated); zero pose is the straight arm.
