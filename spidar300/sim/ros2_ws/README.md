# SPIDAR300 ROS 2 workspace

Target: ROS 2 Jazzy + Gazebo Harmonic (`ros-jazzy-ros-gz`, `ros-jazzy-xacro`,
`ros-jazzy-joint-state-publisher-gui`, `ros-jazzy-teleop-twist-keyboard`).

```
teleop_twist_keyboard ─/cmd_vel─▶ spider_gait ─/joint_command─▶ joint_command_adapter
   ─/spider/<joint>/cmd_pos─▶ ros_gz_bridge ─▶ gz JointPositionController
gz JointStatePublisher, /clock ─▶ ros_gz_bridge ─▶ /joint_states, /clock
```

The motor contract is `/joint_command` (`sensor_msgs/JointState`, 18 joints with
position and velocity). On the real robot a servo driver replaces the adapter and Gazebo.

| package | contents |
|---|---|
| `spider_description` | `config/legs.yaml` (legs, joints, geometry, limits, gains — the single source), `config/links.yaml` (generated mesh/inertia/collision data), `urdf/spider.urdf.xacro`, meshes, `worlds/flat.sdf`, `launch/view.launch.py` |
| `spider_gait` | `kinematics.py` (analytic IK/FK, mount table, no ROS), `gait.py` (tripod gait + stand/lie, no ROS), `gait_node`, `joint_command_adapter`, `keyboard_teleop`, `pi_bridge` |
| `spider_bringup` | `launch/sim.launch.py` (Gazebo, spawn, bridge generated from legs.yaml), `launch/robot.launch.py` (real robot via the Pi), `config/gait.yaml` |

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd sim/ros2_ws
rosdep install --from-paths src -iy
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select spider_gait && colcon test-result --verbose
```

## Joints

Names are `leg_<leg>_<joint>`, legs `lf lm lr rf rm rr`, joints `coxa femur tibia`.
`base_link`: x forward, z up.

| joint | axis | zero | positive |
|---|---|---|---|
| coxa | +z | leg points radially out from the body | counter-clockwise seen from above |
| femur | -y | femur horizontal | knee up |
| tibia | -y | tibia vertical (with femur at 0) | foot swings outward/up |

Limits are the collision-free range around zero (other joints at zero) minus 0.05 rad.

## Lying, standing up, lying down

The simulation starts with the spider lying on its belly, legs spread straight out
(`rest_pose` in `legs.yaml`: coxa 0, femur 0, tibia 1.5). The model spawns 25 cm up in the
URDF zero pose and the position controllers fold the legs into `rest_pose` while it falls.

- `stand_up`: legs tuck in until the feet hover just above the ground at `rise_reach`,
  the feet press straight down until the body is at `body_height`, then the gait steps
  the feet in to `stance_reach`.
- `lie_down`: the gait steps the feet out to `rise_reach`, the body lowers onto the
  ground, the legs spread back to `rest_pose`.

`/cmd_vel` is ignored unless standing. Buttons: `t` / `b` in `keyboard_teleop`, or
`ros2 service call /stand_up std_srvs/srv/Trigger` and `/lie_down`.

## Step-by-step checks

1. **Joints in RViz** — `ros2 launch spider_description view.launch.py`. Every slider
   moves exactly one leg in the direction from the table above.
2. **Physics, no commands** — `ros2 launch spider_bringup sim.launch.py gait:=false`.
   The controllers fold the legs into `rest_pose`; the robot lands on its belly and lies still.
3. **Bridge** — `ros2 topic echo /joint_states --once` lists 18 joints;
   `ros2 topic pub --once /spider/leg_lf_femur/cmd_pos std_msgs/msg/Float64 "{data: 0.5}"`
   lifts one knee.
4. **Adapter** — with `gait:=false`, publish the default stance (height 9 cm, reach 15 cm) by hand
   (from lying it pushes itself up in one go; feet slide a bit, that is expected):
   ```bash
   ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{
     name: [leg_lf_femur, leg_lm_femur, leg_lr_femur, leg_rf_femur, leg_rm_femur, leg_rr_femur,
            leg_lf_tibia, leg_lm_tibia, leg_lr_tibia, leg_rf_tibia, leg_rm_tibia, leg_rr_tibia],
     position: [0.77, 0.77, 0.77, 0.77, 0.77, 0.77, -0.53, -0.53, -0.53, -0.53, -0.53, -0.53]}"
   ```
   Tune `controller: p` in `legs.yaml` (velocity mode, 1/s) until the body neither sags nor rocks.
   Higher p = stiffer; joint speed is capped at `servo.velocity`, torque at `servo.effort`.
5. **Kinematics** — `pytest src/spider_gait/test` (no ROS needed).
6. **Gait** — `ros2 launch spider_bringup sim.launch.py`. It lies; then
   `ros2 service call /stand_up std_srvs/srv/Trigger` and
   `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.03}}"`.
7. **Teleop** — in another terminal:
   ```bash
   ros2 run spider_gait keyboard_teleop --ros-args -p speed:=0.05 -p turn:=0.4
   ```
   ```
     u i o     i , forward / back     j l  turn on the spot
     j k l     u o m .  arcs          k / space  stop
     m , .     Shift: J L  step sideways, I <  forward / back, U O M >  diagonals
     t  stand up     b  lie down      q / z  faster / slower
   ```
   A key moves the robot while it is held (terminal auto-repeat); 0.6 s after the last
   key the command drops to zero and the gait steps into stance and stops.
   `teleop_twist_keyboard` works too (same keys), just without stand / lie down.
8. **Tuning** — live, no rebuild:
   `ros2 param set /spider_gait body_height 0.08`, likewise `step_length`, `period`,
   `lift_height`, `stance_reach`, `max_linear`, `max_angular`, `rise_reach`, `tuck_time`, `rise_time`. Defaults in `spider_bringup/config/gait.yaml`.

## Real robot

The Raspberry Pi runs the gait without ROS (`pi/README.md`); on the laptop
`ros2 launch spider_bringup robot.launch.py pi_host:=<pi>` starts `pi_bridge`, which offers the
same `/cmd_vel`, `/stand_up`, `/lie_down` and `/joint_states` as the simulation, so
`keyboard_teleop` and RViz work unchanged.

## Without ROS

`sim/tools/pybullet_check.py` (venv `/workspace/venvs/spidar-tools`, needs `xacro` from
github.com/ros/xacro@ros2) expands the same xacro and runs the same `kinematics.py`/`gait.py`
in pybullet: URDF vs FK cross-check, drop test, and lying -> stand up -> all walking
commands -> lie down -> stand up again, with travel, height, tilt and stance-foot slip. Gait parameters are flags, e.g. `--period 0.6`.

## Regenerating from CAD

`sim/tools/build_description.py` converts `sim/urdf/robot.urdf` (CAD export) into
`legs.yaml`, `links.yaml` and the renamed meshes. It overwrites hand edits in `legs.yaml`.
Leg `lr` (CAD leg 1) has a ~8 mm deviation in the CAD femur/tibia placement; it uses the
reference leg's femur and tibia parts so its mesh matches the shared kinematic chain.
