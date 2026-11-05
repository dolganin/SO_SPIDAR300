# SPIDAR300 on the Raspberry Pi

The Pi runs the gait and talks to the 18 STS3215 servos; the laptop only sends
velocity and stand / lie commands over Wi-Fi (UDP). No ROS on the Pi.

```
laptop (ROS)                                   Raspberry Pi (python3, no ROS)
keyboard_teleop -/cmd_vel-> pi_bridge  ==UDP==>  spider_pi.robot: gait.py + kinematics.py
RViz <-/joint_states------- pi_bridge  <==UDP==        -> STS3215 bus (USB adapter, 1 Mbaud)
```

The gait code is the same `gait.py` / `kinematics.py` as in simulation
(`sim/ros2_ws/src/spider_gait`), with the same `legs.yaml`. Without a velocity command for
0.5 s the Pi stops walking by itself, so a lost Wi-Fi link just makes the robot stand.

## Install

Use the same Python environment as the working `test_spider_control.ipynb`.
It must provide `scservo_sdk.sms_sts` and pyserial, plus numpy and PyYAML.
The custom SDK compatibility implementation has been removed.

## Run

On the Pi (activate the notebook environment first):

```bash
cd /workspace/SO_SPIDAR300/spidar300/pi
python3 -m spider_pi.robot
```

On the laptop, after rebuilding and sourcing `sim/ros2_ws`:

```bash
ros2 launch spider_bringup robot.launch.py pi_host:=192.168.42.199
```

In a second sourced ROS terminal:

```bash
ros2 run spider_gait keyboard_teleop
```

1. Put the belly on a high support with feet free. Press Enter.
2. The runtime calls the actual `Spider.enable_torque()` and
   `Spider.move_to_default_position(speed=2000, acc=0, blocking=False)`.
   These are the original notebook calls; completion is checked asynchronously
   so Ctrl-C and UDP remain responsive. Default is sent once, without a second
   trajectory or grouped writes. Its encoder positions come from `Spider`.
3. After encoder feedback confirms the default, the controller automatically
   starts lifting legs from the measured joint angles into the lying pose.
   The joint speed limiter is initialized from those actual angles.
4. Wait for `lying pose reached; holding`. Move belly-down onto the floor,
   then press Enter to enable stand-up and walking.
5. `t` stands up; `b` lies down; walking keys match simulation.

To run only the original default without any IK transition or ROS:

```bash
python3 -m spider_pi.calibrate default
```

It waits for Enter, invokes the original Spider methods, checks arrival and
leaves torque on. No EEPROM or zero settings are written.

Other utilities: `calibrate read`, `inspect`, `scan`, `release`.
Legacy `zero`, `record`, `set-id` and `jog` utilities have been removed.

Every physical write now goes through the existing `spider_control.Spider`:
`move_to`, `enable_torque`, `disable_torque`, `move_to_default_position`.
`spider_pi/spider_bus.py` only adapts those methods to the runtime interface;
it contains no packet encoding or SDK replacement.

The subsequent IK motions and tibia direction still require hardware validation.
Gazebo starts in the lying pose and does not use the hardware preparation steps.
Ctrl-C releases torque; after successful preparation it tries to lie down first.

## Tests

```bash
python3 -m pytest tests
```

Tests verify delegation to the original Spider and startup sequencing on a fake
bus. They do not establish mechanical calibration or performance on hardware.
