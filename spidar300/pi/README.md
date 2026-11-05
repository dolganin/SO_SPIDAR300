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

## Replay the original notebooks without the runtime

```bash
cd /workspace/SO_SPIDAR300/spidar300/pi
python -m spider_pi.notebook_replay --show
python -m spider_pi.notebook_replay
```

The module executes source cells directly from `test_spider_control.ipynb`:
construct `Spider()`, ping, enable torque, move to default with speed=2000,
acc=0 and blocking=True, then disable torque. It waits for Enter before executing.
No ROS, IK, runtime adapter or EEPROM calibration is involved. The first serial
port is selected exactly as in the original notebook.

For the direct SDK default cell from `leg_twister.ipynb` (speed=50, acc=10):

```bash
python -m spider_pi.notebook_replay leg-default
```

That original cell sends goals asynchronously and leaves torque enabled; module
completion does not mean the servos reached their goals. The `left-tibia-test`
recipe reproduces its separate test of IDs 113/123/133 at 1000, speed=300, acc=50.
Offset-writing experiment cells are excluded. Errors propagate visibly; on an
error or Ctrl-C during execution, the module requests torque off and closes the port.
Stop all other programs using the servo port before replay.

## Measured software reference (2026-11-05)

`servos.yaml` now contains the latest user-positioned reference, rounded to 10
encoder ticks (2048 preserved). Runtime and `calibrate` use these values for
angle zero and configure the existing Spider instance's default targets with
them. The original class file and servo EEPROM are unchanged. `notebook_replay`
still executes the historical notebooks and their old targets, including tibia
700; do not use it to move to the new measured reference.

Use `calibrate read` first: in the reference pose the reported angles should be
near zero. `calibrate default` moves to the new reference using the original
Spider method. The configured coxa zero is the supplied physical pose; its yaw
alignment with the simulated radial mounts and all motor directions still need
verification before full IK movements.

## Individual mechanical limits

`servos.yaml` supports `limits: {min: ..., max: ...}` in radians relative to the
measured reference, in the model's positive direction. For left rear tibia 133,
the observed physical stop was about 975 ticks; max=1.15 sets the commanded
limit to about 1030 ticks, leaving approximately 5 degrees of clearance.
The runtime intersects this with model/encoder limits before constructing the
gait. Startup and lie-down poses are clipped per leg, IK uses the same bounds,
and the jog utility rejects movements past the configured stop. Other legs
retain their existing limits; their physical stops have not been measured.
