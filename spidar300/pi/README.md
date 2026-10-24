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

## Install on the Pi

The servo bus goes through Feetech's `scservo_sdk` (`sms_sts`, `PortHandler`) — the same
library as `servo.py`, `spider_control.py` and the notebooks in the repo root. If those
notebooks work on the Pi, everything is already installed. Otherwise:

```bash
sudo apt install -y git python3-numpy python3-yaml python3-serial
pip install git+https://github.com/ftservo/FTServo_Python.git   # provides scservo_sdk.sms_sts
sudo usermod -aG dialout $USER        # serial port access, then log out and back in
```

Either flavour of `scservo_sdk` works: the FTServo one (with `sms_sts`) or `feetech-servo-sdk`
from PyPI (what LeRobot installs, e.g. in `lerobot-venv`). Don't install one over the other
in a LeRobot environment; LeRobot needs the PyPI one.

Board: Waveshare / Feetech bus servo driver over USB, `/dev/ttyACM0`, 1 Mbaud (`config.yaml`).
The servos need their own supply, not the Pi's 5 V.

## First time: calibration

All commands from `pi/`. `--config` defaults to `pi/config.yaml`.

1. **Ids** — `<side><leg><joint>` as numbered with `numerate_motors.ipynb`:
   side 1 = left, 2 = right; leg 1 = front, 2 = middle, 3 = rear; joint 1 = coxa,
   2 = femur, 3 = tibia (`servos.yaml`).
   ```bash
   python3 -m spider_pi.calibrate scan          # must list all 18: 111..133, 211..233
   ```
2. **Zeros** — like `leg_twister.ipynb`: the current pose is written into each servo's
   EEPROM offset (register 31) so that it reads 2048. This replaces offsets set earlier
   from the notebooks. Torque goes off; hold the robot in the air in the zero pose: every
   coxa straight out from the body, every femur horizontal, every tibia hanging vertically.
   ```bash
   python3 -m spider_pi.calibrate zero                       # all 18
   python3 -m spider_pi.calibrate zero leg_lf_femur leg_lf_tibia   # or a few at a time
   ```
3. **Directions** — for each joint:
   ```bash
   python3 -m spider_pi.calibrate jog leg_lf_femur 0.3
   ```
   Positive must be: coxa counter-clockwise seen from above, femur knee up, tibia foot
   outward. If it went the other way, set `sign: -1` for that joint in `servos.yaml`.
   Left and right servos are usually mounted mirrored, so expect some `-1`.
4. **Check** — `python3 -m spider_pi.calibrate read` prints live joint angles in degrees;
   in the zero pose all of them are about 0.

## Run

On the Pi:
```bash
python3 -m spider_pi.robot            # --fake: no servos, for a dry run
```
At start it reads the servo positions and moves slowly (0.8 rad/s) into the rest pose:
lying, legs spread out. **Put the robot on the ground on its belly first.**
Ctrl-C makes it lie down, then switches the torque off.

On the laptop (`sim/ros2_ws` built and sourced):
```bash
ros2 launch spider_bringup robot.launch.py pi_host:=raspberrypi.local   # or the Pi's IP
ros2 run spider_gait keyboard_teleop                                   # second terminal
```
`t` stand up, `b` lie down, walking keys as in simulation. RViz shows the measured pose.
Gait parameters live: `ros2 param set /pi_bridge period 0.6` (defaults in `config.yaml`;
`ros2 param get` on the bridge shows its own defaults, not the Pi's values).

Autostart at boot: see `spidar-robot.service`.

## Tests (no hardware)

```bash
python3 -m pytest tests     # packet format, and the whole runtime over UDP on a fake bus
```

## Not verified on hardware yet

Uses the same `scservo_sdk` calls as the notebooks (`ReadPos`, `SyncWritePosEx` +
`groupSyncWrite.txPacket()`, offsets via register 31), checked against the SDK source and on
a fake bus only. Positions are read 3 servos per control tick (`bus.reads_per_tick`) to keep
the loop at 50 Hz; if the log shows the loop running late, lower it. Keep a hand on the
power switch on the first run.
