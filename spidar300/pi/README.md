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

Raspberry Pi OS (Bookworm, 64-bit) or Ubuntu:

```bash
sudo apt install -y git python3-numpy python3-yaml python3-serial
git clone <repo> ~/spidar300          # only spidar300/ is needed
sudo usermod -aG dialout $USER        # serial port access, then log out and back in
```

Servos: Feetech USB bus adapter (FE-URT-1 or similar) -> `/dev/ttyUSB0` (sometimes `/dev/ttyACM0`).
Set the port in `config.yaml`. The servos need their own supply (6-8.4 V for STS3215), not the Pi's 5 V.

## First time: ids and calibration

All commands from `pi/`. `--config` defaults to `pi/config.yaml`.

1. **Ids.** New servos are all id 1. Connect them one at a time and number them as in
   `servos.yaml` (lf coxa = 1, lf femur = 2, ... rr tibia = 18):
   ```bash
   python3 -m spider_pi.calibrate scan
   python3 -m spider_pi.calibrate set-id 1 7
   ```
   With all 18 connected, `scan` must list 1..18.
2. **Zeros.** Torque goes off; hold the robot in the air in the zero pose: every coxa
   straight out from the body, every femur horizontal, every tibia hanging vertically.
   ```bash
   python3 -m spider_pi.calibrate zero
   ```
3. **Directions.** For each joint (start with one leg):
   ```bash
   python3 -m spider_pi.calibrate jog leg_lf_femur 0.3
   ```
   Positive must be: coxa counter-clockwise seen from above, femur knee up, tibia foot
   outward. If it went the other way, set `sign: -1` for that joint in `servos.yaml`.
4. **Check.** `python3 -m spider_pi.calibrate read` prints live joint angles in degrees;
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

Written against the Feetech STS protocol and the STS3215 register map, tested only on
the fake bus. Things to watch on the first run: the sync read of positions (some
firmware/adapters answer slowly; raise `timeout` in `StsBus`), and the speed scaling.
Keep a hand on the power switch.
