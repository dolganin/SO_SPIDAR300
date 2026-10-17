import argparse
import math
import os

import pybullet as p

from walk import JOINT_NAMES, STANCE_HEIGHT, load_chains, stance_joints

_HERE = os.path.dirname(os.path.abspath(__file__))

SERVO_EFFORT = 1.9
SERVO_VELOCITY = 4.7
DT = 1.0 / 240.0


def main() -> None:
    parser = argparse.ArgumentParser(description="Hold the IK stance in pybullet and report base stability")
    parser.add_argument("--urdf", default=os.path.join(_HERE, "robot.urdf"))
    parser.add_argument("--time", type=float, default=5.0, help="Simulated seconds")
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()

    p.connect(p.GUI if args.gui else p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(DT)
    plane = p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE))
    p.changeDynamics(plane, -1, lateralFriction=1.0)

    robot = p.loadURDF(args.urdf, [0, 0, STANCE_HEIGHT + 0.02], flags=p.URDF_USE_INERTIA_FROM_FILE)
    target = stance_joints(load_chains(args.urdf))
    index = {p.getJointInfo(robot, i)[1].decode(): i for i in range(p.getNumJoints(robot))}
    for name in JOINT_NAMES:
        p.resetJointState(robot, index[name], target[name])
        p.changeDynamics(robot, index[name], lateralFriction=1.0)

    steps = int(args.time / DT)
    for step in range(steps + 1):
        for name in JOINT_NAMES:
            p.setJointMotorControl2(robot, index[name], p.POSITION_CONTROL, targetPosition=target[name],
                                    force=SERVO_EFFORT, maxVelocity=SERVO_VELOCITY)
        if step % int(0.5 / DT) == 0:
            pos, orn = p.getBasePositionAndOrientation(robot)
            roll, pitch, _ = p.getEulerFromQuaternion(orn)
            err = max(abs(p.getJointState(robot, index[n])[0] - target[n]) for n in JOINT_NAMES)
            print(f"t={step * DT:4.1f}s  z={pos[2]:.4f} m  roll={math.degrees(roll):5.1f}  "
                  f"pitch={math.degrees(pitch):5.1f}  max joint err={math.degrees(err):4.1f} deg")
        p.stepSimulation()


if __name__ == "__main__":
    main()
