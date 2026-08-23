from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

COXA_MM = 54.0
FEMUR_MM = 70.0
TIBIA_MM = 155.0

COXA_OFFSET_X_MM = 78.0
COXA_OFFSET_Y_MM = 55.0
MIDDLE_COXA_OFFSET_Y_MM = 78.0

FOOT_DISTANCE_MM = 120.0
FOOT_HEIGHT_MM = -110.0

MM = 1.0e-3
COXA, FEMUR, TIBIA = COXA_MM * MM, FEMUR_MM * MM, TIBIA_MM * MM

COXA_LIMIT = (math.radians(-45.0), math.radians(45.0))
FEMUR_LIMIT = (math.radians(-100.0), math.radians(10.0))
TIBIA_LIMIT = (math.radians(5.0), math.radians(175.0))

JOINT_LIMITS = {"coxa": COXA_LIMIT, "femur": FEMUR_LIMIT, "tibia": TIBIA_LIMIT}

SERVO_EFFORT = 1.86
SERVO_VELOCITY = 5.8

@dataclass(frozen=True)
class LegDef:
    name: str
    full: str
    mount_x: float
    mount_y: float
    default_deg: float
    pair: str
    side: str

LEGS: tuple[LegDef, ...] = (
    LegDef("lf", "left_front", COXA_OFFSET_X_MM * MM, COXA_OFFSET_Y_MM * MM, 45.0, "front", "left"),
    LegDef("rf", "right_front", COXA_OFFSET_X_MM * MM, -COXA_OFFSET_Y_MM * MM, -45.0, "front", "right"),
    LegDef("lm", "left_middle", 0.0, MIDDLE_COXA_OFFSET_Y_MM * MM, 90.0, "middle", "left"),
    LegDef("rm", "right_middle", 0.0, -MIDDLE_COXA_OFFSET_Y_MM * MM, -90.0, "middle", "right"),
    LegDef("lr", "left_rear", -COXA_OFFSET_X_MM * MM, COXA_OFFSET_Y_MM * MM, 135.0, "rear", "left"),
    LegDef("rr", "right_rear", -COXA_OFFSET_X_MM * MM, -COXA_OFFSET_Y_MM * MM, -135.0, "rear", "right"),
)

LEG_NAMES = tuple(l.name for l in LEGS)
PAIRS = ("front", "middle", "rear")
JOINT_ORDER = ("coxa", "femur", "tibia")

PAIR_LEGS = {p: tuple(l.name for l in LEGS if l.pair == p) for p in PAIRS}

def joint_name(leg: str, joint: str) -> str:
    return f"{leg}_{joint}_joint"

JOINT_NAMES_18: tuple[str, ...] = tuple(
    joint_name(leg.name, j) for leg in LEGS for j in JOINT_ORDER
)

FEMUR_Z_OFFSET = 0.022

def fk_leg_local(coxa: float, femur: float, tibia: float) -> np.ndarray:
    r = COXA + FEMUR * math.cos(femur) + TIBIA * math.cos(femur + tibia)
    z = FEMUR_Z_OFFSET - (FEMUR * math.sin(femur) + TIBIA * math.sin(femur + tibia))
    x = r * math.cos(coxa)
    y = r * math.sin(coxa)
    return np.array([x, y, z])

def fk_leg_body(leg: LegDef, coxa: float, femur: float, tibia: float) -> np.ndarray:
    local = fk_leg_local(coxa, femur, tibia)
    a = math.radians(leg.default_deg)
    rot = np.array([[math.cos(a), -math.sin(a), 0.0],
                    [math.sin(a), math.cos(a), 0.0],
                    [0.0, 0.0, 1.0]])
    return rot @ local + np.array([leg.mount_x, leg.mount_y, 0.0])

def ik_leg_local(x: float, y: float, z: float) -> tuple[float, float, float]:
    coxa = math.atan2(y, x)
    r = math.hypot(x, y) - COXA
    zr = z - FEMUR_Z_OFFSET
    rho = math.hypot(r, zr)
    if rho > FEMUR + TIBIA or rho < abs(FEMUR - TIBIA):
        raise ValueError(f"foot ({x:.3f},{y:.3f},{z:.3f}) out of reach: rho={rho:.3f}")

    cos_t = (rho * rho - FEMUR * FEMUR - TIBIA * TIBIA) / (2.0 * FEMUR * TIBIA)
    cos_t = max(-1.0, min(1.0, cos_t))
    tibia = math.acos(cos_t)

    phi = math.atan2(-zr, r)
    psi = math.atan2(TIBIA * math.sin(tibia), FEMUR + TIBIA * math.cos(tibia))
    femur = phi - psi
    return coxa, femur, tibia

def _rot_z(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a), 0.0],
                     [math.sin(a), math.cos(a), 0.0],
                     [0.0, 0.0, 1.0]])

def foot_body_to_local(leg: LegDef, p_body: np.ndarray) -> np.ndarray:
    a = math.radians(leg.default_deg)
    return _rot_z(-a) @ (np.asarray(p_body, float) - np.array([leg.mount_x, leg.mount_y, 0.0]))

def ik_leg_body(leg: LegDef, p_body: np.ndarray) -> tuple[float, float, float]:
    return ik_leg_local(*foot_body_to_local(leg, p_body))

def nominal_foot_body(leg: LegDef) -> np.ndarray:
    return fk_leg_body(leg, *default_stance_angles())

def clamp(joint: str, value: float) -> float:
    lo, hi = JOINT_LIMITS[joint]
    return min(hi, max(lo, value))

def default_stance_angles() -> tuple[float, float, float]:
    return ik_leg_local(FOOT_DISTANCE_MM * MM, 0.0, FOOT_HEIGHT_MM * MM)

def stance_joint_vector() -> np.ndarray:
    c, f, t = default_stance_angles()
    return np.array([v for _ in LEGS for v in (c, f, t)])

def within_limits(joint: str, value: float, eps: float = 1e-6) -> bool:
    lo, hi = JOINT_LIMITS[joint]
    return lo - eps <= value <= hi + eps
