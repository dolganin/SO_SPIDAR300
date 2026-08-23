from __future__ import annotations

import math
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from .. import kinematics as K

_ASSET_DIR = os.path.dirname(__file__)
SO_SPIDAR300_USD = os.path.join(_ASSET_DIR, "robot", "so_spidar300.usd")

_COXA0, _FEMUR0, _TIBIA0 = K.default_stance_angles()

_STANCE_HEIGHT = abs(K.FOOT_HEIGHT_MM) * K.MM + 0.012

SO_SPIDAR300_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=SO_SPIDAR300_USD,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=2,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, _STANCE_HEIGHT),
        joint_pos={
            ".*_coxa_joint": _COXA0,
            ".*_femur_joint": _FEMUR0,
            ".*_tibia_joint": _TIBIA0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.95,
    actuators={
        "coxa": ImplicitActuatorCfg(
            joint_names_expr=[".*_coxa_joint"],
            effort_limit_sim=K.SERVO_EFFORT,
            velocity_limit_sim=K.SERVO_VELOCITY,
            stiffness=15.0,
            damping=0.5,
        ),
        "femur": ImplicitActuatorCfg(
            joint_names_expr=[".*_femur_joint"],
            effort_limit_sim=K.SERVO_EFFORT,
            velocity_limit_sim=K.SERVO_VELOCITY,
            stiffness=20.0,
            damping=0.6,
        ),
        "tibia": ImplicitActuatorCfg(
            joint_names_expr=[".*_tibia_joint"],
            effort_limit_sim=K.SERVO_EFFORT,
            velocity_limit_sim=K.SERVO_VELOCITY,
            stiffness=20.0,
            damping=0.6,
        ),
    },
)
"""Articulation config for the SO-SPIDAR300, default-stance initial pose."""

def stance_joint_pos_dict() -> dict[str, float]:
    return {
        K.joint_name(leg.name, j): a
        for leg in K.LEGS
        for j, a in zip(K.JOINT_ORDER, (_COXA0, _FEMUR0, _TIBIA0))
    }
