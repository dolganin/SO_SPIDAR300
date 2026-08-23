from __future__ import annotations

import math
import os

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from ...assets import SO_SPIDAR300_CFG
from ... import kinematics as K
from . import mdp

LEDGE_HEIGHT = 0.09
CAM_W, CAM_H = 224, 224

CAM_PITCH_DEG = 25.0

def _cam_rot(pitch_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(pitch_deg) / 2.0
    qy = (math.cos(half), 0.0, math.sin(half), 0.0)
    qb = (0.5, -0.5, 0.5, -0.5)
    w1, x1, y1, z1 = qy
    w2, x2, y2, z2 = qb
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )

@configclass
class SpidarSceneCfg(InteractiveSceneCfg):

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.05)),
        spawn=sim_utils.CuboidCfg(
            size=(400.0, 400.0, 0.1),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=1.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.25, 0.27, 0.30)),
        ),
    )

    robot: ArticulationCfg = SO_SPIDAR300_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    ledge = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Ledge",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.5, 0.0, LEDGE_HEIGHT / 2.0)),
        spawn=sim_utils.CuboidCfg(
            size=(0.45, 0.7, LEDGE_HEIGHT),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.30, 0.25)),
        ),
    )

    camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link/front_cam",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.08, 0.0, 0.08), rot=_cam_rot(CAM_PITCH_DEG), convention="ros"
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=6.0, focus_distance=400.0, horizontal_aperture=20.955,
            clipping_range=(0.02, 20.0),
        ),
        width=CAM_W,
        height=CAM_H,
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.95)),
    )

@configclass
class ActionsCfg:

    joints = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=list(K.JOINT_NAMES_18),
        preserve_order=True,
        scale=0.5,
        use_default_offset=True,
    )

@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):

        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.5, n_max=0.5))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.02, n_max=0.02))
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.05, n_max=0.05))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.05, n_max=0.05))
        target_vec = ObsTerm(func=mdp.target_vector_b)
        on_back = ObsTerm(func=mdp.is_on_back)
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class RGBCfg(ObsGroup):

        image = ObsTerm(func=mdp.camera_rgb)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()
    rgb: RGBCfg = RGBCfg()

@configclass
class EventsCfg:
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.1, 0.1), "y": (-0.1, 0.1), "yaw": (-0.2, 0.2)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )
    reset_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (0.95, 1.05), "velocity_range": (0.0, 0.0)},
    )

@configclass
class RewardsCfg:
    progress = RewTerm(func=mdp.progress_to_goal, weight=15.0)
    height_gain = RewTerm(func=mdp.goal_height_gain, weight=1.0)
    reached = RewTerm(func=mdp.reached_goal_bonus, weight=5.0, params={"eps": 0.08})
    upright = RewTerm(func=mdp.upright, weight=0.5)
    on_back = RewTerm(func=mdp.on_back_penalty, weight=-1.0)
    alive = RewTerm(func=mdp.is_alive, weight=0.1)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)

@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    success = DoneTerm(func=mdp.reached_goal, params={"eps": (0.06, 0.06, 0.06)})
    on_back_timeout = DoneTerm(func=mdp.on_back_timeout, params={"time_s": 5.0, "speed_thresh": 0.05})
    fell = DoneTerm(func=mdp.base_too_low, params={"min_height": -0.5})

@configclass
class SpidarClimbEnvCfg(ManagerBasedRLEnvCfg):
    scene: SpidarSceneCfg = SpidarSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventsCfg = EventsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    goal_marker: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/goal_marker",
        markers={
            "target": sim_utils.SphereCfg(
                radius=0.04,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.1, 0.1)),
            )
        },
    )
    goal_offset: tuple[float, float, float] = (0.5, 0.0, LEDGE_HEIGHT + 0.12)
    goal_randomization: tuple[float, float, float] = (0.08, 0.12, 0.0)

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 1.0 / 120.0
        self.sim.render_interval = self.decimation
        self.viewer.eye = (2.0, 2.0, 1.2)

@configclass
class SpidarClimbEnvCfg_PLAY(SpidarClimbEnvCfg):

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
        self.scene.env_spacing = 3.0
        self.observations.policy.enable_corruption = False
        self.episode_length_s = 1800.0
        if os.environ.get("SPIDAR_NO_CAMERA", "") not in ("", "0"):
            self.scene.camera = None
            self.observations.rgb = None
