# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.markers.config import BLUE_ARROW_X_MARKER_CFG, FRAME_MARKER_CFG, GREEN_ARROW_X_MARKER_CFG
from isaaclab.actuators import ImplicitActuatorCfg

# from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG
from isaaclab.sim import PhysxCfg
from isaaclab.terrains import TerrainGeneratorCfg
from isaaclab.terrains.height_field import HfRandomUniformTerrainCfg

@configclass
class Rob6323Go2EnvCfg(DirectRLEnvCfg):
    # env
    decimation = 4
    episode_length_s = 20.0
    # - spaces definition
    action_scale = 0.25
    action_space = 12
    observation_space = 48 + 4 # Adding # of clock inputs
    state_space = 0
    debug_vis = True

    # Termination Conditions
    base_height_min = 0.20  # Termination height

    # PD Control gains
    Kp = 20.0
    Kd = 0.5
    torque_limits = 100.0

    # Reward Scales
    raibert_heuristic_reward_scale = -10.0
    feet_clearence_reward_scale = -60.0
    tracking_contacts_shaped_force_reward_scale = 40.0

    # Additional reward scales
    orient_reward_scale = -5.0
    lin_vel_z_reward_scale = -0.02
    dof_vel_reward_scale = -0.0001
    ang_vel_xy_reward_scale = -0.001

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 200,
        render_interval=decimation,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        physx=PhysxCfg(
            gpu_max_rigid_patch_count=2**24,
        ),
    )

    # random-uniform terrain generator (height-field)
    RANDOM_UNIFORM_TERRAIN_CFG = TerrainGeneratorCfg(
        curriculum=False,                 # random (not row-based curriculum) :contentReference[oaicite:2]{index=2}
        difficulty_range=(0.0, 1.0),      # difficulty sampled U(low, high) when curriculum=False :contentReference[oaicite:3]{index=3}
        size=(8.0, 8.0),                  # sub-terrain size in meters :contentReference[oaicite:4]{index=4}
        num_rows=10,
        num_cols=20,
        horizontal_scale=0.1,
        vertical_scale=0.005,
        slope_threshold=0.75,
        sub_terrains={
            "hf_random_uniform": HfRandomUniformTerrainCfg(
                proportion=1.0,            # always choose this terrain type :contentReference[oaicite:5]{index=5}
                noise_range=(0.0, 0.06),   # min/max height noise (m) :contentReference[oaicite:6]{index=6}
                noise_step=0.01,           # height quantization step (m) :contentReference[oaicite:7]{index=7}
                # downsampled_scale=0.2,   # optional: sample on coarser grid then interpolate :contentReference[oaicite:8]{index=8}
            )
        },
    )

    # terrain importer
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=RANDOM_UNIFORM_TERRAIN_CFG,
        max_init_terrain_level=6,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )
    # robot(s)
    robot_cfg: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    robot_cfg.actuators["base_legs"] = ImplicitActuatorCfg(
        joint_names_expr=[".*hip_joint", ".*thigh_joint", ".*calf_joint"],
        effort_limit=23.5,
        velocity_limit=30.0,
        stiffness=0.0,          # 0.0 to disable impmlicit P gain
        damping=0.0,            # 0.0 to disable impmlicit D gain
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)
    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*", history_length=3, update_period=0.005, track_air_time=True
    )
    goal_vel_visualizer_cfg: VisualizationMarkersCfg = GREEN_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/velocity_goal"
    )
    """The configuration for the goal velocity visualization marker. Defaults to GREEN_ARROW_X_MARKER_CFG."""

    current_vel_visualizer_cfg: VisualizationMarkersCfg = BLUE_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/velocity_current"
    )
    """The configuration for the current velocity visualization marker. Defaults to BLUE_ARROW_X_MARKER_CFG."""

    # Set the scale of the visualization markers to (0.5, 0.5, 0.5)
    goal_vel_visualizer_cfg.markers["arrow"].scale = (0.5, 0.5, 0.5)
    current_vel_visualizer_cfg.markers["arrow"].scale = (0.5, 0.5, 0.5)

    # reward scales
    action_rate_reward_scale = -0.1
    lin_vel_reward_scale = 1.0
    yaw_rate_reward_scale = 0.5
