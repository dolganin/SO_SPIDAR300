import gymnasium as gym

from . import agents

gym.register(
    id="SO-SPIDAR300-Climb-v0",
    entry_point=f"{__name__}.climb_env:SpidarClimbEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.climb_env_cfg:SpidarClimbEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:SpidarClimbPPORunnerCfg",
    },
)

gym.register(
    id="SO-SPIDAR300-Climb-Play-v0",
    entry_point=f"{__name__}.climb_env:SpidarClimbEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.climb_env_cfg:SpidarClimbEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:SpidarClimbPPORunnerCfg",
    },
)
