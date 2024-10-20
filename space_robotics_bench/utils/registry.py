from typing import Any, Dict, List, Literal, Optional, Union

import gymnasium

from space_robotics_bench.paths import SRB_HYPERPARAMS_DIR
from space_robotics_bench.utils.cfg import parse_algo_configs


def register_tasks(
    tasks: Dict[
        str,
        Dict[
            Literal["entry_point", "task_cfg", "cfg_dir"],
            Union[gymnasium.Env, Any, str],
        ],
    ],
    *,
    default_entry_point: Optional[gymnasium.Env] = None,
    default_task_cfg: Optional[Any] = None,
    default_cfg_dir: Optional[str] = SRB_HYPERPARAMS_DIR,
    namespace: str = "srb",
):
    for id, cfg in tasks.items():
        entry_point = cfg.get("entry_point", default_entry_point)
        gymnasium.register(
            id=f"{namespace}/{id}",
            entry_point=f"{entry_point.__module__}:{entry_point.__name__}",
            kwargs={
                "task_cfg": cfg.get("task_cfg", default_task_cfg),
                **parse_algo_configs(cfg.get("cfg_dir", default_cfg_dir)),
            },
            disable_env_checker=True,
        )


def get_srb_tasks() -> List[str]:
    return [env_id for env_id in gymnasium.registry.keys() if "srb" in env_id]
