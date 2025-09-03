from srb.utils.registry import register_srb_tasks

from .task import Task, TaskCfg

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {
            "entry_point": Task,
            "task_cfg": TaskCfg,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)


