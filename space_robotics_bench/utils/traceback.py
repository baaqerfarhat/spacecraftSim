from importlib.util import find_spec
from os import environ


def try_enable_rich_traceback():
    if environ.get("SRB_WITH_TRACEBACK", "false").lower() not in ["true", "1"]:
        return

    if find_spec("rich") is None:
        return

    from rich import traceback  # isort:skip
    import numpy

    traceback.install(
        width=120,
        show_locals=environ.get("SRB_WITH_TRACEBACK_LOCALS", "false").lower()
        in ["true", "1"],
        suppress=(numpy,),
    )
