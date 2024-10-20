from os import listdir, path
from typing import List


def abs_listdir(dir: str) -> List[str]:
    return [path.realpath(path.join(dir, file)) for file in listdir(dir)]
