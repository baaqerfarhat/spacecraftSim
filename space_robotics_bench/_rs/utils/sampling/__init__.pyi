from typing import List, Tuple

def sample_poisson_disk_2d(
    num_samples: int,
    bounds: Tuple[Tuple[float, float], Tuple[float, float]],
    radius: float,
) -> List[Tuple[float, float]]: ...
def sample_poisson_disk_2d_looped(
    num_samples: Tuple[int, int],
    bounds: Tuple[Tuple[float, float], Tuple[float, float]],
    radius: float,
) -> List[List[Tuple[float, float]]]: ...
def sample_poisson_disk_3d(
    num_samples: int,
    bounds: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    radius: float,
) -> List[Tuple[float, float, float]]: ...
def sample_poisson_disk_3d_looped(
    num_samples: Tuple[int, int],
    bounds: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    radius: float,
) -> List[List[Tuple[float, float, float]]]: ...
