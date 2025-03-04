import numpy
import torch
from pxr import Usd, UsdGeom, Vt


def particle_positions(prim: Usd.Prim) -> numpy.ndarray:
    return numpy.array(UsdGeom.Points(prim).GetPointsAttr().Get())


def particle_velocities(prim: Usd.Prim) -> numpy.ndarray:
    return numpy.array(UsdGeom.Points(prim).GetVelocitiesAttr().Get())


def set_particle_positions(prim: Usd.Prim, positions: torch.Tensor | numpy.ndarray):
    if isinstance(positions, torch.Tensor):
        positions = positions.cpu().numpy()
    UsdGeom.Points(prim).GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(positions))


def set_particle_velocities(prim: Usd.Prim, velocities: torch.Tensor | numpy.ndarray):
    if isinstance(velocities, torch.Tensor):
        velocities = velocities.cpu().numpy()
    UsdGeom.Points(prim).GetVelocitiesAttr().Set(Vt.Vec3fArray.FromNumpy(velocities))
