import simforge_foundry

from srb.core.action import (
    ActionGroup,
    BodyAccelerationActionCfg,
    BodyAccelerationActionGroup,
    ThrustActionCfg,
    ThrustActionGroup,
    ThrusterCfg,
)
from srb.core.asset import Frame, OrbitalRobot, RigidObjectCfg, Transform, AssetRegistry
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    SimforgeAssetCfg,
    UsdFileCfg,
)
from srb.utils.math import deg_to_rad, rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class Gateway(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/gateway",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("gateway.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
            ),
            mass_props=MassPropertiesCfg(density=1500.0),
        ),
    )

    ## Actions
    actions: ActionGroup = BodyAccelerationActionGroup(
        BodyAccelerationActionCfg(asset_name="robot", scale=0.05)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(-0.5, 0.0, -0.1),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_onboard_camera: Frame = Frame(
        prim_relpath="base/camera_onboard",
        offset=Transform(
            pos=(11.148, 0.05865, -1.63578),
            rot=rpy_to_quat(0.0, 0.0, 180.0),
        ),
    )


class Cubesat(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cubesat",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.Cubesat()],
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
            ),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
    )

    ## Actions
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            thrusters=(
                ThrusterCfg(
                    offset=(-0.05, -0.05, 0.05),
                    direction=(-0.5, -0.5, 1.0),
                    power=10.0,
                ),
                ThrusterCfg(
                    offset=(-0.05, 0.05, 0.05),
                    direction=(-0.5, 0.5, 1.0),
                    power=10.0,
                ),
                ThrusterCfg(
                    offset=(0.05, -0.05, 0.05),
                    direction=(0.5, -0.5, 1.0),
                    power=10.0,
                ),
                ThrusterCfg(
                    offset=(0.05, 0.05, 0.05),
                    direction=(0.5, 0.5, 1.0),
                    power=10.0,
                ),
                ThrusterCfg(
                    offset=(-0.05, -0.05, -0.05),
                    direction=(-0.5, -0.5, -1.0),
                    power=10.0,
                ),
                ThrusterCfg(
                    offset=(-0.05, 0.05, -0.05),
                    direction=(-0.5, 0.5, -1.0),
                    power=10.0,
                ),
                ThrusterCfg(
                    offset=(0.05, -0.05, -0.05),
                    direction=(0.5, -0.5, -1.0),
                    power=10.0,
                ),
                ThrusterCfg(
                    offset=(0.05, 0.05, -0.05),
                    direction=(0.5, 0.5, -1.0),
                    power=10.0,
                ),
            ),
            fuel_capacity=5.0,
            fuel_consumption_rate=(5.0 / (8 * 10.0)) / 20.0,
        )
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="cubesat")
    frame_payload_mount: Frame = Frame(
        prim_relpath="cubesat",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="cubesat",
        offset=Transform(
            pos=(0.0, 0.0, 0.05),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_onboard_camera: Frame = Frame(
        prim_relpath="base/camera_onboard",
        offset=Transform(
            pos=(0.075, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )


class VenusExpress(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/venus_express",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("venus_express.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
            ),
            mass_props=MassPropertiesCfg(density=1500.0),
        ),
    )

    ## Actions
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            thrusters=(
                # +X side (right)
                ThrusterCfg(
                    offset=(1.0, 0.25, 0.0),
                    direction=(1.0, 0.0, 0.0),
                    power=2000.0,
                ),
                ThrusterCfg(
                    offset=(1.0, -0.25, 0.0),
                    direction=(1.0, 0.0, 0.0),
                    power=2000.0,
                ),
                # -X side (left)
                ThrusterCfg(
                    offset=(-1.0, 0.25, 0.0),
                    direction=(-1.0, 0.0, 0.0),
                    power=2000.0,
                ),
                ThrusterCfg(
                    offset=(-1.0, -0.25, 0.0),
                    direction=(-1.0, 0.0, 0.0),
                    power=2000.0,
                ),
                # +Y side (front)
                ThrusterCfg(
                    offset=(0.25, 1.0, 0.0),
                    direction=(0.0, 1.0, 0.0),
                    power=2000.0,
                ),
                ThrusterCfg(
                    offset=(-0.25, 1.0, 0.0),
                    direction=(0.0, 1.0, 0.0),
                    power=2000.0,
                ),
                # -Y side (back)
                ThrusterCfg(
                    offset=(0.25, -1.0, 0.0),
                    direction=(-1.0, 0.0, 0.0),
                    power=2000.0,
                ),
                ThrusterCfg(
                    offset=(-0.25, -1.0, 0.0),
                    direction=(-1.0, 0.0, 0.0),
                    power=2000.0,
                ),
            ),
            fuel_capacity=2500.0,
            fuel_consumption_rate=(2500.0 / (8 * 2000.0)) / 20.0,
        )
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.06691, -0.838982, 0.051877),
            rot=rpy_to_quat(90.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, -0.543748, 0.417019),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_onboard_camera: Frame = Frame(
        prim_relpath="base/camera_onboard",
        offset=Transform(
            pos=(1.0, 1.0, 2.0),
            rot=rpy_to_quat(0.0, 0.0, 90.0),
        ),
    )


class Starship(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/starship",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("starship.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
            ),
            mass_props=MassPropertiesCfg(mass=100000.0),
        ),
    )

    ## Actions
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            thrusters=(
                ThrusterCfg(
                    offset=(0.0, 0.7726, 0.76),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-0.669, -0.3863, 0.76),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(0.669, -0.3863, 0.76),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-1.70384, 0.9837, 0.54),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(0.0, -1.96743, 0.54),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(1.70384, 0.9837, 0.54),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
            ),
            fuel_capacity=0.5 * 1200000.0,
            fuel_consumption_rate=(0.5 * 1200000.0 / (6 * 2300000.0)) / 20.0,
        )
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_onboard_camera: Frame = Frame(
        prim_relpath="base/camera_onboard",
        offset=Transform(
            pos=(0.0, 3.6437, -0.0155),
            rot=rpy_to_quat(0.0, 90.0, 0.0),
        ),
    )


class SuperHeavy(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/super_heavy",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("super_heavy.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
            ),
            mass_props=MassPropertiesCfg(mass=200000.0),
        ),
    )

    ## Actions
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            thrusters=(
                # Gimbaled (13)
                ThrusterCfg(
                    offset=(0.750003, 0.0, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-0.375002, 0.649522, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-0.375002, -0.649522, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(1.78697, 0.478816, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(1.16425, 1.43772, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(0.096822, 1.84747, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-1.00758, 1.55154, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-1.72713, 0.662982, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-1.78697, -0.478816, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-1.16425, -1.43772, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-0.096822, -1.84747, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(1.00758, -1.55154, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(1.72713, -0.662982, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                # Fixed (20)
                ThrusterCfg(
                    offset=(3.0, 0.0, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(2.85317, 0.927052, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(2.42705, 1.76336, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(1.76335, 2.42705, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(0.92705, 2.85317, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(0.0, 3.0, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(-0.92705, 2.85317, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(-1.76335, 2.42705, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(-2.42705, 1.76336, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(-2.85317, 0.927052, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(-3.0, 0.0, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(-2.85317, -0.927048, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(-2.42705, -1.76335, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(-1.76335, -2.42705, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(-0.92705, -2.85317, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(0.0, -3.0, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(0.92705, -2.85317, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(1.76335, -2.42705, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(2.42705, -1.76335, 1.8),
                    power=2300000.0,
                ),
                ThrusterCfg(
                    offset=(2.85317, -0.927048, 1.8),
                    power=2300000.0,
                ),
            ),
            fuel_capacity=0.5 * 3400000.0,
            fuel_consumption_rate=(0.5 * 3400000.0 / (33 * 2300000.0)) / 20.0,
        )
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_onboard_camera: Frame = Frame(
        prim_relpath="base/camera_onboard",
        offset=Transform(
            pos=(0.0, -4.45, 2.25),
            rot=rpy_to_quat(0.0, 90.0, 0.0),
        ),
    )


class LabSc(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/robot",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("untitled.usd")
            .as_posix(),
            scale=(0.1, 0.1, 0.1),  # Scale down to 10% of original size
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
            ),
            mass_props=MassPropertiesCfg(density=1000.0),  # 1000x density to compensate for 0.1³ = 0.001 volume scaling
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),  # At origin
            rot=(0.0, 0.0, 0.0, 1.0),  # identity (no unintended roll)
            lin_vel=(0.0, 0.0, 0.0),  # No initial velocity
            ang_vel=(0.0, 0.0, 0.0),  # No initial angular velocity
        ),
    )

    ## Actions
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            # Main propulsion thrusters (reduced power for precise control)
            thrusters=(
                # Main X-axis thrusters (reduced from 2000N to 100N)
                ThrusterCfg(offset=(1.0, 0.25, 0.0), direction=(1.0, 0.0, 0.0), power=100.0),
                ThrusterCfg(offset=(1.0, -0.25, 0.0), direction=(1.0, 0.0, 0.0), power=100.0),
                ThrusterCfg(offset=(-1.0, 0.25, 0.0), direction=(-1.0, 0.0, 0.0), power=100.0),
                ThrusterCfg(offset=(-1.0, -0.25, 0.0), direction=(-1.0, 0.0, 0.0), power=100.0),
                
                # Main Y-axis thrusters (reduced from 2000N to 100N)
                ThrusterCfg(offset=(0.25, 1.0, 0.0), direction=(0.0, 1.0, 0.0), power=100.0),
                ThrusterCfg(offset=(-0.25, 1.0, 0.0), direction=(0.0, 1.0, 0.0), power=100.0),
                ThrusterCfg(offset=(0.25, -1.0, 0.0), direction=(0.0, -1.0, 0.0), power=100.0),
                ThrusterCfg(offset=(-0.25, -1.0, 0.0), direction=(0.0, -1.0, 0.0), power=100.0),
                
                # Attitude control thrusters (new - for roll/pitch/yaw control)
                ThrusterCfg(offset=(0.0, 0.0, 0.5), direction=(0.0, 0.0, 1.0), power=20.0),   # Roll control (+Z)
                ThrusterCfg(offset=(0.0, 0.0, -0.5), direction=(0.0, 0.0, -1.0), power=20.0),  # Roll control (-Z)
                ThrusterCfg(offset=(0.5, 0.0, 0.0), direction=(1.0, 0.0, 0.0), power=20.0),   # Pitch control (+X)
                ThrusterCfg(offset=(-0.5, 0.0, 0.0), direction=(-1.0, 0.0, 0.0), power=20.0), # Pitch control (-X)
                ThrusterCfg(offset=(0.0, 0.5, 0.0), direction=(0.0, 1.0, 0.0), power=20.0),   # Yaw control (+Y)
                ThrusterCfg(offset=(0.0, -0.5, 0.0), direction=(0.0, -1.0, 0.0), power=20.0), # Yaw control (-Y)
            ),
            fuel_capacity=1.0,
            fuel_consumption_rate=0.0,
            # Enable attitude control features
            planar_lock=True,        # Keep Z motion minimal
            lock_roll_pitch=False,   # Allow roll/pitch control via thrusters
            lock_yaw=False,          # Allow yaw control via thrusters
            level_body=False,        # Don't force orientation
            control_xy=False,        # Direct thruster control (not Fx,Fy commands)
        )
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_onboard_camera: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),  # Camera at robot center
            rot=rpy_to_quat(0.0, 0.0, 90.0),  # Test: yaw=90° to see what Isaac Sim Orient XYZ shows
        ),
    )
    frame_onboard_imu: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 1.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_imu: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 1.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

