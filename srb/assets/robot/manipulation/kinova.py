from srb.core.action import (
    ActionGroup,
    DifferentialInverseKinematicsActionCfg,
    InverseKinematicsActionGroup,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, SerialManipulator, Tool, Transform
from srb.core.controller import DifferentialIKControllerCfg
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import deg_to_rad, rpy_to_quat
from srb.utils.nucleus import ISAAC_NUCLEUS_DIR

# TODO[mid]: Separate Kinova Jaco 2 gripper


class KinovaJ2n6s300(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/j2n6s300",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Kinova/Jaco2/J2N6S300/j2n6s300_instanceable.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "j2n6s300_joint_1": 0.0,
                "j2n6s300_joint_2": 2.76,
                "j2n6s300_joint_3": 2.76,
                "j2n6s300_joint_4": 2.5,
                "j2n6s300_joint_5": 2.0,
                "j2n6s300_joint_6": 0.0,
                "j2n6s300_joint_finger_[1-3]": 0.2,  # close: 1.2, open: 0.2
                "j2n6s300_joint_finger_tip_[1-3]": 0.2,  # close: 1.2, open: 0.2
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*_joint_[1-6]"],
                velocity_limit=100.0,
                effort_limit={
                    ".*_joint_[1-2]": 80.0,
                    ".*_joint_3": 40.0,
                    ".*_joint_[4-6]": 20.0,
                },
                stiffness={
                    ".*_joint_[1-3]": 4000.0,
                    ".*_joint_[4-6]": 1500.0,
                },
                damping={
                    ".*_joint_[1-3]": 1000.0,
                    ".*_joint_[4-6]": 500.0,
                },
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=[".*_finger_[1-3]", ".*_finger_tip_[1-3]"],
                velocity_limit=100.0,
                effort_limit=2.0,
                stiffness=1200.0,
                damping=10.0,
            ),
        },
    )
    end_effector: Tool | None = None

    ## Actions
    actions: ActionGroup = InverseKinematicsActionGroup(
        DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[".*_joint_.*"],
            base_name="j2n6s300_link_base",
            body_name="j2n6s300_end_effector",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="j2n6s300_link_base")
    frame_flange: Frame = Frame(
        prim_relpath="j2n6s300_end_effector",
    )
    frame_base_camera: Frame = Frame(
        prim_relpath="j2n6s300_link_base/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="j2n6s300_end_effector/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class KinovaJ2n7s300(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/j2n7s300",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Kinova/Jaco2/J2N7S300/j2n7s300_instanceable.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "j2n7s300_joint_1": 0.0,
                "j2n7s300_joint_2": 2.76,
                "j2n7s300_joint_3": 0.0,
                "j2n7s300_joint_4": 2.0,
                "j2n7s300_joint_5": 2.0,
                "j2n7s300_joint_6": 0.0,
                "j2n7s300_joint_7": 0.0,
                "j2n7s300_joint_finger_[1-3]": 0.2,  # close: 1.2, open: 0.2
                "j2n7s300_joint_finger_tip_[1-3]": 0.2,  # close: 1.2, open: 0.2
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*_joint_[1-7]"],
                velocity_limit=100.0,
                effort_limit={
                    ".*_joint_[1-2]": 8000.0,
                    ".*_joint_[3-4]": 4000.0,
                    ".*_joint_[5-7]": 2000.0,
                },
                stiffness={
                    ".*_joint_[1-4]": 4000.0,
                    ".*_joint_[5-7]": 1000.0,
                },
                damping={
                    ".*_joint_[1-4]": 1000.0,
                    ".*_joint_[5-7]": 500.0,
                },
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=[".*_finger_[1-3]", ".*_finger_tip_[1-3]"],
                velocity_limit=100.0,
                effort_limit=2.0,
                stiffness=1200.0,
                damping=10.0,
            ),
        },
    )
    end_effector: Tool | None = None

    ## Actions
    actions: ActionGroup = InverseKinematicsActionGroup(
        DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[".*_joint_.*"],
            base_name="j2n7s300_link_base",
            body_name="j2n7s300_end_effector",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="j2n7s300_link_base")
    frame_flange: Frame = Frame(
        prim_relpath="j2n7s300_end_effector",
    )
    frame_base_camera: Frame = Frame(
        prim_relpath="j2n7s300_link_base/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="j2n7s300_end_effector/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class KinovaGen3(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/kinova_gen3",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Kinova/Gen3/gen3n7_instanceable.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "joint_1": 0.0,
                "joint_2": deg_to_rad(45),
                "joint_3": 0.0,
                "joint_4": deg_to_rad(45),
                "joint_5": 0.0,
                "joint_6": deg_to_rad(90),
                "joint_7": 0.0,
            },
        ),
        actuators={
            "shoulder": ImplicitActuatorCfg(
                joint_names_expr=["joint_[1-4]"],
                effort_limit=87.0,
                velocity_limit=2.175,
                stiffness=4000.0,
                damping=800.0,
            ),
            "forearm": ImplicitActuatorCfg(
                joint_names_expr=["joint_[5-7]"],
                effort_limit=12.0,
                velocity_limit=2.61,
                stiffness=4000.0,
                damping=800.0,
            ),
        },
    )
    # TODO[mid]: Fix Kinova Gen3 with articulated end-effectors (failed to find an articulation when resolving)
    # end_effector: Tool | None = FrankaHand()

    ## Actions
    actions: ActionGroup = InverseKinematicsActionGroup(
        DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["joint_.*"],
            base_name="base_link",
            body_name="end_effector_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(
        prim_relpath="end_effector_link",
    )
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="end_effector_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )
