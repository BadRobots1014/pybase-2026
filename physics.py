from typing import TYPE_CHECKING

from pyfrc.physics import core
from wpimath.geometry import Pose2d, Rotation2d

from constants import SIM
from hardware.impl import (
    CoaxialSwerveModule,
    DummyCamera,
    DummyCoaxialAzimuthComponent,
    DummyCoaxialDriveComponent,
    DummyGyro,
)
from hardware.impl.camera import create_apriltag_manager

# explicit importing causes errors, only needed for type checking
if TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine(core.PhysicsEngine):
    def __init__(self, physics_controller, robot: "Robot"):
        self.physics_controller = physics_controller

        # Switch all physical components to dummy versions
        self.swerve = robot.container.swerve

        self.swerve._modules = tuple(
            CoaxialSwerveModule(
                drive=DummyCoaxialDriveComponent(),
                azimuth=DummyCoaxialAzimuthComponent(),
                placement=module.placement,
                name=module.name,
            )
            for module in self.swerve._modules
        )

        self.swerve._gyro = DummyGyro()

        # Create AprilTag manager for the field
        self.tag_manager = create_apriltag_manager(self.swerve.field)

        # Initialize DummyCamera with proper parameters
        self.swerve._camera_list = []

        # Set initial robot position
        initial_pose = Pose2d(
            SIM.initial_x,
            SIM.initial_y,
            Rotation2d.fromDegrees(SIM.initial_heading_deg),
        )
        self.swerve.reset_odometry(initial_pose)

        return

    def update_sim(self, now, tm_diff):
        """
        Update the simulation state based on the robot's current state.
        """
        return
