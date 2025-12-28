import math
from typing import TYPE_CHECKING

from pyfrc.physics import core
from wpilib import Field2d
from wpimath.geometry import Pose2d, Rotation2d

from hardware.impl import CoaxialSwerveModule, DummyGyro
from hardware.impl.camera import DummyCamera
from hardware.impl.motor import DummyCoaxialAzimuthComponent, DummyCoaxialDriveComponent

# explicit importing causes errors, only needed for type checking
if TYPE_CHECKING:
    from robot import Robot


def add_apriltags_to_field(field: Field2d):
    """
    Add AprilTags to an existing Field2d object.

    Note:
        Update each season with new tags for accurate display

    Args:
        field: existing Field2d object (e.g., swerve.field)

    Usage:
        add_apriltags_to_field(self.swerve.field)
    """
    # AprilTag data: ID, X, Y, Z, Z-Rotation, Y-Rotation (in inches and degrees)
    tags = [
        (1, 657.37, 25.80, 58.50, 126, 0),
        (2, 657.37, 291.20, 58.50, 234, 0),
        (3, 455.15, 317.15, 51.25, 270, 0),
        (4, 365.20, 241.64, 73.54, 0, 30),
        (5, 365.20, 75.39, 73.54, 0, 30),
        (6, 530.49, 130.17, 12.13, 300, 0),
        (7, 546.87, 158.50, 12.13, 0, 0),
        (8, 530.49, 186.83, 12.13, 60, 0),
        (9, 497.77, 186.83, 12.13, 120, 0),
        (10, 481.39, 158.50, 12.13, 180, 0),
        (11, 497.77, 130.17, 12.13, 240, 0),
        (12, 33.51, 25.80, 58.50, 54, 0),
        (13, 33.51, 291.20, 58.50, 306, 0),
        (14, 325.68, 241.64, 73.54, 180, 30),
        (15, 325.68, 75.39, 73.54, 180, 30),
        (16, 235.73, -0.15, 51.25, 90, 0),
        (17, 160.39, 130.17, 12.13, 240, 0),
        (18, 144.00, 158.50, 12.13, 180, 0),
        (19, 160.39, 186.83, 12.13, 120, 0),
        (20, 193.10, 186.83, 12.13, 60, 0),
        (21, 209.49, 158.50, 12.13, 0, 0),
        (22, 193.10, 130.17, 12.13, 300, 0),
    ]

    # Convert all tags to poses
    poses = []
    for tag_id, x_in, y_in, z_in, z_rot, y_rot in tags:
        # Convert inches to meters
        x_m = x_in * 0.0254
        y_m = y_in * 0.0254

        # Convert rotation to radians
        rotation_rad = math.radians(z_rot)

        # Create pose
        pose = Pose2d(x_m, y_m, Rotation2d(rotation_rad))
        poses.append(pose)

    # Add all tags as a single object so they share the same icon
    field.getObject("AprilTags").setPoses(poses)


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
        self.swerve._camera_list = [DummyCamera()]

        add_apriltags_to_field(self.swerve.field)

        return

    def update_sim(self, now, tm_diff):
        return
