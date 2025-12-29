from typing import TYPE_CHECKING

from pyfrc.physics import core
from wpilib import Field2d
from wpimath.geometry import Pose2d, Rotation2d

from constants import SIM
from hardware.impl import CoaxialSwerveModule, DummyGyro
from hardware.impl.camera import AprilTag, AprilTagManager, DummyCamera
from hardware.impl.motor import DummyCoaxialAzimuthComponent, DummyCoaxialDriveComponent

# explicit importing causes errors, only needed for type checking
if TYPE_CHECKING:
    from robot import Robot


def create_apriltag_manager(field: Field2d) -> AprilTagManager:
    """
    Create AprilTagManager with all field tags.

    Note:
        Update each season with new tags for accurate simulation

    Args:
        field: existing Field2d object (e.g., swerve.field)

    Returns:
        AprilTagManager instance with all tags
    """
    # AprilTag data: ID, X, Y, Z, Z-Rotation, Y-Rotation (in inches and degrees)
    tag_data = [
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

    # Create AprilTag objects
    tags = []
    for tag_id, x_in, y_in, z_in, z_rot, y_rot in tag_data:
        # Convert inches to meters
        x_m = x_in * 0.0254
        y_m = y_in * 0.0254
        z_m = z_in * 0.0254

        # Create AprilTag (z_rotation in degrees will be converted to radians internally)
        tag = AprilTag(x_m, y_m, z_m, z_rot)
        tags.append(tag)

    # Create and return manager (it will handle displaying tags on field)
    return AprilTagManager(field, tags)


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
        self.swerve._camera_list = [
            DummyCamera(
                horizontal_fov=SIM.camera_horizontal_fov_deg,
                horizontal_pixels=SIM.camera_horizontal_pixels,
                frame_rate=SIM.camera_frame_rate,
                latency=SIM.camera_latency_ms,
                tag_manager=self.tag_manager,
                is_rolling_shutter=SIM.camera_is_rolling_shutter,
                enabled=True,
                name="sim_camera",
            )
        ]

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

        This prevents rubberbanding by continuously telling the physics controller
        where the robot thinks it is based on odometry.
        """
        # Get the current pose from swerve odometry
        current_pose = self.swerve.pose

        # Update the physics controller with the current pose
        # This is what prevents rubberbanding - the physics controller needs to know
        # where your odometry thinks the robot is
        self.physics_controller.field.setRobotPose(current_pose)

        return
