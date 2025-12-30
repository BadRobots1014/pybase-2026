import math
from typing import TYPE_CHECKING, List, Tuple

from pyfrc.physics import core
from wpilib import Field2d
from wpimath.geometry import Pose2d, Rotation2d

from constants import SIM
from hardware.impl import (
    CoaxialSwerveModule,
    DummyCamera,
    DummyCoaxialAzimuthComponent,
    DummyCoaxialDriveComponent,
    DummyGyro,
)

# explicit importing causes errors, only needed for type checking
if TYPE_CHECKING:
    from robot import Robot


class AprilTag:
    pose: Pose2d

    def __init__(self, x: float, y: float, z: float, z_rotation: float):
        """
        Create an digital verison of an april tags.
        0,0 is bottom left of field. Refer to the Field2d image for reference.

        :param x: the x-offset in meters
        :param y: the y-offset in meters
        :param z: the z-offset in meters
        :param z_rotation: the rotation of the tag in degrees
        """
        rotation_rad = math.radians(z_rotation)
        self.pose = Pose2d(x, y, rotation_rad)

    def get_visibility_and_distance(
        self,
        robot_x: float,
        robot_y: float,
        robot_rotation: float,
        max_dist: float,
        horizontal_fov: float,
    ) -> Tuple[bool, float]:
        """
        Check if an AprilTag is visible to the camera.

        :param robot_x: Robot's x position in meters
        :param robot_y: Robot's y position in meters
        :param robot_rotation: Robot's heading in radians (0 = facing +X)
        :param max_dist: maximum distance to be visible by robot
        :param horizontal_fov: the fov of the camera in radians
        :return: True if tag is visible
        """
        if max_dist <= 0:
            return False, 0.0

        # Distance check
        dx = self.pose.X() - robot_x
        dy = self.pose.Y() - robot_y
        distance = math.hypot(dx, dy)

        if distance > max_dist:
            return False, distance

        # FOV check
        angle_to_tag = math.atan2(dy, dx)
        relative_angle = self._normalize_angle(angle_to_tag - robot_rotation)

        if abs(relative_angle) > (horizontal_fov / 2):
            return False, distance

        # orientation check (can the camera see the front of the tag?)
        tag_to_camera_angle = math.atan2(-dy, -dx)
        tag_rotation = self.pose.rotation().radians()
        angle_diff = self._normalize_angle(tag_to_camera_angle - tag_rotation)

        if abs(angle_diff) > SIM.max_tag_viewing_angle_rad:
            return False, distance

        return True, distance

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))


class AprilTagManager:
    """
    Manager for digital representation of april tags on the field.
    """

    field: Field2d
    tags: List[AprilTag]

    def __init__(self, field: Field2d, tags: List[AprilTag]):
        self.field = field
        self.tags = tags

        self._display_tags()

    def _display_tags(self):
        poses = []
        for tag in self.tags:
            poses.append(tag.pose)

        self.field.getObject("AprilTags").setPoses(poses)

    def get_visible_tags(
        self,
        robot_x: float,
        robot_y: float,
        robot_rotation: float,
        max_dist: float,
        horizontal_fov: float,
    ) -> List[AprilTag]:
        """
        Get all visible AprilTags given current robot state.

        Returns list of tags that pass all visibility checks.
        """
        visible_pairs = []

        for tag in self.tags:
            is_visible, dist = tag.get_visibility_and_distance(
                robot_x, robot_y, robot_rotation, max_dist, horizontal_fov
            )
            if is_visible:
                # Store as a tuple (distance, tag) for easy sorting
                visible_pairs.append((dist, tag))

        # Sort based on the first element of the tuple (the distance)
        visible_pairs.sort(key=lambda x: x[0])

        # Return only the AprilTag objects, stripped of the distance metadata
        return [pair[1] for pair in visible_pairs]


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
        """
        return
