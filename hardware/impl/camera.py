import math
from typing import List, Tuple

from wpilib import Field2d, Timer
from wpimath.geometry import Pose2d, Rotation2d

from constants import SIM
from hardware.abstract.camera import Camera


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

    def can_see_tag(
        self,
        robot_x: float,
        robot_y: float,
        robot_rotation: float,
        max_dist: float,
        horizontal_fov: float,
    ) -> bool:
        """
        Check if an AprilTag is visible to the camera.

        :param robot_x: Robot's x position in meters
        :param robot_y: Robot's y position in meters
        :param robot_rotation: Robot's heading in radians (0 = facing +X)
        :param max_dist: maximum distance to be visible by robot
        :param horizontal_fov: the fov of the camera in radians
        :return: True if tag is visible
        """
        if max_dist == 0:
            # Moving too fast for detection
            return False

        # Distance check
        dx = self.pose.X() - robot_x
        dy = self.pose.Y() - robot_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance > max_dist:
            return False

        # FOV check
        angle_to_tag = math.atan2(dy, dx)
        relative_angle = self._normalize_angle(angle_to_tag - robot_rotation)

        half_fov = horizontal_fov / 2
        if abs(relative_angle) > half_fov:
            return False

        # Tag orientation check
        tag_to_camera_angle = math.atan2(-dy, -dx)
        tag_rotation = self.pose.rotation().radians()
        angle_diff = self._normalize_angle(tag_to_camera_angle - tag_rotation)

        max_viewing_angle = SIM.max_tag_viewing_angle_rad
        if abs(angle_diff) > max_viewing_angle:
            return False

        return True

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
        if max_dist == 0:
            # Moving too fast for detection
            return []

        visible_tags = []
        for tag in self.tags:
            if tag.can_see_tag(
                robot_x, robot_y, robot_rotation, max_dist, horizontal_fov
            ):
                visible_tags.append(tag)

        return visible_tags


class Limelight(Camera):
    def __init__(self, enabled: bool = True, name: str = "limelight") -> None:
        super().__init__(enabled, name)

        # nt_table and nt_instance defined in super
        # setup subscriptions
        # returns [x, y, x, roll, pitch, yaw, latency]
        self.pose_sub = self.nt_table.getDoubleArrayTopic("botpose_wpiblue").subscribe(
            [0] * 7
        )
        # tv = target valid
        self.tv_sub = self.nt_table.getBooleanTopic("tv").subscribe(False)
        # tc = tag count
        self.tc_sub = self.nt_table.getIntegerTopic("tc").subscribe(0)

    def array_to_pose2d(self, arr: list[float]):
        return Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[5]))

    def vision_measurement_valid(self) -> bool:
        return self.tv_sub.get() and self.enabled

    # TODO: implement actual algorithm based on tag count, tag distance, and speed/rotation of robot
    # algorithm is used to tell the kalman filter how much to trust the pose estimation. lower is more confidant
    def get_deviation(self) -> Tuple[float, float, float]:
        return 0.7, 0.7, 0.7  # placeholder values

    def get_vision_measurement(
        self,
    ) -> tuple[Pose2d, float, tuple[float, float, float]]:
        # get pose array
        arr = self.pose_sub.get()

        pose = self.array_to_pose2d(arr)
        # arr[6] is latency in ms, and is used to compute absolute timestamp of pose estimate
        timestamp = Timer.getFPGATimestamp() - (arr[6] / 1000.0)
        deviation = self.get_deviation()
        return pose, timestamp, deviation


class DummyCamera(Camera):
    """A psuedo camera instance to use in physics sim"""

    def __init__(
        self,
        horizontal_fov: float,
        horizontal_pixels: float,
        frame_rate: float,
        latency: float,
        tag_manager: AprilTagManager,
        is_rolling_shutter: bool = False,
        enabled: bool = True,
        name: str = "DummyCamera",
    ) -> None:
        """
        Construct a camera for sim purposes. For most accuarte sim, refer to camera manual
        and plug in the settings here.

        :param horizontal_fov: Maximum fov in degrees
        :param horizontal_pixels: The number of horizontal pixels per image
        :param frame_rate: The number of frames per second
        :param latency: avg. expected latency of camera in ms
        :param tag_manager: AprilTagManager containing all field tags
        :param is_rolling_shutter: Whether to simulate a rolling shutter
        """
        super().__init__(enabled, name)

        self.horizontal_fov = math.radians(horizontal_fov)  # Convert to radians
        self.horizontal_pixels = horizontal_pixels
        self.frame_rate = frame_rate
        self.latency = latency
        self.is_rolling_shutter = is_rolling_shutter
        self.tag_manager = tag_manager

        # Compute focal length in pixels
        self.focal_length = horizontal_pixels / (2 * math.tan(self.horizontal_fov / 2))

        # compute the absolute maximum viewable distance.
        self.absolute_max_distance = (
            SIM.april_tag_physical_size * self.focal_length
        ) / SIM.minimum_pixels_per_tag

        # Cache for robot state (will be updated each cycle)
        self._cached_robot_x = 0.0
        self._cached_robot_y = 0.0
        self._cached_robot_heading = 0.0
        self._cached_linear_velocity = 0.0
        self._cached_angular_velocity = 0.0

    def update_robot_state(
        self,
        robot_x: float,
        robot_y: float,
        robot_heading: float,
        vx: float,
        vy: float,
        omega: float,
    ):
        """
        Update the cached robot state for vision calculations.
        Should be called by the swerve subsystem before calling vision methods.
        """
        self._cached_robot_x = robot_x
        self._cached_robot_y = robot_y
        self._cached_robot_heading = robot_heading
        self._cached_linear_velocity = math.sqrt(vx**2 + vy**2)
        self._cached_angular_velocity = abs(omega)

    def dynamic_max_distance(
        self, linear_velocity: float, angular_velocity: float
    ) -> float:
        """
        Calculate maximum viewable distance accounting for motion blur and rolling shutter.

        Returns 0.0 if motion constraints make detection impossible.
        """
        # don't waste time computing if not moving
        if linear_velocity == 0 and angular_velocity == 0:
            return self.absolute_max_distance

        # Rolling shutter hard angular constraint (distance-independent)
        if self.is_rolling_shutter:
            readout_time = 1.0 / self.frame_rate
            angular_distortion = angular_velocity * readout_time

            if angular_distortion > SIM.max_angular_distortion_rad:
                # Rotating too fast for rolling shutter detection
                return 0.0

        # Motion blur constraint (applies to both shutter types)
        tag_edge_velocity = (
            linear_velocity + angular_velocity * SIM.april_tag_physical_size / 2
        )
        dynamic_max_blur = (tag_edge_velocity * self.focal_length) / (
            SIM.pixel_threshold * self.frame_rate
        )

        # Rolling shutter geometric distortion
        if self.is_rolling_shutter:
            dynamic_max_geometric = (linear_velocity * self.focal_length) / (
                SIM.max_pixel_skew * self.frame_rate
            )

            dynamic_max = min(dynamic_max_blur, dynamic_max_geometric)
        else:
            dynamic_max = dynamic_max_blur

        # Can never exceed resolution limit
        return min(dynamic_max, self.absolute_max_distance)

    def vision_measurement_valid(self) -> bool:
        """Check if camera has any visible tags"""
        if not self.enabled:
            return False

        # Calculate max distance
        max_dist = self.dynamic_max_distance(
            self._cached_linear_velocity, self._cached_angular_velocity
        )

        # Check if any tags are visible
        visible_tags = self.tag_manager.get_visible_tags(
            self._cached_robot_x,
            self._cached_robot_y,
            self._cached_robot_heading,
            max_dist,
            self.horizontal_fov,
        )

        return len(visible_tags) > 0

    def get_deviation(
        self, visible_tags: List[AprilTag], robot_x: float, robot_y: float
    ) -> Tuple[float, float, float]:
        """
        Calculate standard deviation based on visible tags.
        Lower deviation = higher confidence for Kalman filter
        """
        if not visible_tags:
            return 100.0, 100.0, 100.0  # Very high uncertainty if no tags

        # For simulation, use reasonable uncertainty
        base_deviation = SIM.vision_base_std_dev

        if SIM.vision_multi_tag_improvement:
            tag_count_factor = 1.0 / math.sqrt(len(visible_tags))
            deviation = base_deviation * tag_count_factor
        else:
            deviation = base_deviation

        return deviation, deviation, deviation

    def get_vision_measurement(
        self,
    ) -> Tuple[Pose2d, float, Tuple[float, float, float]]:
        """
        Simulate getting a vision measurement from visible tags.

        :return: Estimated pose, timestamp, and deviation
        """
        # Use cached robot state
        robot_x = self._cached_robot_x
        robot_y = self._cached_robot_y
        robot_heading = self._cached_robot_heading

        # Calculate max distance based on motion
        max_dist = self.dynamic_max_distance(
            self._cached_linear_velocity, self._cached_angular_velocity
        )

        # Get visible tags
        visible_tags = self.tag_manager.get_visible_tags(
            robot_x, robot_y, robot_heading, max_dist, self.horizontal_fov
        )

        if not visible_tags:
            # Should not happen since vision_measurement_valid is checked first
            pose = Pose2d(robot_x, robot_y, Rotation2d(robot_heading))
            timestamp = Timer.getFPGATimestamp() - (self.latency / 1000.0)
            return pose, timestamp, (100.0, 100.0, 100.0)

        # For now, just return the current pose (no actual estimation)
        # This simulates a perfect vision system
        pose = Pose2d(robot_x, robot_y, Rotation2d(robot_heading))

        # Account for camera latency
        timestamp = Timer.getFPGATimestamp() - (self.latency / 1000.0)

        # Calculate deviation based on visible tags
        deviation = self.get_deviation(visible_tags, robot_x, robot_y)

        return pose, timestamp, deviation
