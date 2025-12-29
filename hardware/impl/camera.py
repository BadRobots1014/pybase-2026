import math
from typing import TYPE_CHECKING, List, Tuple

from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d

from constants import SIM
from hardware.abstract.camera import Camera

if TYPE_CHECKING:
    from physics import AprilTag, AprilTagManager


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
        tag_manager: "AprilTagManager",
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

        # History for latency compensation
        self._pose_history: List[Tuple[float, Pose2d]] = []

        # Cache for vision measurements
        self._visible_tag_count = 0
        self._current_max_distance = 0.0
        self._current_std_dev = (0.0, 0.0, 0.0)

        # Create NetworkTables publishers for telemetry
        self._visible_tags_pub = self.nt_table.getIntegerTopic(
            "Visible Tag Count"
        ).publish()
        self._max_distance_pub = self.nt_table.getDoubleTopic(
            "Max Detection Distance (m)"
        ).publish()
        self._current_distance_pub = self.nt_table.getDoubleTopic(
            "Current Max Distance (m)"
        ).publish()
        self._std_dev_x_pub = self.nt_table.getDoubleTopic("Std Dev X").publish()
        self._std_dev_y_pub = self.nt_table.getDoubleTopic("Std Dev Y").publish()
        self._std_dev_theta_pub = self.nt_table.getDoubleTopic(
            "Std Dev Theta"
        ).publish()
        self._has_target_pub = self.nt_table.getBooleanTopic("Has Target").publish()

        # Camera specifications (for reference)
        self._fov_pub = self.nt_table.getDoubleTopic("FOV (deg)").publish()
        self._fov_pub.set(math.degrees(self.horizontal_fov))
        self._frame_rate_pub = self.nt_table.getDoubleTopic(
            "Frame Rate (fps)"
        ).publish()
        self._frame_rate_pub.set(self.frame_rate)
        self._is_rolling_shutter_pub = self.nt_table.getBooleanTopic(
            "Rolling Shutter"
        ).publish()
        self._is_rolling_shutter_pub.set(self.is_rolling_shutter)
        self._absolute_max_pub = self.nt_table.getDoubleTopic(
            "Absolute Max Distance (m)"
        ).publish()
        self._absolute_max_pub.set(self.absolute_max_distance)

        # Visible tag IDs (as string for easy reading)
        self._visible_tag_ids_pub = self.nt_table.getStringTopic(
            "Visible Tag IDs"
        ).publish()

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

        # Store pose history for latency lookup
        self._pose_history.append(
            (
                Timer.getFPGATimestamp(),
                Pose2d(robot_x, robot_y, Rotation2d(robot_heading)),
            )
        )
        if len(self._pose_history) > 50:  # Keep roughly 1 second of history at 50Hz
            self._pose_history.pop(0)

    def dynamic_max_distance(
        self, linear_velocity: float, angular_velocity: float
    ) -> float:
        """
        Calculate maximum viewable distance accounting for motion blur and rolling shutter.

        Returns 0.0 if motion constraints make detection impossible.
        """
        # Add tolerance for near-zero velocities
        velocity_tolerance = 1e-6

        # don't waste time computing if not moving
        if (
            linear_velocity < velocity_tolerance
            and angular_velocity < velocity_tolerance
        ):
            return self.absolute_max_distance

        # Rolling shutter hard angular constraint (distance-independent)
        if self.is_rolling_shutter:
            readout_time = 1.0 / self.frame_rate
            angular_distortion = angular_velocity * readout_time

            if angular_distortion > SIM.max_angular_distortion_rad:
                # Rotating too fast for rolling shutter detection
                return 0.0

        # Motion blur penalties (reduce max distance as speed increases)
        # Instead of calculating threshold distance, we treat speed as a range multiplier
        linear_penalty = linear_velocity * SIM.motion_blur_constant
        angular_penalty = angular_velocity * SIM.angular_blur_constant

        # Calculate dynamic max by subtracting penalties from the resolution limit
        dynamic_max = self.absolute_max_distance - linear_penalty - angular_penalty

        # Geometric distortion for rolling shutter further penalizes range
        if self.is_rolling_shutter and linear_velocity > velocity_tolerance:
            dynamic_max -= linear_velocity * SIM.rolling_shutter_penalty

        return max(0.0, dynamic_max)

    def vision_measurement_valid(self) -> bool:
        """Check if camera has any visible tags"""
        if not self.enabled:
            self._has_target_pub.set(False)
            self._visible_tags_pub.set(0)
            self._current_distance_pub.set(0.0)
            return False

        # Calculate max distance
        max_dist = self.dynamic_max_distance(
            self._cached_linear_velocity, self._cached_angular_velocity
        )

        self._current_max_distance = max_dist

        # Check if any tags are visible
        visible_tags = self.tag_manager.get_visible_tags(
            self._cached_robot_x,
            self._cached_robot_y,
            self._cached_robot_heading,
            max_dist,
            self.horizontal_fov,
        )

        self._visible_tag_count = len(visible_tags)
        has_target = len(visible_tags) > 0

        # Publish telemetry
        self._has_target_pub.set(has_target)
        self._visible_tags_pub.set(self._visible_tag_count)
        self._current_distance_pub.set(max_dist)

        # Publish visible tag IDs
        if visible_tags:
            tag_ids = []
            for i, tag in enumerate(self.tag_manager.tags):
                if tag in visible_tags:
                    tag_ids.append(str(i + 1))
            self._visible_tag_ids_pub.set(", ".join(tag_ids))
        else:
            self._visible_tag_ids_pub.set("None")

        return has_target

    def get_deviation(
        self, visible_tags: List["AprilTag"], robot_pose: Pose2d
    ) -> Tuple[float, float, float]:
        """
        Calculate standard deviation based on visible tags and distance.
        Lower deviation = higher confidence for Kalman filter
        """
        if not visible_tags:
            deviation = (100.0, 100.0, 100.0)
        else:
            base_deviation = SIM.vision_base_std_dev

            # Calculate average distance to all visible tags
            total_dist = 0.0
            for tag in visible_tags:
                dx = tag.pose.X() - robot_pose.X()
                dy = tag.pose.Y() - robot_pose.Y()
                total_dist += math.sqrt(dx**2 + dy**2)
            avg_dist = total_dist / len(visible_tags)

            # Accuracy drops quadratically with distance (common vision model)
            dist_factor = 1.0 + (avg_dist**2 / 10.0)

            if SIM.vision_multi_tag_improvement:
                tag_count_factor = 1.0 / math.sqrt(len(visible_tags))
                dev_value = base_deviation * tag_count_factor * dist_factor
            else:
                dev_value = base_deviation * dist_factor

            deviation = (
                dev_value,
                dev_value,
                dev_value * 2.0,
            )  # Rotation is usually less certain

        # Cache and publish standard deviation
        self._current_std_dev = deviation
        self._std_dev_x_pub.set(deviation[0])
        self._std_dev_y_pub.set(deviation[1])
        self._std_dev_theta_pub.set(deviation[2])

        return deviation

    def get_vision_measurement(
        self,
    ) -> Tuple[Pose2d, float, Tuple[float, float, float]]:
        """
        Simulate getting a vision measurement from visible tags.

        :return: Estimated pose, timestamp, and deviation
        """
        # Account for camera latency by looking back in history
        timestamp = Timer.getFPGATimestamp() - (self.latency / 1000.0)

        # Find the pose the robot was at when the "shutter fired"
        latent_pose = Pose2d(
            self._cached_robot_x,
            self._cached_robot_y,
            Rotation2d(self._cached_robot_heading),
        )
        for hist_time, hist_pose in reversed(self._pose_history):
            if hist_time <= timestamp:
                latent_pose = hist_pose

                # When a latent pose is found, the timestamp of the recorded pos should be used, not current.
                timestamp = hist_time
                break

        # Get visible tags from the perspective of the latent pose
        visible_tags = self.tag_manager.get_visible_tags(
            latent_pose.X(),
            latent_pose.Y(),
            latent_pose.rotation().radians(),
            self._current_max_distance,
            self.horizontal_fov,
        )

        if not visible_tags:
            return latent_pose, timestamp, (100.0, 100.0, 100.0)

        # Calculate deviation based on the actual pose at capture time
        deviation = self.get_deviation(visible_tags, latent_pose)

        return latent_pose, timestamp, deviation
