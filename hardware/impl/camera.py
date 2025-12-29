import math
from typing import List, Tuple

from hal.simulation import setDIOValue
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
        :param is_rolling_shutter: Whether to simulate a rolling shutter
        """
        super().__init__(enabled, name)

        self.horizontal_fov = horizontal_fov
        self.horizontal_pixels = horizontal_pixels
        self.frame_rate = frame_rate
        self.latency = latency
        self.is_rolling_shutter = is_rolling_shutter

        # Compute focal length. It's essentially the phsyical size of each pixel.
        self.focal_length = horizontal_pixels / (2 * math.tan(horizontal_fov / 2))

        # compute the absolute maximum viewable distance.
        self.absolute_max_distance = (
            SIM.april_tag_physical_size * self.focal_length
        ) / SIM.minimum_pixels_per_tag

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
        return self.enabled

    # TODO: implement actual algorithm based on tag count, tag distance, and speed/rotation of robot
    # algorithm is used to tell the kalman filter how much to trust the pose estimation. lower is more confidant
    def get_deviation(self) -> Tuple[float, float, float]:
        return 0.7, 0.7, 0.7  # placeholder values

    def get_vision_measurement(
        self,
    ) -> tuple[Pose2d, float, tuple[float, float, float]]:
        # arr[6] is latency in ms, and is used to compute absolute timestamp of pose estimate
        timestamp = Timer.getFPGATimestamp() - (arr[6] / 1000.0)
        deviation = self.get_deviation()
        return pose, timestamp, deviation
