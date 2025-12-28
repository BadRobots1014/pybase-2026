from typing import Tuple

from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d

from hardware.abstract.camera import Camera


class AprilTag:
    def __init__(self, x: float, y: float, z: float, z_rotation: float):
        """
        Create an digital verison of an april tags.
        0,0 is bottom left of field. Refer to the Field2d image for reference.

        :param x: the x-offset in meters
        :param y: the y-offset in meters
        :param z: the z-offset in meters
        :param z_rotation: the rotation of the tag in degrees
        """
        self.x = x
        self.y = y
        self.z = z
        self.z_rotation = z_rotation


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

    def __init__(self, enabled: bool = True, name: str = "DummyCamera") -> None:
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
