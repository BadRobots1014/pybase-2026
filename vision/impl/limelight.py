from typing import Tuple

from ntcore import NetworkTableInstance
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d


def array_to_pose2d(arr: list[float]):
    return Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[5]))


class Limelight:

    def __init__(self, name: str = "limelight") -> None:
        self.name = name
        self.nt_inst = NetworkTableInstance.getDefault()
        self.nt_table = self.nt_inst.getTable(name)
        self.pose_sub = self.nt_table.getDoubleArrayTopic("botpose_wpiblue").subscribe([0]*7)
        self.tv_sub = self.nt_table.getBooleanTopic("tv").subscribe(False)
        self.tc_sub = self.nt_table.getIntegerTopic("tc").subscribe(0)

    def vision_measurement_valid(self) -> bool:
        return self.tv_sub.get()

    def get_deviation(self) -> Tuple[float, float, float]:
        return .7, .7, .7

    def get_vision_measurement(self) -> tuple[Pose2d, float, tuple[float, float, float]]:
        arr = self.pose_sub.get()
        pose = array_to_pose2d(arr)
        timestamp = Timer.getFPGATimestamp() - (arr[6] / 1000.0)
        deviation = self.get_deviation()
        return pose, timestamp, deviation




