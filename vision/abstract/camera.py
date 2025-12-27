from abc import abstractmethod, ABC
from typing import Tuple

import ntcore
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d

class Camera(ABC):

    def __init__(self, enabled: bool = True, name: str = "camera") -> None:
        self.enabled = enabled
        self.name = name

        # setup network tables
        self.nt_inst = NetworkTableInstance.getDefault()
        self.nt_table = self.nt_inst.getTable(name)

        # setup enabled listener
        self.enabled_topic = self.nt_table.getBooleanTopic("enabled")
        self.enabled_listener = self.nt_inst.addListener(self.enabled_topic, ntcore.EventFlags.kValueAll,
                                                         self._enabled_changed)

    def _enabled_changed(self, event: ntcore.Event):
        self.enabled = event.data.value.getBoolean()

    # will be called to check if vision has computed a valid pose and is enabled
    @abstractmethod
    def vision_measurement_valid(self) -> bool:
        raise NotImplementedError

    # returns
    # robot pose,
    # absolute timestamp of computation,
    # (x, y, z) standard deviation (confidence) of pose (lower is better)
    # all three will be passed into the kalman filter that keeps track of robot pose
    @abstractmethod
    def get_vision_measurement(self) -> Tuple[Pose2d, float, (float, float, float)]:
        raise NotImplementedError


