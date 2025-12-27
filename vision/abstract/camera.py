from abc import abstractmethod, ABC
from typing import Tuple

from wpimath.geometry import Pose2d

class Camera(ABC):

    @abstractmethod
    def vision_measurement_valid(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def get_vision_measurement(self) -> Tuple[Pose2d, int, (float, float, float)]:
        raise NotImplementedError


