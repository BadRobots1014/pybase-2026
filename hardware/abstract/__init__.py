"""
Contains interfaces for components used in a swerve drive base. These are sensors, motors, and the swerve module itself.
Implementations can be found in the impl module, or the user may define their own.
"""

__all__ = [
    "Camera",
    "SendableABCMeta",
    "CoaxialDriveComponent",
    "CoaxialAzimuthComponent",
    "Gyro",
    "AbsoluteEncoder",
    "SwerveModule",
]

from abc import ABCMeta

from wpiutil import Sendable


class SendableABCMeta(ABCMeta, type(Sendable)):
    pass


from .camera import Camera
from .motor import CoaxialAzimuthComponent, CoaxialDriveComponent
from .sensor import AbsoluteEncoder, Gyro
from .system import SwerveModule
