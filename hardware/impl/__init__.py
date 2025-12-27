"""
Contains default implementations of components (motors, sensors, modules). The user should instantiate these when
creating their drive base.
"""

__all__ = [
    "NeutralMode",
    "TypicalDriveComponentParameters",
    "TypicalAzimuthComponentParameters",
    "Falcon500CoaxialAzimuthComponent",
    "Falcon500CoaxialDriveComponent",
    "NEOCoaxialAzimuthComponent",
    "NEOCoaxialDriveComponent",
    "NEOCoaxialAzimuthComponent",
    "AbsoluteCANCoder",
    "AbsoluteDutyCycleEncoder",
    "PigeonGyro",
    "Pigeon2Gyro",
    "CoaxialSwerveModule",
    "SparkMaxEncoderType",
    "SparkMaxAbsoluteEncoder",
    "DummyGyro",
]

from .motor import (
    Falcon500CoaxialAzimuthComponent,
    Falcon500CoaxialDriveComponent,
    NEOCoaxialAzimuthComponent,
    NEOCoaxialDriveComponent,
    NeutralMode,
    TypicalAzimuthComponentParameters,
    TypicalDriveComponentParameters,
)
from .sensor import (
    AbsoluteCANCoder,
    AbsoluteDutyCycleEncoder,
    DummyGyro,
    Pigeon2Gyro,
    PigeonGyro,
    SparkMaxAbsoluteEncoder,
    SparkMaxEncoderType,
)
from .system import CoaxialSwerveModule
