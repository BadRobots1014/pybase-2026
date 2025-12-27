"""
A modular swerve drive library extendable to any swerve configuration.
"""

__all__ = ["u", "SwerveDrive", "TrajectoryFollowerParameters"]

# Initialize the unit registry before importing anything that relies on it
from pint import UnitRegistry

u = UnitRegistry()

from .subsystem import SwerveDrive, TrajectoryFollowerParameters
