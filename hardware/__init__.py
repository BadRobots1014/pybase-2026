"""Implementations for all hardware components on the robot outside of entire components"""

# Initialize the unit registry to provide consistent types
from pint import UnitRegistry

u = UnitRegistry()
__all__ = ["u"]
