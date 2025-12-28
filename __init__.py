# Initialize the unit registry to provide consistent types
from pint import UnitRegistry

u = UnitRegistry()
__all__ = ["u"]
