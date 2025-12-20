from abc import abstractmethod

from ntcore import NetworkTableInstance
from wpimath.geometry import Rotation2d
from wpiutil import Sendable, SendableBuilder

from . import SendableABCMeta


class Gyro(Sendable, metaclass=SendableABCMeta):
    def __init__(self, name: str = "Gyro"):
        """
        Initialize the Gyro sensor.

        :param name: Name for the NetworkTables entry (default: "Gyro")
        """
        super().__init__()

        self._nt_inst = NetworkTableInstance.getDefault()
        self._table = self._nt_inst.getTable(f"Sensors/{name}")

        # Create publishers for the sensor data
        self._heading_deg_pub = self._table.getDoubleTopic("Heading (deg)").publish()
        self._heading_rad_pub = self._table.getDoubleTopic("Heading (rad)").publish()

    @abstractmethod
    def zero_heading(self):
        """Set the gyro sensor's current heading as zero"""
        raise NotImplementedError

    def simulation_periodic(self, delta_position: float):
        """
        Update gyro angular position and velocity. Called periodically during simulation.
        :param delta_position: Change in angular position (CCW+) in radians since the last time this method was called
        """

    @property
    @abstractmethod
    def heading(self) -> Rotation2d:
        """CCW+ chassis yaw angle"""
        raise NotImplementedError

    def periodic(self):
        """
        Update NetworkTables with current sensor values.
        Should be called periodically (e.g., in subsystem periodic()).
        """
        current_heading = self.heading
        self._heading_deg_pub.set(current_heading.degrees())
        self._heading_rad_pub.set(current_heading.radians())

    def initSendable(self, builder: SendableBuilder):
        """Maintain compatibility with SmartDashboard/Sendable interface"""
        builder.setSmartDashboardType("Gyro")
        builder.addDoubleProperty(
            "Value", lambda: self.heading.degrees(), lambda _: None
        )
        builder.addDoubleProperty(
            "Heading (rad)", lambda: self.heading.radians(), lambda _: None
        )


class AbsoluteEncoder(Sendable, metaclass=SendableABCMeta):
    def __init__(self, name: str = "AbsoluteEncoder"):
        """
        Initialize the AbsoluteEncoder sensor.

        :param name: Name for the NetworkTables entry (default: "AbsoluteEncoder")
        """
        # MUST call Sendable.__init__() first when inheriting from Sendable
        super().__init__()

        self._nt_inst = NetworkTableInstance.getDefault()
        self._table = self._nt_inst.getTable(f"Sensors/{name}")

        # Create publishers for the sensor data
        self._position_deg_pub = self._table.getDoubleTopic(
            "Absolute Rotation (deg)"
        ).publish()
        self._position_rad_pub = self._table.getDoubleTopic(
            "Absolute Rotation (rad)"
        ).publish()

    @property
    @abstractmethod
    def absolute_position(self) -> Rotation2d:
        """Absolute rotation"""
        raise NotImplementedError

    def periodic(self):
        """
        Update NetworkTables with current sensor values.
        Should be called periodically (e.g., in subsystem periodic()).
        """
        current_position = self.absolute_position
        self._position_deg_pub.set(current_position.degrees())
        self._position_rad_pub.set(current_position.radians())

    def initSendable(self, builder: SendableBuilder):
        """Maintain compatibility with SmartDashboard/Sendable interface"""
        builder.setSmartDashboardType("AbsoluteEncoder")
        builder.addDoubleProperty(
            "Absolute Rotation (rad)",
            lambda: self.absolute_position.radians(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Absolute Rotation (deg)",
            lambda: self.absolute_position.degrees(),
            lambda _: None,
        )
