from abc import abstractmethod, ABC

from ntcore import NetworkTableInstance
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpiutil import Sendable

from . import SendableABCMeta


class SwerveModule(Sendable, ABC, metaclass=SendableABCMeta):
    placement: Translation2d
    name: str
    last_commanded_drive_velocity: float = 0
    last_commanded_azimuth_angle = Rotation2d.fromDegrees(0)

    def __init__(self, name: str):
        """
        Initialize the SwerveModule with NetworkTables support.

        :param name: Name for the NetworkTables entry (e.g., "FrontLeft", "BackRight")
        """
        # MUST call Sendable.__init__() first when inheriting from Sendable
        super().__init__()

        # Set up NetworkTables
        self.name = name
        self._nt_inst = NetworkTableInstance.getDefault()
        self._table = self._nt_inst.getTable(f"SwerveModules/{name}")

        # Create publishers for module telemetry
        self._drive_velocity_pub = self._table.getDoubleTopic(
            "Drive Velocity (mps)"
        ).publish()
        self._drive_distance_pub = self._table.getDoubleTopic(
            "Drive Distance (m)"
        ).publish()
        self._drive_voltage_pub = self._table.getDoubleTopic("Drive Voltage").publish()
        self._azimuth_velocity_pub = self._table.getDoubleTopic(
            "Azimuth Velocity (radps)"
        ).publish()
        self._azimuth_position_rad_pub = self._table.getDoubleTopic(
            "Azimuth Position (rad)"
        ).publish()
        self._azimuth_position_deg_pub = self._table.getDoubleTopic(
            "Azimuth Position (deg)"
        ).publish()
        self._desired_drive_velocity_pub = self._table.getDoubleTopic(
            "Desired Drive Velocity (mps)"
        ).publish()
        self._desired_azimuth_rad_pub = self._table.getDoubleTopic(
            "Desired Azimuth Position (rad)"
        ).publish()
        self._desired_azimuth_deg_pub = self._table.getDoubleTopic(
            "Desired Azimuth Position (deg)"
        ).publish()

    def desire_state(
        self, state: SwerveModuleState, drive_open_loop: bool, rotate_in_place: bool
    ):
        """
        Command the module to follow a speed and angle
        :param state: SwerveModuleState representing the module's desired speed and angle
        :param drive_open_loop: Use open loop (True) or closed loop (False) velocity control to drive the wheel
        :param rotate_in_place: Whether the modules will rotate while not driving. Set False to prevent wheels from
        wearing down by spinning in place
        """
        state = optimize(state, self.azimuth_angle)
        # Prevent rotating the module if drive speed is less than 2 cm/s to prevent feedback-loop jitter
        angle = (
            state.angle
            if rotate_in_place or abs(state.speed) > 0.02
            else self.azimuth_angle
        )
        self.desire_drive_velocity(state.speed, drive_open_loop)
        self.desire_azimuth_angle(angle)

    @property
    def module_position(self) -> SwerveModulePosition:
        """The swerve module's driven distance (in metres) and facing angle"""
        return SwerveModulePosition(self.drive_distance, self.azimuth_angle)

    @property
    def module_state(self) -> SwerveModuleState:
        """The swerve module's current velocity (in metres/sec) and facing angle"""
        return SwerveModuleState(self.drive_velocity, self.azimuth_angle)

    def periodic(self):
        """
        Update NetworkTables with current module state.
        Should be called periodically (e.g., in subsystem periodic()).
        """
        self._drive_velocity_pub.set(self.drive_velocity)
        self._drive_distance_pub.set(self.drive_distance)
        self._drive_voltage_pub.set(self.drive_voltage)
        self._azimuth_velocity_pub.set(self.azimuth_velocity)
        self._azimuth_position_rad_pub.set(self.azimuth_angle.radians())
        self._azimuth_position_deg_pub.set(self.azimuth_angle.degrees())
        self._desired_drive_velocity_pub.set(self.last_commanded_drive_velocity)
        self._desired_azimuth_rad_pub.set(self.last_commanded_azimuth_angle.radians())
        self._desired_azimuth_deg_pub.set(self.last_commanded_azimuth_angle.degrees())

    @abstractmethod
    def desire_drive_velocity(self, velocity: float, open_loop: bool):
        """
        Drive the wheel
        :param velocity: Desired velocity in m/s
        :param open_loop: Use open loop (True) or closed loop (False) velocity control
        """
        raise NotImplementedError

    @abstractmethod
    def set_drive_voltage(self, volts: float):
        """
        Apply a voltage to the underlying drive motor. For use with SysID characterization
        :param volts: Voltage in volts
        """
        # TODO: This method creates a compatibility concern for differential swerve drives
        raise NotImplementedError

    @abstractmethod
    def desire_azimuth_angle(self, angle: Rotation2d):
        """
        Turn the wheel
        :param angle: Desired facing angle of the wheel
        """
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        """Reset sensor readings. Should be called during initialization."""
        raise NotImplementedError

    @abstractmethod
    def simulation_periodic(self, delta_time: float):
        """
        Update simulated motors and sensors. Called periodically during simulation.
        :param delta_time: Time in seconds since this method was last called
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def drive_velocity(self) -> float:
        """Drive wheel velocity in m/s"""
        raise NotImplementedError

    @property
    @abstractmethod
    def drive_distance(self) -> float:
        """Driven distance in metres"""
        raise NotImplementedError

    @property
    @abstractmethod
    def drive_voltage(self) -> float:
        """Applied output voltage of the drive motor"""
        raise NotImplementedError

    @property
    @abstractmethod
    def azimuth_angle(self) -> Rotation2d:
        """CCW+ wheel angle"""
        raise NotImplementedError

    @property
    @abstractmethod
    def azimuth_velocity(self) -> float:
        """CCW+ wheel angular velocity in rad/s"""
        raise NotImplementedError


def sign(num):
    return 1 if num > 0 else -1 if num < 0 else 0


def place_in_proper_0_to_360_scope(scope_reference: float, new_angle: float) -> float:
    # Place the new_angle in the range that is a multiple of [0, 360] (e.g., [360, 720]) which is closest
    # to the scope_reference
    lower_offset = scope_reference % 360
    lower_bound = scope_reference - lower_offset
    upper_bound = lower_bound + 360

    while new_angle < lower_bound:
        new_angle += 360
    while new_angle > upper_bound:
        new_angle -= 360

    if new_angle - scope_reference > 180:
        new_angle -= 360
    elif new_angle - scope_reference < -180:
        new_angle += 360

    return new_angle


def optimize(desired_state: SwerveModuleState, current_angle: Rotation2d):
    # There are two ways for a swerve module to reach its goal
    # 1) Rotate to its intended rotation and drive at its intended speed
    # 2) Rotate to the mirrored rotation (subtract 180) and drive at the opposite of its intended speed
    # Optimizing finds the option that requires the smallest rotation by the module

    target_angle = place_in_proper_0_to_360_scope(
        current_angle.degrees(), desired_state.angle.degrees()
    )
    target_speed = desired_state.speed
    delta = target_angle - current_angle.degrees()

    if abs(delta) > 90:
        target_speed *= -1
        target_angle -= 180 * sign(delta)

    return SwerveModuleState(target_speed, Rotation2d.fromDegrees(target_angle))
