from wpimath.geometry import Rotation2d, Translation2d

from ..abstract.motor import CoaxialAzimuthComponent, CoaxialDriveComponent
from ..abstract.system import SwerveModule


class CoaxialSwerveModule(SwerveModule):
    def __init__(
        self,
        drive: CoaxialDriveComponent,
        azimuth: CoaxialAzimuthComponent,
        placement: Translation2d,
        name: str,
    ):
        """
        Initialize the CoaxialSwerveModule.

        :param drive: Drive motor component
        :param azimuth: Azimuth motor component
        :param placement: Module placement on the robot
        :param name: Name for the NetworkTables entry (e.g., "FrontLeft", "BackRight")
        """
        super().__init__(name)
        self._drive = drive
        self._azimuth = azimuth
        self.placement = placement

    def desire_drive_velocity(self, velocity: float, open_loop: bool):
        self.last_commanded_drive_velocity = velocity
        if open_loop:
            self._drive.follow_velocity_open(velocity)
        else:
            self._drive.follow_velocity_closed(velocity)

    def set_drive_voltage(self, volts: float):
        self._drive.set_voltage(volts)

    def desire_azimuth_angle(self, angle: Rotation2d):
        self.last_commanded_azimuth_angle = angle
        self._azimuth.follow_angle(angle)

    def reset(self):
        self._drive.reset()
        self._azimuth.reset()

    def simulation_periodic(self, delta_time: float):
        self._drive.simulation_periodic(delta_time)
        self._azimuth.simulation_periodic(delta_time)

    @property
    def drive_velocity(self) -> float:
        return self._drive.velocity

    @property
    def drive_distance(self) -> float:
        return self._drive.distance

    @property
    def drive_voltage(self) -> float:
        return self._drive.voltage

    @property
    def azimuth_angle(self) -> Rotation2d:
        return self._azimuth.angle

    @property
    def azimuth_velocity(self) -> float:
        return self._azimuth.rotational_velocity
