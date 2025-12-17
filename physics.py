from typing import TYPE_CHECKING

from pyfrc.physics import core

from swervepy.impl import CoaxialSwerveModule, DummyGyro
from swervepy.impl.motor import DummyCoaxialAzimuthComponent, DummyCoaxialDriveComponent

# explicit importing causes errors, only needed for type checking
if TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine(core.PhysicsEngine):
    def __init__(self, physics_controller, robot: "Robot"):
        self.physics_controller = physics_controller

        # Switch all swerve motors to fake components for sim
        self.swerve = robot.container.swerve
        self.swerve._modules = tuple(
            CoaxialSwerveModule(
                drive=DummyCoaxialDriveComponent(),
                azimuth=DummyCoaxialAzimuthComponent(),
                placement=module.placement,
            )
            for module in self.swerve._modules
        )

        self.swerve._gyro = DummyGyro()

        return

    def update_sim(self, now, tm_diff):
        return
