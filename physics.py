from pyfrc.physics import core

from .robot import Robot


class PhysicsEngine(core.PhysicsEngine):
    def __init__(self, physics_controller, robot: Robot):
        self.physics_controller = physics_controller

        self.swerve = robot.container.swerve

    def update_sim(self, now, tm_diff):
        #self.physics_controller.drive(speeds, tm_diff)
