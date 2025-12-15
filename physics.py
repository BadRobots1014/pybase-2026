import hal.simulation
from pyfrc.physics import drivetrains


class PhysicsEngine:
    def __init__(self, physics_controller):
        self.physics_controller = physics_controller
        self.drivetrain = drivetrains.TwoMotorDrivetrain(
            deadzone=drivetrains.linear_deadzone(0.2)
        )

        self.l_motor = hal.simulation.PWMSim(1)
        self.r_motor = hal.simulation.PWMSim(2)

    def update_sim(self, now, tm_diff):
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        speeds = self.drivetrain.calculate(l_motor, r_motor)
        self.physics_controller.drive(speeds, tm_diff)
