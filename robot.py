from typing import Optional

import commands2

from container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    """
    Robot class is the entry-point of the code, handling initialization and scheduling of commands.
    """

    # Runs no matter the state
    def robotInit(self):
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.autonomous_command: Optional[commands2.Command] = None

    # Schedule Auto command when switched to "Auto"
    def autonomousInit(self) -> None:
        self.autonomous_command = self.container.get_autonomous_command()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    # Runs every cycle while the robot is in "Auto"
    def autonomousPeriodic(self) -> None:
        return

    # When no command is scheduled, Teleop command is scheduled automatically
    def teleopInit(self) -> None:
        if self.autonomous_command:
            self.autonomous_command.cancel()

    # Runs every cycle while the robot is in "Teleop"
    def teleopPeriodic(self) -> None:
        return
