from typing import Optional

import commands2
import wpilib
from pathplannerlib.auto import AutoBuilder

from container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    """
    Robot class is the entry-point of the code, handling initialization and scheduling of commands.
    """

    # Runs no matter the state
    def robotInit(self):
        # Init robot and scheduler
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.autonomous_command: Optional[commands2.Command] = None
        # Init dashboard values
        self.auto_chooser = AutoBuilder.buildAutoChooser("MasterAuto")
        wpilib.SmartDashboard.putData("Auto Chooser", self.auto_chooser)

    # Runs every cycle no matter the state
    def robotPeriodic(self):
        match_time = wpilib.DriverStation.getMatchTime()
        wpilib.SmartDashboard.putNumber("Match Info/Match Time", match_time)

    # Schedule Auto command when switched to "Auto"
    def autonomousInit(self) -> None:
        self.autonomous_command = self.auto_chooser.getSelected()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    # Runs every cycle while the robot is in "Auto"
    def autonomousPeriodic(self) -> None:
        return

    # Cleanup auto after exit
    def autonomousExit(self):
        if self.autonomous_command:
            self.autonomous_command.cancel()

    # When no command is scheduled, Teleop command is scheduled automatically
    def teleopInit(self) -> None:
        return

    # Runs every cycle while the robot is in "Teleop"
    def teleopPeriodic(self) -> None:
        return
