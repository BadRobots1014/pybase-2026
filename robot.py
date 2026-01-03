from typing import Optional

import commands2
import wpilib
from ntcore import NetworkTableInstance
from pathplannerlib.auto import AutoBuilder

from container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    """
    Robot class is the entry-point of the code, handling initialization and scheduling of commands.
    """

    # Robot functions are run no matter the mode.
    # Universal functions should be handled here
    #
    def robotInit(self) -> None:
        # Init robot and scheduler
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.autonomous_command: Optional[commands2.Command] = None

        # Init auto path toggle
        self.auto_chooser = AutoBuilder.buildAutoChooser("MasterAuto")
        wpilib.SmartDashboard.putData("Auto Chooser", self.auto_chooser)

        # Init high-level network table values
        self.nt_inst = NetworkTableInstance.getDefault()
        self.match_time_pub = self.nt_inst.getFloatTopic(
            "Match Info/Match Time"
        ).publish()

    def robotPeriodic(self) -> None:
        # Update network table values
        match_time = wpilib.DriverStation.getMatchTime()
        self.match_time_pub.set(match_time)

    # Autonomous functions only run when robot is in "Auto" mode.
    #
    def autonomousInit(self) -> None:
        self.autonomous_command = self.auto_chooser.getSelected()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def autonomousExit(self) -> None:
        if self.autonomous_command:
            self.autonomous_command.cancel()

    # Teleop functions only run when robot is in "TeleOp" mode.
    #
    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

    # Test functions only run when robot is in "Test" mode.
    # This should be used for debug testing and added verbosity that is unnecessary in TeleOp and Auto
    #
    def testInit(self) -> None:
        # TODO: Add functionality to hot-swap PID for swerve modules
        pass

    def testPeriodic(self) -> None:
        pass
