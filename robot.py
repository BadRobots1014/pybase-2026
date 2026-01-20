#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import wpilib
import wpilib.drive
import wpimath
import wpimath.controller
import wpimath.filter

from components.swerve.subsystem import SwerveSubsystem


class Robot(wpilib.TimedRobot):
    # MECH
    maxSpeed: float = 3.0  # 3 meters per second
    maxAngularSpeed: float = math.pi  # 1/2 rotation per second
    maxModuleAngularVelocity: float = math.pi
    maxModuleAngularAcceleration: float = math.tau

    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.PS4Controller(0)
        self.swerve = SwerveSubsystem(
            maxSpeed=self.maxAngularSpeed,
            maxModuleAngularVelocity=self.maxModuleAngularVelocity,
            maxModuleAngularAcceleration=self.maxModuleAngularAcceleration,
        )

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(True)

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.5)
            )
            * self.maxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.5)
            )
            * self.maxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.5)
            )
            * self.maxSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
