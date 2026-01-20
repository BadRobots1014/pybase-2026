#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import navx
import wpilib
import wpimath.geometry
import wpimath.kinematics

from hardware.impl.swerve_module import SwerveModule


class SwerveSubsystem:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(
        self,
        maxSpeed: float,
        maxModuleAngularVelocity: float,
        maxModuleAngularAcceleration: float,
    ) -> None:
        # Constants
        self.maxSpeed = maxSpeed
        self.maxModuleAngularVelocity = maxModuleAngularVelocity
        self.maxModuleAngularAcceleration = maxModuleAngularAcceleration

        # Swerve Module Locations
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

        # Swerve Modules
        self.frontLeft = SwerveModule(
            21, 22, 23, maxModuleAngularVelocity, maxModuleAngularAcceleration
        )
        self.frontRight = SwerveModule(
            11, 12, 13, maxModuleAngularVelocity, maxModuleAngularAcceleration
        )
        self.backLeft = SwerveModule(
            31, 32, 33, maxModuleAngularVelocity, maxModuleAngularAcceleration
        )
        self.backRight = SwerveModule(
            41, 42, 43, maxModuleAngularVelocity, maxModuleAngularAcceleration
        )

        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        # Push data
        wpilib.SmartDashboard.putData("gyro", self.gyro)

        # Reset to compensate for drift
        self.gyro.reset()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """

        wpilib.SmartDashboard.putString("xspeed", str(xSpeed))
        wpilib.SmartDashboard.putString("yspeed", str(ySpeed))
        wpilib.SmartDashboard.putString("rotation", str(rot))

        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                    )
                    if fieldRelative
                    else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
                ),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, self.maxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
