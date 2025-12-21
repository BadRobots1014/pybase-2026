import logging
import math

import wpilib
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import (
    GoalEndState,
    IdealStartingState,
    PathConstraints,
    PathPlannerPath,
)
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

import components
from constants import ELEC, MECH, OP, PHYS, SW
from swervepy import SwerveDrive
from swervepy.impl import CoaxialSwerveModule

logger = logging.getLogger("your.robot")


class RobotContainer:
    """
    RobotContainer contains all the subsystems and components needed for the robot to work, allowing them to be referenced
    by other systems.
    """

    def __init__(self):
        gyro = components.gyro_component_class(**components.gyro_param_values)

        # The Azimuth component included the absolute encoder because it needs
        # to be able to reset to absolute position.
        #
        self.lf_enc = components.absolute_encoder_class(ELEC.LF_encoder_DIO)
        self.lb_enc = components.absolute_encoder_class(ELEC.LB_encoder_DIO)
        self.rb_enc = components.absolute_encoder_class(ELEC.RB_encoder_DIO)
        self.rf_enc = components.absolute_encoder_class(ELEC.RF_encoder_DIO)

        modules = (
            # Left Front module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.LF_drive_CAN_ID,
                    parameters=components.drive_params,
                ),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.LF_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(0),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.lf_enc,
                ),
                placement=Translation2d(*components.module_locations["LF"]),
                name="LeftFront",
            ),
            # Right Front module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.RF_drive_CAN_ID,
                    parameters=components.drive_params,
                ),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.RF_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(0),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.rf_enc,
                ),
                placement=Translation2d(*components.module_locations["RF"]),
                name="RightFront",
            ),
            # Left Back module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.LB_drive_CAN_ID,
                    parameters=components.drive_params,
                ),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.LB_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(0),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.lb_enc,
                ),
                placement=Translation2d(*components.module_locations["LB"]),
                name="LeftBack",
            ),
            # Right Back module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.RB_drive_CAN_ID,
                    parameters=components.drive_params,
                ),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.RB_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(0),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.rb_enc,
                ),
                placement=Translation2d(*components.module_locations["RB"]),
                name="RightBack",
            ),
        )

        self.stick = wpilib.Joystick(0)

        self.speed_limit_ratio = 1.0
        if OP.speed_limit:
            if OP.speed_limit > OP.max_speed:
                wpilib.reportWarning(
                    "Speed limit is greater than max_speed and won't be used"
                )
            else:
                self.speed_limit_ratio = OP.speed_limit / OP.max_speed

        self.angular_velocity_limit_ratio = 1.0
        if OP.angular_velocity_limit:
            if OP.angular_velocity_limit > OP.max_angular_velocity:
                wpilib.reportWarning(
                    "Angular velocity limit is greater than max_angular_velocity and won't be used"
                )
            else:
                self.angular_velocity_limit_ratio = (
                    OP.angular_velocity_limit / OP.max_angular_velocity
                )

        # Define a swerve drive subsystem by passing in a list of SwerveModules
        # and some options
        self.swerve = SwerveDrive(
            modules,
            gyro,
            OP.max_speed,
            OP.max_angular_velocity,
            SW.auto_follower_params,
        )

        # Cache Auto path to reduce calculations in match
        bezier_points = PathPlannerPath.waypointsFromPoses(
            [
                Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)),
            ]
        )
        self.autoPeriodPath = PathPlannerPath(
            waypoints=bezier_points,
            constraints=PathConstraints(3.0, 3.0, 2 * math.pi, 4 * math.pi),
            ideal_starting_state=IdealStartingState(0.0, Rotation2d.fromDegrees(0)),
            goal_end_state=GoalEndState(
                0.0, Rotation2d.fromDegrees(-90)
            ),  # Zero velocity and facing 90 degrees clockwise
        )

        # Set the swerve subsystem's default command to teleoperate using
        # the controller joysticks
        self.swerve.setDefaultCommand(
            self.swerve.teleop_command(
                translation=self.get_translation_input,
                strafe=self.get_strafe_input,
                rotation=self.get_rotation_input,
                field_relative=SW.field_relative,
                drive_open_loop=SW.drive_open_loop,
            )
        )

    @staticmethod
    def deadband(value, band):
        return value if abs(value) > band else 0

    def process_joystick_input(
        self, val, deadband=0.1, exponent=1, limit_ratio=1.0, invert=False
    ):
        """
        Given a raw joystick reading, return the processed value after adjusting
        for real-world UX considerations:
          * apply a deadband to ignore jitter around zero
          * apply an exponent for greater low-velocity control
        """
        deadbanded_input = self.deadband(val, deadband)
        input_sign = +1 if val > 0 else -1  # this works for val=0 also
        invert_sign = -1 if invert else +1
        # abs required for fractional exponents
        scaled_input = limit_ratio * abs(deadbanded_input) ** exponent
        return invert_sign * input_sign * scaled_input

    def get_translation_input(self, invert=True):
        raw_stick_val = self.stick.getRawAxis(OP.translation_joystick_axis)
        return self.process_joystick_input(
            raw_stick_val, invert=invert, limit_ratio=self.speed_limit_ratio
        )

    def get_strafe_input(self, invert=True):
        raw_stick_val = self.stick.getRawAxis(OP.strafe_joystick_axis)
        return self.process_joystick_input(
            raw_stick_val, invert=invert, limit_ratio=self.speed_limit_ratio
        )

    def get_rotation_input(self, invert=True):
        raw_stick_val = self.stick.getRawAxis(OP.rotation_joystick_axis)
        return self.process_joystick_input(
            raw_stick_val, invert=invert, limit_ratio=self.angular_velocity_limit_ratio
        )
