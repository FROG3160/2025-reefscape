import math

from commands2.subsystem import Subsystem
from roborio.FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
import constants
from phoenix6.configs import Slot0Configs, Slot1Configs, MotorOutputConfigs
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import Follower, VelocityVoltage, PositionVoltage, VoltageOut
from phoenix6.hardware import CANrange
from typing import Callable
from commands2 import Command

# Objects needed for Auto setup (AutoBuilder)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    ReplanningConfig,
    PIDConstants,
)
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpilib import DriverStation, Field2d
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Transform3d, Rotation3d


class Grabber(Subsystem):
    # STATES
    # Empty, HasCoral, HasAlgae

    def __init__(self):
        self.motor = FROGTalonFX(
            id=constants.kGrabberMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(90),
                slot0gains=Slot0Configs(),
            ).with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE)
            ),
            parent_nt="Grabber",
            motor_name="motor",
        )
        self.range = CANrange(constants.kGrabberSensorID)

    def Joystick_move_command(self, control: Callable[[], float]) -> Command:
        """Returns a command that takes a joystick control giving values between
        -1.0 and 1.0 and calls it to apply motor voltage of -10 to 10 volts.

        Args:
            control (Callable[[], float]): A control from the joystick that provides
            a value from -1.0 to 1.0

        Returns:
            Command: The command that will cause the motor to move from joystick control.
        """
        return self.run(
            lambda: self.motor.set_control(
                VoltageOut(control() * 2.5, enable_foc=False)
            )
        )

    def get_range(self):
        return self.range.get_distance().value
