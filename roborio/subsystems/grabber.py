import math

from commands2.subsystem import Subsystem
from roborio.FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
import constants
from phoenix6.configs import Slot0Configs, Slot1Configs, MotorOutputConfigs
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import Follower, VelocityVoltage, PositionVoltage, VoltageOut
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

class grabber:

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
        self._follower = FROGTalonFX(id=constants.kGrabberFollowerID)
        # set the follower's control to ALWAYS follow the main motor
        self._follower.set_control(Follower(self.motor.device_id, False))

