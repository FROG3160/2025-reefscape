import math
from enum import Enum
from commands2.subsystem import Subsystem
from commands2.button import Trigger
from FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
import constants
from phoenix6.hardware import TalonFXS
from phoenix6.configs import (
    Slot0Configs,
    Slot1Configs,
    MotorOutputConfigs,
    TalonFXSConfiguration,
    CommutationConfigs,
    CANrangeConfiguration,
    ProximityParamsConfigs,
)
from phoenix6.signals import NeutralModeValue
from phoenix6.signals.spn_enums import BrushedMotorWiringValue, MotorArrangementValue
from phoenix6.controls import (
    Follower,
    VelocityVoltage,
    PositionVoltage,
    VoltageOut,
    StaticBrake,
)
from phoenix6.hardware import CANrange
from typing import Callable
from commands2 import Command
from configs.ctre import motorOutputCCWPandBrake
from ntcore import NetworkTableInstance


class Climber(Subsystem):
    # STATES
    # Empty, HasCoral, HasAlgae

    def __init__(self):
        self.motor = FROGTalonFX(
            id=constants.kClimberMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(25),
                slot0gains=Slot0Configs(),
                slot1gains=Slot1Configs(),
            )
            # Inverting the motor so positive voltage extends
            .with_motor_output(motorOutputCCWPandBrake),
            # Adding software limit so we don't break the nut AGAIN!
            # .with_software_limit_switch(
            #     SoftwareLimitSwitchConfigs()
            #     .with_forward_soft_limit_enable(True)
            #     .with_forward_soft_limit_threshold(7)
            # ),
            parent_nt="Climber",
            motor_name="motor",
        )

        self.motor_intake = VoltageOut(output=1.0, enable_foc=False)
        self.motor.set_position(0)
