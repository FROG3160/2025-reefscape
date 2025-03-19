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
    HardwareLimitSwitchConfigs
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
from configs.ctre import motorOutputCWPandBrake
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
            # Inverting the motor so positive voltage winches the climber up
            .with_motor_output(motorOutputCWPandBrake)
            .with_hardware_limit_switch(HardwareLimitSwitchConfigs().with_forward_limit_enable().with_
                                        )
            parent_nt="Climber",
            motor_name="motor",
        )

        self.motor_request = VoltageOut(output=2.0, enable_foc=False)

    
    def run_motor(self) -> Command:
        return self.runOnce(
            lambda: self.motor.set_control(self.motor_request)
        )
