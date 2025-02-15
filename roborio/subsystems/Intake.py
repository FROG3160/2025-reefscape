from commands2.subsystem import Subsystem
from roborio.FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
import constants
from phoenix6.configs import Slot0Configs, Slot1Configs, MotorOutputConfigs
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import Follower, VelocityVoltage, PositionVoltage, VoltageOut
from typing import Callable
from commands2 import Command

class Intake:
    def __init__(self):
    self.motor = FROGTalonFX(
            id=constants.kIntakeMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(90),
                slot0gains=Slot0Configs(),
            ).with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE)
            ),
            parent_nt="Intake",
            motor_name="motor",
        )
    self.motor2 = FROGTalonFX(
            id=constants.kRollersMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(90),
                slot1gains=Slot1Configs(), 
            ).with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE)
            ),
            parent_nt="Roller",
            motor_name="motor",
        )


    