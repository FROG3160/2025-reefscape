from commands2.subsystem import Subsystem
from roborio.FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
import constants
from phoenix6.configs import Slot0Configs, Slot1Configs, MotorOutputConfigs
from phoenix6.signals import NeutralModeValue
from phoenix6.controls import Follower, VelocityVoltage, PositionVoltage, VoltageOut
from typing import Callable
from commands2 import Command

def __init__(self):
        self.motor = FROGTalonFX(
            id=constants.kLiftMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(90),
                slot0gains=Slot0Configs(),
                slot1gains=Slot1Configs(),
            ).with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE)
            ),
            parent_nt="Lift",
            motor_name="motor",
        )
        self._follower = FROGTalonFX(id=constants.kLiftFollowerID)
        # set the follower's control to ALWAYS follow the main motor
        self._follower.set_control(Follower(self.motor.device_id, False))
