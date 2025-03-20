import math
from commands2.subsystem import Subsystem
from commands2.button import Trigger
from FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
import constants

from phoenix6.configs import Slot0Configs, Slot1Configs, HardwareLimitSwitchConfigs

from phoenix6.signals.spn_enums import (
    ReverseLimitSourceValue,
    ReverseLimitTypeValue,
)
from phoenix6.controls import (
    VoltageOut,
)

from commands2 import Command, waitcommand
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
            # Inverting the motor so positive voltage winches the climber up
            .with_motor_output(motorOutputCCWPandBrake).with_hardware_limit_switch(
                HardwareLimitSwitchConfigs()
                .with_reverse_limit_enable(True)
                .with_reverse_limit_source(ReverseLimitSourceValue.LIMIT_SWITCH_PIN)
                .with_reverse_limit_type(ReverseLimitTypeValue.NORMALLY_OPEN)
            ),
            parent_nt="Climber",
            motor_name="motor",
        )

        self.motor_winch_request = VoltageOut(output=-2.5, enable_foc=False)
        self.motor_deploy_request = VoltageOut(output=2.0, enable_foc=False)

    def run_motor(self) -> Command:
        return self.runOnce(lambda: self.motor.set_control(self.motor_winch_request))

    def stop_motor(self) -> Command:
        return self.runOnce(lambda: self.motor.stopMotor())

    def deploy_climber(self, time) -> Command:
        return (
            self.runOnce(lambda: self.motor.set_control(self.motor_deploy_request))
            .andThen(waitcommand(time))
            .andThen(self.stop_motor())
        )
