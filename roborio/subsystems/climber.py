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

from commands2 import Command, WaitCommand
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
            .with_motor_output(motorOutputCWPandBrake).with_hardware_limit_switch(
                HardwareLimitSwitchConfigs()
                .with_reverse_limit_enable(True)
                .with_reverse_limit_source(ReverseLimitSourceValue.LIMIT_SWITCH_PIN)
                .with_reverse_limit_type(ReverseLimitTypeValue.NORMALLY_OPEN)
            ),
            parent_nt="Climber",
            motor_name="motor",
        )
        self.motor_winch_request = VoltageOut(output=2.5, enable_foc=False)
        self.motor_deploy_request = VoltageOut(output=2.0, enable_foc=False)

        nt_table = f"Subsystems/{self.__class__.__name__}"
        self._torque_current_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/torque_current")
            .publish()
        )
        self._position_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/position")
            .publish()
        )
        self._voltage_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/voltage")
            .publish()
        )
        self._limit_switch_pub = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/limit_switch")
            .publish()
        )

    def run_motor(self) -> Command:
        return self.runOnce(lambda: self.motor.set_control(self.motor_winch_request))

    def stop_motor(self) -> Command:
        return self.runOnce(lambda: self.motor.stopMotor())

    def deploy_climber(self, time) -> Command:
        return (
            self.runOnce(lambda: self.motor.set_control(self.motor_deploy_request))
            .andThen(WaitCommand(time))
            .andThen(self.stop_motor())
        )

    def periodic(self):
        self._position_pub.set(self.motor.get_position())
        self._torque_current_pub.set(self.motor.get_torque_current())
        self._voltage_pub.set(self.motor.get_motor_voltage())
        self._limit_switch_pub.set(self.motor.get_forward_limit())
