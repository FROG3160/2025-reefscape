from commands2.subsystem import Subsystem
from FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
import constants
from phoenix6.configs import (
    Slot0Configs,
    Slot1Configs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
)
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.controls import Follower, VelocityVoltage, PositionVoltage, VoltageOut
from typing import Callable
from commands2 import Command
from configs.ctre import motorOutputCWPandBrake


class Arm(Subsystem):
    def __init__(self):
        self.motor = FROGTalonFX(
            id=constants.kArmMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(16),
                slot0gains=Slot0Configs(),
            )
            .with_motor_output(motorOutputCWPandBrake)
            .with_software_limit_switch(
                SoftwareLimitSwitchConfigs()
                .with_forward_soft_limit_enable(True)
                .with_forward_soft_limit_threshold(8)
            ),
            parent_nt="Arm",
            motor_name="motor",
        )
        self.limitswitch = None
        self.homing_voltage = -0.25  # motor is inverted, negative values retract
        self.homing_torque_limit = 8.0

    def joystick_retract_command(self, retract_control: Callable[[], float]) -> Command:
        """Returns a command that takes a joystick control giving values between
        -1.0 and 1.0 and calls it to apply motor voltage of -10 to 10 volts.

        Args:
            control (Callable[[], float]): A control from the joystick that provides
            a value from 0 to 1.0

        Returns:
            Command: The command that will cause the motor to move from joystick control.
        """
        return self.run(
            lambda: self.motor.set_control(
                VoltageOut(retract_control() * -4, enable_foc=False)
            )
        )

    def joystick_extend_command(self, extend_control: Callable[[], float]) -> Command:
        """Returns a command that takes a joystick control giving values between
        -1.0 and 1.0 and calls it to apply motor voltage of -10 to 10 volts.

        Args:
            control (Callable[[], float]): A control from the joystick that provides
            a value from 0 to 1.0

        Returns:
            Command: The command that will cause the motor to move from joystick control.
        """
        return self.run(
            lambda: self.motor.set_control(
                VoltageOut(extend_control() * 4, enable_foc=False)
            )
        )

    def reset_position(self):
        self.motor.set_position(0)

    def get_torque(self):
        return self.motor.get_torque_current().value

    def stop_homing(self):
        return abs(self.get_torque()) > self.homing_torque_limit

    def set_home(self) -> Command:
        return (
            self.startEnd(
                # START
                lambda: self.motor.set_control(
                    VoltageOut(self.homing_voltage, enable_foc=False)
                ),
                # END
                lambda: self.motor.set_control(VoltageOut(0, enable_foc=False)),
            )
            .until(self.stop_homing)
            .andThen(self.runOnce(self.reset_position))
        )
