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
)
from phoenix6.signals import NeutralModeValue
from phoenix6.signals.spn_enums import BrushedMotorWiringValue
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


class Grabber(Subsystem):
    # STATES
    # Empty, HasCoral, HasAlgae

    def __init__(self):
        self.motor = TalonFXS(constants.kGrabberMotorID)
        self.motor.configurator.apply(
            TalonFXSConfiguration()
            .with_commutation(
                CommutationConfigs().with_brushed_motor_wiring(
                    BrushedMotorWiringValue.LEADS_A_AND_B
                )
            )
            .with_motor_output(
                MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE)
            )
        )

        self.range = CANrange(constants.kGrabberSensorID)
        self.motor_intake = VoltageOut(output=5.0, enable_foc=False)
        self.motor_stop = StaticBrake()
        self.motor_voltage = 5

    def joystick_move_command(self, control: Callable[[], float]) -> Command:
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

    def detecting_coral(self) -> bool:
        return self.get_range() < 0.05

    def detecting_algae(self) -> bool:
        return 0.08 > self.get_range() > 0.05

    def intake_coral(self) -> Command:
        return self.startEnd(
            # start running the motor
            lambda: self.motor.set_control(self.motor_intake),
            # stop the motor
            lambda: self.motor.set_control(VoltageOut(0, enable_foc=False)),
        ).until(self.detecting_coral)

    def intake_algae(self) -> Command:
        return self.startEnd(
            # start running the motor
            lambda: self.motor.set_control(self.motor_intake),
            # stop the motor
            lambda: self.motor.set_control(VoltageOut(0, enable_foc=False)),
        ).until(self.detecting_algae)
