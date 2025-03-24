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
from commands2 import Command, WaitCommand
from configs.ctre import motorOutputCWPandBrake
from ntcore import NetworkTableInstance
from configs.scoring import ScoringConfigs


class Grabber(Subsystem):
    # STATES
    # Empty, HasCoral, HasAlgae

    def __init__(self):
        self.motor = TalonFXS(constants.kGrabberMotorID)
        self.motor.configurator.apply(
            TalonFXSConfiguration()
            .with_commutation(
                CommutationConfigs()
                .with_brushed_motor_wiring(BrushedMotorWiringValue.LEADS_A_AND_B)
                .with_motor_arrangement(MotorArrangementValue.BRUSHED_DC)
            )
            .with_motor_output(motorOutputCWPandBrake)
        )

        self.range = CANrange(constants.kGrabberSensorID)
        self.range.configurator.apply(
            CANrangeConfiguration().with_proximity_params(
                ProximityParamsConfigs()
                .with_proximity_threshold(0.065)
                .with_min_signal_strength_for_valid_measurement(4000)
            )
        )
        self.scoring_config = ScoringConfigs(grabber_v=0)

        self.motor_intake = VoltageOut(output=8.0, enable_foc=False)
        self.motor_eject = VoltageOut(output=8.0, enable_foc=False)
        self.motor_voltage = 5

        nt_table = f"Subsystems/{self.__class__.__name__}"
        self._range_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/range")
            .publish()
        )
        self._coral_detected_pub = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/coral_detected")
            .publish()
        )
        self._algae_detected_pub = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/algae_detected")
            .publish()
        )
        self._scoring_config_pub = (
            NetworkTableInstance.getDefault()
            .getStringTopic(f"{nt_table}/scoring_config")
            .publish()
        )

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
        return self.range.get_is_detected().value == True

    def not_detecting_coral(self) -> bool:
        return not self.detecting_coral()

    def detecting_algae(self) -> bool:
        return 0.15 > self.get_range() > 0.055

    def not_detecting_algae(self) -> bool:
        return not self.detecting_algae()

    def intake_coral(self) -> Command:
        return self.startEnd(
            # start running the motor
            lambda: self.motor.set_control(self.motor_intake),
            # stop the motor
            lambda: self.motor.stopMotor(),
        ).until(self.detecting_coral)

    def eject_coral(self) -> Command:
        return self.startEnd(
            # start running the motor
            lambda: self.motor.set_control(self.motor_eject),
            # stop the motor
            lambda: self.motor.stopMotor(),
        ).until(self.not_detecting_coral)

    def eject_coral_L1(self) -> Command:
        return (
            self.runOnce(
                # start running the motor
                lambda: self.motor.set_control(VoltageOut(-12, enable_foc=False))
            )
            .until(self.not_detecting_coral)
            .andThen(WaitCommand(2))
            .andThen(
                # stop the motor
                lambda: self.motor.stopMotor(),
            )
        )

    def intake_algae(self) -> Command:
        return self.runOnce(
            # start running the motor
            lambda: self.motor.set_control(self.motor_intake),
        )

    def eject_algae(self) -> Command:
        return self.runOnce(
            # start running the motor
            lambda: self.motor.set_control(-self.motor_eject),
        ).until(self.not_detecting_algae)

    def run_motor(self, voltage) -> Command:
        return self.runOnce(
            lambda: self.motor.set_control(VoltageOut(voltage, enable_foc=False))
        )

    def run_motor_with_variable(self, callable: Callable[[], float]) -> Command:
        return self.runOnce(
            lambda: self.motor.set_control(VoltageOut(callable(), enable_foc=False))
        )

    def stop_motor(self) -> Command:
        return self.runOnce(self.motor.stopMotor())

    def periodic(self):
        self._range_pub.set(self.get_range())
        self._coral_detected_pub.set(self.detecting_coral())
        self._algae_detected_pub.set(self.detecting_algae())
        self._scoring_config_pub.set(self.scoring_config.get_name())
