from commands2.subsystem import Subsystem
from commands2.button import Trigger
from FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
import constants
from phoenix6.configs import (
    Slot0Configs,
    Slot1Configs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
    MotionMagicConfigs,
)
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.controls import (
    VoltageOut,
    MotionMagicVoltage,
)
from typing import Callable
from commands2 import Command
from configs.ctre import motorOutputCWPandBrake
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts


class Arm(Subsystem):
    class Position:
        RETRACTED = 0
        CORAL_PICKUP = 0.6
        CORAL_L2_PLACE = 4
        CORAL_L4_PLACE = 5
        ALGAE_PICKUP = 3

    def __init__(self):
        self.motor = FROGTalonFX(
            id=constants.kArmMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(16),
                slot0gains=Slot0Configs()
                .with_k_a(0.1)
                .with_k_p(8)
                .with_k_s(0.19)
                .with_k_v(1.9),
            )
            # Inverting the motor so positive voltage extends
            .with_motor_output(motorOutputCWPandBrake).with_motion_magic(
                MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(6)
                .with_motion_magic_acceleration(18)
            ),
            # Adding software limit so we don't break the nut AGAIN!
            # .with_software_limit_switch(
            #     SoftwareLimitSwitchConfigs()
            #     .with_forward_soft_limit_enable(True)
            #     .with_forward_soft_limit_threshold(7)
            # ),
            parent_nt="Arm",
            motor_name="motor",
        )
        self.motor.get_position().set_update_frequency(50)
        self.motor.get_torque_current().set_update_frequency(50)
        self.motor.optimize_bus_utilization()
        self.limitswitch = None
        self.homing_voltage = -0.25  # motor is inverted, negative values retract
        self.homing_torque_limit = 8.0

        self.position_tolerance = 0.1
        self.control = MotionMagicVoltage(0, slot=0, enable_foc=False)

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
            lambda: self.motor.set_control(VoltageOut(control() * 2, enable_foc=False))
        )

    def reset_position(self):
        self.motor.set_position(0)
        self.motor.config.with_software_limit_switch(
            SoftwareLimitSwitchConfigs()
            .with_reverse_soft_limit_enable(True)
            .with_reverse_soft_limit_threshold(0.0)
            .with_forward_soft_limit_enable(True)
            .with_forward_soft_limit_threshold(7)
        )
        self.motor.configurator.apply(self.motor.config)

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
                lambda: self.motor.stopMotor(),
            )
            .until(self.stop_homing)
            .andThen(self.runOnce(self.reset_position))
        )

    def move(self, position) -> Command:
        return self.runOnce(
            lambda: self.motor.set_control(self.control.with_position(position))
        )

    def at_position(self, position) -> bool:
        return abs(self.motor.get_position().value - position) < self.position_tolerance

    def at_home(self) -> bool:
        return abs(self.motor.get_position().value - 0) < self.position_tolerance
