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
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts


class Arm(Subsystem):
    def __init__(self):
        self.motor = FROGTalonFX(
            id=constants.kArmMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(16),
                slot0gains=Slot0Configs(),
            )
            # Inverting the motor so positive voltage extends
            .with_motor_output(motorOutputCWPandBrake)
            # Adding software limit so we don't break the nut AGAIN!
            .with_software_limit_switch(
                SoftwareLimitSwitchConfigs()
                .with_forward_soft_limit_enable(True)
                .with_forward_soft_limit_threshold(7)
            ),
            parent_nt="Arm",
            motor_name="motor",
        )
        self.limitswitch = None
        self.homing_voltage = -0.25  # motor is inverted, negative values retract
        self.homing_torque_limit = 8.0

        self.sys_id_routine = SysIdRoutine(
            # a lower rampRate means quasistatic tests will take longer and log more data points
            # the stepvoltage is the voltage used by the dynamic tests.  Generally would want
            # a voltage strong enough to take the sytstem closer to its limit.  In this case, we
            # just don't have a mechanism that takes long to move.
            SysIdRoutine.Config(rampRate=0.5, stepVoltage=4.0),
            SysIdRoutine.Mechanism(
                self.sysid_move_motor,
                self.sysid_log_motor,
                self,
            ),
        )

    def sysid_move_motor(self, voltage: volts) -> None:
        self.motor.set_control(VoltageOut(output=voltage, enable_foc=False))

    def sysid_log_motor(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Record a frame for each module.  Since these share an encoder, we consider
        # the entire group to be one motor.
        with self.motor as m:
            sys_id_routine.motor("Lift_motor").voltage(
                m.get_motor_voltage().value
            ).position(m.get_position().value).velocity(m.get_velocity().value)

    def joystick_retract_command(self, retract_control: Callable[[], float]) -> Command:
        """Returns a command that takes a joystick control giving values between
        0 and 1.0 and calls it to apply motor voltage of -4 to 0 volts.

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
        0 and 1.0 and calls it to apply motor voltage of 0 to 4 volts.

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

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)
