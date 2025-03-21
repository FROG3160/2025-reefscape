from commands2.subsystem import Subsystem
from ntcore import NetworkTableInstance
from FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
import constants
from configs.ctre import motorOutputCCWPandBrake
from phoenix6.configs import (
    Slot0Configs,
    Slot1Configs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
    MotionMagicConfigs,
)
from phoenix6.signals import NeutralModeValue, GravityTypeValue
from phoenix6.controls import Follower, VoltageOut, MotionMagicVoltage
from typing import Callable
from commands2 import Command
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts


class Lift(Subsystem):
    # States
    # home,  coralLvl1, coralLvl2 coralLvl3, algeLvl1, algeLvl2 algelLvl3,

    class Position:
        HOME = 0
        LEVEL1 = 3
        LEVEL2 = 5
        LEVEL3 = 7
        LEVEL4 = 9
        ALGAE2 = 7
        ALGAE3 = 9

    def __init__(self):

        self.motor = FROGTalonFX(
            id=constants.kLiftMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(
                    constants.kLiftGearReduction
                ),
                slot0gains=Slot0Configs()
                .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
                .with_k_p(4)
                .with_k_s(0.13)  # voltage to barely move elevator down
                .with_k_v(0.5)
                .with_k_g(0.3),  # voltage to overcome gravity
                slot1gains=Slot1Configs(),
            )
            .with_motor_output(motorOutputCCWPandBrake)
            .with_motion_magic(
                MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(12)
                .with_motion_magic_acceleration(24)
                .with_motion_magic_jerk(48)
            ),
            # .with_software_limit_switch(
            #     SoftwareLimitSwitchConfigs()
            #     .with_reverse_soft_limit_enable(True)
            #     .with_reverse_soft_limit_threshold(0.1)
            #     .with_forward_soft_limit_enable(True)
            #     .with_forward_soft_limit_threshold(13.5)
            # ),
            parent_nt="Lift",
            motor_name="motor",
        )
        self._follower = FROGTalonFX(id=constants.kLiftFollowerID)
        # set the follower's control to ALWAYS follow the main motor
        self._follower.set_control(Follower(self.motor.device_id, False))

        self.position_tolerance = 0.1
        self.control = MotionMagicVoltage(0, slot=0, enable_foc=False)
        nt_table = f"Subsystems/{self.__class__.__name__}"
        self._position_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/position")
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
            lambda: self.motor.set_control(VoltageOut(control() * 10, enable_foc=False))
        )

    def lift_is_at_home(self):
        return abs(self.motor.get_torque_current().value) > 8

    def reset_lift_to_home(self):
        self.motor.set_position(0)
        self.motor.config.with_software_limit_switch(
            SoftwareLimitSwitchConfigs()
            .with_reverse_soft_limit_enable(True)
            .with_reverse_soft_limit_threshold(0.0)
            .with_forward_soft_limit_enable(True)
            .with_forward_soft_limit_threshold(13.5)
        )
        self.motor.configurator.apply(self.motor.config)

    def home(self) -> Command:
        return (
            self.startEnd(
                lambda: self.motor.set_control(VoltageOut(-0.25, enable_foc=False)),
                lambda: self.motor.stopMotor(),
            )
            .until(self.lift_is_at_home)
            .andThen(self.runOnce(self.reset_lift_to_home))
        )

    def move(self, position) -> Command:
        return self.runOnce(
            lambda: self.motor.set_control(self.control.with_position(position))
        )

    def at_position(self, position):
        return abs(self.motor.get_position().value - position) < self.position_tolerance

    def periodic(self):
        self._position_pub.set(self.motor.get_position().value)
