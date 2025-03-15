from enum import Enum
from commands2.subsystem import Subsystem
from FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
import constants
from phoenix6.configs import (
    Slot0Configs,
    Slot1Configs,
    MotorOutputConfigs,
    MotionMagicConfigs,
)
from phoenix6.signals import NeutralModeValue, GravityTypeValue
from phoenix6.controls import (
    Follower,
    VelocityVoltage,
    PositionVoltage,
    VoltageOut,
    MotionMagicVoltage,
)
from typing import Callable
from commands2 import Command
from configs.ctre import (
    motorOutputCWPandBrake,
    motorOutputCCWPandCoast,
    motorOutputCWPandCoast,
)


class Intake(Subsystem):
    # States
    # upAndOff, downAndOn

    class Position:
        HOME = 0.3
        DEPLOYED = 0.0

    def __init__(self):
        self.lower_motor = FROGTalonFX(
            id=constants.kIntakeLowerMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(1),
                slot0gains=Slot0Configs(),
            ).with_motor_output(motorOutputCWPandCoast),
            parent_nt="Intake",
            motor_name="lower_motor",
        )
        self.upper_motor = FROGTalonFX(
            id=constants.kIntakeUpperMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(1),
                slot1gains=Slot1Configs(),
            ).with_motor_output(motorOutputCCWPandCoast),
            parent_nt="Intake",
            motor_name="upper_motor",
        )
        self.deploy_motor = FROGTalonFX(
            id=constants.kIntakeDeployMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(48),
                slot0gains=Slot0Configs()
                .with_gravity_type(GravityTypeValue.ARM_COSINE)
                .with_k_p(12)
                .with_k_s(0.2)
                .with_k_v(3.0)
                .with_k_a(0.1)
                .with_k_g(0.7),
            )
            .with_motor_output(motorOutputCWPandBrake)
            .with_motion_magic(
                MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(0.5)
                .with_motion_magic_acceleration(1)
            ),
            parent_nt="Intake",
            motor_name="deploy_motor",
        )
        # when the robot is powered on, the intake should be in the upright position
        self._position = self.Position.HOME
        self.reset_position()
        self.homing_torque_limit = 8.0
        self.deploy_control = MotionMagicVoltage(0, slot=0, enable_foc=False)

    def reset_position(self):
        self.deploy_motor.set_position(self.Position.HOME)

    def get_torque(self):
        return self.deploy_motor.get_torque_current().value

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

    def deploy(self) -> Command:
        return self.runOnce(
            lambda: self.deploy_motor.set_control(
                self.deploy_control().with_position(self.Position.DEPLOYED)
            )
        )

    def retract(self) -> Command:
        return self.runOnce(
            lambda: self.deploy_motor.set_control(
                self.deploy_control().with_position(self.Position.HOME)
            )
        )
