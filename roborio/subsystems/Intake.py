from enum import Enum
from commands2.subsystem import Subsystem
from commands2.button import Trigger
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

    class Status:
        OFF = 0
        ON = 1
        CORAL_DETECTED = 2
        CORAL_CLEARED = 3

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
        self.deploy_motor.get_torque_current().set_update_frequency(50)
        self.deploy_motor.get_position().set_update_frequency(50)
        self.deploy_motor.optimize_bus_utilization()
        # when the robot is powered on, the intake should be in the upright position
        self._position = self.Position.HOME
        self.deploy_position_tolerance = 0.01
        self.reset_position()
        self.homing_torque_limit = 8.0
        self.homing_voltage = 1.0
        self.deploy_control = MotionMagicVoltage(0, slot=0, enable_foc=False)
        self.detected_torque = 0
        self.intake_status = self.Status.OFF

    def reset_position(self):
        self.deploy_motor.set_position(self.Position.HOME)

    def get_deploy_torque(self):
        return self.deploy_motor.get_torque_current().value

    def get_intake_torque(self):
        return self.deploy_motor.get_torque_current().value

    def stop_homing(self):
        return abs(self.get_deploy_torque()) > self.homing_torque_limit

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

    def run_intake(self):
        self.upper_motor.set_control(VoltageOut(2.5, enable_foc=False))
        self.lower_motor.set_control(VoltageOut(1.5, enable_foc=False))

    def stop_intake(self):
        self.upper_motor.stopMotor()
        self.lower_motor.stopMotor()

    ### COMMANDS

    def move(self, position) -> Command:
        return self.runOnce(
            lambda: self.deploy_motor.set_control(
                self.deploy_control().with_position(position)
            )
        )

    def start(self) -> Command:
        return self.runOnce(self.run_intake)

    def stop(self) -> Command:
        return self.runOnce(self.stop_intake)

    def at_position(self, position):
        return (
            abs(self.deploy_motor.get_position() - position)
            < self.deploy_position_tolerance
        )

    def coral_cleared(self):
        return self.intake_status == self.Status.CORAL_CLEARED

    def get_deployed_trigger(self):
        return Trigger(lambda: self.at_position(self.Position.DEPLOYED))

    def get_home_trigger(self):
        return Trigger(lambda: self.at_position(self.Position.HOME))

    def get_coral_cleared_trigger(self):
        return Trigger(lambda: self.coral_cleared())

    def registerIntakeTriggers(self):
        self.get_deployed_trigger().onTrue(self.runOnce(self.run_intake()))

        self.get_coral_cleared_trigger().onTrue(
            self.runOnce(self.stop_intake).andThen(self.retract())
        )

    def periodic(self):
        previous_torque = self.detected_torque
        self.detected_torque = self.get_intake_torque()
        if self.detected_torque > previous_torque and self.detected_torque > 9:
            self.intake_status = self.Status.CORAL_DETECTED
        elif (
            self.detected_torque < previous_torque
            and self.detected_torque < 9
            and self.intake_status == self.Status.CORAL_DETECTED
        ):
            self.intake_status = self.Status.CORAL_CLEARED
