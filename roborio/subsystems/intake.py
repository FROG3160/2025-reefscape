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
    SoftwareLimitSwitchConfigs,
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
from commands2 import Command, WaitCommand, WaitUntilCommand
from configs.ctre import (
    motorOutputCWPandBrake,
    motorOutputCCWPandCoast,
    motorOutputCWPandCoast,
)


class Intake(Subsystem):
    # States
    # upAndOff, downAndOn

    class Position:
        HOME = 0.33
        DEPLOYED = 0.0

    class State:
        EMPTY = 1
        CORAL_DETECTED = 2
        CORAL_CLEARED = 3
        CORAL_LOADED = 4

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
                .with_k_g(0.4),
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
        self.deploy_position_tolerance = 0.02
        self.homing_torque_limit = 8.0
        self.homing_voltage = 1.0
        self.deploy_control = MotionMagicVoltage(0, slot=0, enable_foc=False)
        self.detected_torque = 0
        self.intake_state = self.State.EMPTY
        self.watch_intake_torque = False

    def _reset_position(self):
        self.deploy_motor.set_position(self.Position.HOME)
        self.deploy_motor.config.with_software_limit_switch(
            SoftwareLimitSwitchConfigs()
            .with_reverse_soft_limit_enable(True)
            .with_reverse_soft_limit_threshold(0.0)
            .with_forward_soft_limit_enable(True)
            .with_forward_soft_limit_threshold(0.33)
        )
        self.deploy_motor.configurator.apply(self.deploy_motor.config)

    def _at_position(self, position) -> bool:
        return (
            abs(self.deploy_motor.get_position().value - position)
            < self.deploy_position_tolerance
        )

    # def _coral_detected(self):
    #     return self.intake_state == self.State.CORAL_DETECTED

    # def _coral_loaded(self):
    #     return self.intake_state == self.State.CORAL_LOADED

    def _get_deploy_torque(self):
        return self.deploy_motor.get_torque_current().value

    def _get_intake_torque(self):
        return self.upper_motor.get_torque_current().value

    def _stop_homing(self):
        return abs(self._get_deploy_torque()) > self.homing_torque_limit

    def _run_intake_motors(self):
        self.upper_motor.set_control(VoltageOut(2.5, enable_foc=False))
        self.lower_motor.set_control(VoltageOut(2.0, enable_foc=False))

    def _stop_intake_motors(self):
        self.upper_motor.stopMotor()
        self.lower_motor.stopMotor()

    def _set_intake_state(self, state):
        self.intake_state = state

    def _enable_watch_torque(self):
        self.watch_intake_torque = True

    def _disable_watch_torque(self):
        self.watch_intake_torque = False

    ### COMMANDS

    def set_home(self) -> Command:
        return (
            self.startEnd(
                # START
                lambda: self.deploy_motor.set_control(
                    VoltageOut(self.homing_voltage, enable_foc=False)
                ),
                # END
                lambda: self.deploy_motor.stopMotor(),
            )
            .until(self._stop_homing)
            .andThen(self.runOnce(self._reset_position))
        )

    def move_intake(self, position) -> Command:
        return self.runOnce(
            lambda: self.deploy_motor.set_control(
                self.deploy_control.with_position(position)
            )
        )

    def start_intake(self) -> Command:
        return (
            self.runOnce(self._run_intake_motors)
            .andThen(WaitCommand(0.1))
            .andThen(self.runOnce(self._enable_watch_torque))
        )

    def stop_intake(self) -> Command:
        return self.runOnce(self._stop_intake_motors).andthen(
            self.runOnce(self._disable_watch_torque)
        )

    def set_intake_loaded(self) -> Command:
        return self.runOnce(self._set_intake_state(self.State.CORAL_LOADED))

    def set_intake_empty(self) -> Command:
        return self.runOnce(self._set_intake_state(self.State.CORAL_CLEARED))

    # TRIGGERS

    def has_state(self, state):
        return Trigger(lambda: self.intake_state == state)

    def at_position(self, position):
        return Trigger(lambda: self._at_position(position))

    """at_position above COULD take the place of the two more specific
        triggers below"""

    def intake_deployed(self):
        return Trigger(lambda: self._at_position(self.Position.DEPLOYED))

    def intake_retracted(self):
        return Trigger(lambda: self._at_position(self.Position.HOME))

    # def coral_detected(self):
    #     # return Trigger(self._coral_detected)
    #     return self.has_state(self.State.CORAL_DETECTED)

    # def coral_loaded(self):
    #     # return Trigger(self._coral_loaded)
    #     return self.has_state(self.State.CORAL_LOADED)

    def periodic(self):
        if self.watch_intake_torque:
            if (
                self.intake_state != self.State.CORAL_DETECTED
                and self._get_intake_torque() > 10
            ):
                self._set_intake_state(self.State.CORAL_DETECTED)
            elif (
                self.intake_state == self.State.CORAL_DETECTED
                and self._get_intake_torque() < 8
            ):
                self._set_intake_state(self.State.CORAL_LOADED)
