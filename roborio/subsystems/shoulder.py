from commands2.subsystem import Subsystem
from ntcore import NetworkTableInstance
from FROGlib.ctre import (
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGFeedbackConfig,
    FROGCanCoder,
    FROGCANCoderConfig,
)
import constants
from phoenix6.configs import (
    Slot0Configs,
    Slot1Configs,
    MotorOutputConfigs,
    MagnetSensorConfigs,
    CANcoderConfiguration,
    MotionMagicConfigs,
)
from phoenix6.signals import NeutralModeValue, GravityTypeValue
from phoenix6.signals.spn_enums import (
    FeedbackSensorSourceValue,
    SensorDirectionValue,
    ControlModeValue,
)
from phoenix6.controls import Follower, VoltageOut, MotionMagicVoltage
from typing import Callable
from commands2 import Command
from commands2.button import Trigger
from configs.ctre import motorOutputCWPandBrake, motorOutputCCWPandBrake
from configs.scoring import ScoringConfigs


class Shoulder(Subsystem):
    # States
    # home, coralLvl1, coralLvl2, coralLvl3, algeLvl1, algeLvl2, algeLvl3, processor

    class Position:
        LOAD = -0.25
        READY = -0.15
        LEVEL1 = -0.15
        LEVEL2 = 0.1
        LEVEL3 = 0.1
        LEVEL4 = 0.125
        ALGAE2 = -0.1
        ALGAE3 = -0.1

    def __init__(self):
        self.shoulder_encoder = FROGCanCoder(
            constants.kShoulderSensorID,
            CANcoderConfiguration().with_magnet_sensor(
                MagnetSensorConfigs()
                .with_magnet_offset(constants.KShoulderOffset)
                .with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE)
            ),
        )
        self.motor = FROGTalonFX(
            id=constants.kShoulderMotorID,
            motor_config=FROGTalonFXConfig(
                feedback_config=FROGFeedbackConfig(
                    remote_sensor_id=self.shoulder_encoder.device_id,
                    sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
                ).with_rotor_to_sensor_ratio(90),
                slot0gains=Slot0Configs()
                .with_gravity_type(GravityTypeValue.ARM_COSINE)
                .with_k_p(20)
                .with_k_s(0.24)
                .with_k_v(8.0)
                .with_k_a(0.01)
                .with_k_g(0.21),
                slot1gains=Slot1Configs(),
            )
            .with_motor_output(motorOutputCWPandBrake)
            .with_motion_magic(
                MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(0.5)
                .with_motion_magic_acceleration(1)
            ),
            parent_nt="Shoulder",
            motor_name="motor",
        )
        self._follower = FROGTalonFX(id=constants.kShoulderFollowerID)
        # set the follower's control to ALWAYS follow the main motor
        self._follower.set_control(Follower(self.motor.device_id, False))

        self.position_tolerance = 0.005
        self.motion_magic_request = MotionMagicVoltage(
            0, slot=0, enable_foc=False
        ).with_

        self.scoring_config = ScoringConfigs(
            shoulder_start_pos=-0.25, shoulder_end_pos=-0.25
        )

        nt_table = f"Subsystems/{self.__class__.__name__}"
        self._position_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/position")
            .publish()
        )
        self._cl_output_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/cl_output")
            .publish()
        )
        self._cl_error_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/cl_error")
            .publish()
        )
        self._cl_goal_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/cl_goal")
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
            lambda: self.motor.set_control(VoltageOut(control() * 10, enable_foc=False))
        )

    def at_position(self, position) -> bool:
        # return Trigger(
        #     lambda: abs(self.motor.get_position().value - position)
        #     < self.position_tolerance
        # )
        return abs(self.motor.get_position().value - position) < self.position_tolerance

    def _move(self, position):
        self.motor.set_control(self.motion_magic_request.with_position(position))

    def move(self, position) -> Command:
        return self.runOnce(lambda: self._move(position))

    def move_to_scoring_start(self) -> Command:
        return self.runOnce(lambda: self._move(self.scoring_config.shoulder_start_pos))

    def move_to_scoring_end(self) -> Command:
        return self.runOnce(lambda: self._move(self.scoring_config.shoulder_end_pos))

    def set_goal(self, position) -> None:
        self.scoring_position = position

    def move_with_variable(self, callable: Callable[[], float]) -> Command:
        return self.runOnce(
            lambda: self.motor.set_control(
                self.motion_magic_request.with_position(callable())
            )  # VoltageOut(callable() * 10, enable_foc=False))
        )

    def periodic(self):
        self._position_pub.set(self.motor.get_position().value)
        self._cl_error_pub.set(self.motor.get_closed_loop_error().value)
        self._cl_output_pub.set(self.motor.get_closed_loop_output().value)
        self._cl_goal_pub.set(
            self.motor.get_position().value + self.motor.get_closed_loop_error().value
        )
        self._scoring_config_pub.set(self.scoring_config.get_name())
