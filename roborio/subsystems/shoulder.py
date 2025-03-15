from commands2.subsystem import Subsystem
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
from phoenix6.signals.spn_enums import FeedbackSensorSourceValue, SensorDirectionValue
from phoenix6.controls import Follower, VoltageOut, MotionMagicVoltage
from typing import Callable
from commands2 import Command
from configs.ctre import motorOutputCWPandBrake, motorOutputCCWPandBrake


class Shoulder(Subsystem):
    # States
    # home, coralLvl1, coralLvl2, coralLvl3, algeLvl1, algeLvl2, algeLvl3, processor

    class Position:
        HOME = 0
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
                .with_k_p(24)
                .with_k_s(0.24)
                .with_k_v(6)
                .with_k_a(0.01)
                .with_k_g(0.2),
                slot1gains=Slot1Configs(),
            )
            .with_motor_output(motorOutputCWPandBrake)
            .with_motion_magic(
                MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(1)
                .with_motion_magic_acceleration(2)
            ),
            parent_nt="Shoulder",
            motor_name="motor",
        )
        self._follower = FROGTalonFX(id=constants.kShoulderFollowerID)
        # set the follower's control to ALWAYS follow the main motor
        self._follower.set_control(Follower(self.motor.device_id, False))

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
            lambda: self.motor.set_control(VoltageOut(control() * 10, enable_foc=False))
        )

    def move(self, position) -> Command:
        return self.runOnce(
            lambda: self.motor.set_control(self.control().with_position(position))
        )

    def at_position(self, position):
        return abs(self.motor.get_position() - position) < self.position_tolerance
