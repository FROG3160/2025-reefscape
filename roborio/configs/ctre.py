import math
import constants
from wpimath.geometry import Translation2d, Translation3d, Transform3d, Rotation3d
from FROGlib import ctre  # import FROGTalonFXConfig, FROGFeedbackConfig
from FROGlib import swerve  # import SwerveModuleConfig
from phoenix6.configs.config_groups import (
    Slot0Configs,
    Slot1Configs,
    MotorOutputConfigs,
    MotionMagicConfigs,
)
from phoenix6.signals.spn_enums import FeedbackSensorSourceValue
from pathplannerlib.config import PIDConstants
from wpimath.units import inchesToMeters
from phoenix6.signals.spn_enums import NeutralModeValue, InvertedValue

steerGains = Slot0Configs().with_k_p(constants.kSteerP).with_k_i(constants.kSteerI)
driveDutyCycleGains = (
    Slot0Configs()
    .with_k_s(constants.kDutyCycleDriveS)
    .with_k_v(constants.kDutyCycleDriveV)
)
driveVoltageGains = (
    Slot1Configs()
    .with_k_s(constants.kVoltageDriveS)
    .with_k_v(constants.kVoltageDriveV)
    .with_k_p(constants.kVoltageDriveP)
    .with_k_a(constants.kVoltageDriveA)
)


autobuilderHolonomicTranslationPID = PIDConstants(1.0, 0.0, 0.0)
autobuilderHolonomicRotationPID = PIDConstants(0.4, 0.0, 0.0)

motorOutputCCWPandBrake = (
    MotorOutputConfigs()
    .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    .with_neutral_mode(NeutralModeValue.BRAKE)
)
motorOutputCCWPandCoast = (
    MotorOutputConfigs()
    .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    .with_neutral_mode(NeutralModeValue.COAST)
)
motorOutputCWPandBrake = (
    MotorOutputConfigs()
    .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
    .with_neutral_mode(NeutralModeValue.BRAKE)
)
motorOutputCWPandCoast = (
    MotorOutputConfigs()
    .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
    .with_neutral_mode(NeutralModeValue.COAST)
)

swerveModuleFrontLeft = swerve.SwerveModuleConfig(
    # swerveModuleFrontLeft =
    **{
        "name": "FrontLeft",
        "location": Translation2d(
            constants.kWheelBaseMeters / 2, constants.kTrackWidthMeters / 2
        ),
        "drive_gearing": constants.kSwerveDriveGearing,
        "wheel_diameter": constants.kWheelDiameter,
        "drive_motor_id": constants.kFrontLeftDriveID,
        "drive_motor_config": ctre.FROGTalonFXConfig(
            slot0gains=driveDutyCycleGains, slot1gains=driveVoltageGains
        ),
        "steer_motor_id": constants.kFrontLeftSteerID,
        "steer_motor_config": ctre.FROGTalonFXConfig(
            feedback_config=(
                ctre.FROGFeedbackConfig(
                    remote_sensor_id=constants.kFrontLeftSensorID,
                    sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
                )
            ),
            slot0gains=steerGains,
        ),
        "cancoder_id": constants.kFrontLeftSensorID,
        "cancoder_config": ctre.FROGCANCoderConfig(constants.kFrontLeftOffset),
    }
)
swerveModuleFrontRight = swerve.SwerveModuleConfig(
    **{
        "name": "FrontRight",
        "location": Translation2d(
            constants.kWheelBaseMeters / 2, -constants.kTrackWidthMeters / 2
        ),
        "drive_gearing": constants.kSwerveDriveGearing,
        "wheel_diameter": constants.kWheelDiameter,
        "drive_motor_id": constants.kFrontRightDriveID,
        "drive_motor_config": ctre.FROGTalonFXConfig(
            slot0gains=driveDutyCycleGains, slot1gains=driveVoltageGains
        ),
        "steer_motor_id": constants.kFrontRightSteerID,
        "steer_motor_config": ctre.FROGTalonFXConfig(
            feedback_config=(
                ctre.FROGFeedbackConfig(
                    remote_sensor_id=constants.kFrontRightSensorID,
                    sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
                )
            ),
            slot0gains=steerGains,
        ),
        "cancoder_id": constants.kFrontRightSensorID,
        "cancoder_config": ctre.FROGCANCoderConfig(constants.kFrontRightOffset),
    }
)
swerveModuleBackLeft = swerve.SwerveModuleConfig(
    **{
        "name": "BackLeft",
        "location": Translation2d(
            -constants.kWheelBaseMeters / 2, constants.kTrackWidthMeters / 2
        ),
        "drive_gearing": constants.kSwerveDriveGearing,
        "wheel_diameter": constants.kWheelDiameter,
        "drive_motor_id": constants.kBackLeftDriveID,
        "drive_motor_config": ctre.FROGTalonFXConfig(
            slot0gains=driveDutyCycleGains, slot1gains=driveVoltageGains
        ),
        "steer_motor_id": constants.kBackLeftSteerID,
        "steer_motor_config": ctre.FROGTalonFXConfig(
            feedback_config=(
                ctre.FROGFeedbackConfig(
                    remote_sensor_id=constants.kBackLeftSensorID,
                    sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
                )
            ),
            slot0gains=steerGains,
        ),
        "cancoder_id": constants.kBackLeftSensorID,
        "cancoder_config": ctre.FROGCANCoderConfig(constants.kBackLeftOffset),
    }
)
swerveModuleBackRight = swerve.SwerveModuleConfig(
    **{
        "name": "BackRight",
        "location": Translation2d(
            -constants.kWheelBaseMeters / 2, -constants.kTrackWidthMeters / 2
        ),
        "drive_gearing": constants.kSwerveDriveGearing,
        "wheel_diameter": constants.kWheelDiameter,
        "drive_motor_id": constants.kBackRightDriveID,
        "drive_motor_config": ctre.FROGTalonFXConfig(
            slot0gains=driveDutyCycleGains, slot1gains=driveVoltageGains
        ),
        "steer_motor_id": constants.kBackRightSteerID,
        "steer_motor_config": ctre.FROGTalonFXConfig(
            feedback_config=(
                ctre.FROGFeedbackConfig(
                    remote_sensor_id=constants.kBackRightSensorID,
                    sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
                )
            ),
            slot0gains=steerGains,
        ),
        "cancoder_id": constants.kBackRightSensorID,
        "cancoder_config": ctre.FROGCANCoderConfig(constants.kBackRightOffset),
    }
)
