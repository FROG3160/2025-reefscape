import math
from ntcore import NetworkTableInstance
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware.pigeon2 import Pigeon2
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue
from phoenix6.configs.config_groups import Slot0Configs, Slot1Configs, FeedbackConfigs
from phoenix6.signals.spn_enums import GravityTypeValue, SensorDirectionValue


class FROGFeedbackConfig(FeedbackConfigs):
    def __init__(
        self, remote_sensor_id=0, sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR
    ):
        super().__init__()
        self.feedback_remote_sensor_id = remote_sensor_id
        self.feedback_sensor_source = sensor_source
        # TODO: Add this in if it makes sense
        # self.rotor_to_sensor_ratio =
        # self.sensor_to_mechanism_ratio =


class FROGTalonFXConfig(TalonFXConfiguration):
    """A subclass of TalonFXConfiguration that adds the ability to pass parameters to __init__
    during instantiation instead of creating an instance and then setting attributes."""

    def __init__(
        self,
        feedback_config=FROGFeedbackConfig(),
        slot0gains=Slot0Configs(),
        slot1gains=Slot1Configs(),
    ):
        super().__init__()
        self.feedback = feedback_config
        self.slot0 = slot0gains
        self.slot1 = slot1gains


class FROGTalonFX(TalonFX):
    """A subclass of TalonFX that allows us to pass in the config and apply it during
    instantiation.
    """

    def __init__(
        self,
        id: int = 0,
        motor_config: FROGTalonFXConfig = FROGTalonFXConfig(),
        parent_nt: str = "Undefined",
        motor_name: str = "",
    ):
        """Creates a TalonFX motor object with applied configuration

        Args:
            id (int, required): The CAN ID assigned to the motor.
            motor_config (FROGTalonFXConfig, required): The configuration to apply to the motor.
            table_name: NetworksTable to put the motor data on
            motor_name: NetworksTable name
        """
        super().__init__(device_id=id)
        self.config = motor_config
        self.configurator.apply(self.config)
        if motor_name == "":
            motor_name = f"TalonFX({id})"
        table = f"{parent_nt}/{motor_name}"
        self._motorVelocityPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/velocity")
            .publish()
        )
        self._motorPositionPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/position")
            .publish()
        )
        self._motorVoltagePub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/voltage")
            .publish()
        )

    def getMotorVoltage(self):
        return self.get_motor_voltage().value

    def logData(self):
        """Logs data to network tables for this motor"""
        self._motorVelocityPub.set(self.get_velocity().value)


class FROGPigeonGyro:
    "Gyro class that creates an instance of the Pigeon 2.0 Gyro"

    def __init__(self, can_id: int):
        self.gyro = Pigeon2(can_id)
        self.gyro.reset()

    def getAngleCCW(self):
        # returns gyro heading
        # and inverts it to change from bearing to
        # cartesian angles with CCW positive.
        # return -self.gyro.getYaw()
        return self.gyro.get_yaw().value

    def getRoll(self):
        return self.gyro.get_roll().value

    def getPitch(self):
        return self.gyro.get_pitch().value

    def getDegreesPerSecCCW(self):
        return self.gyro.get_angular_velocity_z_world().value

    def getRadiansPerSecCCW(self):
        return math.radians(self.getDegreesPerSecCCW())

    def getRotation2d(self):
        return self.gyro.getRotation2d()

    def setAngleAdjustment(self, angle):
        self.gyro.set_yaw(angle)


class FROGCANCoderConfig(CANcoderConfiguration):
    """Inheretis from CANcoderConfiguration and add the ability to pass in steer offset
    during instantiation."""

    def __init__(self, steer_offset=0):
        super().__init__()
        self.magnet_sensor.absolute_sensor_discontinuity_point = 0.5
        self.magnet_sensor.magnet_offset = steer_offset
        self.magnet_sensor.sensor_direction = (
            SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )


class FROGCanCoder(CANcoder):
    def __init__(self, id, config: FROGCANCoderConfig):
        super().__init__(id)
        self.configurator.apply(config)
        # self._motorPositionPub.set(self.get_position().value)
        # self._motorVoltagePub.set(self.get_motor_voltage().value)
