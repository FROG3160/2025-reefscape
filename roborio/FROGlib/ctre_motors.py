import math
from ntcore import NetworkTableInstance
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue
from phoenix6.configs.config_groups import Slot0Configs, Slot1Configs, FeedbackConfigs
from phoenix6.signals.spn_enums import GravityTypeValue


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
        # self._motorPositionPub.set(self.get_position().value)
        # self._motorVoltagePub.set(self.get_motor_voltage().value)


class GearTrain:
    def __init__(self, gear_stages: list):
        """
        Constructs a GearStages object that stores data about the gear stages.
        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
        """
        self.gear_ratio = math.prod(gear_stages)

    def input_rotations(self, output_rotations):
        """Calculates motor rotations given the rotation at the other end of the gears."""
        return output_rotations / self.gear_ratio

    def output_rotations(self, input_rotations):
        """Calculates final gear rotation given the motor's rotation"""
        return input_rotations * self.gear_ratio


class DriveTrain:
    def __init__(self, gear_stages: list, wheel_diameter: float):
        """Constructs a DriveTrain object that stores data about the gear stages and wheel.

        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
            diameter (float): Diameter of the attached wheel in meters
        """
        self.gearing = GearTrain(gear_stages)
        self.circumference = math.pi * wheel_diameter

    def speed_to_input_rps(self, speed: float) -> float:
        """Converts the system linear speed to a motor velocity
        Args:
            speed (float): desired linear speed in meters per second
        Returns:
            float: motor rotations per second
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = self.gearing.input_rotations(wheel_rotations_sec)
        return motor_rotations_sec

    def input_rps_to_speed(self, rotations_per_sec: float) -> float:
        """Converts motor velocity to the system linear speed

        Args:
            rotations_per_sec (float): motor rotational speed in rotations per second
        Returns:
            float: system linear speed in meters per second
        """
        wheel_rotations_sec = self.gearing.output_rotations(rotations_per_sec)
        return wheel_rotations_sec * self.circumference

    def rotations_to_distance(self, rotations: float) -> float:
        """Takes  and returns distance

        Args:
            rotations (float): number of motor rotations
        Returns:
            float: distance in meters
        """
        wheel_rotations = self.gearing.output_rotations(rotations)
        return wheel_rotations * self.circumference
