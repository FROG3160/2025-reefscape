import math
from navx import AHRS
from phoenix6.hardware.pigeon2 import Pigeon2
from wpimath.geometry import Rotation2d
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.signals.spn_enums import AbsoluteSensorRangeValue, SensorDirectionValue
from phoenix6.hardware.cancoder import CANcoder


class FROGNavXGyro:
    """Gyro class that creates and instance of the NavX gyro and uses it to get AHRS data,
    converting it for use by the swerve drivetrain.  All swerve calculations use radians
    with CCW rotation being positive, and field-oriented driving uses moving forward and
    moving left as positive."""

    def __init__(self):
        # TODO Make sure if we need this.
        self.gyro = AHRS.create_spi()
        self.starting_angle = 0.0
        self.offset = 0
        # self.field_heading = 360-242
        # self.gyro.reset()
        self.gyro.setAngleAdjustment(self.offset)

    def getAngleCCW(self):
        # returns gyro heading
        # and inverts it to change from bearing to
        # cartesian angles with CCW positive.
        # return -self.gyro.getYaw()
        return -self.gyro.getAngle()

    def getRoll(self):
        return self.gyro.getRoll()

    def getPitch(self):
        return self.gyro.getPitch()

    def setOffset(self, offset):
        self.offset = offset

    def getDegreesPerSecCCW(self):
        return -self.gyro.getRate()

    def getRadiansPerSecCCW(self):
        return math.radians(self.getDegreesPerSecCCW())

    def getRotation2d(self):
        return Rotation2d.fromDegrees(self.getAngleCCW())

    def resetGyro(self, on_red: bool):
        # sets yaw reading to 0
        if on_red:
            self.setAngleAdjustment(180)
        else:
            self.setAngleAdjustment(0)
        self.gyro.reset()

    def getAngleConstrained(self):
        angle = self.getAngle()
        return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

    def setAngleAdjustment(self, angle):
        self.gyro.setAngleAdjustment(angle)

    def getAngleAdjustment(self):
        return self.gyro.getAngleAdjustment()


class FROGPigeonGyro:
    "Gyro class that creates an instance of the Pigeon 2.0 Gyro"

    def __init__(self):
        # TODO Make sure if we need this.
        self.gyro = Pigeon2(5, "")
        self.starting_angle = 0.0  # Not sure if needed
        self.offset = 0  # Not sure if needed
        # self.field_heading = 360-242
        # self.gyro.reset()
        self.gyro.reset()

    def getAngleCCW(self):
        # returns gyro heading
        # and inverts it to change from bearing to
        # cartesian angles with CCW positive.
        # return -self.gyro.getYaw()
        return self.gyro.get_yaw()

    def getRoll(self):
        return self.gyro.get_roll()

    def getPitch(self):
        return self.gyro.get_pitch()

    def setOffset(self, offset):
        self.offset = offset

    def getDegreesPerSecCCW(self):
        return self.gyro.get_angular_velocity_z_world()

    def getRadiansPerSecCCW(self):
        return math.radians(self.getDegreesPerSecCCW())

    def getRotation2d(self):
        return self.gyro.getRotation2d()

    def resetGyro(self, on_red: bool):
        # sets yaw reading to 0
        if on_red:
            self.setAngleAdjustment(180)
        else:
            self.setAngleAdjustment(0)
        self.gyro.reset()

    def setAngleAdjustment(self, angle):
        self.gyro.set_yaw(angle)


# TODO https://github.com/FROG3160/2025-reefscape/issues/3
class FROGCANCoderConfig(CANcoderConfiguration):
    """Inheretis from CANcoderConfiguration and add the ability to pass in steer offset
    during instantiation."""

    def __init__(self, steer_offset):
        super().__init__()
        self.magnet_sensor.absolute_sensor_range = (
            AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        )
        self.magnet_sensor.magnet_offset = steer_offset
        self.magnet_sensor.sensor_direction = (
            SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )


class FROGCanCoder(CANcoder):
    def __init__(self, id, config: FROGCANCoderConfig):
        super().__init__(id)
        self.configurator.apply(config)
