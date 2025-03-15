#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#


# This is where we hold the basic settings/configuration of our robot.
# This should only hold numerical, string, or boolean constants, using
# the C++ naming convention.

from wpimath.geometry import Translation2d, Translation3d, Transform3d, Rotation3d
from enum import Enum
import math
import wpilib
from wpimath.units import feetToMeters, inchesToMeters, lbsToKilograms
from wpilib import DriverStation
from FROGlib.utils import GearStage

# CANCoder offsets
kFrontLeftOffset = 0.168701  # -0.246338
kFrontRightOffset = -0.260986  # 0.036377
kBackLeftOffset = 0.383057  # 0.478027
kBackRightOffset = -0.244141  # 0.207031
KShoulderOffset = 0.123047  # -0.05859375  # offset with arm parallel to the floor

# steer motor gains
kSteerP = 39.236  # 2.402346
kSteerI = 0  # 0.200195\
kSteerS = 0.15
kSteerV = 0.10583

# drive motor gains
kDriveFeedForward = 0.53
kDutyCycleDriveV = 0.00916
kDutyCycleDriveS = 0.01125

# These numbers were calculated from the SYSID characterization tool
# The SYSID tool calculates kV, kA, kP as values per meter
# these numbers come from determining that a meter of travel
# requires the motor to rotate 25+ times.  This was determined by
# factoring in the wheel diameter (4 inches) and the 8.14 gear reduction
# resulting in a value of 25.5024 rotations for every meter of travel
# The calculated kV was near 3, so 3/25.5024 = 0.117
kVoltageDriveV = 1.928
kVoltageDriveS = 0.14
kVoltageDriveP = 2.0
kVoltageDriveA = 0.027631

# Swerve Drive Motor/Encoder IDs
kFrontLeftDriveID = 11
kFrontLeftSteerID = 21
kFrontLeftSensorID = 31
kFrontRightDriveID = 12
kFrontRightSteerID = 22
kFrontRightSensorID = 32
kBackLeftDriveID = 13
kBackLeftSteerID = 23
kBackLeftSensorID = 33
kBackRightDriveID = 14
kBackRightSteerID = 24
kBackRightSensorID = 34

kGyroID = 39

kLiftMotorID = 41
kLiftFollowerID = 42
kShoulderMotorID = 43
kShoulderFollowerID = 44
kArmMotorID = 45
kGrabberMotorID = 46
kShoulderSensorID = 48
kGrabberSensorID = 49

kIntakeDeployMotorID = 51
kIntakeLowerMotorID = 52
kIntakeUpperMotorID = 53

kClimberMotorID = 61

kLiftGearReduction = 5.0  # 4:1 x 2.5:1 x 1:2
kShoulderGearReduction = 90

# Falcon 500 max rps
kFalconMaxRps = 106


# Swerve Drive Physical Attributes
kTrackWidthMeters = inchesToMeters(22.750)
kWheelBaseMeters = inchesToMeters(25.750)
kWheelDiameter = inchesToMeters(4)  # The tread has worn down
kSwerveDriveGearing = [
    GearStage(16, 50),
    GearStage(28, 16),
    GearStage(15, 45),
]  # Mk4c L3
# kDriveBaseRadius is the distance from the center of the robot
# to the farthest module. This is needed for the construction
# of Autobuilder in the drivetrain init method.
kDriveBaseRadius = math.sqrt(
    ((kTrackWidthMeters / 2) ** 2) + ((kWheelBaseMeters / 2) ** 2)
)


kRobotKilograms = lbsToKilograms(95)
kMOI = (kTrackWidthMeters**2 + kWheelBaseMeters**2) * kRobotKilograms * (1 / 12)

# Swerve Drive Performance
kMaxMetersPerSecond = feetToMeters(16)  # max fps for Mk4c L1=14.7, L2=17.7, L3=19.5
kMaxChassisRadiansPerSec = 2 * math.tau  # revolutions per sec * tau

# Swerve Drive Trajectory Constraints
kMaxTrajectorySpeed = kMaxMetersPerSecond
kMaxTrajectoryAccel = 2.8  # determined from testing pathplanner

kProfiledRotationMaxVelocity = 1.5 * math.tau  # 1.5 rotations per second
kProfiledRotationMaxAccel = 2 * math.tau  # 2 rotations per second per second

kProfiledRotationP = 0.4
kProfiledRotationI = 0.0
kProfiledRotationD = 0.0

# Xbox controller ports
kDriverControllerPort = 0
kOperatorControllerPort = 1

# Xbox controller constants
kDeadband = 0.15
kDebouncePeriod = 0.5
kTranslationSlew = 2
kRotSlew = 2

kFrameLength = inchesToMeters(20)
kFrameWidth = inchesToMeters(20)
kBumperDepth = inchesToMeters(3.25)
kTotalLength = kFrameLength + kBumperDepth * 2
kTotalWidth = kFrameWidth + kBumperDepth * 2

kTargetSizeThreshold = 14.0

kShoulderRatio = 90

# Camera names and configs
kCamera1Name = "Arducam_OV9281_Apriltag_1"
kCamera2Name = "Arducam_OV9281_Apriltag_2"


class Tags:
    class Blue:
        class Reef(Enum):
            DSRIGHT = 17
            DSCENTER = 18
            DSLEFT = 19
            BARGELEFT = 20
            BARGECENTER = 21
            BARGERIGHT = 22

        class Station(Enum):
            LEFT = 13
            RIGHT = 12

        class Barge(Enum):
            LEFT = 14
            RIGHT = 15

        class Processor(Enum):
            RIGHT = 16

    class Red:
        class Reef(Enum):
            DSRIGHT = 8
            DSCENTER = 7
            DSLEFT = 6
            BARGELEFT = 11
            BARGECENTER = 10
            BARGERIGHT = 9

        class Station(Enum):
            LEFT = 1
            RIGHT = 2

        class Barge(Enum):
            LEFT = 5
            RIGHT = 4

        class Processor(Enum):
            RIGHT = 3


robotToCamera = Transform3d(
    Translation3d(
        inchesToMeters(15.5),  # Forward from center
        inchesToMeters(0),  # Left from center
        inchesToMeters(3.6),  # Up from the floor
    ),
    Rotation3d(0, 0, 0),
)

robotToCamera2 = Transform3d(
    Translation3d(
        inchesToMeters(-15.5),  # Forward from center
        inchesToMeters(0),  # Left from center
        inchesToMeters(3.6),  # Up from the floor
    ),
    Rotation3d(0, 0, math.pi),
)


kReefTags = {
    DriverStation.Alliance.kBlue: {"Reef": Tags.Blue.Reef},
    DriverStation.Alliance.kRed: {"Reef": Tags.Red.Reef},
}
