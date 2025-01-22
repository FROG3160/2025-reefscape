#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#


# This is where we hold the basic settings/configuration of our robot.
# This should only hold numerical, string, or boolean constants, using
# the C++ naming convention.

from enum import Enum, member
import math
import wpilib
from wpimath.geometry import Pose3d, Rotation3d
from wpimath.units import feetToMeters, inchesToMeters

# CANCoder offsets
kFrontLeftOffset = -0.246338
kFrontRightOffset = 0.036377
kBackLeftOffset = 0.478027
kBackRightOffset = 0.207031

# steer motor gains
kSteerP = 2.402346
kSteerI = 0.200195

# drive motor gains
kDriveFeedForward = 0.53
kDutyCycleDriveV = 0.00916
kDutyCycleDriveS = 0.01125

# These numbers were calculated from the SYSID characterization tool
# The SYSID tool calculates kV, kA, kP as values per meter
# these numbers come from determining that a meter of travel
# requires the motor to rotate 25+ times.  This was determined by
# factooring in the wheel diameter (4 inches) and the 8.14 gear reduction
# resulting in a value of 25.5024 rotations for every meter of travel
# The calculated kV was near 3, so 3/25.5024 = 0.117
kVoltageDriveV = 0.113
kVoltageDriveS = 0.14
kVoltageDriveP = 0.02
kVoltageDriveA = 0.01

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


# Falcon 500 max rps
kFalconMaxRps = 106


# Swerve Drive Physical Attributes
kTrackWidthMeters = inchesToMeters(20)
kWheelBaseMeters = inchesToMeters(20)
kWheelDiameter = inchesToMeters(3.91)  # The tread has worn down
kSwerveDriveGearing = [
    (14.0 / 50.0),
    (28.0 / 16.0),
    (15.0 / 45.0),
]  # Mk4 L3 from V.O.L.T.S.
# kDriveBaseRadius is the distance from the center of the robot
# to the farthest module. This is needed for the construction
# of Autobuilder in the drivetrain init method.
kDriveBaseRadius = math.sqrt(
    ((kTrackWidthMeters / 2) ** 2) + ((kWheelBaseMeters / 2) ** 2)
)

# Swerve Drive Performance
kMaxMetersPerSecond = feetToMeters(16)  # max fps for L1=13.5, L2=16.3, L3=18
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


# class AprilTagPlacement:

#     class Red:
#         SPEAKER = 4
#         AMP = 5
#         STAGE_CENTER = 13
#         STAGE_AMP = 12
#         STAGE_SOURCE = 11

#     class Blue:
#         SPEAKER = 7
#         AMP = 6
#         STAGE_CENTER = 14
#         STAGE_AMP = 15
#         STAGE_SOURCE = 16


kFrameLength = inchesToMeters(20)
kFrameWidth = inchesToMeters(20)
kBumperDepth = inchesToMeters(3.25)
kTotalLength = kFrameLength + kBumperDepth * 2
kTotalWidth = kFrameWidth + kBumperDepth * 2

kTargetSizeThreshold = 14.0
