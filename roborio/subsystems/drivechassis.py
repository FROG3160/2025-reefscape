# High level objects that control our drivetrain
import math

# from robotpy_apriltag import AprilTagField, loadAprilTagLayoutField
from FROGlib.swerve import SwerveChassis, SwerveModule
from FROGlib.sensors import FROGGyro
from constants import (
    AprilTagPlacement,
    kMaxChassisRadiansPerSec,
    kMaxMetersPerSecond,
    kDriveBaseRadius,
    kSteerP,
    kSteerI,
)
import configs

# Objects needed for Auto setup (AutoBuilder)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    ReplanningConfig,
    PIDConstants,
)
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpilib import DriverStation, Field2d
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Transform3d, Rotation3d
# from subsystems.vision import PositioningSubsystem
# from subsystems.elevation import ElevationSubsystem
from wpilib import SmartDashboard
from commands2 import Subsystem, Command
from FROGlib.utils import RobotRelativeTarget, remap
import constants
from wpimath.units import degreesToRadians
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
# from subsystems.leds import LEDSubsystem


class DriveTrain(SwerveChassis):
    def __init__(
        self,
        parent_nt: str = "Subsystems",
    ):
        super().__init__(
            swerve_module_configs=(
                configs.swerveModuleFrontLeft,
                configs.swerveModuleFrontRight,
                configs.swerveModuleBackLeft,
                configs.swerveModuleBackRight,
            ),
            # modules=(
            #     SwerveModule(**configs.s,werveModuleFrontLeft),
            #     SwerveModule(**configs.swerveModuleFrontRight),
            #     SwerveModule(**configs.swerveModuleBackLeft),
            #     SwerveModule(**configs.swerveModuleBackRight),
            # ),
            gyro=FROGGyro(),
            max_speed=kMaxMetersPerSecond,
            max_rotation_speed=kMaxChassisRadiansPerSec,
            parent_nt=parent_nt,
        )
        self.resetController = True
        # TODO https://github.com/FROG3160/2025-reefscape/issues/5


        self.estimatorPose = Pose2d(0, 0, Rotation2d(0))

        self.profiledRotationConstraints = TrapezoidProfileRadians.Constraints(
            constants.kProfiledRotationMaxVelocity, constants.kProfiledRotationMaxAccel
        )
        self.profiledRotationController = ProfiledPIDControllerRadians(
            constants.kProfiledRotationP,
            constants.kProfiledRotationI,
            constants.kProfiledRotationD,
            self.profiledRotationConstraints,
        )
        self.profiledRotationController.enableContinuousInput(-math.pi, math.pi)

        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            self.getPose,  # Robot pose supplier
            self.resetPose,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.setChassisSpeeds,  # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig(  # HolonomicPathFollowerConfig, this should likely live in your Constants class
                configs.autobuilderHolonomicTranslationPID,  # Translation PID constants
                configs.autobuilderHolonomicRotationPID,  # Rotation PID constants
                self.max_speed,  # Max module speed, in m/s
                constants.kDriveBaseRadius,  # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig(),  # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self,  # Reference to this subsystem to set requirements
        )
        self.robotToSpeaker = None
        self.robotToAmp = None
        self.field = Field2d()
        SmartDashboard.putData("DrivePose", self.field)

    def onRedAlliance(self):
        # Returns boolean that equals true if we are on the Red Alliance
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return self.onRedAlliance()

    def getSpeakerTagNum(self):
        # should return 7 is false and 4 if True
        return [AprilTagPlacement.Blue.SPEAKER, AprilTagPlacement.Red.SPEAKER][
            self.onRedAlliance()
        ]

    def getAmpTagNum(self):
        return [AprilTagPlacement.Blue.AMP, AprilTagPlacement.Red.AMP][
            self.onRedAlliance()
        ]

    def getStageTagNum(self):
        # if our X value on the field falls between the blue and red
        # wing lines
        if 5.320792 < self.estimatorPose.x < 11.220196:
            return [
                AprilTagPlacement.Blue.STAGE_CENTER,
                AprilTagPlacement.Red.STAGE_CENTER,
            ][self.onRedAlliance()]
        # if we are inside our wing, determine if we are on the amp or
        # source side of the stage
        elif self.estimatorPose.y > 4.105148:
            return [
                AprilTagPlacement.Blue.STAGE_AMP,
                AprilTagPlacement.Red.STAGE_AMP,
            ][self.onRedAlliance()]
        else:
            return [
                AprilTagPlacement.Blue.STAGE_SOURCE,
                AprilTagPlacement.Red.STAGE_SOURCE,
            ][self.onRedAlliance()]

    def getStagePose(self):
        # get the tag's pose and transform it for the robot position
        # in this case we need the robot facing the opposite direction
        # of the tag and about 0.5 meters in front of it.
        return (
            self.fieldLayout.getTagPose(self.getStageTagNum())
            .toPose2d()
            .transformBy(
                Pose2d(0.5, 0, Rotation2d(math.pi)) - Pose2d(0, 0, Rotation2d(0))
            )
        )

    def getPathToStage(self) -> str:

        if (self.estimatorPose.x > 6 and not self.onRedAlliance()) or (
            self.estimatorPose.x < 11 and self.onRedAlliance()
        ):
            return "Center Stage Approach"
        elif self.estimatorPose.y > 4.105148:
            return "Amp Side Stage Approach"
        else:
            return "Source Side Stage Approach"

    def driveToStageCommand(self) -> Command:
        # return AutoBuilder.pathfindToPose(
        #     self.getStagePose(), self.pathfindingConstraints, 0.0
        # )
        pathname = self.getPathToStage()
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathname)).withName(
            pathname
        )

    def resetRotationController(self):
        self.profiledRotationController.reset(
            self.getRotation2d().radians(),
            self.gyro.getRadiansPerSecCCW(),
        )

    def enableResetController(self):
        self.resetController = True

    def resetRotationControllerCommand(self):
        return self.runOnce(self.enableResetController)

    def setFieldPositionFromVision(self):
        self.resetPose(self.vision.getLatestData().botPose.toPose2d())
        # self.estimator.resetPosition(
        #     self.gyro.getRotation2d(),
        #     tuple(self.getModulePositions()),
        #     pose,
        # )

    # def resetGyroCommand(self) -> Command:
    #     return self.runOnce(self.gyro.resetGyro(self.onRedAlliance())

    def calculateRobotRelativeTargets(self):
        speakerPose = self.fieldLayout.getTagPose(self.getSpeakerTagNum())
        self.robotToSpeaker = RobotRelativeTarget(self.estimatorPose, speakerPose)
        ampPose = self.fieldLayout.getTagPose(self.getAmpTagNum())
        self.robotToAmp = RobotRelativeTarget(self.estimatorPose, ampPose)

    def getvTtoTag(self):
        tagPose = self.fieldLayout.getTagPose(self.getSpeakerTagNum())
        robotToTarget = RobotRelativeTarget(self.estimatorPose, tagPose)
        return robotToTarget.driveVT

    def getFiringHeadingForSpeaker(self) -> Rotation2d:
        """Get's the robot-relative change in heading the robot needs to make
            to aim the shooter at the speaker

        Returns:
            Rotation2d : the change in heading
        """
        return self.robotToSpeaker.firingHeading

    def periodic(self):
        self.estimatorPose = self.estimator.update(
            self.gyro.getRotation2d(), tuple(self.getModulePositions())
        )
        latestVisionResult = self.vision.getLatestData()
        if latestVisionResult.tagCount > 0:
            if not self.enabled:
                # the drivetrain isn't enabled yet, but we want the apriltags to update
                # the drivetrain's pose
                translationStdDev = remap(
                    latestVisionResult.tagData[0].distanceToRobot, 2, 6, 0.3, 1.0
                )
                rotationStdDev = math.pi
                SmartDashboard.putNumber("TranslationStdDev", translationStdDev)
                SmartDashboard.putNumber(
                    "distanceToTag", latestVisionResult.tagData[0].distanceToRobot
                )
                SmartDashboard.putNumber("tagID", latestVisionResult.tagData[0].id)
                SmartDashboard.putNumber(
                    "tagAmbiguity", latestVisionResult.tagData[0].ambiguity
                )
                self.estimator.addVisionMeasurement(
                    latestVisionResult.botPose.toPose2d(),
                    latestVisionResult.timestamp,
                    (translationStdDev, translationStdDev, rotationStdDev),
                )
                self.leds.drivePoseSet()

            elif (
                abs(latestVisionResult.botPose.x - self.estimatorPose.x) < 1
                and abs(latestVisionResult.botPose.y - self.estimatorPose.y) < 1
            ):
                # TODO: https://github.com/FROG3160/2025-reefscape/issues/6  
                # We may want to validate the first instance of tagData
                # is a valid tag by checking tagData[0].id > 0
                translationStdDev = remap(
                    latestVisionResult.tagData[0].distanceToRobot, 2, 8, 0.3, 1.0
                )
                rotationStdDev = math.pi
                SmartDashboard.putNumber("TranslationStdDev", translationStdDev)
                SmartDashboard.putNumber(
                    "distanceToTag", latestVisionResult.tagData[0].distanceToRobot
                )
                SmartDashboard.putNumber("tagID", latestVisionResult.tagData[0].id)
                SmartDashboard.putNumber(
                    "tagAmbiguity", latestVisionResult.tagData[0].ambiguity
                )
                self.estimator.addVisionMeasurement(
                    latestVisionResult.botPose.toPose2d(),
                    latestVisionResult.timestamp,
                    (translationStdDev, translationStdDev, rotationStdDev),
                )
            # self.estimator.addVisionMeasurement(
            #     visionPose.toPose2d(), visionTimestamp, (0.2, 0.2, math.pi / 8)
            # )

        self.field.setRobotPose(self.estimator.getEstimatedPosition())
        SmartDashboard.putNumberArray(
            "Drive Pose",
            [
                self.estimatorPose.x,
                self.estimatorPose.y,
                self.estimatorPose.rotation().radians(),
            ],
        )
        SmartDashboard.putNumber(
            "Pose Rotation", self.estimatorPose.rotation().degrees()
        )
        # SmartDashboard.putData(
        #     "Drive Pose w/Vision ", self.estimator.getEstimatedPosition()
        # )
        self.calculateRobotRelativeTargets()
        speakerDistance = self.robotToSpeaker.distance
        # update elevation with the needed distance
        self.elevation.setSpeakerDistance(speakerDistance)
        SmartDashboard.putNumber("DistanceToSpeaker", speakerDistance)
        # SmartDashboard.putNumber("Calculated Firing Heading", azimuth.degrees())
        # SmartDashboard.putNumber("Calculated VT", vt)

        # Gyro data
        # SmartDashboard.putNumber("Gyro Angle", self.gyro.getRotation2d().degrees())
        # SmartDashboard.putNumber("Gyro Adjustment", self.gyro.getAngleAdjustment())
        # SmartDashboard.putNumber("RAW Gyro Angle CCW", -self.gyro.gyro.getYaw())

        # run periodic method of the superclass, in this case SwerveChassis.periodic()
        super().periodic()
