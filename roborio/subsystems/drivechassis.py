import math
from FROGlib.swerve import SwerveBase
from FROGlib.ctre import FROGPigeonGyro
from constants import (
    kMaxChassisRadiansPerSec,
    kMaxMetersPerSecond,
    kDriveBaseRadius,
    kSteerP,
    kSteerI,
)
from configs import ctre

from wpilib import DriverStation, Field2d
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Transform3d, Rotation3d
from wpimath.units import volts
from wpilib.sysid import SysIdRoutineLog

# from subsystems.vision import PositioningSubsystem
# from subsystems.elevation import ElevationSubsystem
from wpilib import SmartDashboard
from commands2 import Subsystem, Command
from commands2.sysid import SysIdRoutine
from FROGlib.utils import RobotRelativeTarget, remap
import constants
from wpimath.units import degreesToRadians
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians

from phoenix6.controls import (
    PositionDutyCycle,
    VelocityVoltage,
    PositionVoltage,
    VoltageOut,
)

# from subsystems.leds import LEDSubsystem


class DriveChassis(SwerveBase):
    def __init__(
        self,
        parent_nt: str = "Subsystems",
    ):
        super().__init__(
            swerve_module_configs=(
                ctre.swerveModuleFrontLeft,
                ctre.swerveModuleFrontRight,
                ctre.swerveModuleBackLeft,
                ctre.swerveModuleBackRight,
            ),
            gyro=FROGPigeonGyro(constants.kGyroID),
            max_speed=kMaxMetersPerSecond,
            max_rotation_speed=kMaxChassisRadiansPerSec,
            parent_nt=parent_nt,
        )
        self.resetController = True

        # initializing the estimator to 0, 0, 0
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

        self.field = Field2d()
        SmartDashboard.putData("DrivePose", self.field)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine_drive = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(self.sysid_drive, self.sysid_log_drive, self),
        )
        self.sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(self.sysid_steer, self.sysid_log_steer, self),
        )

    # Tell SysId how to plumb the driving voltage to the motors.
    def sysid_drive(self, voltage: volts) -> None:
        for module in self.modules:
            module.steer_motor.set_control(
                PositionVoltage(
                    position=0,
                    slot=0,  # Duty Cycle gains for steer
                )
            )
            module.drive_motor.set_control(VoltageOut(output=voltage, enable_foc=False))

    def sysid_steer(self, voltage: volts) -> None:
        for module in self.modules:
            module.steer_motor.set_control(VoltageOut(output=voltage, enable_foc=False))
            module.drive_motor.stopMotor()

    def sysid_log_drive(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Record a frame for each module.  Since these share an encoder, we consider
        # the entire group to be one motor.
        for module in self.modules:
            with module.drive_motor as m:
                sys_id_routine.motor(module.name).voltage(
                    m.get_motor_voltage().value
                ).position(m.get_position().value).velocity(m.get_velocity().value)

    def sysid_log_steer(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Record a frame for each module.  Since these share an encoder, we consider
        # the entire group to be one motor.
        for module in self.modules:
            with module.steer_motor as m:
                sys_id_routine.motor(module.name).voltage(
                    m.get_motor_voltage().value
                ).angularPosition(m.get_position().value).angularVelocity(
                    m.get_velocity().value
                )

    def sysIdQuasistaticDrive(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_drive.quasistatic(direction)

    def sysIdDynamicDrive(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_drive.dynamic(direction)

    def sysIdQuasistaticSteer(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_steer.quasistatic(direction)

    def sysIdDynamicSteer(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_steer.dynamic(direction)

    def resetRotationController(self):
        self.profiledRotationController.reset(
            self.getRotation2d().radians(),
            self.gyro.getRadiansPerSecCCW(),
        )

    def enableResetController(self):
        self.resetController = True

    def resetRotationControllerCommand(self):
        return self.runOnce(self.enableResetController)

    def periodic(self):
        self.estimatorPose = self.estimator.update(
            self.gyro.getRotation2d(), tuple(self.getModulePositions())
        )

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

        # run periodic method of the superclass, in this case SwerveChassis.periodic()
        super().periodic()
