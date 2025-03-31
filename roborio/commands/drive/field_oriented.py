import math
from commands2 import Command
from wpilib import DriverStation
from subsystems.drivechassis import DriveChassis
from FROGlib.xbox import FROGXboxDriver
import constants
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard

# povSpeed = 0.1
# povSpeeds = {
#     0: (povSpeed, 0),
#     45: (povSpeed, -povSpeed),
#     90: (0, -povSpeed),
#     135: (-povSpeed, -povSpeed),
#     180: (-povSpeed, 0),
#     225: (-povSpeed, povSpeed),
#     270: (0, povSpeed),
#     315: (povSpeed, povSpeed),
# }


class ManualDrive(Command):
    def __init__(
        self, controller: FROGXboxDriver, drive: DriveChassis, table: str = "Undefined"
    ) -> None:
        """Allows manual control of the drivetrain through use of the specified
        controller.

        Args:
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
        self.addRequirements(self.drive)
        self.resetController = True
        profiledRotationConstraints = TrapezoidProfileRadians.Constraints(
            constants.kProfiledRotationMaxVelocity, constants.kProfiledRotationMaxAccel
        )
        self.profiledRotationController = ProfiledPIDControllerRadians(
            constants.kProfiledRotationP,
            constants.kProfiledRotationI,
            constants.kProfiledRotationD,
            profiledRotationConstraints,
        )
        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vTPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )
        self._rotation_degreesPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/rotation_degrees")
            .publish()
        )

    def resetRotationController(self):
        self.profiledRotationController.reset(
            self.drive.getRotation2d().radians(),
            self.drive.gyro.getRadiansPerSecCCW(),
        )

    def execute(self) -> None:
        # read right joystick Y to see if we are using it
        driveRotation2d = self.drive.getRotation2d()

        vT = self.controller.getSlewLimitedFieldRotation()
        self._rotation_degreesPub.set(driveRotation2d.degrees())

        # pov = self.controller.getPOVDebounced()
        # if pov != -1:
        #     vX, vY = povSpeeds[pov]
        # else:

        vX = self.controller.getSlewLimitedFieldForward()
        vY = self.controller.getSlewLimitedFieldLeft()

        self.drive.fieldOrientedDrive(
            # self._vX, self._vY, self._vT, self._throttle
            vX,
            vY,
            vT,
            self.controller.getFieldThrottle(),
        )


class AutoMoveOffLine(Command):

    def __init__(self, drive: DriveChassis) -> None:

        self.drive = drive
        self.addRequirements(self.drive)

    def execute(self) -> None:

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            vX = 0.25
        else:
            vX = -0.25

        self.drive.fieldOrientedAutoRotateDrive(
            # self._vX, self._vY, self._vT, self._throttle
            vX,
            0,
            0,
        )
