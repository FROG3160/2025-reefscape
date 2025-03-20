import math
from commands2 import Command
from FROGlib.xbox import FROGXboxDriver
from FROGlib.swerve import SwerveBase
from subsystems.drivechassis import DriveChassis


class ManualRobotOrientedDrive(Command):
    def __init__(
        self, controller: FROGXboxDriver, drive: DriveChassis, table: str = "Undefined"
    ) -> None:
        """Allows manual control of the drivetrain through use of the specified
        controller.

        Args:
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveChassis): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
        self.addRequirements(self.drive)

    def execute(self) -> None:
        throttle = self.controller.getFieldThrottle()
        max_speed = self.drive.max_speed

        self.drive.robotOrientedDrive(
            # self._vX, self._vY, self._vT, self._throttle
            self.controller.getRobotForward(),
            self.controller.getRobotLeft(),
            self.controller.getRotation(),
        )
