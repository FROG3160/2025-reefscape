import math
from commands2 import Command
from subsystems.drivechassis import DriveChassis
from subsystems.vision import TargetingSubsystem
import constants
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from ntcore import NetworkTableInstance
from FROGlib.xbox import FROGXboxDriver
from wpimath.kinematics import ChassisSpeeds


class DriveToTarget(Command):

    def __init__(
        self,
        drive: DriveChassis,
        targeting: TargetingSubsystem,
        table: str = "Undefined",
    ) -> None:
        """Allows robot to drive in a robot oriented towards the target

        Args:
            drive (DriveTrain): The drive to control.
            targeting (TargetingSubsystem): The targeting subsystem that uses a limelight
            to calculate chassis speeds to drive towards the target
            table (str, optional): The name of the network table telemetry data will go into.
        """

        self.drive = drive
        self.targeting = targeting
        self.addRequirements(self.drive)
        self.nt_table = f"{table}/{type(self).__name__}"

    def execute(self):
        vX = self.targeting.calculate_vx()
        vT = self.targeting.calculate_vt()
        chassisSpeeds = ChassisSpeeds(
            vX,
            0,
            vT * self.drive.max_rotation_speed * 0.12,
        )
        self.drive.robotOrientedDrive(*chassisSpeeds)
