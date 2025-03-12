from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from wpimath.geometry import Transform3d
from commands2.subsystem import Subsystem
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from constants import kLLTargeting, kTargetAreaThreshold
from ntcore import NetworkTableInstance
from wpimath.filter import MedianFilter
import math
from wpimath.kinematics import ChassisSpeeds


class VisionPose:
    def __init__(
        self, poseCameraName: str, cameraTransform: Transform3d = Transform3d()
    ):

        self.estimator = PhotonPoseEstimator(
            fieldTags=AprilTagFieldLayout().loadField(
                AprilTagField.k2025ReefscapeWelded
            ),
            strategy=PoseStrategy.LOWEST_AMBIGUITY,
            camera=PhotonCamera(poseCameraName),
            robotToCamera=cameraTransform,
        )

    def periodic(self):
        self.latestVisionPose = self.estimator.update()
        return self.latestVisionPose


class VisionTargeting:
    def __init__(self, limelight_name: str = "limelight"):
        self.network_table = NetworkTableInstance.getDefault().getTable(
            key=limelight_name
        )
        self.nt_tclass = self.network_table.getStringTopic("tclass").subscribe("None")
        self.nt_ta = self.network_table.getFloatTopic("ta").subscribe(0)
        self.nt_tx = self.network_table.getFloatTopic("tx").subscribe(-999)
        self.nt_ty = self.network_table.getFloatTopic("ty").subscribe(-999)
        self.nt_tv = self.network_table.getIntegerTopic("tv").subscribe(0)

        self.zeroValues()

    def getTarget(self):
        if self.nt_tv.get():
            self.tClass = self.nt_tclass.get()
            self.ta = self.nt_ta.get()
            self.tx = self.nt_tx.get()
            self.ty = self.nt_ty.get()
            self.tv = self.nt_tv.get()
        else:
            self.zeroValues()

    def zeroValues(self):
        self.tClass = self.ta = self.tx = self.ty = self.tv = None

    def hasObjectTarget(self):
        return self.tv == 1


class TargetingSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.tracker = VisionTargeting(kLLTargeting)
        self.filterVX = MedianFilter(9)
        self.filterVT = MedianFilter(9)

    def getTargetInRange(self):
        """Returns true if ta is more than 18"""

        return float(self.tracker.ta or 0) > kTargetAreaThreshold

    def calculate_vx(self):
        """Calculate X robot-oriented speed from the Y value of the target in the camera frame.

        Args:
            targetY (Float):  The target Y value determined by limelight.

        Returns:
            Float: Velocity (m/s) in the X direction (robot oriented)
        """
        if ty := self.tracker.ty:
            return self.filterVX.calculate(
                min(
                    2.0, ((0.45 * math.log(max(ty, -11) + 11.5))) + 0.4
                )  # (0.020833 * (14.7 * math.exp(0.0753 * ty) * 2))
            )
        else:
            return self.filterVX.calculate(0)

    def calculate_vt(self):
        """Calculate the rotational speed from the X value of the target in the camera frame.
        Return is inverted to make left rotation positive from a negative X value, meaning the
        target is to the left of center of the camera's view.

        Args:
            targetX (Float): The X value of the target in the camera frame, 0 is straight ahead,
            to the left is negative, to the right is positive.

        Returns:
            Float: Rotational velocity (radians/sec) with CCW (left, robot oriented) positive.
        """
        if tx := self.tracker.tx:
            return self.filterVT.calculate(-(tx / 35))
        else:
            return self.filterVT.calculate(0)

    def getChassisSpeeds(self):
        """Get calculated velocities from vision target data"""
        return ChassisSpeeds(self.calculate_vx(), 0, self.calculate_vt())
