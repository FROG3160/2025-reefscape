from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from wpimath.geometry import Transform3d
from wpilib import SmartDashboard

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


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

    def get_result(self):
        estimated_pose = self.estimator.update()
        if estimated_pose:
            targets = estimated_pose.targetsUsed[0].getBestCameraToTarget()
            SmartDashboard.putNumber(
                f"{self.estimator._camera.getName()} Targets Found", len(targets)
            )
            target1 = targets[0]
            SmartDashboard.putNumber(
                f"{self.estimator._camera.getName()} First Target Ambiguity",
                target1.getPoseAmbiguity(),
            )

    def periodic(self):
        self.latestVisionPose = self.estimator.update()
        return self.latestVisionPose
