from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from wpimath.geometry import Transform3d

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
            offset=0,
        )

    def periodic(self):
        self.latestVisionPose = self.estimator.update()
        return self.latestVisionPose
