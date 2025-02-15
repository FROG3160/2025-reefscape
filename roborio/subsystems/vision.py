from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from wpimath.geometry import Transform3d

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


class VisionPose:
    def __init__(self, poseCameraName: str):
        self.estimator = PhotonPoseEstimator(
            fieldTags=AprilTagFieldLayout().loadField(
                AprilTagField.k2025ReefscapeWelded
            ),
            strategy=PoseStrategy.LOWEST_AMBIGUITY,
            camera=PhotonCamera(poseCameraName),
            robotToCamera=Transform3d(),
        )


    def periodic(self):
        self.latestVisionPose = self.estimator.update()
