from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from wpimath.geometry import Transform3d

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


class VisionPose:
    def __init__(self):
        self.estimator = PhotonPoseEstimator(
            fieldTags=AprilTagFieldLayout().loadField(AprilTagField.k2025Reefscape),
            strategy=PoseStrategy(),
            camera=PhotonCamera("FROGPositioning"),
            robotToCamera=Transform3d(),
        )
