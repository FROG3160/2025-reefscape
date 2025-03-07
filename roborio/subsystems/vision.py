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
        )

    def periodic(self):
        self.latestVisionPose = self.estimator.update()
        return self.latestVisionPose


class ColorVision:
    def __init__(self, colorCameraName: str):
        self.camera = PhotonCamera(colorCameraName)

    def periodic(self):
        self.latestTargetResult = self.camera.getLatestResult()
        self.latestTargets = self.latestTargetResult.getTargets()
        for target in self.latestTargets:
            pass
