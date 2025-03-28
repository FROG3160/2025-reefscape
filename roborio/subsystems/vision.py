from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from wpimath.geometry import Transform3d
from wpilib import SmartDashboard
from wpimath.geometry import Pose3d, Pose2d
from FROGlib.utils import PoseBuffer
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from ntcore import NetworkTableInstance


class VisionPose:
    def __init__(
        self, poseCameraName: str, cameraTransform: Transform3d = Transform3d()
    ):
        self.camera_name = poseCameraName
        self.camera = PhotonCamera(poseCameraName)
        self.estimator = PhotonPoseEstimator(
            fieldTags=AprilTagFieldLayout().loadField(
                AprilTagField.k2025ReefscapeWelded
            ),
            strategy=PoseStrategy.LOWEST_AMBIGUITY,
            # strategy=PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera=self.camera,
            robotToCamera=cameraTransform,
        )
        # self.estimator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

        nt_table = f"Subsystems/Vision/{self.camera_name}"
        self.pose_buffer = PoseBuffer(25)
        self._latest_pose_pub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(f"{nt_table}/estimated_pose", Pose2d)
            .publish()
        )
        # self._detected_tags_pub = (
        #     NetworkTableInstance.getDefault()
        #     .getStructArrayTopic(f"{nt_table}/detected_tags", dict)
        #     .publish()
        # )
        self._stdev_x_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/stdev_x")
            .publish()
        )
        self._stdev_y_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/stdev_y")
            .publish()
        )
        self._stdev_rotation_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/stdev_rotation")
            .publish()
        )

    def get_result(self):
        estimated_pose = self.estimator.update()
        if estimated_pose:
            targets = estimated_pose.targetsUsed
            SmartDashboard.putNumber(
                f"{self.estimator._camera.getName()} Targets Found", len(targets)
            )
            target1 = targets[0]
            SmartDashboard.putNumber(
                f"{self.estimator._camera.getName()} First Target Ambiguity",
                target1.getPoseAmbiguity(),
            )

    def get_distance_to_tag(self, pose: Pose3d, tag_num: int) -> float:
        return (
            pose.toPose2d()
            .translation()
            .distance(self.getTagPose(tag_num).toPose2d().translation())
        )

    def getTagPose(self, tag_id: int):
        return self.estimator.fieldTags.getTagPose(tag_id)

    def periodic(self):
        self.latestVisionPose = self.estimator.update()
        # result = self.estimator._camera.getLatestResult()
        # if result.hasTargets():
        #     target = result.getBestTarget()
        #     ambiguity = target.getPoseAmbiguity()
        if self.latestVisionPose:
            self.pose_buffer.append(self.latestVisionPose.estimatedPose.toPose2d())
            # target_details = []
            # for photon_target in self.latestVisionPose.targetsUsed():
            #     target_details.append(
            #         {
            #             "tag": photon_target.fiducialId,
            #             "ambiguity": photon_target.poseAmbiguity,
            #             "transform": photon_target.bestCameraToTarget,
            #             "distance": self.get_distance_to_tag(
            #                 self.latestVisionPose.estimatedPose.toPose2d(),
            #                 photon_target.fiducialId,
            #             ),
            #         }
            #     )
            self._latest_pose_pub.set(self.latestVisionPose.estimatedPose.toPose2d())
            self._stdev_x_pub.set(self.pose_buffer.x_stddev())
            self._stdev_y_pub.set(self.pose_buffer.y_stddev())
            self._stdev_rotation_pub.set(self.pose_buffer.rotation_stddev())
            return self.latestVisionPose
