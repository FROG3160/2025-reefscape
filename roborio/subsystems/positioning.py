from FROGlib.field import FROGField
from robotpy_apriltag import AprilTagField
from wpimath.geometry import (
    Transform3d,
    Translation3d,
    Rotation3d,
    Pose3d,
    Transform2d,
    Rotation2d,
    Pose2d,
)

from wpilib import DriverStation
from constants import kReefTags
from wpimath.units import inchesToMeters
import math

TAG_TO_STEM_DISTANCE = inchesToMeters(6.5)
ROBOT_WIDTH_TO_CENTER = 0.44
ROBOT_GRABBER_TO_CENTER = inchesToMeters(1)
TAG_TO_LEFT_STEM_ROBOT_TRANSFORM = Transform2d(
    ROBOT_WIDTH_TO_CENTER,
    -TAG_TO_STEM_DISTANCE + ROBOT_GRABBER_TO_CENTER,
    Rotation2d(-math.pi / 2),
)
TAG_TO_RIGHT_STEM_ROBOT_TRANSFORM = Transform2d(
    ROBOT_WIDTH_TO_CENTER,
    TAG_TO_STEM_DISTANCE + ROBOT_GRABBER_TO_CENTER,
    Rotation2d(-math.pi / 2),
)
TAG_TO_CENTER_ROBOT_TRANSFORM = Transform2d(
    ROBOT_WIDTH_TO_CENTER,
    ROBOT_GRABBER_TO_CENTER,
    Rotation2d(-math.pi / 2),
)


class Position(FROGField):
    TRANSFORMS = [
        TAG_TO_LEFT_STEM_ROBOT_TRANSFORM,
        TAG_TO_CENTER_ROBOT_TRANSFORM,
        TAG_TO_RIGHT_STEM_ROBOT_TRANSFORM,
    ]
    LEFT = 0
    CENTER = 1
    RIGHT = 2

    def __init__(self):
        super().__init__(AprilTagField.k2025ReefscapeWelded)
        self._reef_tags = kReefTags[DriverStation.Alliance.kRed]["Reef"]

    def get_closest_reef_tag_num(self, robot_pose: Pose2d) -> int:
        """returns the tag number of the closest side of the Reef

        Args:
            robot_pose (Pose2d): Current robot position

        Returns:
            tag number (): The tag position of the closest side
                of the reef.
        """
        closest_distance = 1000.0
        closest_tag = None

        for tag in self._reef_tags:
            distance = self.get_distance_to_tag(robot_pose, tag.value)
            if distance < closest_distance:
                closest_distance = distance
                closest_tag = tag
        return closest_tag.value

    def get_closest_reef_pose(self, robot_pose: Pose2d) -> Pose2d:
        return self.getTagPose(self.get_closest_reef_tag_num(robot_pose)).toPose2d()

    def get_reef_enum_name(self, tag_num: int) -> str:
        return self._reef_tags(tag_num).name

    def setReefTags(self, alliance: DriverStation.Alliance):
        if alliance:
            self._reef_tags = kReefTags[alliance]["Reef"]

    def get_distance_to_tag(self, pose: Pose2d, tag_num: int) -> float:
        return pose.translation().distance(
            self.getTagPose(tag_num).toPose2d().translation()
        )

    def get_scoring_pose(self, poseToTransform: Pose2d, position: str):
        if position == "left":
            return poseToTransform.transformBy(TAG_TO_LEFT_STEM_ROBOT_TRANSFORM)
        if position == "right":
            return poseToTransform.transformBy(TAG_TO_RIGHT_STEM_ROBOT_TRANSFORM)

        # # distance from center of robot forward to the centerline of the arm swing
        # robot_forward_adjust = 3  # 5 centimeters
        # robot_left_adjust = (
        #     -12.5
        # )  # half the width of the robot, amounts to distance to the bumber's edge
        # stem_offset = 6.5  # the distance from center of the apriltag to the reef stem
        # # this moves the aimpoint to the edge of the robot that we need to match
        # # to one of the reef stems
        # robot_aim_point = Transform2d(
        #     Translation2d(-robot_forward_adjust, -robot_left_adjust),
        #     Rotation3d(0, 0, -math.pi / 2),
        # )
        # rightside_stem_adjust = Transform3d(
        #     Translation3d(-robot_left_adjust, -stem_offset + robot_forward_adjust, 0),
        #     Rotation3d(),
        # )
        # leftside_stem_adjust = Transform3d(
        #     Translation3d(-robot_left_adjust, stem_offset + robot_forward_adjust, 0),
        #     Rotation3d(),
        # )
        # test = Pose3d(0, 0, 0, Rotation3d()).transformBy(rightside_stem_adjust)
        # test2 = Pose3d(0, 0, 0, Rotation3d()).transformBy(leftside_stem_adjust)
        # print(test)
        # print(test2)

    # print(f"    ANGLE: {}")


# f = FROGField()
# # this transform will move the pose half a meter "forward"
# adjust1 = Transform3d(Translation3d(x=0.5, y=0, z=0), Rotation3d())
# # this transform will move the pose half a meter to the left 45 degrees
# adjust2 = Transform3d(
#     Translation3d(distance=0.5, angle=Rotation3d().fromDegrees(0, 0, 45)), Rotation3d()
# )
# # this transform will move the pose half a meter to the right
# adjust3 = Transform3d(Translation3d(x=0, y=-0.5, z=0), Rotation3d())
# # this transform moves the pose 4 meters to the right 30 degrees
# adjust4 = Transform3d(
#     Translation3d(distance=3, angle=Rotation3d().fromDegrees(0, 0, -22)), Rotation3d()
# )
# position = f.getTagPose(f.Tags.Blue.Processor.RIGHT.value).transformBy(adjust4)
# f.getClosestReefSide(position)
# print("\n")
# print(f"Robot Position: {position}")
# relative = position.relativeTo(f.getTagPose(f.Tags.Blue.Reef.BARGECENTER.value))
# print(f"RELATIVE: {relative}")
# print(f"REL ANGLE: {relative.translation().toTranslation2d().angle().degrees()}")

# test = Position()
# print(test.getTagPose(18))
# for transform in [
#     TAG_TO_CENTER_ROBOT_TRANSFORM,
#     TAG_TO_LEFT_STEM_ROBOT_TRANSFORM,
#     TAG_TO_RIGHT_STEM_ROBOT_TRANSFORM,
# ]:
#     robotpose = test.getTagPose(18).toPose2d().transformBy(transform)
#     print(f"X: {robotpose.x}")
#     print(f"Y: {robotpose.y}")
#     print(f"DEG: {robotpose.rotation().degrees()}")
# pass
