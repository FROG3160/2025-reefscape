from FROGlib.field import FROGField
from robotpy_apriltag import AprilTagField
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d
from wpilib import DriverStation
from constants import kReefTags


class Position(FROGField):

    def __init__(self):
        super().__init__(AprilTagField.k2025ReefscapeWelded)
        self._reef_tags = None

    def getTagPose(self, tag_id: int):
        return self._layout.getTagPose(tag_id)

    def getClosestReefPosition(self, robot_pose: Pose3d) -> Pose3d:
        """returns the tag number of the closest side of the Reef

        Args:
            robot_pose (Pose3d): Current robot position

        Returns:
            tag number (Pose3d): The tag position of the closest side
                of the reef.
        """
        closest_distance = 1000.0
        closest_tag = None

        for tag in self._reef_tags:
            distance = robot_pose.translation().distance(
                self.getTagPose(tag.value).translation()
            )
            if distance < closest_distance:
                closest_distance = distance
                closest_tag = tag
        return self.getTagPose(closest_tag.value)
        #     print(f"TAG: {tag}, Distance: {distance}")
        # tag_pose = self.getTagPose(closest_tag.value)
        # print(f"CLOSEST TAG: {closest_tag}")
        # print(f"    POSE: {tag_pose}")
        # print(f"    DISTANCE: {closest_distance}")

    def setReefTags(self, alliance: DriverStation.Alliance):
        self._reef_tags = kReefTags[alliance]["Reef"]

    def getRightSidePose():
        import math

        # distance from center of robot forward to the centerline of the arm swing
        robot_forward_adjust = 3  # 5 centimeters
        robot_left_adjust = (
            -12.5
        )  # half the width of the robot, amounts to distance to the bumber's edge
        stem_offset = 6.5  # the distance from center of the apriltag to the reef stem
        # this moves the aimpoint to the edge of the robot that we need to match
        # to one of the reef stems
        robot_aim_point = Transform3d(
            Translation3d(-robot_forward_adjust, -robot_left_adjust, 0),
            Rotation3d(0, 0, -math.pi / 2),
        )
        rightside_stem_adjust = Transform3d(
            Translation3d(-robot_left_adjust, -stem_offset + robot_forward_adjust, 0),
            Rotation3d(),
        )
        leftside_stem_adjust = Transform3d(
            Translation3d(-robot_left_adjust, stem_offset + robot_forward_adjust, 0),
            Rotation3d(),
        )
        test = Pose3d(0, 0, 0, Rotation3d()).transformBy(rightside_stem_adjust)
        test2 = Pose3d(0, 0, 0, Rotation3d()).transformBy(leftside_stem_adjust)
        print(test)
        print(test2)

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

# pass
