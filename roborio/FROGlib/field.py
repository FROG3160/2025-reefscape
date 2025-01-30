from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


class FROGField:
    def __init__(self):
        self._layout = AprilTagFieldLayout().loadField(AprilTagField.k2025Reefscape)

    def getTagPose(self, tag_id: int):
        return self._layout.getTagPose(tag_id)
