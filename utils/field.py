from enum import Enum

import ntcore
from ntcore import NetworkTable
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Transform2d,
    Translation2d,
    Translation3d,
)

from units.SI import degrees_to_radians, inches_to_meters


def post_pose_as_array(
    table: ntcore.NetworkTableInstance, name: str, pose: Pose2d | Pose3d | Translation2d
) -> None:
    """Helper to post Pose2d or Pose3d as an array to NetworkTables."""
    if isinstance(pose, Pose2d):
        pose_array = [pose.X(), pose.Y(), pose.rotation().degrees()]
        table.putValue(name, pose_array)
    elif isinstance(pose, Pose3d):
        pose_array = [
            pose.X(),
            pose.Y(),
            pose.Z(),
            pose.rotation().roll,
            pose.rotation().yaw,
            pose.rotation().roll,
        ]
        table.putValue(name, pose_array)
    elif isinstance(pose, Translation2d):
        pose_array = [pose.X(), pose.Y()]
        table.putValue(name, pose_array)


class ReefHeight(Enum):
    L4 = (72 * inches_to_meters, -90)
    L3 = (47.625 * inches_to_meters, -35)
    L2 = (31.875 * inches_to_meters, -35)
    L1 = (18 * inches_to_meters, 0)

    def __init__(self, height, pitch):
        self.height = height
        self.pitch = pitch  # in degrees


class FieldConstants:
    """
    This class is designed from looking at a similar class in
    6328's 2025 code.
    https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/FieldConstants.java
    // Copyright (c) 2025 FRC 6328
    // http://github.com/Mechanical-Advantage
    //
    // Use of this source code is governed by an MIT-style
    // license that can be found in the LICENSE file at
    // the root directory of this project.
    """

    fieldLength = 690.876 * inches_to_meters
    fieldWidth = 317 * inches_to_meters
    startingLineX = 299.438 * inches_to_meters
    table: NetworkTable = ntcore.NetworkTableInstance.getDefault().getTable(
        "FieldConstants"
    )

    class Processor:
        centerFace = Pose2d(235.726 * inches_to_meters, 0, Rotation2d.fromDegrees(90))

    class Barge:
        farCage = Translation2d(345.428 * inches_to_meters, 286.779 * inches_to_meters)
        middleCage = Translation2d(
            345.428 * inches_to_meters, 242.855 * inches_to_meters
        )
        closeCage = Translation2d(
            345.428 * inches_to_meters, 199.947 * inches_to_meters
        )

        deepHeight = 3.125 * inches_to_meters
        shallowHeight = 30.125 * inches_to_meters

    class CoralStation:
        leftCenterFace = Pose2d(
            33.526 * inches_to_meters,
            291.176 * inches_to_meters,
            Rotation2d.fromDegrees(90 - 144.011),
        )
        rightCenterFace = Pose2d(
            33.526 * inches_to_meters,
            25.824 * inches_to_meters,
            Rotation2d.fromDegrees(144.011 - 90),
        )

    class Reef:
        center = Translation2d(176.746 * inches_to_meters, 158.501 * inches_to_meters)
        faceToZoneLine = (
            12 * inches_to_meters
        )  # side of reef to inside of reef zone line
        centerFaces = []
        centerFaces[0] = Pose2d(
            144.003 * inches_to_meters,
            158.5 * inches_to_meters,
            Rotation2d.fromDegrees(180),
        )
        centerFaces[1] = Pose2d(
            160.373 * inches_to_meters,
            186.857 * inches_to_meters,
            Rotation2d.fromDegrees(120),
        )
        centerFaces[2] = Pose2d(
            193.116 * inches_to_meters,
            186.858 * inches_to_meters,
            Rotation2d.fromDegrees(60),
        )
        centerFaces[3] = Pose2d(
            209.489 * inches_to_meters,
            158.502 * inches_to_meters,
            Rotation2d.fromDegrees(0),
        )
        centerFaces[4] = Pose2d(
            193.118 * inches_to_meters,
            130.145 * inches_to_meters,
            Rotation2d.fromDegrees(-60),
        )
        centerFaces[5] = Pose2d(
            160.375 * inches_to_meters,
            130.144 * inches_to_meters,
            Rotation2d.fromDegrees(-120),
        )

        def __init__(self):
            self.branch_positions = {}
            branchlabels = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]
            currentbranch = 0
            for face in range(6):
                fill_right = {}
                fill_left = {}

                for level in list(ReefHeight):
                    # Calculate the pose direction
                    pose_direction = Pose2d(
                        self.center, Rotation2d.fromDegrees(180 - (60 * face))
                    )
                    adjust_x = 30.738 * inches_to_meters
                    adjust_y = 6.469 * inches_to_meters

                    # Fill the right poses
                    fill_right[level] = Pose3d(
                        Translation3d(
                            pose_direction.transformBy(
                                Transform2d(adjust_x, adjust_y, Rotation2d())
                            ).X(),
                            pose_direction.transformBy(
                                Transform2d(adjust_x, adjust_y, Rotation2d())
                            ).Y(),
                            level.height,
                        ),
                        Rotation3d(
                            0,
                            level.pitch * degrees_to_radians,
                            pose_direction.rotation().radians(),
                        ),
                    )

                    # Fill the left poses
                    fill_left[level] = Pose3d(
                        Translation3d(
                            pose_direction.transformBy(
                                Transform2d(adjust_x, -adjust_y, Rotation2d())
                            ).X(),
                            pose_direction.transformBy(
                                Transform2d(adjust_x, -adjust_y, Rotation2d())
                            ).Y(),
                            level.height,
                        ),
                        Rotation3d(
                            0,
                            level.pitch * degrees_to_radians,
                            pose_direction.rotation().radians(),
                        ),
                    )

                # Add positions to branch_positions
                self.branch_positions[branchlabels[currentbranch]] = (
                    face * 2
                ) + 1, fill_right
                self.branch_positions[branchlabels[currentbranch]] = (
                    face * 2
                ) + 2, fill_left

    def update_tables(self) -> None:
        # Initialize NetworkTables
        table = self.table

        # Iterate through FieldConstants and its nested classes
        for attr_name, attr_value in vars(FieldConstants).items():
            if (
                isinstance(attr_value, Pose2d)
                or isinstance(attr_value, Pose3d)
                or isinstance(attr_value, Translation2d)
            ):
                post_pose_as_array(table, attr_name, attr_value)

            # If it's a nested class, iterate through its variables
            elif isinstance(attr_value, type):  # Check for nested class
                nested_class_table = table.getSubTable(attr_name)
                for nested_attr_name, nested_attr_value in vars(attr_value).items():
                    if (
                        isinstance(nested_attr_value, Pose2d)
                        or isinstance(nested_attr_value, Pose3d)
                        or isinstance(nested_attr_value, Translation2d)
                    ):
                        post_pose_as_array(
                            nested_class_table, nested_attr_name, nested_attr_value
                        )
