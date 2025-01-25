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


def post_pose(
    table: ntcore.NetworkTableInstance, name: str, pose: Pose2d | Pose3d | Translation2d
) -> None:
    """Helper to post Pose2d, Pose3d, or Translation2d as an array to NetworkTables.

    Args:
        table (ntcore.NetworkTableInstance): NetworkTable instance to post to.
        name (str): Name of the entry in the NetworkTable.
        pose (Pose2d | Pose3d | Translation2d): Pose to post.

    """
    # publisher = ntcore.NetworkTableInstance.getDefault()
    if isinstance(pose, Pose2d):
        pose_array = [pose.X(), pose.Y(), pose.rotation().radians()]
        table.putNumberArray(name, pose_array)
    elif isinstance(pose, Pose3d):
        pose_array = [
            pose.X(),
            pose.Y(),
            pose.Z(),
            pose.rotation().X(),
            pose.rotation().Y(),
            pose.rotation().Z(),
        ]
        table.putNumberArray(name, pose_array)

    elif isinstance(pose, Translation2d):
        pose_array = [pose.X(), pose.Y(), 0]
        table.putNumberArray(name, pose_array)


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

    class StagingPositions:
        # Measured from the center of the ice cream
        leftIceCream = Pose2d(
            48 * inches_to_meters, 230.5 * inches_to_meters, Rotation2d()
        )
        middleIceCream = Pose2d(
            48 * inches_to_meters, 158.5 * inches_to_meters, Rotation2d()
        )
        rightIceCream = Pose2d(
            48 * inches_to_meters, 86.5 * inches_to_meters, Rotation2d()
        )

    class Reef:
        center = Translation2d(176.746 * inches_to_meters, 158.501 * inches_to_meters)
        faceToZoneLine = (
            12 * inches_to_meters
        )  # side of reef to inside of reef zone line
        centerFaces = []
        centerFaces.append(
            Pose2d(
                144.003 * inches_to_meters,
                158.5 * inches_to_meters,
                Rotation2d.fromDegrees(180),
            )
        )
        centerFaces.append(
            Pose2d(
                160.375 * inches_to_meters,
                130.144 * inches_to_meters,
                Rotation2d.fromDegrees(-120),
            )
        )
        centerFaces.append(
            Pose2d(
                193.118 * inches_to_meters,
                130.145 * inches_to_meters,
                Rotation2d.fromDegrees(-60),
            )
        )
        centerFaces.append(
            Pose2d(
                209.489 * inches_to_meters,
                158.502 * inches_to_meters,
                Rotation2d.fromDegrees(0),
            )
        )
        centerFaces.append(
            Pose2d(
                193.116 * inches_to_meters,
                186.858 * inches_to_meters,
                Rotation2d.fromDegrees(60),
            )
        )
        centerFaces.append(
            Pose2d(
                160.373 * inches_to_meters,
                186.857 * inches_to_meters,
                Rotation2d.fromDegrees(120),
            )
        )

        branch_positions_2d = {}
        branch_positions = {}
        branchlabels = ["B", "A", "D", "C", "F", "E", "H", "G", "J", "I", "L", "K"]
        currentbranch = 0
        for face in range(6):
            fill_right = {}
            fill_left = {}

            for level in list(ReefHeight):
                # Calculate the pose direction
                pose_direction = Pose2d(
                    center, Rotation2d.fromDegrees(180 + (60 * face))
                )
                adjust_x = 30.738 * inches_to_meters
                adjust_y = 6.469 * inches_to_meters
                branch_positions_2d[branchlabels[currentbranch]] = Pose2d(
                    pose_direction.transformBy(
                        Transform2d(adjust_x, adjust_y, pose_direction.rotation())
                    ).X(),
                    pose_direction.transformBy(
                        Transform2d(adjust_x, adjust_y, pose_direction.rotation())
                    ).Y(),
                    pose_direction.rotation(),
                )
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
                branch_positions_2d[branchlabels[currentbranch + 1]] = Pose2d(
                    pose_direction.transformBy(
                        Transform2d(adjust_x, -adjust_y, pose_direction.rotation())
                    ).X(),
                    pose_direction.transformBy(
                        Transform2d(adjust_x, -adjust_y, pose_direction.rotation())
                    ).Y(),
                    pose_direction.rotation(),
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
            branch_positions[branchlabels[currentbranch]] = fill_right
            branch_positions[branchlabels[currentbranch + 1]] = fill_left
            currentbranch += 2
        fill_left = None  # so this doesn't get sent to the network table
        fill_right = None  # so this doesn't get sent to the network table
        pose_direction = None  # so this doesn't get sent to the network table

    def update_tables(self) -> None:
        # Initialize NetworkTables
        table = self.table

        def process_value(table, name, value):
            """Recursive processing of a value to post it to NetworkTables."""
            if isinstance(value, (Pose2d, Pose3d, Translation2d)):
                post_pose(table, name, value)
            elif isinstance(value, list):
                count = 0
                # Process each element in the list
                if isinstance(value[0], (Pose2d, Pose3d, Translation2d)):
                    for sub_value in value:
                        post_pose(table, f"{name}{str(count)}", sub_value)
                        count += 1
            elif isinstance(value, dict):
                # Process each key-value pair in the dictionary
                # dict_table = table.getSubTable(name)
                for k, v in value.items():
                    process_value(table, name + str(k), v)
            else:
                # Handle scalar values
                if isinstance(value, float):
                    table.putValue(name, value)

        # Iterate through FieldConstants and its nested classes
        for attr_name, attr_value in vars(FieldConstants).items():
            print(attr_name)
            if isinstance(attr_value, type):  # Handle nested classes
                nested_class_table = table.getSubTable(attr_name)
                for nested_attr_name, nested_attr_value in vars(attr_value).items():
                    process_value(
                        nested_class_table, nested_attr_name, nested_attr_value
                    )
            else:
                process_value(table, attr_name, attr_value)
