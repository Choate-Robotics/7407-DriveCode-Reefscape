from enum import Enum, StrEnum

import ntcore
from constants import reef_scoring_distance
import constants

from ntcore import NetworkTable
from wpilib import DriverStation  # noqa
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
    table: ntcore.NetworkTable,
    name: str,
    pose: Pose2d | Pose3d | Translation2d,
    debug=False,
    red=False,
) -> None:
    """Helper to post Pose2d, Pose3d, or Translation2d as an array to NetworkTables.

    Args:
        table (ntcore.NetworkTableInstance): NetworkTable instance to post to.
        name (str): Name of the entry in the NetworkTable.
        pose (Pose2d | Pose3d | Translation2d): Pose to post.

    """
    pose = FieldConstants.get_red_pose(pose) if red else pose
    if isinstance(pose, Pose2d):
        pose_array = [pose.X(), pose.Y(), pose.rotation().radians()]
        if debug:
            print(f"Pose2d: {name} {pose_array}")
        else:
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
        if debug:
            print(f"Pose3d: {name} {pose_array}")
        else:
            table.putNumberArray(name, pose_array)

    elif isinstance(pose, Translation2d):
        pose_array = [pose.X(), pose.Y(), 0]
        if debug:
            print(f"Translation2d: {name} {pose_array}")
        else:
            table.putNumberArray(name, pose_array)


class Branch(StrEnum):
    A = "A"
    B = "B"
    C = "C"
    D = "D"
    E = "E"
    F = "F"
    G = "G"
    H = "H"
    I = "I"  # noqa
    J = "J"
    K = "K"
    L = "L"

    def __init__(self, label):
        self.label = label
        branches = list(self.__class__)  # Get all enum members
        index = len(branches)  # Find the index of the current face
        adjust_x = (
            30.738 * inches_to_meters + 0.051 + reef_scoring_distance * inches_to_meters
        )  # the extra is to push to the edge of the reef.
        print(adjust_x)
        adjust_y = 6.469 * inches_to_meters
        center = Translation2d(176.746 * inches_to_meters, 158.501 * inches_to_meters)
        face = int(index / 2)

        pose_direction = Pose2d(center, Rotation2d.fromDegrees(-180 + (60 * face)))
        print(f"new pose direction {pose_direction}")

        self.scoring_pose = Pose2d(
            pose_direction.transformBy(
                Transform2d(
                    adjust_x,
                    (-1) ** (index % 2 + 1) * adjust_y,
                    pose_direction.rotation(),
                )
            ).X(),
            pose_direction.transformBy(
                Transform2d(
                    adjust_x,
                    (-1) ** (index % 2 + 1) * adjust_y,
                    pose_direction.rotation(),
                )
            ).Y(),
            pose_direction.rotation() + Rotation2d.fromDegrees(180),
        )


class ReefFace(Enum):
    Face1 = ("Face1", Branch.A, Branch.B)
    Face2 = ("Face2", Branch.C, Branch.D)
    Face3 = ("Face3", Branch.E, Branch.F)
    Face4 = ("Face4", Branch.G, Branch.H)
    Face5 = ("Face5", Branch.I, Branch.J)
    Face6 = ("Face6", Branch.K, Branch.L)

    def __init__(self, label, left, right):
        self.label = label
        self.left = left
        self.right = right
        self.scoring_pose = Pose2d(
            (self.left.scoring_pose.X() + self.right.scoring_pose.X()) / 2,
            (self.left.scoring_pose.Y() + self.right.scoring_pose.Y()) / 2,
            self.left.scoring_pose.rotation(),
        )


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

    debug = False
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
        right_branches = [Branch.B, Branch.D, Branch.F, Branch.H, Branch.J, Branch.L]
        left_branches = [Branch.A, Branch.C, Branch.E, Branch.G, Branch.I, Branch.K]
        # faces=[ReefFace.Face1, ReefFace.Face2, ReefFace.Face3, ReefFace.Face4, ReefFace.Face5, ReefFace.Face6]
        center = Translation2d(176.746 * inches_to_meters, 158.501 * inches_to_meters)
        faceToZoneLine = (
            12 * inches_to_meters
        )  # side of reef to inside of reef zone line

        @staticmethod
        def calculate_pose_3d(center, face, level, adjust_x, adjust_y, is_right=True):
            """Helper to calculate 3D Pose for a given face and level."""
            pose_direction = Pose2d(center, Rotation2d.fromDegrees(180 + (60 * face)))
            adjust_y = adjust_y if is_right else -adjust_y
            return Pose3d(
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

    def update_tables(self, red=False, debug=False) -> None:
        # Initialize NetworkTables
        table = self.table

        def process_value(table: NetworkTable, name, value, debug=False):
            """Recursive processing of a value to post it to NetworkTables."""
            if isinstance(value, (Pose2d, Pose3d, Translation2d)):
                post_pose(table, name, value, self.debug, red)
            elif isinstance(value, list):
                # Process each element in the list
                for i, sub_value in enumerate(value):
                    process_value(table, f"{name}[{i}]", sub_value, debug)
            elif isinstance(value, dict):
                # Process each key-value pair in the dictionary
                for k, v in value.items():
                    process_value(table, f"{name}.{k}", v, debug)
            elif isinstance(value, float):
                # Handle scalar values
                if self.debug:
                    print(f"Scalar: {name} = {value}")
                else:
                    table.putValue(name, value)
            elif isinstance(value, int):  # Handle integers (if needed)
                if self.debug:
                    print(f"Scalar (int): {name} = {value}")
                else:
                    table.putValue(name, value)

        def process_class(table: NetworkTable, prefix, cls, debug=False):
            """Recursively process a class and its nested classes."""
            # cls.__init__(cls)
            for attr_name, attr_value in vars(cls).items():
                attr_path = f"{prefix}.{attr_name}" if prefix else attr_name
                if isinstance(attr_value, type):  # Handle nested classes
                    nested_class_table = table.getSubTable(attr_name)
                    if self.debug:
                        print(f"Nested class: {attr_path}")
                    process_class(nested_class_table, attr_path, attr_value)
                else:
                    process_value(table, attr_path, attr_value, debug)

        def process_enum(table, prefix, enum_class, debug=False):
            for member in enum_class:
                # print(f"\nProcessing {member.name} ({member.value[0]}):")
                for attr_name, attr_value in vars(member).items():
                    attr_path = (
                        f"{prefix}.{str(member)}.{attr_name}" if prefix else attr_name
                    )
                    if isinstance(attr_value, type):  # Handle nested classes
                        nested_class_table = table.getSubTable(attr_name)
                        if self.debug:
                            print(f"Nested class: {attr_path}")
                        process_class(nested_class_table, attr_path, attr_value)
                    elif isinstance(attr_value, Pose2d):
                        process_value(table, attr_path, attr_value, debug)

        # Start processing FieldConstants and its attributes
        process_class(table, "", FieldConstants)
        process_enum(table, "Branch", Branch)
        process_enum(table, "ReefFace", ReefFace)

    @staticmethod
    def get_red_pose(pose: Pose2d | Pose3d):
        if isinstance(pose, Pose2d):
            return Pose2d(
                Translation2d(
                    constants.field_length - pose.X(), constants.field_width - pose.Y()
                ),
                pose.rotation().rotateBy(Rotation2d(180 * degrees_to_radians)),
            )
        elif isinstance(pose, Pose3d):
            return Pose3d(
                Translation3d(
                    constants.field_length - pose.X(),
                    constants.field_width - pose.Y(),
                    pose.Z(),
                ),
                pose.rotation().rotateBy(Rotation3d(0, 0, 180 * degrees_to_radians)),
            )


if __name__ == "__main__":
    FC = FieldConstants()
    FC.update_tables(debug=True)
    # print(FC.Reef.BranchScoringPositions().get_scoring_pose(Branch.L, ReefHeight.L4))
    print(
        f"April Tag closest driver's station x:{144 * inches_to_meters}, y:{158 * inches_to_meters}"
    )
    # branches = Branch()
    # print(branches)
    for branch in Branch:
        print(branch.label, branch.scoring_pose)
    for face in ReefFace:
        # print(here)
        print(f"{face.label} {face.scoring_pose}")
    print(type(Branch.A))
