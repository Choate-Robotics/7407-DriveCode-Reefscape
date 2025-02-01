from enum import Enum, EnumType, StrEnum

import ntcore
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

from constants import reef_scoring_distance
from units.SI import degrees_to_radians, inches_to_meters


def flatten_pose3d(pose: Pose3d) -> list:
    return [
        pose.X(),
        pose.Y(),
        pose.Z(),
        pose.rotation().X(),
        pose.rotation().Y(),
        pose.rotation().Z(),
    ]


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

        adjust_y = 6.469 * inches_to_meters
        center = Translation2d(176.746 * inches_to_meters, 158.501 * inches_to_meters)
        face = int(index / 2)

        pose_direction = Pose2d(center, Rotation2d.fromDegrees(-180 + (60 * face)))

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

    @staticmethod
    def get_right_branches() -> list:
        return [Branch.B, Branch.D, Branch.F, Branch.H, Branch.J, Branch.L]

    @staticmethod
    def get_left_branches() -> list:
        return [Branch.A, Branch.C, Branch.E, Branch.G, Branch.I, Branch.K]


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


class Processor:
    centerFace = Pose2d(235.726 * inches_to_meters, 0, Rotation2d.fromDegrees(90))


class Barge:
    farCage = Translation2d(345.428 * inches_to_meters, 286.779 * inches_to_meters)
    middleCage = Translation2d(345.428 * inches_to_meters, 242.855 * inches_to_meters)
    closeCage = Translation2d(345.428 * inches_to_meters, 199.947 * inches_to_meters)

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
    leftIceCream = Pose2d(48 * inches_to_meters, 230.5 * inches_to_meters, Rotation2d())
    middleIceCream = Pose2d(
        48 * inches_to_meters, 158.5 * inches_to_meters, Rotation2d()
    )
    rightIceCream = Pose2d(48 * inches_to_meters, 86.5 * inches_to_meters, Rotation2d())


class Reef:
    right_branches = [Branch.B, Branch.D, Branch.F, Branch.H, Branch.J, Branch.L]
    left_branches = [Branch.A, Branch.C, Branch.E, Branch.G, Branch.I, Branch.K]
    # faces=[ReefFace.Face1, ReefFace.Face2, ReefFace.Face3, ReefFace.Face4, ReefFace.Face5, ReefFace.Face6]
    center = Translation2d(176.746 * inches_to_meters, 158.501 * inches_to_meters)
    faceToZoneLine = 12 * inches_to_meters  # side of reef to inside of reef zone line


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
    field_length = 690.876 * inches_to_meters
    field_width = 317 * inches_to_meters
    starting_line_x = 299.438 * inches_to_meters


class NT_Updater:
    def __init__(self, table_name: str, table: NetworkTable | None = None):
        if table is None:
            self.table = ntcore.NetworkTableInstance.getDefault().getTable(table_name)
        else:
            self.table = table

    def post_pose(
        self,
        table: NetworkTable,
        name: str,
        pose: Pose2d | Pose3d | Translation2d,
        debug=False,
    ) -> None:
        """Helper to post Pose2d, Pose3d, or Translation2d as an array to NetworkTables.

        Args:
            table (ntcore.NetworkTableInstance): NetworkTable instance to post to.
            name (str): Name of the entry in the NetworkTable.
            pose (Pose2d | Pose3d | Translation2d): Pose to post.

        """
        # pose = FieldConstants.get_red_pose(pose) if red else pose
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

    def process_value(self, table, name, value, debug=False):
        """Recursive processing of a value to post it to NetworkTables."""
        if isinstance(value, (Pose2d, Pose3d, Translation2d)):
            self.post_pose(table, name, value, debug)
        elif isinstance(value, list):
            # Process each element in the list
            for i, sub_value in enumerate(value):
                self.process_value(table, f"{name}[{i}]", sub_value, debug)
        elif isinstance(value, dict):
            # Process each key-value pair in the dictionary
            for k, v in value.items():
                self.process_value(table, f"{name}.{k}", v, debug)
        elif isinstance(value, float):
            # Handle scalar values
            if debug:
                print(f"Scalar: {name} = {value}")
            else:
                table.putValue(name, value)
        elif isinstance(value, int):  # Handle integers (if needed)
            if debug:
                print(f"Scalar (int): {name} = {value}")
            else:
                table.putValue(name, value)

    def process_class(self, table, prefix, cls, debug=False):
        """Recursively process a class and its nested classes."""
        # cls.__init__(cls)
        for attr_name, attr_value in vars(cls).items():
            attr_path = f"{prefix}.{attr_name}" if prefix else attr_name
            if isinstance(attr_value, type):  # Handle nested classes
                nested_class_table = table.getSubTable(attr_name)
                if debug:
                    print(f"Nested class: {attr_path}")
                self.process_class(nested_class_table, attr_path, attr_value)
            else:
                self.process_value(table, attr_path, attr_value, debug)

    def process_enum(self, table, prefix, enum_class: Enum, debug=False):
        for member in enum_class:
            # print(f"\nProcessing {member.name} ({member.value[0]}):")
            for attr_name, attr_value in vars(member).items():
                if not attr_name.startswith("_") and isinstance(
                    attr_value, Pose2d | Pose3d | Translation2d
                ):  # Ignore private/magic attributes
                    attr_path = (
                        f"{prefix}.{str(member)}.{attr_name}" if prefix else attr_name
                    )
                    self.post_pose(table, attr_path, attr_value, debug)

    def update_table(self, my_cls: type | Enum, prefix: str = "", debug=False):
        """Update the NetworkTable with the class or Enum."""
        if type(my_cls) is EnumType:
            self.process_enum(self.table, prefix, my_cls, debug)
        elif isinstance(my_cls, type):
            self.process_class(self.table, prefix, my_cls, debug)
        else:
            raise TypeError("cls must be a class or Enum")


def get_red_pose(
    pose: Pose2d | Pose3d | Translation2d,
) -> Pose2d | Pose3d | Translation2d:
    if isinstance(pose, Pose2d):
        return Pose2d(
            Translation2d(
                FieldConstants.field_length - pose.X(),
                FieldConstants.field_width - pose.Y(),
            ),
            pose.rotation().rotateBy(Rotation2d(180 * degrees_to_radians)),
        )
    elif isinstance(pose, Pose3d):
        return Pose3d(
            Translation3d(
                FieldConstants.field_length - pose.X(),
                FieldConstants.field_width - pose.Y(),
                pose.Z(),
            ),
            pose.rotation().rotateBy(Rotation3d(0, 0, 180 * degrees_to_radians)),
        )
    elif isinstance(pose, Translation2d):
        return Translation2d(
            FieldConstants.field_length - pose.X(),
            FieldConstants.field_width - pose.Y(),
        )


def apply_rotation_to_pose2d(instance):
    """Iterates through all attributes of an instance,
    including class and enum attributes,
    and applies rotate_field to Pose2d attributes."""
    # print(type(instance) is EnumType)
    if type(instance) is EnumType:
        for member in instance:
            for attr_name, attr_value in vars(member).items():
                if isinstance(attr_value, Pose2d):
                    rotated_pose = get_red_pose(attr_value)
                    setattr(member, attr_name, rotated_pose)
                # elif isinstance(attr_value, type):  # If it's a class, apply to its attributes
                #     apply_rotation_to_pose2d(attr_value)
                # elif isinstance(attr_value, Enum):  # If it's an Enum, check its attributes
                #     for enum_member in attr_value:
                #         if isinstance(enum_member.value, Pose2d):
                #             rotated_pose = get_red_pose(enum_member.value)
                #             setattr(enum_member, "value", rotated_pose)

    elif isinstance(instance, type):
        for attr_name in dir(instance):
            if not attr_name.startswith("_"):  # Ignore private/magic attributes
                attr_value = getattr(instance, attr_name)
                if isinstance(attr_value, Pose2d | Pose3d | Translation2d):
                    rotated_pose = get_red_pose(attr_value)
                    setattr(instance, attr_name, rotated_pose)
                elif isinstance(
                    attr_value, type
                ):  # If it's a class, apply to its attributes
                    apply_rotation_to_pose2d(attr_value)
                elif isinstance(
                    attr_value, Enum
                ):  # If it's an Enum, check its attributes
                    for enum_member in attr_value:
                        if isinstance(enum_member, Pose2d):
                            rotated_pose = get_red_pose(enum_member)
                            setattr(enum_member, "value", rotated_pose)


def flip_poses():
    apply_rotation_to_pose2d(Branch)
    apply_rotation_to_pose2d(ReefFace)
    apply_rotation_to_pose2d(Processor)
    apply_rotation_to_pose2d(Barge)
    apply_rotation_to_pose2d(CoralStation)
    apply_rotation_to_pose2d(StagingPositions)
    apply_rotation_to_pose2d(Reef)


def update_table(nt_reporter, debug=False):
    # = NT_Updater(table_name)
    nt_reporter.update_table(Branch, "Branch", debug)
    nt_reporter.update_table(ReefFace, "ReefFace", debug)
    nt_reporter.update_table(Processor, "Processor", debug)
    nt_reporter.update_table(Barge, "Barge", debug)
    nt_reporter.update_table(CoralStation, "CoralStation", debug)
    nt_reporter.update_table(StagingPositions, "StagingPositions", debug)
    nt_reporter.update_table(Reef, "Reef", debug)


if __name__ == "__main__":
    nt_reporter = NT_Updater("Field")
    flip_poses()
    update_table(nt_reporter, True)
