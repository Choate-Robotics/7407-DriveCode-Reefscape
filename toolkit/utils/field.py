from units.SI import inches_to_meters
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Pose3d
from enum import Enum

class ReefHeight(Enum):
    L4 = (72*inches_to_meters, -90)
    L3 = (47.625*inches_to_meters, -35)
    L2 = (31.875*inches_to_meters, -35)
    L1 = (18*inches_to_meters, 0)

    def __init__(self, height, pitch):
        self.height = height
        self.pitch = pitch #in degrees

class FieldConstants:
    fieldLength = 690.876*inches_to_meters
    fieldWidth = 317*inches_to_meters
    startingLineX = 299.438*inches_to_meters

    class Processor:
        centerFace = Pose2d(235.726*inches_to_meters, 0, Rotation2d.fromDegrees(90))

    class Barge:
        farCage = Translation2d(345.428*inches_to_meters, 286.779*inches_to_meters)
        middleCage = Translation2d(345.428*inches_to_meters, 242.855*inches_to_meters)
        closeCage = Translation2d(345.428*inches_to_meters, 199.947*inches_to_meters)

        deepHeight = 3.125*inches_to_meters
        shallowHeight = 30.125*inches_to_meters

    class CoralStation:
        leftCenterFace = Pose2d(33.526*inches_to_meters, 291.176*inches_to_meters, Rotation2d.fromDegrees(90-144.011))
        rightCenterFace = Pose2d(33.526*inches_to_meters, 25.824*inches_to_meters, Rotation2d.fromDegrees(144.011-90))

    class Reef:
        center = Translation2d(176.746*inches_to_meters, 158.501*inches_to_meters)
        faceToZoneLine = 12*inches_to_meters #side of reef to inside of reef zone line
        centerFaces = []
        centerFaces[0] = Pose2d(144.003*inches_to_meters, 158.5*inches_to_meters, Rotation2d.fromDegrees(180))
        centerFaces[1] = Pose2d(160.373*inches_to_meters, 186.857*inches_to_meters, Rotation2d.fromDegrees(120))
        centerFaces[2] = Pose2d(193.116*inches_to_meters, 186.858*inches_to_meters, Rotation2d.fromDegrees(60))
        centerFaces[3] = Pose2d(209.489*inches_to_meters, 158.502*inches_to_meters, Rotation2d.fromDegrees(0))
        centerFaces[4] = Pose2d(193.118*inches_to_meters, 130.145*inches_to_meters, Rotation2d.fromDegrees(-60))
        centerFaces[5] = Pose2d(160.375*inches_to_meters, 130.144*inches_to_meters, Rotation2d.fromDegrees(-120))

        for face in range(6):