import subsystem
import sensors  # noqa
import wpilib  # noqa
from utils.field import (
    FieldConstants,
    ReefFace,
    ReefHeight,
    Branch,
    Reef,
    Barge,
    StagingPositions,
    CoralStation,
    Processor,
    flip_poses,
    update_table,
    NT_Updater,
)


class Robot:
    drivetrain = subsystem.Drivetrain()


class Pneumatics:
    pass


class Sensors:
    pass


class LEDs:
    pass


class PowerDistribution:
    pass


class Field:
    field_constants = FieldConstants()
    reef_face = ReefFace
    branch = Branch
    reef_height = ReefHeight
    reef = Reef
    barge = Barge
    staging_positions = StagingPositions
    coral_station = CoralStation
    processor = Processor
    nt_reporter = NT_Updater("Field")

    @staticmethod
    def flip_poses():
        print("Flipping Pos")
        flip_poses()

    @staticmethod
    def update_field_table(debug=False):
        print("Updating Table")
        update_table(Field.nt_reporter, False)
