import wpilib  # noqa

import sensors  # noqa
import subsystem
from utils.field import (
    Barge,
    Branch,
    CoralStation,
    FieldConstants,
    NT_Updater,
    Processor,
    Reef,
    ReefFace,
    ReefHeight,
    StagingPositions,
    flip_poses,
    update_table,
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
