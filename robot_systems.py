import wpilib  # noqa
import config
import constants
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
    ManualBranchesRed,
    ManualBranchesBlue,
    flip_poses,
    update_table,
    NT_Updater,
)

import sensors
import subsystem


class Robot:
    elevator = subsystem.Elevator()
    wrist = subsystem.Wrist()
    drivetrain = subsystem.Drivetrain()
    intake = subsystem.Intake()
    climber = subsystem.Climber()


class Pneumatics:
    pass


class Sensors:
    right_cam = sensors.PhotonCamCustom(config.right_cam_name, constants.robot_to_right_cam)
    left_cam = sensors.PhotonCamCustom(config.left_cam_name, constants.robot_to_left_cam)
    cam_controller = sensors.PhotonController([left_cam, right_cam])
    # cam_controller = None


class LEDs:
    pass


class PowerDistribution:
    pass


class Field:
    odometry = sensors.FieldOdometry(Robot.drivetrain, Sensors.cam_controller)
    field_constants = FieldConstants()
    reef_face = ReefFace
    branch = Branch
    reef_height = ReefHeight
    reef = Reef
    barge = Barge
    staging_positions = StagingPositions
    coral_station = CoralStation
    processor = Processor
    manual_branches_red = ManualBranchesRed
    manual_branches_blue = ManualBranchesBlue
    nt_reporter = NT_Updater("Field")

    @staticmethod
    def flip_poses():
        print("Flipping Pos")
        flip_poses()

    @staticmethod
    def get_branches():
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            return Field.manual_branches_red
        else:
            return Field.manual_branches_blue

    @staticmethod
    def update_field_table(debug=False):
        print("Updating Table")
        update_table(Field.nt_reporter, False)
