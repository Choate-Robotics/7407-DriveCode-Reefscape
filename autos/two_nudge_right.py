from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *
import config

from autos import AutoRoutine

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup, WaitCommand, ParallelRaceGroup, ConditionalCommand, ParallelDeadlineGroup

path_name = "One Nudge Right"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(5)]

command = SequentialCommandGroup(
    InstantCommand(lambda: Robot.wrist.set_coral(True)),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[0]),
        Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[1]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
    ),
    FeedOut(Robot.wrist).withTimeout(.3),
    ParallelDeadlineGroup(
        SequentialCommandGroup(
            AutoBuilder.followPath(paths[2]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
            WaitCommand(0.3),
            AutoBuilder.followPath(paths[3]),
        ),
        SequentialCommandGroup(
            WaitCommand(0.2),
            Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
            IntakeCoral(Robot.intake, Robot.wrist),
            Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
        )
    ),
    AutoBuilder.followPath(paths[4]),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[5]),
        Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator)
    ),
)

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())