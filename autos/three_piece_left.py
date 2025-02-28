from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

import config

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *

from autos import AutoRoutine

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup, ParallelDeadlineGroup, WaitCommand, ConditionalCommand

path_name = "Four L4 Left"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(9)]

command = SequentialCommandGroup(
    InstantCommand(lambda: Robot.wrist.set_coral(True)),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[0]),
        Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[1]),
        Target(config.target_positions["L4"], Robot.wrist, Robot.elevator)
    ),
    FeedOut(Robot.wrist).withTimeout(.4),
    ParallelDeadlineGroup(
        SequentialCommandGroup(
            AutoBuilder.followPath(paths[2]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
            WaitCommand(0.3),
            AutoBuilder.followPath(paths[3]),
        ),
        SequentialCommandGroup(
            WaitCommand(0.3),
            Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
            IntakeCoral(Robot.intake, Robot.wrist),
            Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
        )
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[4]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        ConditionalCommand(
            Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
            WaitCommand(0),
            lambda: Robot.wrist.coral_in_feed
        )
    ),
    ConditionalCommand(
        FeedOut(Robot.wrist).withTimeout(.4),
        WaitCommand(0),
        lambda: Robot.wrist.coral_in_feed
    ),
    ParallelDeadlineGroup(
        SequentialCommandGroup(
            AutoBuilder.followPath(paths[5]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
            WaitCommand(0.3),
            AutoBuilder.followPath(paths[6]),
        ),
        SequentialCommandGroup(
            WaitCommand(0.3),
            Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
            IntakeCoral(Robot.intake, Robot.wrist),
            Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
        )
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[7]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        ConditionalCommand(
            Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
            WaitCommand(0),
            lambda: Robot.wrist.coral_in_feed
        )
    ),
    ConditionalCommand(
        FeedOut(Robot.wrist).withTimeout(.4),
        WaitCommand(0),
        lambda: Robot.wrist.coral_in_feed
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[8]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
    ),
    IntakeCoral(Robot.intake, Robot.wrist),
)

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())