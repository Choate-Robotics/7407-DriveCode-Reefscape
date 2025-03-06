from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *
import config

from autos import AutoRoutine

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup, WaitCommand, ParallelRaceGroup, ConditionalCommand, ParallelDeadlineGroup

path_name = "Four L4 Right"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(9)]

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
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[4]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        SequentialCommandGroup(
            IntakeCoral(Robot.intake, Robot.wrist).withTimeout(1).unless(lambda: Robot.wrist.coral_in_feed),
            Target(config.target_positions["L4"], Robot.wrist, Robot.elevator).onlyIf(lambda: Robot.wrist.coral_in_feed)
        )
    ),
    FeedOut(Robot.wrist).withTimeout(.3).onlyIf(lambda: Robot.wrist.coral_in_feed),
    ParallelDeadlineGroup(
        SequentialCommandGroup(
            AutoBuilder.followPath(paths[5]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
            WaitCommand(0.3),
            AutoBuilder.followPath(paths[6]),
        ),
        SequentialCommandGroup(
            Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
            IntakeCoral(Robot.intake, Robot.wrist),
            Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
        )
    ),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[7]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        SequentialCommandGroup(
            IntakeCoral(Robot.intake, Robot.wrist).withTimeout(1).unless(lambda: Robot.wrist.coral_in_feed),
            Target(config.target_positions["L4"], Robot.wrist, Robot.elevator).onlyIf(lambda: Robot.wrist.coral_in_feed)
        )
    ),
    FeedOut(Robot.wrist).withTimeout(.3).onlyIf(lambda: Robot.wrist.coral_in_feed),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[8]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator).andThen(IntakeCoral(Robot.intake, Robot.wrist)),   
    )
)

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())