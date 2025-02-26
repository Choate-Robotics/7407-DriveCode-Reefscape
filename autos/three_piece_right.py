from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *
import config

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup, WaitCommand, ParallelRaceGroup, ConditionalCommand, ParallelDeadlineGroup

path_name = "Four L4 Right"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(9)]
starting_pose = get_red_pose(paths[0].getStartingHolonomicPose()) if DriverStation.getAlliance() == DriverStation.Alliance.kRed else paths[0].getStartingHolonomicPose()

auto = SequentialCommandGroup(
    # InstantCommand(lambda: Robot.drivetrain.reset_odometry_auto(starting_pose)),
    InstantCommand(lambda: Robot.wrist.set_coral(True)),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[0]),
        Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
    ),
    # ParallelCommandGroup(
    AutoBuilder.followPath(paths[1]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
    Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
    # ),
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
    ParallelRaceGroup(
        SequentialCommandGroup(
            WaitCommand(0),
            Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
            IntakeCoral(Robot.intake, Robot.wrist)
        ),
        AutoBuilder.followPath(paths[5]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))).andThen(WaitCommand(0.3)),
    ),
    AutoBuilder.followPath(paths[6]).alongWith(Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[7]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
    ),
    FeedOut(Robot.wrist).withTimeout(.4),
    ParallelRaceGroup(
        SequentialCommandGroup(
            WaitCommand(0),
            Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
            IntakeCoral(Robot.intake, Robot.wrist)
        ),
        AutoBuilder.followPath(paths[8]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))).andThen(WaitCommand(0.3)),
    ),
    # AutoBuilder.followPath(paths[0]),
    # AutoBuilder.followPath(paths[1]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
    # WaitCommand(1),
    # AutoBuilder.followPath(paths[2]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
    # WaitCommand(1),
    # AutoBuilder.followPath(paths[3]),
    # AutoBuilder.followPath(paths[4]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
    # WaitCommand(1),
    # AutoBuilder.followPath(paths[5]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
    # WaitCommand(1),
    # AutoBuilder.followPath(paths[6]),
    # AutoBuilder.followPath(paths[7]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
    # WaitCommand(1),
    # AutoBuilder.followPath(paths[8]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
    # WaitCommand(1),
)