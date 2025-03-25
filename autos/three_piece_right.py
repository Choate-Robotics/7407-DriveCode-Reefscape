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

    # Start to Waypoint
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[0]),
        Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
    ),

    # Waypoint to first L4
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[1]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
    ),
    FeedOut(Robot.wrist).withTimeout(.3),
    ParallelCommandGroup(
        # First L4 to Station
        AutoBuilder.followPath(paths[2]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        SequentialCommandGroup(
            WaitCommand(0.2),
            Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator)
        )
    ),
    ParallelCommandGroup(
        ParallelCommandGroup(
            WaitCommand(0.3),
            # Station to Waypoint
            AutoBuilder.followPath(paths[3]),
            # Waypoint to second L4
            AutoBuilder.followPath(paths[4]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0)))
        ),
        ParallelCommandGroup(
            IntakeCoral(Robot.intake, Robot.wrist).withTimeout(1),
            Target(config.target_positions["L4"], Robot.wrist, Robot.elevator).onlyIf(lambda: Robot.wrist.coral_in_feed)
        )
    ),
    FeedOut(Robot.wrist).withTimeout(.3).onlyIf(lambda: Robot.wrist.coral_in_feed),
    ParallelCommandGroup(
        # Second L4 to Station
        AutoBuilder.followPath(paths[2]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        SequentialCommandGroup(
            WaitCommand(0.2),
            Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator)
        )
    ),
    ParallelCommandGroup(
        ParallelCommandGroup(
            WaitCommand(0.3),
            # Station to Waypoint
            AutoBuilder.followPath(paths[6]),
            # Waypoint to Third L4
            AutoBuilder.followPath(paths[7]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0)))
        ),
        ParallelCommandGroup(
            IntakeCoral(Robot.intake, Robot.wrist).withTimeout(1),
            Target(config.target_positions["L4"], Robot.wrist, Robot.elevator).onlyIf(lambda: Robot.wrist.coral_in_feed)
        )
    ),
    FeedOut(Robot.wrist).withTimeout(.3).onlyIf(lambda: Robot.wrist.coral_in_feed),
    ParallelCommandGroup(
        # Third L4 to Station
        AutoBuilder.followPath(paths[8]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator).andThen(IntakeCoral(Robot.intake, Robot.wrist)),   
    )
)
auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())