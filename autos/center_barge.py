from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

import config

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *

from autos import AutoRoutine

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup, ParallelDeadlineGroup, WaitCommand, ConditionalCommand

path_name = "Center Barge Auto"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(8)]

command = SequentialCommandGroup(
    InstantCommand(lambda: Robot.wrist.set_coral(True)),

    ParallelCommandGroup(
        # Start to Waypoint
        AutoBuilder.followPath(paths[0]),
        Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
    ),
    ParallelCommandGroup(
        # Waypoint to first L4
        AutoBuilder.followPath(paths[1]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
    ),
    FeedOut(Robot.wrist).withTimeout(.3),
    ParallelCommandGroup(
        # Move back to de-algae
        AutoBuilder.followPath(paths[2]),
        SequentialCommandGroup(
            WaitCommand(0.2),
            Target(config.target_positions["DEALGAE_LOW"], Robot.wrist, Robot.elevator),
        )
    ),
    ParallelDeadlineGroup(
        # Move in and out while de-algae
        AutoBuilder.followPath(paths[3]),
        WristAlgaeIn(Robot.wrist)
    ),
    ParallelCommandGroup(
        # Move back to barge algae
        AutoBuilder.followPath(paths[4]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        Target(config.target_positions["SCORE_BARGE"], Robot.wrist, Robot.elevator)
    ),
    WristAlgaeOut(Robot.wrist).withTimeout(.3),
    ParallelCommandGroup(
        # Move back to de-algae
        AutoBuilder.followPath(paths[5]),
        SequentialCommandGroup(
            WaitCommand(0.2),
            Target(config.target_positions["DEALGAE_HIGH"], Robot.wrist, Robot.elevator),
        )
    ),
    ParallelDeadlineGroup(
        # Move in and out while de-algae
        AutoBuilder.followPath(paths[6]),
        WristAlgaeIn(Robot.wrist)
    ),
    ParallelCommandGroup(
        # Move back to barge algae
        AutoBuilder.followPath(paths[7]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        SequentialCommandGroup(
            WaitCommand(0.2),
            Target(config.target_positions["SCORE_BARGE"], Robot.wrist, Robot.elevator),
        )
    ),
    WristAlgaeOut(Robot.wrist).withTimeout(.3),
    ParallelCommandGroup(
        #score second algae on barge
        AutoBuilder.followPath(paths[8]),
        SequentialCommandGroup(
            WaitCommand(0.2),
            Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator),
        )
    ),
)

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())