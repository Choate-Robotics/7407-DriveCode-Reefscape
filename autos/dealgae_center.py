from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

import config

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *

from autos import AutoRoutine

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup, ParallelDeadlineGroup, WaitCommand, ConditionalCommand

path_name = "Center L4 2 Barge Algae"
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
        AutoBuilder.followPath(paths[1]),
        Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
    ),
    FeedOut(Robot.wrist).withTimeout(.3),
    ParallelCommandGroup(
        Target(config.target_positions["DEALGAE_LOW"], Robot.wrist, Robot.elevator),
        # Move back to de-algae
        AutoBuilder.followPath(paths[2])
    ),
    ParallelDeadlineGroup(
        # Move in and out while de-algae
        AutoBuilder.followPath(paths[3]),
        WristAlgaeIn(Robot.wrist)
    ),
    ParallelCommandGroup(
        Target(config.target_positions["DEALGAE_HIGH"], Robot.wrist, Robot.elevator),
        # Move back to next algae
        AutoBuilder.followPath(paths[4]),
    ),
    ParallelDeadlineGroup(
        # Move in and out while de-algae
        AutoBuilder.followPath(paths[5]),
        WristAlgaeIn(Robot.wrist)
    ),
    ParallelCommandGroup(
        Target(config.target_positions["DEALGAE_HIGH"], Robot.wrist, Robot.elevator),
        # Move back to next algae
        AutoBuilder.followPath(paths[6]),
    ),
    ParallelDeadlineGroup(
        # Move in and out while de-algae
        AutoBuilder.followPath(paths[7]),
        WristAlgaeIn(Robot.wrist)
    ),
    )

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())