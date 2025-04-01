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
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(7)]

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
        Target(config.target_positions["DEALGAE_LOW"], Robot.wrist, Robot.elevator),
        # Move back a bit and pick up algae low
        AutoBuilder.followPath(paths[2]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
    ),
    WristAlgaeIn(Robot.wrist),
    ParallelCommandGroup(
        # Move to barge
        AutoBuilder.followPath(paths[3]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        SequentialCommandGroup(
            WaitCommand(0.2),
            Target(config.target_positions["SCORE_BARGE"], Robot.wrist, Robot.elevator),
        )
    ),
    WristAlgaeOut(Robot.wrist).withTimeout(0.3),
    ParallelCommandGroup(
        # Move to the algae high
        AutoBuilder.followPath(paths[4]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        SequentialCommandGroup(
            WaitCommand(0.2),
            Target(config.target_positions["DEALGAE_HIGH"], Robot.wrist, Robot.elevator),
        )
    ),
    WristAlgaeIn(Robot.wrist),
    ParallelCommandGroup(
        # Move to barge
        AutoBuilder.followPath(paths[5]).andThen(InstantCommand(lambda: Robot.drivetrain.set_driver_centric((0, 0), 0))),
        SequentialCommandGroup(
            WaitCommand(0.2),
            Target(config.target_positions["SCORE_BARGE"], Robot.wrist, Robot.elevator),
        )
    ),  
    WristAlgaeOut(Robot.wrist).withTimeout(0.3),
    AutoBuilder.followPath(paths[6])
)

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())