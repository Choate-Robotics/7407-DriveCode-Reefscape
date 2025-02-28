from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

import config

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *

from autos import AutoRoutine

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup

path_name = "One L4 Right"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(2)]

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
    FeedOut(Robot.wrist).withTimeout(.2),
    ParallelCommandGroup(
        AutoBuilder.followPath(paths[2]),
        Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
    ),
)

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())