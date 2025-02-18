from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

import config

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup

path_name = "Three L4 Left"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(6)]
starting_pose = get_red_pose(paths[0].getStartingHolonomicPose()) if DriverStation.getAlliance() == DriverStation.Alliance.kRed else paths[0].getStartingHolonomicPose()

auto = SequentialCommandGroup(
    InstantCommand(lambda: Robot.drivetrain.reset_odometry_auto(starting_pose)),
    InstantCommand(lambda: Robot.wrist.set_coral(True)),
    
    Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
    FeedOut(Robot.wrist).withTimeout(.2),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[1]),
        Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
    ),

    AutoBuilder.followPath(paths[2]),
    Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
    FeedOut(Robot.wrist).withTimeout(.2),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[3]),
        Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator),
    ),

    AutoBuilder.followPath(paths[4]),
    Target(config.target_positions["L4"], Robot.wrist, Robot.elevator),
    FeedOut(Robot.wrist).withTimeout(.2),


)