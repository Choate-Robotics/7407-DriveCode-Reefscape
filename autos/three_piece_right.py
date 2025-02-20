from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *
import config

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup

path_name = "Four L4 Right"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(6)]
starting_pose = get_red_pose(paths[0].getStartingHolonomicPose()) if DriverStation.getAlliance() == DriverStation.Alliance.kRed else paths[0].getStartingHolonomicPose()

auto = SequentialCommandGroup(
     InstantCommand(lambda: Robot.drivetrain.reset_odometry_auto(starting_pose)),
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
        Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator)
    ),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[3]),
        Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
    ),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[4]),
        Target(config.target_positions["L4"], Robot.wrist, Robot.elevator)
    ),
    FeedOut(Robot.wrist).withTimeout(.2),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[5]),
        Target(config.target_positions["STATION_INTAKING"], Robot.wrist, Robot.elevator)
    ),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[6]),
        Target(config.target_positions["IDLE"], Robot.wrist, Robot.elevator)
    ),

    ParallelCommandGroup(
        AutoBuilder.followPath(paths[7]),
        Target(config.target_positions["L4"], Robot.wrist, Robot.elevator)
    ),
    FeedOut(Robot.wrist).withTimeout(.2),
)