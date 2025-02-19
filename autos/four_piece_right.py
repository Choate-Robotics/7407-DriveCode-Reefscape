from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand

path_name = "Four L4 Right"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(6)]
starting_pose = get_red_pose(paths[0].getStartingHolonomicPose()) if DriverStation.getAlliance() == DriverStation.Alliance.kRed else paths[0].getStartingHolonomicPose()

auto = SequentialCommandGroup(
    InstantCommand(lambda: Robot.drivetrain.reset_odometry_auto(starting_pose)),
    AutoBuilder.followPath(paths[0]),
    AutoBuilder.followPath(paths[1]),
    AutoBuilder.followPath(paths[2]),
    AutoBuilder.followPath(paths[3]),
    AutoBuilder.followPath(paths[4]),
    AutoBuilder.followPath(paths[5]),
    AutoBuilder.followPath(paths[6])
)