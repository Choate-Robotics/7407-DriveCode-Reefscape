from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

import config

from robot_systems import Robot, Field
from utils.field import get_red_pose
from command import *

from autos import AutoRoutine

from wpilib import DriverStation
from commands2 import SequentialCommandGroup, InstantCommand, ParallelCommandGroup, ParallelDeadlineGroup, WaitCommand, ConditionalCommand

path_name = "Four L4 Left"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(9)]

command = SequentialCommandGroup(
    InstantCommand(lambda: Robot.wrist.set_coral(True)),
    )

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())