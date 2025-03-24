from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from robot_systems import Robot

from autos import AutoRoutine

from commands2 import SequentialCommandGroup, InstantCommand

path_name = "Leave"
paths = [PathPlannerPath.fromChoreoTrajectory(path_name, i) for i in range(1)]

command = SequentialCommandGroup(
    InstantCommand(lambda: Robot.wrist.set_coral(True)),
    AutoBuilder.followPath(paths[0])
)

auto = AutoRoutine(command, paths[0].getStartingHolonomicPose())