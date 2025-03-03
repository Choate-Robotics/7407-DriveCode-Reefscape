from dataclasses import dataclass

from commands2 import Command
from wpimath.geometry import Pose2d

from utils.field import get_red_pose

@dataclass
class AutoRoutine:
    def __init__(self, command: Command, start_pose: Pose2d):
        self.command = command
        self.blue_start_pose = start_pose
        self.red_start_pose = get_red_pose(start_pose)