from __future__ import annotations

import wpilib

from robot_systems import Field
from toolkit.command import SubsystemCommand

import config
from subsystem import Drivetrain
from toolkit.utils.toolkit_math import bounded_angle_diff
from math import radians
from wpimath.units import seconds
from enum import Enum
import logging
import math

def deadzone(x, d=config.drivetrain_deadzone):
    if abs(x) < d:
        return 0
    if x < 0:
        return (x+d)/(1-d)
    return (x-d)/(1-d)

def curve(x, d=config.drivetrain_deadzone, c=config.drivetrain_curve):
    if abs(x) < d:
        return 0
    elif x < 0:
        return -1*math.pow((-1*(x+d)/(1-d)), c)
    return math.pow(((x-d)/(1-d)), c)

def bound_angle(degrees: float):
    degrees = degrees % 360
    if degrees > 180:
        degrees -= 360
    return degrees


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    """
    Main drive command
    """
    driver_centric = False
    driver_centric_reversed = True

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        dx, dy, d_theta = (
            self.subsystem.axis_dx.value * -1,
            self.subsystem.axis_dy.value * 1,
            self.subsystem.axis_rotation.value,
        )

        dx = deadzone(dx)
        dy = deadzone(dy)
        d_theta = curve(d_theta)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel
        
        d_theta *= self.subsystem.max_angular_vel
        

        if config.driver_centric:
            self.subsystem.set_driver_centric((dy, dx), -d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False

class DrivetrainZero(SubsystemCommand[Drivetrain]):
    """
    Zeroes drivetrain
    """

    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        print("ZEROING DRIVETRAIN")
        self.subsystem.reset_gyro(config.drivetrain_zero)
        self.subsystem.n_front_left.zero()
        self.subsystem.n_front_right.zero()
        self.subsystem.n_back_left.zero()
        self.subsystem.n_back_right.zero()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        logging.info("Successfully re-zeroed swerve pods.")
        ...