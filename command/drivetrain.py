from __future__ import annotations

import wpilib
import ntcore

from robot_systems import Field
from toolkit.command import SubsystemCommand

import config
import constants
from subsystem import Drivetrain
from toolkit.utils.toolkit_math import bounded_angle_diff
from wpimath.units import seconds
from enum import Enum
import logging
import math
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpilib import DriverStation

from units.SI import radians, meters_to_inches


def deadzone(x, d=config.drivetrain_deadzone):
    if abs(x) < d:
        return 0
    if x < 0:
        return (x + d) / (1 - d)
    return (x - d) / (1 - d)


def curve(x, d=config.drivetrain_deadzone, c=config.drivetrain_curve):
    if abs(x) < d:
        return 0
    elif x < 0:
        return -1 * math.pow((-1 * (x + d) / (1 - d)), c)
    return math.pow(((x - d) / (1 - d)), c)


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
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("Drivetrain Command")
        self.table.putBoolean("DriveSwerveCustom", True)

    def execute(self) -> None:
        self.subsystem.aligned=False
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
            self.subsystem.set_robot_centric((dy, -dx, d_theta))

    def end(self, interrupted: bool) -> None:
        self.table.putBoolean("DriveSwerveCustom", False)
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class DriveSwerveAim(SubsystemCommand[Drivetrain]):
    """
    aim drivetrain at a given angle (radians)
    """

    driver_centric = False
    driver_centric_reversed = True

    def __init__(
        self,
        subsystem: Drivetrain,
        target_angle: radians
    ):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.angle = target_angle
        self.theta_pid_controller = PIDController(
            config.drivetrain_rotation_kp,
            config.drivetrain_rotation_ki,
            config.drivetrain_rotation_kd,
            config.period,
        )
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("Aim Drivetrain")

    def initialize(self) -> None:
        self.theta_pid_controller.enableContinuousInput(radians(-180), radians(180))
        self.theta_pid_controller.reset()
        self.theta_pid_controller.setSetpoint(self.angle)

    def execute(self) -> None:
        self.table.putNumber("target angle", math.degrees(self.angle))
        dx, dy = (
            self.subsystem.axis_dx.value * -1,
            self.subsystem.axis_dy.value * 1
        )

        current = self.subsystem.get_heading().radians()
        d_theta = self.theta_pid_controller.calculate(current)

        dx = curve(dx)
        dy = curve(dy)
        dx *= self.subsystem.max_vel
        dy *= self.subsystem.max_vel

        if config.driver_centric:
            self.subsystem.set_driver_centric((dy, dx), d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx, d_theta))

        self.table.putNumber("current angle", math.degrees(current))

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

class DrivetrainZero(SubsystemCommand[Drivetrain]):
    """
    Zeroes drivetrain
    """

    def __init__(self, subsystem: Drivetrain, angle: radians = config.drivetrain_zero):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.angle = angle

    def initialize(self) -> None:
        print("ZEROING DRIVETRAIN")
        if (self.angle == config.drivetrain_zero) and (
            DriverStation.getAlliance() == DriverStation.Alliance.kRed
        ):
            self.angle = bounded_angle_diff((self.angle + math.pi), 0)
        self.subsystem.reset_gyro(self.angle)
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


class DrivetrainXMode(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.x_mode()

    def execute(self) -> None:
        pass

    def isFinished(self):
        return False

    def end(self, interrupted: bool) -> None:
        pass


class DriveToPose(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, poses: list[Pose2d] = None):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.poses = poses
        self.current_pose: Pose2d

        self.x_controller = PIDController(
            config.drivetrain_x_kp,
            config.drivetrain_x_ki,
            config.drivetrain_x_kd,
            config.period,
        )
        self.y_controller = PIDController(
            config.drivetrain_y_kp,
            config.drivetrain_y_ki,
            config.drivetrain_y_kd,
            config.period,
        )
        self.theta_controller = PIDController(
            config.drivetrain_rotation_kp,
            config.drivetrain_rotation_ki,
            config.drivetrain_rotation_kd,
            config.period,
        )

        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("drive to pose")

    def initialize(self):
        self.subsystem.aligned=False
        self.current_pose = self.subsystem.get_estimated_pose()
        pose = self.current_pose.nearest(self.poses)

        self.theta_controller.enableContinuousInput(0, math.radians(360))

        self.x_controller.setTolerance(config.drivetrain_x_tolerance)
        self.y_controller.setTolerance(config.drivetrain_y_tolerance)
        self.theta_controller.setTolerance(
            math.radians(config.drivetrain_rotation_tolerance)
        )

        self.x_controller.setSetpoint(pose.X())
        self.y_controller.setSetpoint(pose.Y())
        self.theta_controller.setSetpoint(pose.rotation().radians())

    def execute(self):
        self.current_pose = self.subsystem.get_estimated_pose()

        vx = self.x_controller.calculate(self.current_pose.X())
        vy = self.y_controller.calculate(self.current_pose.Y())
        vtheta = self.theta_controller.calculate(self.current_pose.rotation().radians())

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            vx *= -1
            vy *= -1

        self.subsystem.set_driver_centric((vx, vy), vtheta)

        self.nt.putNumber("goal x", self.x_controller.getSetpoint())
        self.nt.putNumber("goal y", self.y_controller.getSetpoint())
        self.nt.putNumber(
            "goal theta", math.degrees(self.theta_controller.getSetpoint())
        )

        self.nt.putNumber("current x", self.current_pose.X())
        self.nt.putNumber("current y", self.current_pose.Y())
        self.nt.putNumber(
            "current theta",
            math.degrees(bounded_angle_diff(self.current_pose.rotation().radians(), 0)),
        )

    def isFinished(self) -> bool:
        self.subsystem.aligned= (
            self.x_controller.atSetpoint()
            and self.y_controller.atSetpoint()
            and self.theta_controller.atSetpoint()
        )
        return self.subsystem.aligned

    def end(self, interrupted):
        self.subsystem.set_driver_centric((0, 0), 0)


class FindWheelRadius(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("Wheel Radius")

    def initialize(self):
        self.subsystem.n_front_left.m_move.set_sensor_position(0)
        self.subsystem.n_front_right.m_move.set_sensor_position(0)
        self.subsystem.n_back_left.m_move.set_sensor_position(0)
        self.subsystem.n_back_right.m_move.set_sensor_position(0)
        self.subsystem.gyro.reset_angle()

        self.subsystem.set_robot_centric((0, 0, math.radians(36)))

    def execute(self):
        pass

    def isFinished(self):
        return (self.subsystem.gyro._gyro.get_yaw().value >= 360) | (
            self.subsystem.gyro._gyro.get_yaw().value <= -360
        )

    def end(self, interrupted: bool):
        self.subsystem.set_robot_centric((0, 0, 0))
        rotations = [
            abs(self.subsystem.n_front_left.m_move.get_sensor_position()),
            abs(self.subsystem.n_front_right.m_move.get_sensor_position()),
            abs(self.subsystem.n_back_left.m_move.get_sensor_position()),
            abs(self.subsystem.n_back_right.m_move.get_sensor_position()),
        ]
        average = sum(rotations) / 4
        if average > 0:
            self.nt.putNumber(
                "diameter",
                abs(
                    constants.drivetrain_radius
                    * self.subsystem.gyro._gyro.get_yaw().value
                    / 360
                    / average
                    * meters_to_inches
                    * constants.drivetrain_wheel_gear_ratio
                    * 2
                ),
            )
