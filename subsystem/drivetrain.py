from __future__ import annotations
from typing import overload

import math
import config
import constants

from dataclasses import dataclass
from wpilib import AnalogEncoder, TimedRobot
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import SwerveDrive4Odometry, SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, \
    SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator

from units.SI import meters, meters_per_second, \
    radians_per_second, radians, degrees_per_second__to__radians_per_second, degrees, meters_per_second_squared

from oi.keymap import Keymap
from toolkit.oi.joysticks import JoystickAxis
from toolkit.subsystem import Subsystem
from toolkit.utils import logger
from toolkit.motors.ctre_motors import TalonFX
from toolkit.sensors.gyro import Pigeon2
from toolkit.subsystem import Subsystem
from toolkit.subsystem_templates.drivetrain import (
    SwerveNode
)
import ntcore

# auto imports
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.path import DriveFeedforwards
from wpilib import DriverStation


class Drivetrain(Subsystem):
    """
    Swerve Drivetrain Extendable class. Contains driving functions.
    """
    n_front_left = SwerveNode(
        TalonFX(config.front_left_move_id, foc=config.foc_active, config=config.MOVE_CONFIG, inverted=config.front_left_move_inverted),
        TalonFX(config.front_left_turn_id, config=config.TURN_CONFIG, inverted=config.front_left_turn_inverted),
        config.front_left_encoder_port,
        absolute_encoder_zeroed_pos=config.front_left_encoder_zeroed_pos,
        name="n_front_left",
    )
    n_front_right = SwerveNode(
        TalonFX(config.front_right_move_id, foc=config.foc_active, config=config.MOVE_CONFIG, inverted=config.front_right_move_inverted),
        TalonFX(config.front_right_turn_id, config=config.TURN_CONFIG, inverted=config.front_right_turn_inverted),
        config.front_right_encoder_port,
        absolute_encoder_zeroed_pos=config.front_right_encoder_zeroed_pos,
        name="n_front_right",
    )
    n_back_left = SwerveNode(
        TalonFX(config.back_left_move_id, foc=config.foc_active, config=config.MOVE_CONFIG, inverted=config.back_left_move_inverted),
        TalonFX(config.back_left_turn_id, config=config.TURN_CONFIG, inverted=config.back_left_turn_inverted),
        config.back_left_encoder_port,
        absolute_encoder_zeroed_pos=config.back_left_encoder_zeroed_pos,
        name="n_back_left",
    )
    n_back_right = SwerveNode(
        TalonFX(config.back_right_move_id, foc=config.foc_active, config=config.MOVE_CONFIG, inverted=config.back_right_move_inverted),
        TalonFX(config.back_right_turn_id, config=config.TURN_CONFIG, inverted=config.back_right_turn_inverted),
        config.back_right_encoder_port,
        absolute_encoder_zeroed_pos=config.back_right_encoder_zeroed_pos,
        name="n_back_right",
    )
    gyro: Pigeon2 = Pigeon2(config.gyro_id)
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    track_width: meters = constants.track_width
    track_length: meters = constants.track_length
    max_vel: meters_per_second = constants.drivetrain_max_vel
    max_accel: meters_per_second_squared = constants.drivetrain_max_accel
    max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
    deadzone_velocity: meters_per_second = 0.05  # Does not run within this speed
    deadzone_angular_velocity: radians_per_second = 5 * degrees_per_second__to__radians_per_second  # Will not turn within this speed
    start_pose: Pose2d = Pose2d(0, 0, 0)  # Starting pose of the robot from wpilib Pose (x, y, rotation)
    start_angle: degrees = 0
    gyro_start_angle: radians = 0
    gyro_offset: radians = math.radians(0)

    def __init__(self):
        super().__init__()
        self.kinematics: SwerveDrive4Kinematics | None = None
        self.odometry: SwerveDrive4Odometry | None = None
        self.odometry_estimator: SwerveDrive4PoseEstimator | None = None
        self.chassis_speeds: ChassisSpeeds | None = ChassisSpeeds(0, 0, 0)
        self._omega: radians_per_second = 0
        self._sim_fl: meters_per_second = 0
        self._sim_fr: meters_per_second = 0
        self._sim_bl: meters_per_second = 0
        self._sim_br: meters_per_second = 0
        self._sim_omega: radians_per_second = 0
        self.sim_node_positions: tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition] = None
        self.sim_node_states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState] = None
        self.node_translations: tuple[Translation2d] | None = None
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("Drivetrain")

        # auto setup
        self.pp_config = constants.auto_config

        # setup autobuilder
        AutoBuilder.configure(
            self.get_estimated_pose,
            self.reset_odometry_auto,
            self.get_speeds,
            lambda spds, ffs: self.set_robot_centric(spds),
            PPHolonomicDriveController(
                config.auto_translation_pid,
                config.auto_rotation_pid,
                config.period
            ),
            self.pp_config,
            self.should_flip_path,
            self
        )

    def init(self):
        """
        Initialize the swerve drivetrain, kinematics, odometry, and gyro.
        """
        logger.info("initializing swerve drivetrain", "[swerve_drivetrain]")
        self.n_front_left.init()
        self.n_front_right.init()
        self.n_back_left.init()
        self.n_back_right.init()
        self.gyro.init(self.gyro_start_angle)

        logger.info("initializing odometry", "[swerve_drivetrain]")

        self.node_translations = (
            Translation2d(.5 * self.track_length, .5 * self.track_width),
            Translation2d(.5 * self.track_length, -.5 * self.track_width),
            Translation2d(-.5 * self.track_length, .5 * self.track_width),
            Translation2d(-.5 * self.track_length, -.5 * self.track_width)
        )

        self.kinematics = SwerveDrive4Kinematics(
            *self.node_translations
        )

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.get_heading(),
            self.node_positions,
            self.start_pose
        )
        self.odometry_estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_heading(),
            self.node_positions,
            self.start_pose
        )
        

        logger.info("initialization complete", "[swerve_drivetrain]")

    @property
    def node_positions(self) -> tuple[
        SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition
    ]:
        """
        Get the node positions.
        """
        return (
            self.n_front_left.get_node_position(),
            self.n_front_right.get_node_position(),
            self.n_back_left.get_node_position(),
            self.n_back_right.get_node_position()
        )

    @property
    def node_states(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        """
        Get the node states.
        """
        return (
            self.n_front_left.get_node_state(),
            self.n_front_right.get_node_state(),
            self.n_back_left.get_node_state(),
            self.n_back_right.get_node_state()
        )


    def set_driver_centric(self, vel: tuple[meters_per_second, meters_per_second], angular_vel: radians_per_second):
        """
        Set the driver centric velocity and angular velocity. Driver centric runs with perspective of driver.

        Args:
            vel: velocity in x and y direction as (meters per second, meters per second)
            angular_vel: angular velocity in radians per second
        """
        # vel = rotate_vector(vel[0], vel[1], -self.gyro.get_robot_heading())

        heading = self.get_heading()
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            heading = heading.rotateBy(Rotation2d(math.pi))

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vel[0], vel[1], angular_vel, heading)

        self.set_robot_centric(speeds)


    def set_robot_centric(self, speeds: ChassisSpeeds | tuple[meters_per_second, meters_per_second, radians_per_second]):
        """
        Set the robot centric velocity and angular velocity. Robot centric runs with perspective of robot.
        Args:
            vel: velocity in x and y direction as (meters per second, meters per second)
            angular_vel: angular velocity in radians per second
        """

        if isinstance(speeds, tuple):

            speeds = ChassisSpeeds(*speeds)

            
        self.chassis_speeds = speeds

        new_states = self.kinematics.toSwerveModuleStates(self.chassis_speeds)
        normalized_states = self.kinematics.desaturateWheelSpeeds(new_states, self.max_vel)
        # normalized_states = new_states
        fl, fr, bl, br = normalized_states
            
        self._sim_omega += self.chassis_speeds.omega * .03
            
        self.n_front_left.set(fl.speed, fl.angle.radians())
        self.n_front_right.set(fr.speed, fr.angle.radians())
        self.n_back_left.set(bl.speed, bl.angle.radians())
        self.n_back_right.set(br.speed, br.angle.radians())
        

        self.odometry.update(
            self.get_heading(),
            self.node_positions
        )

        # self.odometry_estimator.update(
        #     self.get_heading(),
        #     self.node_positions
        # )

        # self.chassis_speeds = self.kinematics.toChassisSpeeds(*self.node_states)

    def should_flip_path(self)-> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def get_pose(self) -> Pose2d:
        pose = self.odometry.getPose()
        return Pose2d(pose.translation(), self.get_heading())
    
    def get_estimated_pose(self) -> Pose2d:
        pose = self.odometry_estimator.getEstimatedPosition()
        return pose
    
    def get_speeds(self) -> ChassisSpeeds:
        return self.chassis_speeds

    def periodic(self):
        pass

    def stop(self):
        """
        Stop the drivetrain and all pods.
        """
        self.n_front_left.set(0, 0)
        self.n_front_right.set(0, 0)
        self.n_back_left.set(0, 0)
        self.n_back_right.set(0, 0)
        
    def reset_gyro(self, radians: float = 0):
        """
        Reset the gyro to a given angle.

        Args:
            radians (float): The angle to reset the gyro to in radians.
        """
        if TimedRobot.isSimulation():
            self._sim_omega = radians
        self.gyro.reset_angle(radians)

    def get_heading(self) -> Rotation2d:
        """
        Get the robot heading.

        Returns:
            Heading (Rotation2d): the robot heading
        """
        if TimedRobot.isSimulation():
            return Rotation2d(self._sim_omega + self.gyro_offset)
        return Rotation2d((self.gyro.get_robot_heading() + self.gyro_offset))

    def reset_odometry(self, pose: Pose2d) -> None:
        """
        Reset the odometry to a given pose.

        Args:
            pose (Pose2d): The pose to reset the odometry to.
        """
        self.odometry.resetPosition(
            gyroAngle=self.get_heading(),
            wheelPositions=self.node_positions,
            pose=pose,
        )
        self.odometry_estimator.resetPosition(
            gyroAngle=self.get_heading(),
            wheelPositions=self.node_positions,
            pose=pose,
        )
        
    def reset_odometry_auto(self, pose: Pose2d):
        """
        Reset the odometry to a given pose.

        Args:
            pose (Pose2d): The pose to reset the odometry to.
        """
        self.odometry.resetPosition(
            gyroAngle=pose.rotation(),
            wheelPositions=self.node_positions,
            pose=pose,
        )
        self.odometry_estimator.resetPosition(
            gyroAngle=pose.rotation(),
            wheelPositions=self.node_positions,
            pose=pose,
        )


    @staticmethod
    def _calculate_swerve_node(node_x: meters, node_y: meters, dx: meters_per_second, dy: meters_per_second,
                               d_theta: radians_per_second) -> tuple[meters_per_second, radians]:
        tangent_x, tangent_y = -node_y, node_x
        tangent_m = math.sqrt(tangent_x ** 2 + tangent_y ** 2)
        tangent_x /= tangent_m
        tangent_y /= tangent_m

        r = math.sqrt(2) / 2
        sx = dx + r * d_theta * tangent_x
        sy = dy + r * d_theta * tangent_y

        theta = math.atan2(sy, sx)
        magnitude = math.sqrt(sx ** 2 + sy ** 2)
        return magnitude, theta
    
    def x_mode(self):
        self.n_front_left.set(0, math.radians(45))
        self.n_front_right.set(0, math.radians(-45))
        self.n_back_left.set(0, math.radians(-45))
        self.n_back_right.set(0, math.radians(45))

    def get_abs(self):
        fl = self.n_front_left.get_abs()
        fr = self.n_front_right.get_abs()
        bl = self.n_back_left.get_abs()
        br = self.n_back_right.get_abs()
        return [fl, fr, bl, br]
    
    def update_tables(self):
        self.nt.putNumberArray("encoder poses", self.get_abs())
        n_states = self.node_states

        self.nt.putNumberArray('Node States', [
            n_states[0].angle.radians(), n_states[0].speed,
            n_states[1].angle.radians(), n_states[1].speed,
            n_states[2].angle.radians(), n_states[2].speed,
            n_states[3].angle.radians(), n_states[3].speed
        ])

        pose = self.get_pose()

        self.nt.putNumberArray("Estimated pose",[
            pose.X(),
            pose.Y(),
            pose.rotation().radians()
        ])

        self.nt.putNumber("rotation", self.get_heading().degrees())

        self.n_front_left.update_tables()
        self.n_front_right.update_tables()
        self.n_back_left.update_tables()
        self.n_back_right.update_tables()