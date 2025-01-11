import math

from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from toolkit.utils.toolkit_math import rotate_vector, bounded_angle_diff
from units.SI import meters, meters_per_second, \
    radians_per_second, radians
from wpilib import TimedRobot
from toolkit.motors.ctre_motors import TalonFX
from wpilib import AnalogEncoder
import constants
import ntcore

class SwerveNode:
    """
    Swerve node class
    """
    def __init__(self, move: TalonFX, turn: TalonFX, encoder: AnalogEncoder, absolute_encoder_zeroed_pos: float, name: str):
        self.m_move = move
        self.m_turn = turn
        self.encoder = encoder
        self.absolute_encoder_zeroed_pos = absolute_encoder_zeroed_pos
        self.name = name
        self.counter: int = 0
        self.motor_reversed: bool = False
        self.motor_sensor_offset: radians = 0
        self.sim_travel_distance: meters = 0
        self.sim_motor_speed: meters_per_second = 0
        self.sim_motor_angle: radians = 0
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("Pods")

    def init(self):
        """
        Initialize the swerve node.
        """
        print(f"Initializing {self.name}", self.counter)
        self.sim_travel_distance = 0
        self.sim_motor_speed = 0
        self.sim_motor_angle = 0

        self.m_move.init()
        self.m_turn.init()
        self.counter += 1

    def zero(self):
        """
        Zeros the drivetrain

        """

        self.m_turn.set_sensor_position(0)
        abs_encoder_position: float = self.encoder.getAbsolutePosition()

        # Find the difference in current and zero absolute position
        encoder_difference: float = abs_encoder_position - self.absolute_encoder_zeroed_pos

        if encoder_difference > .5:
            encoder_difference -= 1
        elif encoder_difference < -.5:
            encoder_difference += 1

        motor_change = encoder_difference * constants.drivetrain_turn_gear_ratio
        
        ntcore.NetworkTableInstance.getDefault().getTable('swerve').putNumber('where the motor thinks it should be', motor_change)

        self.m_turn.set_sensor_position(motor_change)

        self.m_turn.set_position_duty_cycle(0)


    def set(self, vel: meters_per_second, angle_radians: radians_per_second):
        """
        Set the velocity and angle of the swerve node.

        Args:
            vel (meters_per_second): velocity of the swerve node
            angle_radians (radians_per_second): turning swerve node velocity in radians per second
        """
        
        self._set_angle(angle_radians, self.get_turn_motor_angle() + self.motor_sensor_offset)
        self.set_motor_velocity(vel if not self.motor_reversed else -vel)
        if TimedRobot.isSimulation():
            self.sim_motor_speed = vel
            self.sim_travel_distance += vel * .03
            self.sim_motor_angle = angle_radians

    def set_motor_angle(self, pos: radians):
        """
        Set the angle of the swerve node.

        Args:
            pos (radians): angle of the swerve node in radians
        """
        self.m_turn.set_position_duty_cycle(
            (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
        )

    def get_turn_motor_angle(self) -> radians:
        """
        Get the current angle of the swerve node.
        """
        return (
                (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
                * 2
                * math.pi
        )

    def get_abs(self):
        '''
        Gets the absolute encoder value. Must be overridden.
        '''
        return self.encoder.getAbsolutePosition()

    def set_motor_velocity(self, vel: meters_per_second):
        """
        Set the velocity of the swerve node. Must be overridden.
        Args:
            vel (meters_per_second): velocity of the swerve node in meters per second
        """
        rotations_per_second = (
                vel *
                constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )
        
        rotations_per_second_squared = (
            constants.drivetrain_max_accel *
            constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        ) if constants.drivetrain_max_accel > 0 else 0

        self.m_move.set_target_velocity(rotations_per_second, rotations_per_second_squared)
        

    def get_motor_velocity(self) -> meters_per_second:
        """
        Get the velocity of the swerve node. Must be overridden. Must return meters per second.
        """
        return (
                self.m_move.get_sensor_velocity()
                / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )

    def get_drive_motor_traveled_distance(self) -> meters:
        """
        Get the distance traveled by the drive motor. Must be overridden. Must return meters.
        """
        sensor_position = self.m_move.get_sensor_position()
        return (
                sensor_position
                / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )

    def get_node_position(self) -> SwerveModulePosition:
        """
        Get the position of the swerve node.

        Returns:
            SwerveModulePosition: position of the swerve node
        """
        if TimedRobot.isSimulation():
            return SwerveModulePosition(
                self.sim_travel_distance,
                Rotation2d(self.sim_motor_angle)
            )
        return SwerveModulePosition(
            self.get_drive_motor_traveled_distance(),
            Rotation2d(self.get_turn_motor_angle())
        )

    def get_node_state(self) -> SwerveModuleState:
        """
        Get the state of the swerve node.
        Returns:
            SwerveModuleState: state of the swerve node
        """
        if TimedRobot.isSimulation():
            return SwerveModuleState(
                self.sim_motor_speed,
                Rotation2d(self.sim_motor_angle)
            )
        return SwerveModuleState(
            self.get_motor_velocity(),
            Rotation2d(self.get_turn_motor_angle())
        )
    
    def get_target_angle(self) -> radians:
        return (
                (self.m_turn.get_target() / constants.drivetrain_turn_gear_ratio)
                * 2
                * math.pi
        )
    
    def get_target_velocity(self) -> meters_per_second:
        return (
                self.m_move.get_target_velocity()
                / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )

    # 0 degrees is facing right | "ethan is our FRC lord and saviour" - sid
    def _set_angle(self, target_angle: radians, initial_angle: radians):
        target_sensor_angle, flipped, flip_sensor_offset = SwerveNode._resolve_angles(target_angle, initial_angle)

        target_sensor_angle -= self.motor_sensor_offset

        if flipped:
            self.motor_reversed = not self.motor_reversed
            self.motor_sensor_offset += flip_sensor_offset

        self.set_motor_angle(target_sensor_angle)

    @staticmethod
    def _resolve_angles(target_angle: radians, initial_angle: radians) -> tuple[float, bool, float]:
        """
        :param target_angle: Target node angle
        :param initial_angle: Initial node sensor angle
        :return: (target_sensor_angle, flipped, flip_sensor_offset)
        """

        # Actual angle difference in radians
        diff = bounded_angle_diff(initial_angle, target_angle)

        # Should we flip
        if abs(diff) > 0.65 * math.pi:
            flip_sensor_offset = math.pi if diff > 0 else -math.pi
            diff -= flip_sensor_offset
            return diff + initial_angle, True, flip_sensor_offset

        return diff + initial_angle, False, 0
    
    def update_tables(self):
        self.nt.putNumber(f"{self.name} target angle", bounded_angle_diff(self.get_target_angle(), 0))
        self.nt.putNumber(f"{self.name} current angle", bounded_angle_diff(self.get_turn_motor_angle(), 0))

        self.nt.putNumber(f"{self.name} target velocity", self.get_target_velocity())
        self.nt.putNumber(f"{self.name} current velocity", self.get_motor_velocity())