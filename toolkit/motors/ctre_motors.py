from __future__ import annotations

from phoenix6 import StatusCode, StatusSignal, configs, controls, hardware, signals
from wpilib import TimedRobot

import config
from toolkit.motor import PIDMotor
from units.SI import rotations, rotations_per_second
from utils import LocalLogger
from wpilib import TimedRobot

radians_per_second_squared = float

rotations_per_second_squared = float


class TalonConfig:
    kP: float
    kI: float
    kD: float
    kF: float
    kA: float
    current_limit: float
    break_mode: bool
    output_range: tuple[float, float]

    def __init__(
        self,
        kP: float,
        kI: float,
        kD: float,
        kF: float,
        kA: float,
        kV: float = 0,
        kG: float = 0,
        current_limit: int = 80,
        brake_mode: bool = True,
        output_range: tuple[float, float] = (-1, 1),
        motion_magic_cruise_velocity=20,
        motion_magic_acceleration=600,
        motion_magic_jerk=6000,
    ):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kA = kA
        self.kV = kV
        self.kG = kG
        self.current_limit = current_limit
        self.brake_mode = brake_mode
        self.output_range = output_range
        self.motion_magic_cruise_velocity = motion_magic_cruise_velocity
        self.motion_magic_acceleration = motion_magic_acceleration
        self.motion_magic_jerk = motion_magic_jerk

    def _apply_settings(self, motor: hardware.TalonFX, inverted: bool = False):
        print("applying settings to Talon")
        talon_config = configs.TalonFXConfiguration()

        # PID
        pid = talon_config.slot0
        pid.k_p = self.kP
        pid.k_i = self.kI
        pid.k_d = self.kD
        pid.k_s = self.kF
        pid.k_a = self.kA
        pid.k_v = self.kV
        pid.k_g = self.kG

        # current limits
        current_limits_config = talon_config.current_limits
        current_limits_config.stator_current_limit = self.current_limit
        current_limits_config.stator_current_limit_enable = (
            True if self.current_limit > 0 else False
        )
        current_limits_config.supply_time_threshold = 1
        # current_limits_config.

        # brake mode
        brake_mode_config = talon_config.motor_output
        brake_mode_config.neutral_mode = (
            signals.NeutralModeValue.BRAKE
            if self.brake_mode
            else signals.NeutralModeValue.COAST
        )
        brake_mode_config.inverted = (
            signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if inverted
            else signals.InvertedValue.CLOCKWISE_POSITIVE
        )

        # motion magic
        magic = talon_config.motion_magic
        magic.motion_magic_acceleration = self.motion_magic_acceleration
        magic.motion_magic_jerk = self.motion_magic_jerk
        magic.motion_magic_cruise_velocity = self.motion_magic_cruise_velocity

        res = motor.configurator.apply(talon_config)
        if res != StatusCode.OK:
            print(res)
            print("error! config not applying")
            raise RuntimeError(f"error! config not applying| {res}")
        else:
            print("talon configured")


class TalonFX(PIDMotor):
    _motor: hardware.TalonFX

    _logger: LocalLogger

    _config: configs.TalonFXConfigurator

    _talon_config: TalonConfig = None

    _motor_pos: StatusSignal

    _motor_vel: StatusSignal

    _motor_accel: StatusSignal

    _motor_current: StatusSignal

    _motion_magic_velocity_voltage: controls.MotionMagicVelocityVoltage

    _motion_magic_voltage: controls.MotionMagicVoltage

    _duty_cycle_out: controls.DutyCycleOut

    _position_voltage: controls.PositionVoltage

    _velocity_voltage: controls.VelocityVoltage

    _foc: bool

    _initialized: bool

    _inverted: bool

    _optimized: bool

    def __init__(
            self,
            can_id: int,
            foc: bool = True,
            inverted: bool = False,
            config: TalonConfig = None,
            optimize: bool = True,
    ):
        self._inverted = inverted
        self._foc = foc
        self._can_id = can_id
        self._talon_config = config
        self._logger = LocalLogger(f'TalonFX: {can_id}')
        self._initialized = False
        self._optimized = optimize
        self.target = 0
        self.target_velocity = 0

    def init(self):
        if self._initialized:
            self._logger.warn('already initialized')
            return

        self._logger.setup('initializing')
        self._motor = hardware.TalonFX(self._can_id, 'rio')
        self._config = self._motor.configurator
        self._motor_pos = self._motor.get_position()
        self._motor_vel = self._motor.get_velocity()
        self._motor_accel = self._motor.get_acceleration()
        self._motor_current = self._motor.get_torque_current()
        if self._talon_config is not None:
            self._talon_config._apply_settings(self._motor, self._inverted)

        self.__setup_controls()

        if self._optimized:
            if self.optimize_normal_operation() == StatusCode.OK:
                self._logger.complete('optimized')

        self._initialized = True
        self._logger.complete('initialized')
        pass

    def __setup_controls(self):
        self._motion_magic_velocity_voltage = controls.MotionMagicVelocityVoltage(0)
        self._motion_magic_voltage = controls.MotionMagicVoltage(0)
        self._duty_cycle_out = controls.DutyCycleOut(0)
        self._position_voltage = controls.PositionVoltage(0)
        self._velocity_voltage = controls.VelocityVoltage(0)

    def error_check(self, status: StatusCode, message: str = ''):
        if TimedRobot.isSimulation():
            return
        if status != StatusCode.OK:
            self._logger.error(f'Error: {status} {message}')
            if config.DEBUG_MODE:
                raise RuntimeError(f'Error: {status} {message}')

    def get_sensor_position(self) -> rotations:
        self._motor_pos.refresh()
        return self._motor_pos.value

    def set_target_position(self, pos: rotations, arbFF: float = 0.0):
        self.error_check(self._motor.set_control(self._motion_magic_voltage.with_position(pos)),
                         f'target position: {pos}, arbFF: {arbFF}')
        self.target = pos

    def set_sensor_position(self, pos: rotations):
        self.error_check(self._motor.set_position(pos), f'sensor position: {pos}')

    def set_target_velocity(self, vel: rotations_per_second, accel: rotations_per_second_squared = 0):
        self.error_check(self._motor.set_control(self._motion_magic_velocity_voltage.with_velocity(vel).with_acceleration(accel)), f'target velocity: {vel}, accel: {accel}')
        self.target_velocity = vel

    def set_target_position_voltage(self, pos: rotations):
        self.error_check(self._motor.set_control(self._position_voltage.with_position(pos)), f'target position: {pos}')
        self.target = pos

    def set_target_velocity_voltage(self, vel: rotations_per_second, accel: rotations_per_second_squared = 0):
        self.error_check(self._motor.set_control(self._velocity_voltage.with_velocity(vel).with_acceleration(accel)),
                         f'target velocity: {vel}, accel: {accel}')

    def set_raw_output(self, x: float):
        self.error_check(self._motor.set_control(self._duty_cycle_out.with_output(x)), f'raw output: {x}')

    def follow(self, master: TalonFX, inverted: bool = False) -> StatusCode.OK:
        self.error_check(self._motor.set_control(controls.Follower(master._can_id, inverted)),
                         f'following {master._can_id} inverted: {inverted}')

    def get_sensor_velocity(self) -> rotations_per_second:
        self._motor_vel.refresh()
        return self._motor_vel.value

    def get_sensor_acceleration(self) -> rotations_per_second_squared:
        self._motor_accel.refresh()
        return self._motor_accel.value

    def get_motor_current(self) -> float:
        self._motor_current.refresh()
        return self._motor_current.value

    def get_applied_output(self) -> float:
        return self._motor.get_motor_voltage().value

    def get_target(self) -> rotations:
        return self.target
    
    def get_target_velocity(self) -> rotations_per_second:
        return self.target_velocity

    def get_target_velocity(self) -> rotations_per_second:
        return self.target_velocity

    def optimize_normal_operation(self, ms: int = 25) -> StatusCode.OK:
        """removes every status signal except for motor position, current, and velocty to optimize bus utilization
        Args:
            ms: (int, optional) the update frequency of the status signals (default is 25ms)
        Returns:
            StatusCode.OK: if the talon was optimized
        """
        self._motor_pos.set_update_frequency(ms)
        self._motor_vel.set_update_frequency(ms)
        self._motor_accel.set_update_frequency(ms)
        self._motor_current.set_update_frequency(ms)
        # return self._motor.optimize_bus_utilization()
