import config
import constants
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
import ntcore

import math
from math import pi
from units.SI import radians
from wpilib import Timer


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor: TalonFX = TalonFX(
            config.intake_id,
            config.foc_active,
            inverted=False,
            config=config.INTAKE_CONFIG,
        )
        self.pivot_motor: TalonFX = TalonFX(
            config.intake_pivot_id,
            config.foc_active,
            inverted=False,
            config=config.INTAKE_PIVOT_CONFIG,
        )
        self.intake_running: bool = False
        self.intake_rolling_in: bool = False
        self.intake_rolling_out: bool = False

        self.pivot_angle: radians = math.radians(90)
        self.intake_pivoting: bool = False
        self.intake_up: bool = True
        self.target_intake_position: bool = True

        self.timer = Timer()

    def init(self):
        self.motor.init()
        self.pivot_motor.init()

    def roll_in(self) -> None:
        """
        Spins intake motors inwards.

        """

        self.motor.set_raw_output(config.intake_speed * constants.intake_gear_ratio)
        self.intake_running = True
        self.intake_rolling_in = True
        self.intake_rolling_out = False

    def roll_out(self) -> None:
        """
        Spins intake motors outwards
        
        """

        self.motor.set_raw_output(-config.intake_speed * constants.intake_gear_ratio)
        self.intake_running = True
        self.intake_rolling_in = False
        self.intake_rolling_out = True

    def stop(self) -> None:
        self.motor.set_raw_output(0)
        self.intake_running = False
        self.intake_rolling_in = False
        self.intake_rolling_out = False

    def get_current(self) -> float:
        """
        Returns: 
            float: intake roller current.

        """

        return self.motor.get_motor_current()
    
    def is_pivot_up(self) -> bool:
        """
        Returns:
            bool: if the intake is pivoted up

        """
        
        self.pivot_angle = (
            self.pivot_motor.get_sensor_position() / constants.intake_pivot_gear_ratio
            * pi
            * 2
        )

        self.intake_up = not (self.pivot_angle < config.intake_max_angle - config.intake_angle_threshold)

        return self.intake_up
    
    def pivot_up(self) -> None:
        """
        Pivots the intake up into coral station intaking position.

        """

        self.pivot_motor.set_target_position(
            (config.intake_max_angle / 2 * math.pi) * constants.intake_pivot_gear_ratio
        )
        self.intake_up = True

    def pivot_down(self) -> None:
        """
        Pivots the intake down.
        
        """

        self.pivot_motor.set_target_position(
            (config.intake_min_angle / 2*math.pi ) * constants.intake_pivot_gear_ratio
        )
        self.intake_up = False

    def update_table(self) -> None:
        table = ntcore.NetworkTableInstance.getDefault().getTable('intake')

        table.putBoolean('Intake Running', self.intake_running)
        table.putNumber('Outer Current', self.get_current())
        table.putNumber('Pivot Angle', self.pivot_angle)
        table.putBoolean('Pivot Moving', self.intake_pivoting)