import config
import constants
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
import ntcore

import math
from math import pi

from wpilib import Timer


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor: TalonFX = TalonFX(
            config.intake_id,
            config.foc_active,
            inverted=False,
            # config=config.INTAKE_CONFIG,
        )
        self.pivot_motor: TalonFX = TalonFX(
            config.intake_pivot_id,
            config.foc_active,
            inverted=False,
            config=config.INTAKE_PIVOT_CONFIG,
        )
        self.intake_running: bool = False

        self.pivot_angle = math.radians(90)
        self.intake_pivoting: bool = False
        self.intake_up: bool = True # True is up, False is down
        self.target_intake_position: bool = True # True is up, False is down

        self.timer = Timer()

    def init(self):
        self.motor.init()
        self.pivot_motor.init()

    def roll_in(self) -> None:
        """
        spin the motors inwards to collect the corral
        """
        self.motor.set_raw_output(config.intake_speed * constants.intake_gear_ratio)
        self.intake_running = True

    def stop(self) -> None:
        """
        stop the motors
        """
        self.motor.set_raw_output(0)
        self.intake_running = False

    def roll_out(self) -> None:
        """
        eject coral in the intake
        """
        self.motor.set_raw_output(-config.intake_speed * constants.intake_gear_ratio)
        self.intake_running = True

    def get_current(self) -> float:
        return self.motor.get_motor_current()
    
    def is_pivot_up(self) -> bool:
        "returns the angle in radians of the pivot"
        self.pivot_angle = (
            self.pivot_motor.get_sensor_position() / constants.intake_pivot_gear_ratio
            * pi
            * 2
            )
        self.intake_up = not self.pivot_angle < config.intake_max_angle - config.intake_angle_threshold

        return self.intake_up
    
    def pivot_up(self) -> None:
        """
        pivot the intake up
        """
        self.pivot_motor.set_target_position(
            (config.intake_max_angle / 2*math.pi ) * constants.intake_pivot_gear_ratio
        )
        self.intake_up = True

    def pivot_down(self) -> None:
        """
        pivot the intake down
        """
        self.pivot_motor.set_target_position(
            (config.intake_min_angle / 2*math.pi ) * constants.intake_pivot_gear_ratio
        )
        self.intake_up = False

    def update_table(self) -> None:
        table = ntcore.NetworkTableInstance.getDefault().getTable('intake')

        table.putBoolean('intake running', self.intake_running)
        table.putNumber('outer current', self.get_current())
        table.putNumber('pivot angle', self.pivot_angle)
        table.putBoolean('pivot moving', self.intake_pivoting)
    
    def periodic(self) -> None:
        if config.NT_INTAKE:
            self.update_table()