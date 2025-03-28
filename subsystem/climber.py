import config
import constants

from units.SI import radians
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
from phoenix6.hardware import CANcoder
import ntcore
import math

class Climber(Subsystem):

    # Initialize class
    def __init__(self) -> None:
        super().__init__()
        self.climber_motor = TalonFX(
            can_id=config.climber_motor_id,
            config=config.climber_config,
            inverted=False
        )
        self.moving = False
        self.zeroed = False

    # Start motors
    def init(self) -> None:
        self.climber_motor.init()
        self.zero()

    def zero(self) -> None:
        self.climber_motor.set_sensor_position(0)
        self.zeroed = True

    # Set raw output of climber motor
    def set_raw_output(self, raw_value: float) -> None:
        self.climber_motor.set_raw_output(raw_value)

    # Get motor revolutions
    def get_motor_revolutions(self) -> float:
        return self.climber_motor.get_sensor_position()
        
    def update_table(self) -> None:
        table = ntcore.NetworkTableInstance.getDefault().getTable("intake")

        table.putNumber("climber_motor_revolutions", self.climber_motor.get_sensor_position())
        table.putBoolean("climber_moving", self.moving)
        table.putBoolean("climber_zeroed", self.zeroed)
        table.putNumber("climber_motor_current", self.climber_motor.get_motor_current())

    def periodic(self) -> None:
        if config.NT_CLIMBER:
            self.update_table()
