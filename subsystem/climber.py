import config
import constants

from units.SI import radians
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
from phoenix6.hardware import CANcoder
import math

class Climber(Subsystem):

    # Initialize class
    def __init__(self) -> None:
        super().__init__()
        self.climber_motor = TalonFX(can_id=config.climber_motor_id, config=config.climber_config)
        self.moving = False
        self.zeroed = False
        self.climber_encoder: CANcoder = CANcoder(config.climber_encoder_id)

    # Start motors
    def init(self) -> None:
        self.climber_motor.init()

    def zero_encoder(self) -> None:
        self.climber_encoder.set_position(0)
        self.zeroed = True

    # Set raw output of climber motor
    def set_raw_output(self, raw_value: float) -> None:
        self.climber_motor.set_raw_output(raw_value)

    # Get motor revolutions
    def get_motor_revolutions(self) -> float:
        return self.climber_encoder.get_absolute_position().value
        