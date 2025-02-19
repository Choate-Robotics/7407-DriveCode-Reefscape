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
        self.climber_encoder.set_position(config.climber_encoder_zero)
        self.zeroed = True

    def get_angle(self) -> radians:
        return (self.climber_encoder.get_absolute_position().value - config.climber_encoder_zero) * 2*math.pi
        
    # Set raw output of climber motor
    def set_raw_output(self, raw_value: float) -> None:
        self.climber_motor.set_raw_output(raw_value)

    # Recieve voltage of motor output
    def get_raw_output(self) -> float:
        return self.climber_motor.get_applied_output()
        