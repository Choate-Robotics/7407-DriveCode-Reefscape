import config
import constants

from units.SI import radians
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
import math

class Climber(Subsystem):

    # Initialize class
    def __init__(self) -> None:
        super().__init__()
        self.climber_motor = TalonFX(can_id=config.climber_motor_id, config=config.climber_config)
        self.climber_motor_follower = TalonFX(can_id=config.climber_motor_follower_id, config=config.climber_config)

        self.zeroed = False
        self.moving = False

    # Start motors
    def init(self) -> None:
        self.climber_motor.init()
        self.climber_motor_follower.init()
        self.climber_motor_follower.follow(self.climber_motor, inverted=True)

    # Zero the climber
    def zero(self) -> None:
        self.climber_motor.set_target_position(self.climber_motor.get_sensor_position() * constants.climber_gear_ratio)
        self.climber_motor.set_sensor_position(0) #TODO: Temp value
        self.zeroed = True

    # Set a custom angle for the climber
    def set_angle(self, angle: radians) -> None:
        self.climber_motor.set_target_position(max(min(angle, math.radians(constants.upper_climber_bound)), math.radians(constants.lower_climber_bound)) * constants.climber_gear_ratio)
        self.moving = True

    # Compare current sensor position with target position
    def check_angle(self, angle: radians) -> bool:

        if self.arm_moving and round((self.climber_motor.get_sensor_position() / constants.climber_gear_ratio), 2) == round(angle, 2):
            self.arm_moving = False
            return True
        else:
            return False
        
    def get_angle(self) -> float:
        return self.climber_motor.get_sensor_position() / constants.climber_gear_ratio
        
    # Set raw output of climber motor
    def set_raw_output(self, raw_value: float) -> None:
        self.climber_motor.set_raw_output(raw_value)
        self.climber_motor.set_raw_output(raw_value)

    # Recieve voltage of motor output
    def get_raw_output(self) -> float:
        return self.climber_motor.get_applied_output()
        