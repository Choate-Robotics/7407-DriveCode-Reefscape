from toolkit.subsystem import Subsystem
from toolkit.motors.ctre_motors import TalonFX

import config

class Elevator(Subsystem):
    def _init_(self):
        super._init_()
        self.leader_motor: TalonFX = TalonFX(
            config.elevator_lead_id,
            config.foc_active,
            inverted=False,
        )
        self.follower_motor: TalonFX = TalonFX(
            config.elevator_follower_id,
            config.foc_active,
        )
    
    def init(self):
        self.leader_motor.init()
        self.follower_motor.init()
        self.follower_motor.follow(self.leader_motor, inverted=True)

    def set_position(self, height):
        """
        sets the elevator to a specific height
        """
        self.leader_motor.set_target_position(height)

    def set_zero(self):
        """
        brings the elevator down to the lowest position
        """
        pass

    def get_position(self): 
        """
        gets the current height of the elevator
        """
        self.follower_motor.get_sensor_position()

    def zero(self):
        """
        zero the elevator
        """
        pass