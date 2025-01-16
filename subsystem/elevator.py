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
        Brings the elevator to given height 

        Args:
            height (float): intended elevator height in meters
        """
        self.leader_motor.set_target_position(height)

    def set_zero(self):
        """
        Brings the elevator to the zero position
        """
        pass

    def get_position(self): 
        """
        Obtains the current height of the elevator

        Returns:
            return_float: current elevator height 
        """
        self.follower_motor.get_sensor_position()

    def zero(self):
        """
        Tells the robot that its position is at zero
        """
        pass