import config
import constants
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem

class Elevator(Subsystem):
    def _init_(self):
        super.__init__()
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
        self.rotations = (height * constants.elevator_gear_ratio) / constants.elevator_driver_gear_circumference
        self.leader_motor.set_target_position(self.rotations)

    def set_zero(self):
        """
        Brings the elevator to the zero position
        """
        self.set_position(0)

    def get_position(self):
        """
        Obtains the current height of the elevator

        Returns:
            return_float: current elevator height
        """
        return (
            self.leader_motor.get_sensor_position()
            * constants.elevator_driver_gear_circumference
            / constants.elevator_gear_ratio
        )

    def zero(self):
        """
        Tells the elevator that its position is at zero
        """
        self.leader_motor.set_sensor_position(0)

    def is_at_position(self, height):
        """
        checks if the elevator is at a certain height

        Args:
            height (float): height to be checked in meters
        """
        # Rounding to make sure it's not too precise (will cause err)
        return round(self.get_position(), 2) == round(height, 2)