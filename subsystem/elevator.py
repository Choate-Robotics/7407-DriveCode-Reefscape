import config
import constants

import ntcore
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
from units.SI import meters

class Elevator(Subsystem):
    def _init_(self):
        super().__init__()
        self.leader_motor: TalonFX = TalonFX(
            config.elevator_lead_id,
            config.foc_active,
            inverted=False,
        )
        self.follower_motor: TalonFX = TalonFX(
            config.elevator_follower_id,
            config.foc_active,
        )

        self.zeroed: bool = False
        self.target_height: meters = 0.0

    def init(self):
        self.leader_motor.init()
        self.follower_motor.init()
        self.follower_motor.follow(self.leader_motor, inverted=True)

    def limit_height(self, height: meters):
        """
        limits the height of the elevator to both a max and min
        """
        if height > constants.elevator_max_height:
            return constants.elevator_max_height
        elif height < 0.0:
            return 0.0
        return height

    def set_position(self, height):
        """
        Brings the elevator to given height

        Args:
            height (float): intended elevator height in meters
        """
        height = self.limit_height(height)
        self.target_height = height

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
        self.zeroed = True

    def is_at_position(self, height):
        """
        checks if the elevator is at a certain height

        Args:
            height (float): height to be checked in meters
        """
        # Rounding to make sure it's not too precise (will cause err)
        return round(self.get_position(), 2) == round(height, 2)
    
    def update_network_table(self):
        table = ntcore.NetworkTableInstance.getDefault().getTable('elevator')

        table.putNumber('elevator height', self.get_position())
        table.putBoolean('elevator zeroed', self.zeroed)
        table.putNumber('elevator target height', self.target_height)
        table.putNumber('elevator motor lead applied output', self.leader_motor.get_applied_output())
        table.putNumber('elevator motor follow applied output', self.follower_motor.get_applied_output())

    def periodic(self):
        if config.NT_ELEVATOR:
            self.update_network_table()