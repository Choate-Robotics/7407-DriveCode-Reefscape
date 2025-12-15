import ntcore
from wpilib import DigitalInput

import config
import constants
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
from units.SI import meters, meters_to_inches


class Elevator(Subsystem):
    def __init__(self):
        super().__init__()
        self.leader_motor: TalonFX = TalonFX(
            config.elevator_lead_id,
            config.foc_active,
            inverted=False,
            config=config.ELEVATOR_CONFIG
        )
        self.follower_motor: TalonFX = TalonFX(
            config.elevator_follower_id,
            config.foc_active,
            inverted=False,
            config=config.ELEVATOR_CONFIG
        )

        self.target_height: meters = 0.0
        self.elevator_moving: bool = False

    def init(self):
        self.leader_motor.init()
        self.follower_motor.init()
        self.follower_motor.follow(self.leader_motor, inverted=True)
        self.leader_motor.set_sensor_position(0)

        self.table = ntcore.NetworkTableInstance.getDefault().getTable("elevator")
        self.height_pub = self.table.getDoubleTopic("height").publish()
        self.velocity_pub = self.table.getDoubleTopic("velocity rps").publish()
        self.acceleration_pub = self.table.getDoubleTopic("acceleration rpss").publish()
        self.target_height_pub = self.table.getDoubleTopic("target height").publish()
        self.motor_lead_applied_output_pub = self.table.getDoubleTopic("motor lead applied output").publish()
        self.motor_lead_current_pub = self.table.getDoubleTopic("motor lead current").publish()
        self.motor_follow_applied_output_pub = self.table.getDoubleTopic("motor follow applied output").publish()
   
    @staticmethod
    def limit_height(height: meters) -> meters:
        """
        limits the height of the elevator to both a max and min
        """
        if height > constants.elevator_true_max:
            return constants.elevator_true_max
        elif height < 0.0:
            return 0.0
        return height

    def set_position(self, height: meters) -> None:
        """
        Brings the elevator to given height

        Args:
            height (meters): intended elevator height in meters
        """
        height = self.limit_height(height)
        self.target_height = height

        rotations = (
            height * constants.elevator_gear_ratio
        ) / constants.elevator_driver_gear_circumference
        self.leader_motor.set_target_position(rotations)

    def stop(self) -> None:
        """
        Stops the elevator
        """
        self.set_position(self.get_position())

    def set_zero(self) -> None:
        """
        Brings the elevator to the zero position
        """
        self.set_position(0)

    def get_position(self) -> meters:
        """
        Obtains the current height of the elevator

        Returns:
            return_float: current elevator height in meters
        """
        return (
            self.leader_motor.get_sensor_position()
            * constants.elevator_driver_gear_circumference
            / constants.elevator_gear_ratio
        )

    def is_at_position(self, height: meters, tolerance: meters = config.elevator_height_threshold) -> bool:
        """
        checks if the elevator is at a certain height

        Args:
            height (meters): height to be checked
        """
        return abs(self.get_position() - height) < tolerance

    def update_table(self) -> None:
        table = ntcore.NetworkTableInstance.getDefault().getTable("Elevator")

        self.height_pub.set(self.get_position() * meters_to_inches)
        self.velocity_pub.set(self.leader_motor.get_sensor_velocity())
        self.acceleration_pub.set(self.leader_motor.get_sensor_acceleration())
        self.target_height_pub.set(self.target_height * meters_to_inches)
        self.motor_lead_applied_output_pub.set(self.leader_motor.get_applied_output())
        self.motor_lead_current_pub.set(self.leader_motor.get_motor_current())
        self.motor_follow_applied_output_pub.set(self.follower_motor.get_applied_output())

    def periodic(self):
        if config.NT_ELEVATOR:
            self.update_table()