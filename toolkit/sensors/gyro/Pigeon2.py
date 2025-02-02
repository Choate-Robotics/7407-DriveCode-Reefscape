import phoenix6
import math

from units.SI import radians, radians_per_second
from toolkit.sensors.gyro.base_gyro import BaseGyro


class Pigeon2(BaseGyro):
    """
    Wrapper class for the Pigeon2 IMU gyro.
    """

    def __init__(self, port):
        """
        Args:
            port (int): CAN ID of the Pigeon gyro
        """
        self._gyro = phoenix6.hardware.Pigeon2(port)


    def init(self, gyro_start_angle=0):
        """
        Initialize gyro
        """
        self.reset_angle(gyro_start_angle)

    def get_robot_heading(self) -> radians:
        """
        Returns the angle of the robot's heading in radians (yaw)
        :return: Robot heading (radians)
        """
        return math.radians(self._gyro.get_yaw().value)
    
    def get_robot_heading_rate(self) -> radians_per_second:
        """
        Returns the rate of the robot's heading in radians per second
        :return: Robot heading (radians)
        """
        return math.radians(self._gyro.get_angular_velocity_z_world().value)

    def get_robot_pitch(self) -> radians:
        """
        Returns the angle of the robot's pitch in radians
        :return: Robot pitch (radians)
        """
        return math.radians(self._gyro.get_pitch().value)
    
    def get_robot_pitch_rate(self) -> radians_per_second:
        """
        Returns the rate of the robot's pitch in radians per second
        :return: Robot pitch rate (radians per second)
        """
        return math.radians(self._gyro.get_angular_velocity_y_world().value)

    def get_robot_roll(self) -> radians:
        """
        Returns the angle of the robot's roll in radians
        :return: Robot roll (radians)
        """
        return math.radians(self._gyro.get_roll().value)
    
    def get_robot_roll_rate(self) -> radians_per_second:
        """
        Returns the rate of the robot's roll in radians per second
        :return: Robot roll (radians)
        """
        return math.radians(self._gyro.get_angular_velocity_x_world().value)

    def reset_angle(self, angle: radians = 0):
        """
        Resets the gyro's yaw.
        """
        self._gyro.set_yaw(math.degrees(angle))
        
    def get_x_accel(self):
        
        return self._gyro.get_acceleration_x().value
    
    def get_y_accel(self):
        
        return self._gyro.get_acceleration_y().value
    
    def get_z_accel(self):
        
        return self._gyro.get_acceleration_z().value
    
