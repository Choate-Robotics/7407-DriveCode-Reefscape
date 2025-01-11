from units.SI import radians


class BaseGyro:
    """
    Extendable class for gyro.
    """

    def init(self, gyro_start_angle=0):
        """
        Initialize the gyro. Overridden class.
        """
        ...

    def get_robot_heading(self) -> radians:
        """
        Get the robot heading in radians. Overridden class. Must return radians.
        """
        ...

    def get_robot_pitch(self) -> radians:
        """
        Returns the angle of the robot's pitch in radians. Overridden class. Must return radians.
        :return: Robot pitch (radians)
        """
        ...

    def get_robot_roll(self) -> radians:
        """
        Returns the angle of the robot's roll in radians. Overridden class. Must return radians.
        :return: Robot roll (radians)
        """

    def reset_angle(self, angle: radians = 0):
        """
        Reset the robot heading. Overridden class.
        """
        ...
