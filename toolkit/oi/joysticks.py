from dataclasses import dataclass

import wpilib

DRIVER_CONTROLLER_ID = 0
OPERATOR_CONTROLLER_ID = 1


class Joysticks:
    joysticks: list[wpilib.Joystick] = [
        wpilib.Joystick(DRIVER_CONTROLLER_ID),
        wpilib.Joystick(OPERATOR_CONTROLLER_ID)
    ]


@dataclass
class JoystickAxis:
    """
    Wrapper for wpilib joystick button
    """
    controller_id: int
    axis_id: int

    @property
    def value(self) -> float:
        """
        Gets the value of the axis selected

        Returns:
            float: The value between -1 and 1 as a float
        """
        return Joysticks.joysticks[self.controller_id].getRawAxis(self.axis_id)
