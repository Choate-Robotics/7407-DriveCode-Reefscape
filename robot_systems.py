import wpilib

import sensors
import subsystem
from toolkit.subsystem import Subsystem


class Robot:
    drivetrain = subsystem.Drivetrain()


class Pneumatics:
    pass


class Sensors:
    pass


class LEDs:
    pass


class PowerDistribution:
    pass


class Field:
    pass


# Initialize subsystems
def init_subsystems(myRobot):
    subsystems: list[Subsystem] = list(
        {
            k: v
            for k, v in myRobot.__dict__.items()
            if isinstance(v, Subsystem) and hasattr(v, "init")
        }.values()
    )

    # sensors: list = list(
    #     {k: v for k, v in Sensors.__dict__.items() if isinstance(v, sensors.Sensor) and hasattr(v, 'init')}.values()
    # )

    for subsystem in subsystems:
        subsystem.init()

    # for sensor in sensors:
    #     sensor.init()
