import subsystem
import sensors
import wpilib


class Robot:
    drivetrain = subsystem.Drivetrain()
    wrist = subsystem.Wrist()
    intake = subsystem.Intake()
    elevator = subsystem.Elevator()
    climb = subsystem.Climb()

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
