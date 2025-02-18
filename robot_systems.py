import subsystem
import sensors
import wpilib
import config


class Robot:
    drivetrain = subsystem.Drivetrain()
    led = subsystem.AddressableLEDStrip(
        config.leds_id, 
        config.leds_size, 
        config.leds_speed, 
        config.leds_brightness, 
        config.leds_saturation,
        config.leds_spacing, 
        config.leds_blink_frequency
        )


class Pneumatics:
    pass


class Sensors:
    pass


class PowerDistribution:
    pass


class Field:
    pass
