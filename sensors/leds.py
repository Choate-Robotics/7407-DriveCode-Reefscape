from wpilib import AddressableLED, PowerDistribution, SmartDashboard
import math, config

class ALeds:

    """
    Addressable LEDs

    """

    def __init__(self, 
            id: int, 
            size: int, 
            speed: int, 
            brightness: int, 
            saturation: int,
            spacing: int, 
            blink_frequency: int,
            rightlimit, 
            leftlimit
        ):

        self.size = size
        self.id = id
        self.speed = speed 
        self.brightness = brightness
        self.saturation = saturation
        self.LEDSpacing = spacing
        self.blink_frequency = blink_frequency #seconds
        self.rightlimit = rightlimit 
        self.leftlimit = leftlimit
        self.mode = None
        
    def init(self):
        """
        initialize new LED strip connected to PWM port
        """
        self.led = AddressableLED(self.id)
       
        self.ledBuffer = self.led.LEDData()
        self.ledBuffer.setLength(self.size)

        self.led.setData(self.ledBuffer)

        SmartDashboard.putBoolean("LEDs Initialized", True)

    def enable(self):
        self.led.start()

    def disable(self):
        self.led.stop()

    def set_brightness(self, brightness: float):
        self.brightness = brightness
    
    def set_speed(self, speed: float):
        self.speed = speed
    
    def get_led_data(self):
        return [self.led.LEDData() for i in range(self.size)].copy()
    
    def get_current_type(self):
        return self.mode

    def set_Solid(self, r: int, g: int, b: int):

        self.solid = self.LEDPattern.solid(r,g,b)
        self.solid.applyTo(self.ledBuffer)
        self.led.setData(self.ledBuffer)
       
        self.mode = "solid"

    def set_Rainbow_Ladder(self):
        
        self.rainbow = self.LEDPattern.rainbow(self.saturation, self.brightness)

        #scrolls the rainbow
        self.scrollingRainbow = self.rainbow.scrollAtAbsoluteSpeed(self.speed, self.LEDSpacing)
        
        self.scrollingRainbow.applyTo(self.ledBuffer)
        self.led.setData(self.ledBuffer)

        self.mode = "rainbow"
    
    def set_Blink(self, r: int, g: int, b: int):

        self.base = self.LEDPattern.discontinousGradient(r,g,b)
        self.pattern = self.base.blink(self.blink_frequency)  

        self.pattern.applyTo(self.ledBuffer)
        self.led.setData(self.ledBuffer)

        self.mode = "blink"
    
    def field_position(self, r1, g1, b1, r2, b2, g2):
        """ 
        identify where the robot is on the field

        """
        self.robotposition = 0
    
        if self.robotposition > self.rightlimit:
            
            #to do: left side green, right side red 

            self.mode = "robot position on starting line is too far right"
        
        elif self.robotposition < self.leftlimit:
            
            #to do: left side red, right side green 
            self.mode = "robot positioning on starting line is too far left"
        
        else:
            self.set_solid(0, 100, 0) # robot is where it needs to be
    

class SLEDS:
    """
    Switchable LEDS from Switchable PDH
    """

    def on(self):
        PowerDistribution.setSwitchableChannel(True)

    def off(self):
        PowerDistribution.setSwitchableChannel(False)