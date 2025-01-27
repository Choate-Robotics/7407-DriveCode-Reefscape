from wpilib import AddressableLED, PowerDistribution, SmartDashboard
import math, config

class ALeds():

    """
    Addressable LEDs

    """

    def __init__(self, id: int, size: int ):

        self.size = size
        self.id = id
        self.speed = 5 
        self.brightness = 1
        """ density of *120* per meter"""
        self.LEDSpacing = 1/120.0
        self.blinkfrequency = 1.5 #seconds
        self.rightlimit = 100 
        self.leftlimit = 50
        
    def init(self):
        """
        initialize new LED strip connected to PWM port
        """
        self.led = AddressableLED(self.id)
        """
        create new 
        """
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
    
    def get_led_data(self):
        return [self.led.LEDData() for i in range(self.size)].copy()
    
    def get_current_type(self):
        if self.mode is None:
            return {
                'type': 0,
                'color': {
                    'r': 0,
                    'g': 0,
                    'b': 0
                }
            }
        return self.mode
    
    def set_LED(self, brightness: float = 1.0, speed: int = 5):
        self.speed = speed
        self.brightness = brightness

    def set_Solid(self, r: int, g: int, b: int):

        self.solid = self.LEDPattern.solid(r,g,b)
        self.solid.applyTo(self.ledBuffer)
        self.led.setData(self.ledBuffer)
       
        self.mode = "solid"

    def set_Rainbow_Ladder(self):
        
        self.rainbow = self.LEDPattern.rainbow(self.speed, self.brightness)

        #scrolls the rainbow
        self.scrollingRainbow = self.rainbow.scrollAtAbsoluteSpeed(self.speed, self.LEDSpacing)
        
        self.scrollingRainbow.applyTo(self.ledBuffer)

        self.mode = "rainbow"
    
    def set_Blink(self, r, g, b):

        self.base = self.LEDPattern.discontinousGradient(r,g,b)
        self.pattern = self.base.blink(self.blinkfrequency)  
        self.base.applyTo(self.ledBuffer)
        self.led.setData(self.baseledBuffer)

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