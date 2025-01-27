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
        
        
    def init(self):
        """
        initialize new LED strip connected to PWM port
        """
        self.led = AddressableLED(self.id)
        """
        create new 
        """
        global ledBuffer
        self.ledBuffer = self.led.LEDData()
        self.ledBuffer.setLength(self.size)

        self.led.setData(self.ledBuffer)

        SmartDashboard.putBoolean("LEDs Initialized", True)

    def enable(self):
        self.led.start()

    def disable(self):
        self.led.stop()

    def section(self, starting_pixel: int, ending_pixel: int):
        self.section = ledBuffer.createView(starting_pixel, ending_pixel)

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
        """ density of *120* per meter"""
        self.LEDSpacing = 1/120.0;
        
        #scrolls the rainbow
        self.scrollingRainbow = self.rainbow.scrollAtAbsoluteSpeed(self.speed, self.LEDSpacing)
        
        """
        finish this later 
        """

        self.mode = "rainbow"
    
    def set_Mask(self, r1, g1, b1, r2, b2, g2):
        """
        mask = self.get_led_data()
        for i in range(self.size):
            mask[i].setRGB(r1, g1, b1)
        
        for i in range(self.mask_index, self.size, 4):
            mask[i].setRGB(r2, g2, b2)
        
        self.mask_index += 1

        if self.mask_index > self.size:
            self.mask_index = 0

        """


        self.mode = "mask"
    
    def set_Blink(self, r, g, b):
        blink = self.get_led_data()
        if self.blink_index / (2 * self.speed) <= .5:
            for i in range(self.size):
                blink[i].setRGB(r, g, b)
        else:
            for i in range(self.size):
                blink[i].setRGB(0, 0, 0)

        self.blink_index += 1
        if self.blink_index > 2 * self.speed:
            self.blink_index = 0

        return blink

class SLEDS:
    """
    Switchable LEDS from Switchable PDH
    """

    def on(self):
        PowerDistribution.setSwitchableChannel(True)

    def off(self):
        PowerDistribution.setSwitchableChannel(False)