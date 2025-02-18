from wpilib import AddressableLED, SmartDashboard, Color, LEDPattern
import math
from toolkit.subsystem import Subsystem
import ntcore

class AddressableLEDStrip(Subsystem):

    def __init__(self, 
            id: int, 
            size: int, 
            speed: int, 
            brightness: int, 
            saturation: int,
            spacing: int, 
            blink_frequency: float,
        ):

        self.size = size
        self.id = id
        self.speed = speed 
        self.brightness = brightness
        self.saturation = saturation
        self.LEDSpacing = spacing
        self.blink_frequency = blink_frequency #seconds
        self.pattern = None
        self.mode = "None"
        
    def init(self):
        """
        initialize new LED strip connected to PWM port
        """
        self.led = AddressableLED(self.id)
       
        self.ledBuffer = [self.led.LEDData() for i in range(self.size)]
        self.led.enable()
        self.led.set_brightness()
        self.led.set_speed()
        SmartDashboard.putBoolean("LEDs Initialized", True)

    def enable(self):
        '''
        be sure to enable
        '''
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
        #return self.pattern/mode
        pass

    def set_Solid(self, r: int, g: int, b: int):

        self.pattern = LEDPattern.solid(Color(g/255, r/255, b/255))
        self.mode = f"Solid r:{r} g:{g} b:{b}"

    def set_Alternate(self, r1: int, g1: int, b1: int, r2:int, g2:int, b2: int):

        self.alternate = []
        #red and green are switched for an unknown reason
        for i in range(self.size):
            if i % 2 == 0:
                self.alternate.append((i/self.size, Color(g1/255, r1/255, b1/255)))
            elif i % 2 == 1:
                self.alternate.append((i/self.size, Color(g2/255, r2/255, b2/255)))

        self.pattern = LEDPattern.steps(self.alternate)
        self.mode = f"Alternate"

    def set_Rainbow_Ladder(self):
        """
        creates a scrolling rainbow on LEDs
        """
        self.rainbow = LEDPattern.rainbow(self.saturation, self.brightness)

        self.pattern = self.rainbow.scrollAtAbsoluteSpeed(self.speed, self.LEDSpacing)
        self.mode = "Rainbow Ladder"

    def set_Blink(self, r: int, g: int, b: int):

        self.base = LEDPattern.solid((r/255, g/255, b/255))
        self.pattern = self.base.blink(self.blink_frequency)
        self.mode = f"Blink r:{r} g:{g} b:{b}"  

    def periodic(self):

        self.update_tables()
        def set_pattern_writer(i: int, my_color: Color) -> None:
            self.ledBuffer[i].setLED(my_color)
        ledreader= LEDPattern.LEDReader(self.ledBuffer.__getitem__, self.size)
        self.pattern.applyTo(ledreader, set_pattern_writer)
        self.led.setData(self.ledBuffer)

    def update_tables(self):
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("LEDS")
        self.table.putString("mode", self.mode)