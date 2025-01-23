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
        self.track_index = self.size / 2
        self.blink_index = 0
        self.brightness = 1
        self.active_mode = None
        self.last_active_mode = None
        self.last_brightness = None
        self.last_speed = None
        
        
    def init(self):
        self.rainbowFirstPixelHue = 0
        self.led_data = [self.ledBuffer for i in range(self.size)]
        self.led.setLength(self.size)
        self.led = AddressableLED(self.id)
        self.ledBuffer = self.led.LEDData()
        self.led.setData(self.led_data)

        SmartDashboard.putBoolean("LEDs Initialized", True)

    def enable(self):
        self.led.start()

    def disable(self):
        self.led.stop()

    def set_brightness(self, brightness: float):
        self.brightness = brightness
    
    def get_led_data(self):
        return [self.led.LEDData() for i in range(self.size)].copy()
    
    def get_current_cycle(self):
        return self.led_data
    
    def get_current_type(self):
        if self.active_mode is None:
            return {
                'type': 0,
                'color': {
                    'r': 0,
                    'g': 0,
                    'b': 0
                }
            }
        return self.active_mode

    def store_current(self):
        self.last_active_mode = self.active_mode
        self.last_speed = self.speed
        self.last_brightness = self.brightness
    
    def set_LED(self, type, brightness: float = 1.0, speed: int = 5):
        self.active_mode = type
        self.speed = speed
        self.brightness = brightness

    def set_last_current(self):
        self.active_mode = self.last_active_mode
        self.speed = self.last_speed
        self.brightness = self.last_brightness
    
    def match(self, type: config.LEDType):
        res = self.get_led_data()
        match type['type']:
            case 1:
                color = type['color']
                res = self.set_Solid(color['r'], color['g'], color['b'])
            case 2:
                res = self.set_Rainbow()
            case 3:
                color = type['color']
                res = self.set_Mask(color['r1'], color['g1'], color['b1'], color['r2'], color['g2'], color['b2'])
            case 4:
                color = type['color']
                res = self.set_Blink(color['r'], color['g'], color['b'])
            case _:
                res = self.set_Rainbow()

        return res

    def cycle(self):
        self.led.setData(self.match(self.active_mode))

    def set_Mask(self, r1, g1, b1, r2, b2, g2):
        mask = self.get_led_data()
        for i in range(self.size):
            mask[i].setRGB(r1, g1, b1)
        
        for i in range(self.mask_index, self.size, 4):
            mask[i].setRGB(r2, g2, b2)
        
        self.mask_index += 1

        if self.mask_index > self.size:
            self.mask_index = 0

        return mask

    def set_Rainbow_Ladder(self):
        rainbow = self.get_led_data()
        for i in range(self.size):
            hue = math.floor((self.rainbowFirstPixelHue + (i * 180 / self.size)) % 180)
            rainbow[i].setHSV(hue, 255, 128)

        #move rainbow
        self.rainbowFirstPixelHue += self.speed

        self.rainbowFirstPixelHue %= 180

        return rainbow

    def set_Solid(self, r: int, g: int, b: int):

        solid = self.get_led_data()
        for i in range(self.size):
            solid[i].setRGB(r, g, b)

        return solid
    
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