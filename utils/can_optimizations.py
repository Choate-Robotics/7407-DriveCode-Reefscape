import time
from wpilib import TimedRobot

def CAN_delay(seconds: float):
    '''
    Delays the code by the given amount of seconds, but only if the robot is not in simulation mode
    
    Useful for the CAN bus, which may need to catch up during initialization
    '''
    time.sleep(seconds) if not TimedRobot.isSimulation() else None
