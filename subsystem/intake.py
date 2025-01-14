from toolkit.subsystem import Subsystem
from toolkit.motors.ctre_motors import TalonFX

import config

class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor: TalonFX = TalonFX(
            config.intake_id,
            config.foc_active,
            inverted = False,
            config = config.INTAKE_CONFIG
        )
        self.coral_in_intake: bool = False
        self.intake_running: bool = False
    
    def init(self):
        self.motor.init()

    def roll_in(self):
        """
        spin the motors inwards to collect the corral
        """
        pass

    def stop(self):
        """
        stop the motors
        """
        pass

    def roll_out(self):
        """
        eject coral in the intake
        """
        pass

    def detect_coral(self) -> bool:
        """
        check if coral is in the intake
        """
        self.coral_in_intake = self.imaginarysensor
        return self.coral_in_intake

