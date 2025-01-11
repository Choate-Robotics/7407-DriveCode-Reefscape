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
    
    def init(self):
        self.motor.init()

    