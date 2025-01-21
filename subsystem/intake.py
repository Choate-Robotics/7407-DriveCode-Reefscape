from unittest.mock import MagicMock

import config
import constants
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor: TalonFX = TalonFX(
            config.intake_id,
            config.foc_active,
            inverted=False,
            config=config.INTAKE_CONFIG,
        )
        self.coral_in_intake: bool = False
        self.intake_running: bool = False
        self.sensor = MagicMock()  # placeholder

    def init(self):
        self.motor.init()

    def roll_in(self):
        """
        spin the motors inwards to collect the corral
        """
        self.motor.set_raw_output(1)

    def stop(self):
        """
        stop the motors
        """
        self.motor.set_raw_output(0)

    def roll_out(self):
        """
        eject coral in the intake
        """
        self.motor.set_raw_output(-1)

    def detect_coral(self) -> bool:
        """
        check if coral is in the intake
        """
        self.coral_in_intake = self.sensor.get()
        return self.coral_in_intake
