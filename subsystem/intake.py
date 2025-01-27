from unittest.mock import MagicMock

import config
import constants
from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
import ntcore


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
        self.sensor = MagicMock()

    def init(self):
        self.motor.init()

    def roll_in(self):
        """
        spin the motors inwards to collect the corral
        """
        self.motor.set_raw_output(config.intake_speed * constants.intake_gear_ratio)

    def stop(self):
        """
        stop the motors
        """
        self.motor.set_raw_output(0)

    def roll_out(self):
        """
        eject coral in the intake
        """
        self.motor.set_raw_output(-config.intake_speed * constants.intake_gear_ratio)

    def detect_coral(self) -> bool:
        """
        check if coral is in the intake
        """
        self.coral_in_intake = self.sensor.getVoltage()
        return self.coral_in_intake
    
    def get_current(self) -> int:
        pass

    def getAppliedOutput():
        pass
    
    def periodic(self) -> None:
        if config.NT_INTAKE:
            table = ntcore.NetworkTableInstance.getDefault().getTable('intake')

            table.putBoolean('coral in intake', self.coral_in_intake)
            table.putBoolean('coral detected', self.detect_coral())
            table.putBoolean('intake running', self.intake_running)
            table.putNumber('outer current', self.get_current())
            table.putNumber('intake applied output', self.motor.getAppliedOutput())