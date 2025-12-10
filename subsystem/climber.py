import config
import constants

from units.SI import radians
# from toolkit.motors.ctre_motors import TalonFX
from toolkit.subsystem import Subsystem
from phoenix6 import hardware, controls, configs, signals
import ntcore
import math

class Climber(Subsystem):

    # Initialize class
    def __init__(self) -> None:
        super().__init__()
        self.climber_motor = hardware.TalonFX(config.climber_motor_id)
        self.control = controls.DutyCycleOut(0)

        self.config = configs.TalonFXConfiguration().with_motor_output(
            configs.MotorOutputConfigs()
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
        ).with_feedback(

        )

        self.moving = False
        self.zeroed = False

    # Start motors
    def init(self) -> None:
        self.climber_motor.configurator.apply(self.config)
        self.zero()

    def zero(self) -> None:
        self.climber_motor.set_position(0)
        self.zeroed = True

    # Set raw output of climber motor
    def set_raw_output(self, raw_value: float) -> None:
        self.climber_motor.set_control(self.control.with_output(raw_value))

    # Get motor revolutions
    def get_motor_revolutions(self) -> float:
        return self.climber_motor.get_rotor_position().value_as_double
        
    # def update_table(self) -> None:
    #     table = ntcore.NetworkTableInstance.getDefault().getTable("climber")

    #     table.putNumber("climber_motor_revolutions", self.climber_motor.get_sensor_position())
    #     table.putBoolean("climber_moving", self.moving)
    #     table.putBoolean("climber_zeroed", self.zeroed)
    #     table.putNumber("climber_motor_current", self.climber_motor.get_motor_current())

    # def periodic(self) -> None:
    #     if config.NT_CLIMBER:
    #         self.update_table()
