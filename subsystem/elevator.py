import ntcore
from wpilib import DigitalInput

import config
import constants
from toolkit.subsystem import Subsystem
from units.SI import meters, meters_to_inches
from phoenix6 import hardware, controls, configs, signals


class Elevator(Subsystem):
    def __init__(self):
        super().__init__()
        self.leader_motor = hardware.TalonFX(config.elevator_lead_id)
        self.motion_magic = controls.MotionMagicVoltage(0)

        self.follower_motor = hardware.TalonFX(config.elevator_follower_id)      

        self.config = configs.TalonFXConfiguration().with_motor_output(
            configs.MotorOutputConfigs()
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)
            .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
        ).with_motion_magic(
            configs.MotionMagicConfigs()
            .with_motion_magic_cruise_velocity(110/9)
            .with_motion_magic_acceleration(275/9)
            .with_motion_magic_jerk(1000/9)
        ).with_slot0(
            configs.Slot0Configs()
            .with_k_p(5)
            .with_k_i(0)
            .with_k_d(0.175)
            .with_k_s(0.13)
            .with_k_v(0)
            .with_k_a(0)
            .with_gravity_type(signals.GravityTypeValue.ELEVATOR_STATIC)
            .with_k_g(0.28)
        ).with_feedback(
            configs.FeedbackConfigs()
            .with_sensor_to_mechanism_ratio(constants.elevator_gear_ratio/constants.elevator_driver_gear_circumference)
            .with_feedback_sensor_source(signals.FeedbackSensorSourceValue.FUSED_CANCODER)
        )

        self.target_height: meters = 0.0
        self.elevator_moving: bool = False

    def init(self):
        self.leader_motor.configurator.apply(self.config)
        self.follower_motor.set_control(controls.Follower(config.elevator_lead_id, True))
        self.leader_motor.set_position(0)

    @staticmethod
    def limit_height(height: meters) -> meters:
        """
        limits the height of the elevator to both a max and min
        """
        if height > constants.elevator_true_max:
            return constants.elevator_true_max
        elif height < 0.0:
            return 0.0
        return height


    def set_position(self, height: meters) -> None:
        """
        Brings the elevator to given height

        Args:
            height (meters): intended elevator height in meters
        """
        height = self.limit_height(height)

        self.leader_motor.set_control(self.motion_magic.with_position(height))

    def stop(self) -> None:
        """
        Stops the elevator
        """
        self.set_position(self.get_position())


    def set_zero(self) -> None:
        """
        Brings the elevator to the zero position
        """
        self.set_position(0)

    def get_position(self) -> meters:
        """
        Obtains the current height of the elevator

        Returns:
            return_float: current elevator height in meters
        """
        return self.leader_motor.get_position().value_as_double

    def is_at_position(self, height: meters, tolerance: meters = config.elevator_height_threshold) -> bool:
        """
        checks if the elevator is at a certain height

        Args:
            height (meters): height to be checked
        """
        return abs(self.get_position() - height) < tolerance

    # def update_table(self) -> None:
    #    table = ntcore.NetworkTableInstance.getDefault().getTable("Elevator")

    #     table.putNumber("height", self.get_position() * meters_to_inches)
    #     table.putNumber("velocity rps", self.leader_motor.get_sensor_velocity())
    #     table.putNumber("acceleration rpss", self.leader_motor.get_sensor_acceleration())
    #     table.putNumber("target height", self.target_height * meters_to_inches)
    #     table.putNumber(
    #         "motor lead applied output", self.leader_motor.get_applied_output()
    #     )
    #     table.putNumber(
    #         "motor lead current", self.leader_motor.get_motor_current()
    #     )
    #     table.putNumber(
    #         "motor follow applied output", self.follower_motor.get_applied_output()
    #     )

    # def periodic(self):
    #     if config.NT_ELEVATOR:
    #         self.update_table()