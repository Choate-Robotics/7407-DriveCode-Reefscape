from units.SI import radians
from wpilib import AnalogEncoder
from toolkit.motors.ctre_motors import TalonConfig
import math


DEBUG_MODE: bool = True
# MAKE SURE TO MAKE THIS FALSE FOR COMPETITION
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
LOGGING: bool = True
LOG_OUT_LEVEL: int = 0
LOG_FILE_LEVEL: int = 1
# Levels are how much information is logged
# higher level = less information
# level 0 will log everything
# level 1 will log everything except debug
# and so on
# levels:
# 0 = All
# 1 = INFO
# 2 = WARNING
# 3 = ERROR
# 4 = SETUP
# anything else will log nothing
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

foc_active = False  #foc for TalonFX requires paid subscription

#Drivetrain
gyro_id: int = 13

front_left_move_id: int = 2
front_left_turn_id: int = 1
front_left_encoder_port: AnalogEncoder = AnalogEncoder(0)
front_left_encoder_zeroed_pos: float = 0.362
front_left_turn_inverted = False
front_left_move_inverted = False

front_right_move_id: int = 4
front_right_turn_id: int = 3
front_right_encoder_port: AnalogEncoder = AnalogEncoder(1)
front_right_encoder_zeroed_pos: float = 0.034
front_right_turn_inverted = False
front_right_move_inverted = False

back_left_move_id: int = 8
back_left_turn_id: int = 7
back_left_encoder_port: AnalogEncoder = AnalogEncoder(3)
back_left_encoder_zeroed_pos: float = 0.735
back_left_turn_inverted = False
back_left_move_inverted = False

back_right_move_id: int = 6
back_right_turn_id: int = 5
back_right_encoder_port: AnalogEncoder = AnalogEncoder(2)
back_right_encoder_zeroed_pos: float = 0.713
back_right_turn_inverted = False
back_right_move_inverted = False

driver_centric: bool = True
drivetrain_deadzone: float = 0.1
drivetrain_curve: float = 2
drivetrain_zero: radians = math.radians(180)

TURN_CONFIG = TalonConfig(
    .9, 0, 0, 0, 0, brake_mode=True,
)

MOVE_CONFIG = TalonConfig(
    0.11,
    0,
    0,
    0.25,
    0.01,
    brake_mode=True,
    current_limit=40,
    kV=0.12
)

# TODO: Placeholder
climber_motor_id = 23
climber_motor_follower_id = 24
climber_config = TalonConfig(0, 0, 0, 0, 0, 0)
climber_encoder_id = 25
climber_angle_threshold = 0.01
deploy_climber_speed = 0.5
lift_climber_speed = -0.5
climber_revolutions = 10