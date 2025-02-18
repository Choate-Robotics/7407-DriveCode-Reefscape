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

# DEBUGGING NETWORK TABLES
NT_INTAKE = False
NT_ELEVATOR: bool = False


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


#intake
horizontal_id = 9
# HORIZONTAL_CONFIG = TalonConfig(
#     1,
#     0,
#     0,
#     0,
#     0
# )
vertical_id = 13
# VERTICAL_CONFIG = TalonConfig(
#     1,
#     0,
#     0,
#     0,
#     0
# )

intake_cancoder_id = 400
intake_pivot_id = 10
intake_encoder_zero = 0 #placeholder
INTAKE_PIVOT_CONFIG = TalonConfig(
    1,
    0,
    0,
    0,
    0
)

intake_max_angle = math.radians(90)
intake_min_angle = math.radians(60)
intake_angle_threshold = math.radians(5)
intake_current_threshold = 80 #placeholder
intake_current_time_threshold = 2 #placeholder

intake_max_ff = 0 #placeholder
intake_ff_offset = 0 #placeholder

intake_speed = 1 #placeholder
intake_eject_speed = 1 #placeholder

#elevator
elevator_lead_id = 10 
elevator_follower_id = 11
magsensor_id = 12 #placeholder

