from phoenix6 import degree
from wpilib import AnalogEncoder
from toolkit.motors.ctre_motors import TalonConfig
import math
from pathplannerlib.config import PIDConstants
from units.SI import degrees_to_radians, degrees, radians, meters


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


foc_active = False  # foc for TalonFX requires paid subscription

NT_ELEVATOR: bool = False

# Cameras
left_cam_name = "left_cam"
right_cam_name = "right_cam"

# Drivetrain
gyro_id: int = 20

front_left_move_id: int = 1
front_left_turn_id: int = 2
front_left_encoder_port: AnalogEncoder = AnalogEncoder(2)
front_left_encoder_zeroed_pos: float = 0.945
front_left_turn_inverted = False
front_left_move_inverted = False

front_right_move_id: int = 3
front_right_turn_id: int = 4
front_right_encoder_port: AnalogEncoder = AnalogEncoder(3)
front_right_encoder_zeroed_pos: float = 0.074
front_right_turn_inverted = False
front_right_move_inverted = False

back_left_move_id: int = 7
back_left_turn_id: int = 8
back_left_encoder_port: AnalogEncoder = AnalogEncoder(1)
back_left_encoder_zeroed_pos: float = 0.227
back_left_turn_inverted = False
back_left_move_inverted = False

back_right_move_id: int = 5
back_right_turn_id: int = 6
back_right_encoder_port: AnalogEncoder = AnalogEncoder(0)
back_right_encoder_zeroed_pos: float = 0.597
back_right_turn_inverted = False
back_right_move_inverted = False

driver_centric: bool = True
drivetrain_deadzone: float = 0.1
drivetrain_curve: float = 2.0000
drivetrain_zero: radians = math.radians(0)

drivetrain_rotation_kp: float = 5.5
drivetrain_rotation_ki: float = 0.0
drivetrain_rotation_kd: float = 0.0
drivetrain_rotation_tolerance: degrees = 1  # degrees

drivetrain_x_kp: float = 4.0
drivetrain_x_ki: float = 0.0
drivetrain_x_kd: float = 0.0
drivetrain_x_tolerance: float = 0.05

drivetrain_y_kp: float = 4.0
drivetrain_y_ki: float = 0.0
drivetrain_y_kd: float = 0.0
drivetrain_y_tolerance: float = 0.05

# odometry
odometry_tag_distance_threshold: meters = 2.5

TURN_CONFIG = TalonConfig(7, 0, 0.02, 0, 0, brake_mode=True)

MOVE_CONFIG = TalonConfig(
    0.11,
    0,
    0,
    kF=0.25,
    kA=0.15,
    kV=0.12,
    brake_mode=True,
    current_limit=50,
)

# elevator
elevator_lead_id = 10
elevator_follower_id = 11
magsensor_id = 12  # placeholder

auto_translation_pid = PIDConstants(6, 0.0, 0.1)
auto_rotation_pid = PIDConstants(5.0, 0.0, 0.0)

# TO CHANGE
period: float = 0.03
