from units.SI import radians, inches_to_meters
import math
from dataclasses import dataclass
from wpilib import AnalogEncoder

from toolkit.motors.ctre_motors import TalonConfig
from units.SI import radians, meters, seconds

from wpimath.geometry import Pose2d

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

# TODO: Set deadzone
trigger_threshold = 0

# DEBUGGING NETWORK TABLES
NT_INTAKE: bool = False
NT_ELEVATOR: bool = False
NT_WRIST: bool = False
NT_DRIVETRAIN: bool = False


# DRIVETRAIN
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
    0.9,
    0,
    0,
    0,
    0,
    brake_mode=True,
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


# WRIST
wrist_feed_id = 10
wrist_id = 9
wrist_cancoder_id = 11

WRIST_FEED_CONFIG = TalonConfig(
    1,
    0,
    0,
    0,
    0
)

WRIST_CONFIG = TalonConfig(
    1, 
    0, 
    0, 
    0, 
    0
)

wrist_encoder_zero = 0  # placeholder

wrist_max_angle: radians = 2 * math.pi
wrist_min_angle: radians = 0 * math.pi

# TODO: Change to actual angles
wrist_intake_angle: radians = 0
wrist_l1_angle: radians = 0
wrist_l2_angle: radians = 0
wrist_l3_angle: radians = 0
wrist_l4_angle: radians = 0
wrist_dhigh_angle: radians = 0
wrist_dlow_angle: radians = 0
wrist_barge_angle: radians = 0
wrist_processor_score_angle: radians = 0

wrist_angle_threshold: radians = math.radians(2)

wrist_feed_in_speed = 1
wrist_feed_out_speed = -1

# TODO: Change to actual thresholds
out_current_threshold = 2
back_current_threshold = 10
current_time_threshold: seconds = 0.3


# ELEVATOR
elevator_lead_id = 10
elevator_follower_id = 11
elevator_height_threshold = .1*inches_to_meters #placeholder

ELEVATOR_CONFIG = TalonConfig(
    0,
    0,
    0,
    0,
    0,
    0,
    brake_mode=True
)

# TODO: Change to actual heights
elevator_l1_height: meters = 0
elevator_l2_height: meters = 0
elevator_l3_height: meters = 0
elevator_l4_height: meters = 0
elevator_dhigh_height: meters = 0
elevator_dlow_height: meters = 0
elevator_barge_height: meters = 0

# INTAKE
intake_pivot_id = 0
horizontal_id = 9
vertical_id = 13
intake_cancoder_id = 400
intake_encoder_zero_pos = 0

INTAKE_CONFIG = TalonConfig(
    1, 
    0, 
    0, 
    0, 
    0
)

INTAKE_PIVOT_CONFIG = TalonConfig(
    1, 
    0, 
    0, 
    0, 
    0
)

# TODO: Change to actual speeds
intake_speed = 1
intake_eject_speed = -1

intake_max_angle = 0
intake_min_angle = 0
intake_angle_threshold = 0

# TARGET POSITIONS
@dataclass
class TargetData:
    elevator_idle: bool
    wrist_idle: bool
    intake_idle: bool

    elevator_height: meters

    wrist_angle: radians
    wrist_feed_on: bool
    wrist_score_on: bool
    
    intake_in_run: bool
    intake_out_run: bool

    intake_climb: bool = False


target_positions: dict[str, TargetData] = {

    "IDLE": TargetData(
        elevator_idle=True,
        wrist_idle=True,
        intake_idle=True,
        elevator_height=0,
        wrist_angle=0,
        wrist_feed_on=False,
        wrist_score_on=False,
        intake_in_run=False,
        intake_out_run=False
    ),

    "STATION_INTAKING": TargetData(
        elevator_idle=True,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=0,
        wrist_angle=wrist_intake_angle,
        wrist_feed_on=True,
        wrist_score_on=False,
        intake_in_run=True,
        intake_out_run=False
    ),

    "L1": TargetData(
        elevator_idle=True,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_l1_height,
        wrist_angle=wrist_l1_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_in_run=False,
        intake_out_run=False
    ),

    "L2": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_l2_height,
        wrist_angle=wrist_l2_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_in_run=False,
        intake_out_run=False
    ),

    "L3": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_l3_height,
        wrist_angle=wrist_l3_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_in_run=False,
        intake_out_run=False
    ),

    "L4": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_l4_height,
        wrist_angle=wrist_l4_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_in_run=False,
        intake_out_run=False
    ),

    "DEALGAE_HIGH": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_dhigh_height,
        wrist_angle=wrist_dhigh_angle,
        wrist_feed_on=False,
        wrist_score_on=False,
        intake_in_run=False,
        intake_out_run=False
    ),

    "DEALGAE_LOW": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_dlow_height,
        wrist_angle=wrist_dlow_angle,
        wrist_feed_on=False,
        wrist_score_on=False,
        intake_in_run=False,
        intake_out_run=False
    ),

    "SCORE_BARGE": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_barge_height,
        wrist_angle=wrist_barge_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_in_run=False,
        intake_out_run=False
    ),

    # "SCORE_PROCESSOR_INTAKE": TargetData(
    #     elevator_idle=True,
    #     wrist_idle=True,
    #     intake_idle=True, # Might also be ground intake, tbd
    #     elevator_height=0,
    #     wrist_angle=0,
    #     wrist_feed_on=False,
    #     wrist_score_on=False,
    #     intake_in_run=False,
    #     intake_out_run=True
    # ),

    "SCORE_PROCESSOR_WRIST": TargetData(
        elevator_idle=True,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=0,
        wrist_angle=wrist_processor_score_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_in_run=False,
        intake_out_run=False
    ),

    "INTAKE_ALGAE": TargetData(
        elevator_idle=True,
        wrist_idle=True,
        intake_idle=False,
        elevator_height=0,
        wrist_angle=0,
        wrist_feed_on=False,
        wrist_score_on=False,
        intake_in_run=True,
        intake_out_run=False
    ),

    "CLIMB": TargetData(
        elevator_idle=True,
        wrist_idle=True,
        intake_idle=False,
        elevator_height=0,
        wrist_angle=0,
        wrist_feed_on=False,
        wrist_score_on=False,
        intake_in_run=True,
        intake_out_run=False,
        intake_climb=True,
    )
}
