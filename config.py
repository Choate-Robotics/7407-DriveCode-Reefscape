import math
from dataclasses import dataclass
from wpilib import AnalogEncoder

from toolkit.motors.ctre_motors import TalonConfig
from units.SI import radians, meters

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

# NETWORK TABLES
NT_WRIST = True
NT_INTAKE = True
NT_ELEVATOR = True


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
    0.11, 0, 0, 0.25, 0.01, brake_mode=True, current_limit=40, kV=0.12
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

WRIST_CONFIG = TalonConfig(1, 0, 0, 0, 0)

wrist_max_angle = 2 * math.pi
wrist_min_angle = 0 * math.pi

angle_threshold = math.radians(2)  # radians
out_current_threshold = 2  # amps PLACEHOLDER
back_current_threshold = 10  # amps PLACEHOLDER
current_time_threshold = 0.3  # seconds PLACEHOLDER



# ELEVATOR
elevator_lead_id = 10
elevator_follower_id = 11
magsensor_id = 0


# INTAKE
intake_id = 9
INTAKE_CONFIG = TalonConfig(1, 0, 0, 0, 0)

intake_speed = 1  # placeholder
intake_eject_speed = -1  # placeholder



# SCORING POSITIONS
@dataclass
class TargetData:
    target_pose: Pose2d | None
    elevator_height: meters
    wrist_angle: radians

    intake_enabled: bool
    intake_running: bool

    extake_running: bool


target_positions: dict[str, TargetData] = {
    # ALL PLACEHOLDERS

    "IDLE": TargetData(
        target_pose=None,
        elevator_height=0,
        wrist_angle=0,
        intake_enabled=True,
        intake_running=False,
        extake_running=False
    ),

    "STATION_INTAKING": TargetData(
        target_pose=None,
        elevator_height=0,
        wrist_angle=0,
        intake_enabled=True,
        intake_running=True,
        extake_running=False
    ),

    "LOW": TargetData(
        target_pose=None,
        elevator_height=0,
        wrist_angle=0,
        intake_enabled=False,
        intake_running=False,
        extake_running=True
    ),

    "MID": TargetData(
        target_pose=None,
        elevator_height=0,
        wrist_angle=0,
        intake_enabled=False,
        intake_running=False,
        extake_running=True
    ),

    "HIGH": TargetData(
        target_pose=None,
        elevator_height=0,
        wrist_angle=0,
        intake_enabled=False,
        intake_running=False,
        extake_running=True
    )
}
