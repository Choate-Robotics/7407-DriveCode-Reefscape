from dataclasses import dataclass
from wpilib import AnalogEncoder
from toolkit.motors.ctre_motors import TalonConfig
import math
from pathplannerlib.config import PIDConstants
from units.SI import degrees, radians, meters, inches_to_meters
import constants

DEBUG_MODE: bool = False
# MAKE SURE TO MAKE THIS FALSE FOR COMPETITION
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

foc_active = False  # foc for TalonFX requires paid subscription

trigger_threshold = 0.4

# DEBUGGING NETWORK TABLES
NT_INTAKE: bool = True
NT_ELEVATOR: bool = True
NT_WRIST: bool = True
NT_DRIVETRAIN: bool = True

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

drivetrain_rotation_kp: float = 5
drivetrain_rotation_ki: float = 0.0
drivetrain_rotation_kd: float = 0.0
drivetrain_rotation_tolerance: degrees = 1  # degrees

drivetrain_x_kp: float = 4.0
drivetrain_x_ki: float = 0.0
drivetrain_x_kd: float = 0.0
drivetrain_x_tolerance: float = 0.001

drivetrain_y_kp: float = 4.0
drivetrain_y_ki: float = 0.0
drivetrain_y_kd: float = 0.0
drivetrain_y_tolerance: float = 0.001

auto_translation_pid = PIDConstants(4, 0.0, 0)
auto_rotation_pid = PIDConstants(5.0, 0.0, 0.0)

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

climber_motor_id = 16
climber_config = TalonConfig(0, 0, 0, 0, 0, brake_mode=True)
deploy_climber_speed = 1
climb_speed = 1
manual_climber_speed = 0.2
deploy_position = 285
manual_lower_bound = -50
climb_initial_out = 36


# odometry
odometry_tag_distance_threshold: meters = 2.5

# Wrist
wrist_feed_id = 15
WRIST_FEED_CONFIG = TalonConfig(1, 0, 0, 0, 0, current_limit=60)
wrist_algae_id = 17
WRIST_ALGAE_CONFIG = TalonConfig(1, 0, 0, 0, 0, current_limit=40)
wrist_id = 14
WRIST_CONFIG = TalonConfig(
    48, 0, 0, 0.06, 0, motion_magic_cruise_velocity=97.75, motion_magic_acceleration=350
)  # 97.75
wrist_cancoder_id = 22
wrist_encoder_zero = 0.781

wrist_intake_speed = 0.35
wrist_extake_speed = -0.25
wrist_algae_speed = 1
wrist_algae_extake_speed = -0.5
wrist_algae_hold_volts = 1
algae_moving_hold_volts = 10
wrist_max_angle: radians = math.radians(75)
wrist_min_angle: radians = math.radians(-117)
angle_threshold: radians = math.radians(1)  # radians
out_current_threshold: float = 13  # amps PLACEHOLDER
back_current_threshold: float = 43
current_time_threshold: float = 0.25
wrist_algae_time_threshold: float = 3  # seconds PLACEHOLDER
algae_current_threshold: float = 20

wrist_max_ff = 0.17
wrist_ff_offset = math.radians(30)

# intake
horizontal_id = 13
intake_cancoder_id = 21
intake_pivot_id = 11
intake_encoder_zero = 0.075
INTAKE_CONFIG = TalonConfig(0, 0, 0, 0, 0, brake_mode=True)
INTAKE_PIVOT_CONFIG = TalonConfig(2, 0, 0, -0.195, 0, motion_magic_cruise_velocity=97, brake_mode=True) # 97

intake_max_angle = math.radians(60)
intake_min_angle = math.radians(0)
intake_angle_threshold = math.radians(2)
intake_current_threshold = 80  # placeholder
intake_current_time_threshold = 2  # placeholder

intake_max_ff = -0.075
intake_ff_offset = math.radians(90)

horizontal_intake_speed = 0.5
l1_eject_speed = 0.1
intake_algae_speed = 1
extake_algae_speed = 0.3

# elevator
elevator_lead_id = 9
elevator_follower_id = 10

elevator_height_threshold = 0.1 * inches_to_meters  # placeholder

ELEVATOR_CONFIG = TalonConfig(
    5,
    0,
    0.175,
    0.13,
    0,
    0,
    kG=0.28,
    brake_mode=True,
    motion_magic_cruise_velocity=110,
    motion_magic_acceleration=275,
    motion_magic_jerk=1000
) 


# TO CHANGE
period: float = 0.03

elevator_l1_height: meters = 5 * inches_to_meters
elevator_l2_height: meters = 6 * inches_to_meters
elevator_l3_height: meters = 13.75 * inches_to_meters
elevator_l4_height: meters = constants.elevator_max_height
elevator_dhigh_height: meters = 11 * inches_to_meters
elevator_dlow_height: meters = 2.75 * inches_to_meters
elevator_barge_height: meters = constants.elevator_max_height

intake_algae_ground_angle = math.radians(58)
intake_algae_score_angle = math.radians(32)
intake_climb_angle = math.radians(20)
intake_coral_station_angle = math.radians(-0.095)
intake_l1_angle = math.radians(37)
intake_l1_hold_angle = math.radians(-5)

wrist_idle_angle = math.radians(-10)
wrist_intake_angle = math.radians(-114.5)
wrist_intake_l1_angle = math.radians(-100)
wrist_l1_angle = math.radians(64)
wrist_l2_angle = math.radians(64)
wrist_l3_angle = math.radians(64)
wrist_l4_angle = math.radians(54)
wrist_dhigh_angle = math.radians(55)
wrist_dlow_angle = math.radians(55)
wrist_barge_angle = math.radians(54)
wrist_processor_score_angle = 0


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

    intake_angle: radians
    intake_in_run: bool
    intake_out_run: bool

    intake_climb: bool = False


target_positions: dict[str, TargetData] = {
    "IDLE": TargetData(
        elevator_idle=True,
        wrist_idle=True,
        intake_idle=True,
        elevator_height=0,
        wrist_angle=wrist_idle_angle,
        wrist_feed_on=False,
        wrist_score_on=False,
        intake_angle=0,
        intake_in_run=False,
        intake_out_run=False,
    ),
    "STATION_INTAKING": TargetData(
        elevator_idle=True,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=0,
        wrist_angle=wrist_intake_angle,
        wrist_feed_on=True,
        wrist_score_on=False,
        intake_angle=intake_coral_station_angle,
        intake_in_run=True,
        intake_out_run=False,
    ),
    "INTAKE_L1": TargetData(
        elevator_idle=True,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=0,
        wrist_angle=wrist_intake_l1_angle,
        wrist_feed_on=True,
        wrist_score_on=False,
        intake_angle=intake_coral_station_angle,
        intake_in_run=True,
        intake_out_run=False,
    ),
    "L1": TargetData(
        elevator_idle=True,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_l1_height,
        wrist_angle=wrist_l1_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_angle=0,
        intake_in_run=False,
        intake_out_run=False,
    ),
    "L2": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_l2_height,
        wrist_angle=wrist_l2_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_angle=0,
        intake_in_run=False,
        intake_out_run=False,
    ),
    "L3": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_l3_height,
        wrist_angle=wrist_l3_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_angle=0,
        intake_in_run=False,
        intake_out_run=False,
    ),
    "L4": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_l4_height,
        wrist_angle=wrist_l4_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_angle=0,
        intake_in_run=False,
        intake_out_run=False,
    ),
    "DEALGAE_HIGH": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_dhigh_height,
        wrist_angle=wrist_dhigh_angle,
        wrist_feed_on=False,
        wrist_score_on=False,
        intake_angle=0,
        intake_in_run=False,
        intake_out_run=False,
    ),
    "DEALGAE_LOW": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_dlow_height,
        wrist_angle=wrist_dlow_angle,
        wrist_feed_on=False,
        wrist_score_on=False,
        intake_angle=0,
        intake_in_run=False,
        intake_out_run=False,
    ),
    "SCORE_BARGE": TargetData(
        elevator_idle=False,
        wrist_idle=False,
        intake_idle=True,
        elevator_height=elevator_barge_height,
        wrist_angle=wrist_barge_angle,
        wrist_feed_on=False,
        wrist_score_on=True,
        intake_angle=0,
        intake_in_run=False,
        intake_out_run=False,
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
        intake_angle=0,
        intake_in_run=False,
        intake_out_run=False,
    ),
    "INTAKE_ALGAE": TargetData(
        elevator_idle=True,
        wrist_idle=True,
        intake_idle=False,
        elevator_height=0,
        wrist_angle=0,
        wrist_feed_on=False,
        wrist_score_on=False,
        intake_angle=intake_algae_ground_angle,
        intake_in_run=True,
        intake_out_run=False,
    ),
    "CLIMB": TargetData(
        elevator_idle=True,
        wrist_idle=True,
        intake_idle=False,
        elevator_height=0,
        wrist_angle=0,
        wrist_feed_on=False,
        wrist_score_on=False,
        intake_angle=intake_climb_angle,
        intake_in_run=True,
        intake_out_run=False,
        intake_climb=True,
    ),
}