from pathplannerlib.config import RobotConfig
from wpimath.geometry import Transform3d, Translation3d, Rotation3d
import math
from units.SI import (
    degrees_per_second__to__radians_per_second,
    inches_to_meters,
    meters,
    meters_per_second_squared,
    rotations_per_minute,
    rotations_per_minute,
)

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

# cameras
robot_to_left_cam = Transform3d(
    Translation3d(9.333*inches_to_meters, 12.0925*inches_to_meters, 8.345*inches_to_meters),
    Rotation3d(0, math.radians(-20), math.radians(-20))
)
robot_to_right_cam = Transform3d(
    Translation3d(9.333*inches_to_meters, -12.0925*inches_to_meters, 8.345*inches_to_meters),
    Rotation3d(0, math.radians(-20), math.radians(20))
)

#drivetrain
drivetrain_turn_gear_ratio: float = 18.75
drivetrain_wheel_gear_ratio: float = 7.13
track_width: meters = 23.75 * inches_to_meters # distance between the center of the wheels (front side)
track_length: meters = track_width # (left/right side)
drivetrain_length: meters = 29 * inches_to_meters
bumper_thickness: meters = 3.5 * inches_to_meters
drivetrain_length_with_bumpers: meters = drivetrain_length + (2 * bumper_thickness)
drivetrain_radius: meters = math.sqrt(math.pow(track_length/2, 2) + math.pow(track_width/2, 2))
reef_scoring_distance: meters = drivetrain_length_with_bumpers / 2 + 1 * inches_to_meters
reef_y_offset: meters = -2 * inches_to_meters #positive is right


drivetrain_move_motor_free_speed: rotations_per_minute = (
    6000  # 6000 is the free speed RPM of the Kraken without FOC
)

drivetrain_wheel_diameter: meters = (
    3.873 * inches_to_meters
) 
 

drivetrain_max_vel: meters = (
    (drivetrain_move_motor_free_speed / 60) / drivetrain_wheel_gear_ratio
) * (drivetrain_wheel_diameter * math.pi)

# drivetrain_max_vel = 5.4 m/s, 17.7 ft/s, 12.1 mph
drivetrain_max_accel: meters_per_second_squared = (
    0  # setting to 0 will set to default motor accel
)
drivetrain_max_angular_vel = 720 * degrees_per_second__to__radians_per_second
drivetrain_max_angular_accel = 720 * degrees_per_second__to__radians_per_second

drivetrain_move_gear_ratio_as_rotations_per_meter: float = (
    1 / (drivetrain_wheel_diameter * math.pi)
) * drivetrain_wheel_gear_ratio

# wrist
wrist_gear_ratio = 45
wrist_encoder_gear_ratio = 1.5

# intake
intake_pivot_gear_ratio: float = 175
horizontal_gear_ratio: float = 2
vertical_gear_ratio: float = 2.5
intake_encoder_gear_ratio: float = 3

#elevator
elevator_gear_ratio = 12 #REAL VALUE: 12:1 gear ratio
elevator_driver_gear_circumference = 1.751*inches_to_meters*math.pi
elevator_max_height: meters = 27.5*inches_to_meters #true max=28 inches

# field
field_length = 17.548
field_width = 8.052

auto_config = RobotConfig.fromGUISettings()
