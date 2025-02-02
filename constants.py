import math
from wpimath.geometry import Pose2d, Rotation2d, Pose3d, Rotation3d, Translation2d
from units.SI import (
    degrees_per_second__to__radians_per_second,
    feet_to_meters,
    inches_to_meters,
    meters,
    rotations_per_minute,
    degrees_to_radians,
    meters_per_second_squared,
)

#drivetrain
drivetrain_turn_gear_ratio: float = 150 / 7
drivetrain_wheel_gear_ratio: float = 5.9
track_width: meters = 19.75 * inches_to_meters #distance between the center of the wheels (front side)
track_length: meters = 18.25 * inches_to_meters #(left/right side)
drivetrain_length: meters = 20 #length of one side of the robot, placeholder
bumper_thickness: float = 3.5
drivetrain_length_with_bumpers = drivetrain_length + (2 * bumper_thickness)

drivetrain_move_motor_free_speed: rotations_per_minute = (
    6000 #6000 is the free speed RPM of the Kraken without FOC
)

drivetrain_wheel_diameter: meters = (
        4 * inches_to_meters
)  
 

drivetrain_max_vel: meters = (
    ((drivetrain_move_motor_free_speed / 60) / drivetrain_wheel_gear_ratio) * (drivetrain_wheel_diameter * math.pi)
    )
# drivetrain_max_vel = 5.4 m/s, 17.7 ft/s, 12.1 mph
drivetrain_max_accel: meters_per_second_squared = 0 # setting to 0 will set to default motor accel
drivetrain_max_angular_vel = 720 * degrees_per_second__to__radians_per_second
drivetrain_max_angular_accel = 720 * degrees_per_second__to__radians_per_second


# the below variable is the rotation the motor rotates per meter of wheel movement
drivetrain_move_gear_ratio_as_rotations_per_meter: float = (
    1 / (drivetrain_wheel_diameter * math.pi)
) * drivetrain_wheel_gear_ratio

#field
reef_scoring_distance: meters = 17.5 #placeholder (meters)

#elevator
elevator_gear_ratio = 1 #placeholder
elevator_driver_gear_circumference = 1 #placeholder
elevator_max_height = 2.5 #placeholder

#SIM Constants
kFieldSimTargetKey = "SimTargets"

kSwerveModuleCenterToRobotCenterLength=track_length/2
kSwerveModuleCenterToRobotCenterWidth=track_width/2
kSimMotorResistance = .002
kSimulationRotationalInertia = 0.0002
kSimDefaultRobotLocation = Pose2d(0, 0, Rotation2d(0))
kSimMaxVoltage = 12
kSimRobotVelocityArrayKey = "robot_velocity"
kSimRobotAccelerationArrayKey = "robot_acceleration"
kSimRobotAngularVelocityArrayKey = "robot_angular_velocity"
kSimRobotAngularAccelerationArrayKey = "robot_angular_acceleration"
kSimRobotPoseArrayKey = "sim_robot_pose"
kSimRobotAngularPoseArrayKey = "robot_angular_pose"
kSimRobotMotorVoltageArrayKey = "robot_motor_voltage"
kWheelDiameter = 4 * inches_to_meters
kDriveGearingRatio= (50 / 14) * (16 / 28) * (45 / 15)
"""dimensionless"""
kSteerGearingRatio = 150 / 7
"""dimensionless"""
kFrontLeftDriveInverted = True
kFrontRightDriveInverted = False
kBackLeftDriveInverted = True
kBackRightDriveInverted = False

kFrontLeftSteerInverted = False
kFrontRightSteerInverted = False
kBackLeftSteerInverted = False
kBackRightSteerInverted = False

"""
To determine encoder offsets (with robot ON and DISABLED):
  1. Rotate all swerve modules so that the wheels:
     * are running in the forwards-backwards direction
     * have the wheel bevel gears facing inwards towards the
       center-line of the robot
  2. Run Phoenix Tuner
  3. Select desired encoder
  4. Go to "Config" tab
  5. Click "Factory Default"
  6. Go to "Self-Test Snapshot" tab
  7. Click "Self-Test Snapshot"
  8. Record value from line: "Absolute Position (unsigned):"
"""
kFrontLeftAbsoluteEncoderOffset = 0.417969
"""rotations"""

kFrontRightAbsoluteEncoderOffset = -0.055176
"""rotations"""

kBackLeftAbsoluteEncoderOffset = 0.452637
"""rotations"""

kBackRightAbsoluteEncoderOffset = 0.105225
"""rotations"""


kFrontLeftWheelPosition = Translation2d(
    kSwerveModuleCenterToRobotCenterLength,
    kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kFrontRightWheelPosition = Translation2d(
    kSwerveModuleCenterToRobotCenterLength,
    -kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kBackLeftWheelPosition = Translation2d(
    -kSwerveModuleCenterToRobotCenterLength,
    kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kBackRightWheelPosition = Translation2d(
    -kSwerveModuleCenterToRobotCenterLength,
    -kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kCentimetersPerInch = 2.54
"""centimeters / inch"""

kCentimetersPerMeter = 100
"""centimeters / meter"""

kMetersPerInch = kCentimetersPerInch / kCentimetersPerMeter
"""meters / inch"""
kRadiansPerRevolution = 2 * math.pi
"""radians / revolution"""

kDegreesPerRevolution = 360
"""degrees / revolution"""

kRadiansPerDegree = kRadiansPerRevolution / kDegreesPerRevolution
"""radians / degree"""
"""meters"""
kApriltagPositionDict = {
    1: Pose3d(
        (kMetersPerInch * 657.37),
        (kMetersPerInch * 25.80),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 126 * kRadiansPerDegree),
    ),
    2: Pose3d(
        (kMetersPerInch * 657.37),
        (kMetersPerInch * 291.20),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 234 * kRadiansPerDegree),
    ),
    3: Pose3d(
        (kMetersPerInch * 455.15),
        (kMetersPerInch * 317.15),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
    ),
    4: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 241.64),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    5: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 75.39),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    6: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 120.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
    7: Pose3d(
        (kMetersPerInch * 546.87),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    8: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    9: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    10: Pose3d(
        (kMetersPerInch * 481.39),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    11: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    12: Pose3d(
        (kMetersPerInch * 33.51),
        (kMetersPerInch * 25.80),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 54 * kRadiansPerDegree),
    ),
    13: Pose3d(
        (kMetersPerInch * 33.51),
        (kMetersPerInch * 291.20),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 306 * kRadiansPerDegree),
    ),
    14: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 241.64),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    15: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 75.39),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    16: Pose3d(
        (kMetersPerInch * 235.73),
        (kMetersPerInch * -0.15),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 90 * kRadiansPerDegree),
    ),
    17: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    18: Pose3d(
        (kMetersPerInch * 144.00),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    19: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    20: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    21: Pose3d(
        (kMetersPerInch * 209.49),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    22: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
}

kWheelRadius = kWheelDiameter / 2