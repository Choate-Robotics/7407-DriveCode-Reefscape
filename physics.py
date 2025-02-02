# This file is derived from 1757 Westwood Robotics physics.py on 2/2/2025 in their main
# branch for their 2025 robot codebase. The original file can be found at
# https://github.com/1757WestwoodRobotics/2025-Reefscape/blob/main/physics.py
#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import typing
from ntcore import NetworkTableInstance
from phoenix6.sim.cancoder_sim_state import CANcoderSimState
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState
from phoenix6.unmanaged import feed_enable
#from photonlibpy.simulation.photonCameraSim import PhotonCameraSim
#from photonlibpy.simulation.visionSystemSim import VisionSystemSim
#from photonlibpy.simulation.simCameraProperties import SimCameraProperties
from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d, Pose3d
from wpimath.system.plant import DCMotor, LinearSystemId
import wpimath.kinematics
from pyfrc.physics.core import PhysicsInterface
import constants
from robot import _Robot
from subsystem import Drivetrain
from robot_systems import Robot #DriveSubsystem
#from subsystems.visionsubsystem import VisionSubsystem
from toolkit.utils.toolkit_math import clamp
from utils.motorsimulator import MotorSimulator


class SwerveModuleSim:
    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        position: Translation2d,
        wheelMotorType: DCMotor,
        wheelMotorSim: typing.Callable[[], TalonFXSimState],
        driveMotorGearing: float,
        swerveMotorType: DCMotor,
        swerveMotorSim: typing.Callable[[], TalonFXSimState],
        steerMotorGearing: float,
        swerveEncoderSim: typing.Callable[[], CANcoderSimState],
        encoderOffset: float,
        inverted: bool,
    ) -> None:
        self.position = position
        self.wheelMotorSim = wheelMotorSim
        self.wheelMotorType = wheelMotorType
        self.driveMotorGearing = driveMotorGearing
        self.wheelMotorInternalSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                self.wheelMotorType,
                constants.kSimulationRotationalInertia,
                self.driveMotorGearing,
            ),
            self.wheelMotorType,
        )
        self.swerveMotorSim = swerveMotorSim
        self.swerveMotorType = swerveMotorType
        self.steerMotorGearing = steerMotorGearing
        self.steerMotorIntenalSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                self.swerveMotorType,
                constants.kSimulationRotationalInertia,
                self.steerMotorGearing,
            ),
            self.swerveMotorType,
        )
        self.swerveEncoderSim = swerveEncoderSim
        self.encoderOffset = encoderOffset + 0.25

        self.multiplier = -1 if inverted else 1

    def __str__(self) -> str:
        return f"pos: x={self.position.X():.2f} y={self.position.Y():.2f}"


class SwerveDriveSim:
    def __init__(self, swerveModuleSims: typing.Tuple[SwerveModuleSim, ...]) -> None:
        self.swerveModuleSims = swerveModuleSims
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            *(module.position for module in swerveModuleSims)
        )
        self.pose = constants.kSimDefaultRobotLocation
        self.outputs = None

        self.robotVelocityPublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(
                constants.kSimRobotVelocityArrayKey, wpimath.kinematics.ChassisSpeeds
            )
            .publish()
        )

    def resetPose(self, pose) -> None:
        self.pose = pose

    def getPose(self) -> Pose2d:
        return self.pose

    def getHeading(self) -> Rotation2d:
        return self.pose.rotation()

    def update(self, tm_diff: float, robotVoltage: float) -> None:
        deltaT = tm_diff

        states = []
        for module in self.swerveModuleSims:
            module.wheelMotorInternalSim.setInputVoltage(
                module.wheelMotorSim().motor_voltage
            )
            # print(module.wheelMotorSim().motor_voltage)
            module.wheelMotorInternalSim.update(tm_diff)
            wheel_position_rot = (
                module.wheelMotorInternalSim.getAngularPosition()
                / constants.kRadiansPerRevolution
                * module.driveMotorGearing
            )
            wheel_velocity_rps = (
                module.wheelMotorInternalSim.getAngularVelocity()
                / constants.kRadiansPerRevolution
                * module.driveMotorGearing
            )
            module.wheelMotorSim().set_raw_rotor_position(wheel_position_rot)
            module.wheelMotorSim().set_rotor_velocity(wheel_velocity_rps)
            module.wheelMotorSim().set_supply_voltage(
                clamp(
                    robotVoltage
                    - module.wheelMotorSim().supply_current
                    * constants.kSimMotorResistance,
                    0,
                    robotVoltage,
                )
            )

            module.steerMotorIntenalSim.setInputVoltage(
                module.swerveMotorSim().motor_voltage
            )
            module.steerMotorIntenalSim.update(tm_diff)
            swerve_position_rot = (
                module.steerMotorIntenalSim.getAngularPosition()
                / constants.kRadiansPerRevolution
                * module.steerMotorGearing
            )
            swerve_velocity_rps = (
                module.steerMotorIntenalSim.getAngularVelocity()
                / constants.kRadiansPerRevolution
                * module.steerMotorGearing
            )
            module.swerveMotorSim().set_raw_rotor_position(swerve_position_rot)
            module.swerveMotorSim().set_rotor_velocity(swerve_velocity_rps)
            module.swerveMotorSim().set_supply_voltage(
                clamp(
                    robotVoltage
                    - module.swerveMotorSim().supply_current
                    * constants.kSimMotorResistance,
                    0,
                    robotVoltage,
                )
            )
            module.swerveEncoderSim().set_raw_position(
                -swerve_position_rot / module.steerMotorGearing + module.encoderOffset
            )
            module.swerveEncoderSim().set_velocity(
                -swerve_velocity_rps / module.steerMotorGearing
            )

            wheelLinearVelocity = (
                wheel_velocity_rps
                * module.multiplier
                * constants.kWheelRadius
                * constants.kRadiansPerRevolution
                / constants.kDriveGearingRatio
            )

            state = wpimath.kinematics.SwerveModuleState(
                -wheelLinearVelocity,
                Rotation2d(
                    -swerve_position_rot
                    / module.steerMotorGearing
                    * constants.kRadiansPerRevolution
                ),
            )
            states.append(state)

        chassisSpeed = self.kinematics.toChassisSpeeds(states)
        deltaHeading = chassisSpeed.omega * deltaT
        deltaX = chassisSpeed.vx * deltaT
        deltaY = chassisSpeed.vy * deltaT

        self.robotVelocityPublisher.set(chassisSpeed)

        deltaTrans = Transform2d(deltaX, deltaY, deltaHeading)

        newPose = self.pose + deltaTrans
        self.pose = newPose


# class VisionSim:
#     def __init__(self, visionSubsystem: VisionSubsystem) -> None:
#         self.sim = VisionSystemSim("main")
#         self.sim.addAprilTags(constants.kApriltagFieldLayout)
#
#         cameraProps = SimCameraProperties()
#         cameraProps.setCalibrationFromFOV(
#             1280, 800, Rotation2d.fromDegrees(constants.kCameraFOVVertical)
#         )
#         cameraProps.setCalibError(0.35, 0.1)
#         cameraProps.setFPS(30)
#         cameraProps.setAvgLatency(50)
#         cameraProps.setLatencyStdDev(15)
#
#         self.cameras = [PhotonCameraSim(cam.camera) for cam in visionSubsystem.cameras]
#         for cam, transform in zip(self.cameras, visionSubsystem.cameras):
#             self.sim.addCamera(cam, transform.robotToCameraTransform)
#             # cam.enableDrawWireframe(True) not implemented in current version
#
#     def update(self, robotPose: Pose2d):
#         self.sim.update(robotPose)
#

class PhysicsEngine:
    """
    Simulates a drivetrain
    """

    # pylint: disable-next=unused-argument
    def __init__(self, physics_controller: PhysicsInterface, robot: _Robot):
        #assert robot.container is not None
        self.physics_controller = physics_controller
        self.bot = robot

        driveSubsystem: Drivetrain = Robot.drivetrain

        frontLeftSim = driveSubsystem.n_front_left.getSimulator()
        self.frontLeftModuleSim = SwerveModuleSim(
            constants.kFrontLeftWheelPosition,
            DCMotor.krakenX60(),
            frontLeftSim[0],
            constants.kDriveGearingRatio,
            DCMotor.falcon500(),
            frontLeftSim[1],
            constants.kSteerGearingRatio,
            frontLeftSim[2],
            constants.kFrontLeftAbsoluteEncoderOffset,
            constants.kFrontLeftDriveInverted,
        )
        frontRightSim = driveSubsystem.n_front_right.getSimulator()
        self.frontRightModuleSim = SwerveModuleSim(
            constants.kFrontRightWheelPosition,
            DCMotor.krakenX60(),
            frontRightSim[0],
            constants.kDriveGearingRatio,
            DCMotor.falcon500(),
            frontRightSim[1],
            constants.kSteerGearingRatio,
            frontRightSim[2],
            constants.kFrontRightAbsoluteEncoderOffset,
            constants.kFrontRightDriveInverted,
        )
        backLeftSim = driveSubsystem.n_back_left.getSimulator()
        self.backSimLeftModule = SwerveModuleSim(
            constants.kBackLeftWheelPosition,
            DCMotor.krakenX60(),
            backLeftSim[0],
            constants.kDriveGearingRatio,
            DCMotor.falcon500(),
            backLeftSim[1],
            constants.kSteerGearingRatio,
            backLeftSim[2],
            constants.kBackLeftAbsoluteEncoderOffset,
            constants.kBackLeftDriveInverted,
        )
        backRightSim = driveSubsystem.n_back_right.getSimulator()
        self.backSimRightModule = SwerveModuleSim(
            constants.kBackRightWheelPosition,
            DCMotor.krakenX60(),
            backRightSim[0],
            constants.kDriveGearingRatio,
            DCMotor.falcon500(),
            backRightSim[1],
            constants.kSteerGearingRatio,
            backRightSim[2],
            constants.kBackRightAbsoluteEncoderOffset,
            constants.kBackRightDriveInverted,
        )

        self.swerveModuleSims = [
            self.frontLeftModuleSim,
            self.frontRightModuleSim,
            self.backSimLeftModule,
            self.backSimRightModule,
        ]

        self.driveSim = SwerveDriveSim(tuple(self.swerveModuleSims))
        #self.visionSim = VisionSim(robot.container.vision)

        driveSubsystem.resetSimPosition = self.driveSim.resetPose

        self.gyroSim = driveSubsystem.gyro._gyro.sim_state

        self.sim_initialized = False

        self.motorsim = MotorSimulator()

        self.fieldSimTargetPublisher = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic(constants.kFieldSimTargetKey, Pose3d)
            .publish()
        )
        self.fieldSimTargetPublisher.set(list(constants.kApriltagPositionDict.values()))

        self.simRobotPosePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kSimRobotPoseArrayKey, Pose2d)
            .publish()
        )

    # pylint: disable-next=unused-argument
    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        feed_enable(1 / 50)

        if not self.sim_initialized:
            self.sim_initialized = True
            # self.physics_controller.field, is not set until simulation_init

        self.gyroSim.set_raw_yaw(self.driveSim.getHeading().degrees())

        # Simulate the drivetrain
        voltage = RobotController.getInputVoltage()

        self.motorsim.update(tm_diff, voltage)
        self.driveSim.update(tm_diff, voltage)

        simRobotPose = self.driveSim.getPose()
        self.physics_controller.field.setRobotPose(simRobotPose)

        #self.visionSim.update(simRobotPose)

        # publish the simulated robot pose to nt
        self.simRobotPosePublisher.set(simRobotPose)