import commands2
import wpilib.drive
from toolkit.subsystem import Subsystem

import ntcore
import phoenix6 as ctre
import wpilib

import command
import math
import config

# import constants
from robot_systems import (  # noqa
    Robot,
    Pneumatics,
    Sensors,
    LEDs,
    PowerDistribution,
    Field,
)
from wpilib import DriverStation

# import sensors
# import subsystem
import utils
from oi.OI import OI
from pathplannerlib.auto import PathPlannerPath, FollowPathCommand, AutoBuilder
from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from utils import get_red_pose


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

        self.log = utils.LocalLogger("Robot")
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.color = DriverStation.Alliance.kBlue

    def robotInit(self):
        self.log._robot_log_setup()
        # Initialize Operator Interface
        if config.DEBUG_MODE:
            self.log.setup("WARNING: DEBUG MODE IS ENABLED")
        period = config.period
        self.scheduler.setPeriod(period)

        Field.flip_poses()
        Field.update_field_table("Field")
        self.log.info(f"Scheduler period set to {period} seconds")

        # Initialize subsystems
        def init_subsystems():
            subsystems: list[Subsystem] = list(
                {
                    k: v
                    for k, v in Robot.__dict__.items()
                    if isinstance(v, Subsystem) and hasattr(v, "init")
                }.values()
            )

            # sensors: list = list(
            #     {k: v for k, v in Sensors.__dict__.items()
            #       if isinstance(v, sensors.Sensor) and hasattr(v, 'init')}.values()
            # )

            for subsystem in subsystems:
                subsystem.init()

            # for sensor in sensors:
            #     sensor.init()

        if not config.DEBUG_MODE:
            try:
                init_subsystems()
            except Exception as e:
                self.log.error(e)
                self.nt.getTable("errors").putString("subsystem init", str(e))
        else:
            try:
                init_subsystems()
            except Exception as e:
                self.log.error(e)
                self.nt.getTable("errors").putString("subsystem init", str(e))
                raise e

        ctre.hardware.ParentDevice.optimize_bus_utilization_for_all()
        Field.update_field_table()

        self.log.complete("Robot initialized")
        ...

    def robotPeriodic(self):
        table = ntcore.NetworkTableInstance.getDefault().getTable("Color")
        table.putValue("self.color", self.color)

        fms_table = ntcore.NetworkTableInstance.getDefault().getTable("FMSInfo")
        is_red = fms_table.getBoolean("IsRedAlliance", True)
        if is_red:
            color_now = DriverStation.Alliance.kRed
        else:
            color_now = DriverStation.Alliance.kBlue

        # current_alliance = DriverStation.getAlliance()
        if not color_now == self.color:
            Field.flip_poses()
            self.color = color_now
            Field.update_field_table()
        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        if not config.DEBUG_MODE:
            try:
                self.scheduler.run()
            except Exception as e:
                self.log.error(e)
                self.nt.getTable("errors").putString("command scheduler", str(e))
        else:
            try:
                self.scheduler.run()
            except Exception as e:
                self.log.error(e)
                self.nt.getTable("errors").putString("command scheduler", str(e))
                raise e

        # Field.odometry.disable()
        pose = Field.odometry.update()

        self.nt.getTable("Odometry").putNumberArray(
            "Estimated pose", [pose.X(), pose.Y(), pose.rotation().radians()]
        )

        Robot.drivetrain.update_tables()
        # Sensors.cam_controller.update_tables()
        ...

    # Initialize subsystems

    # Pneumatics

    def teleopInit(self):
        OI.init()
        OI.map_controls()
        self.scheduler.schedule(
            commands2.SequentialCommandGroup(
                command.SetWrist(Robot.wrist, 0),
                command.SetElevator(Robot.elevator, 0),
            )
        )
        self.scheduler.schedule(
            commands2.SequentialCommandGroup(
                command.DrivetrainZero(Robot.drivetrain),
                command.DriveSwerveCustom(Robot.drivetrain),
            )
        )
        self.log.info("Teleop initialized")

    def teleopPeriodic(self):
        pass

    def autonomousInit(self):
        path = PathPlannerPath.fromChoreoTrajectory("Four L4 Left")
        starting_pose = (
            get_red_pose(path.getStartingHolonomicPose())
            if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
            else path.getStartingHolonomicPose()
        )
        Robot.drivetrain.reset_odometry_auto(starting_pose)
        self.scheduler.schedule(
            commands2.SequentialCommandGroup(
                command.DrivetrainZero(
                    Robot.drivetrain, starting_pose.rotation().radians()
                ),
                AutoBuilder.followPath(path),
                commands2.InstantCommand(
                    lambda: Robot.drivetrain.set_robot_centric((0, 0, 0))
                ),
            )
        )

        # self.scheduler.schedule(commands2.SequentialCommandGroup(
        #     command.DrivetrainZero(Robot.drivetrain),
        #     command.FindWheelRadius(Robot.drivetrain)
        # ))

        self.log.info("Autonomous initialized")

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        self.log.info("Robot disabled")

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(_Robot)
