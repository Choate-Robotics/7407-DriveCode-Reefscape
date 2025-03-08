import commands2
import wpilib.drive
import autos.auto_routine
from toolkit.subsystem import Subsystem

import ntcore
import phoenix6 as ctre
import wpilib

import command
import math
import config
import autos

# import constants
from robot_systems import (  # noqa
    Robot,
    Pneumatics,
    Sensors,
    LEDs,
    PowerDistribution,
    Field,
)
from wpilib import DriverStation, SendableChooser

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
        self.color = DriverStation.Alliance.kRed

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

        try:
            init_subsystems()
        except Exception as e:
            self.log.error(e)
            self.nt.getTable("errors").putString("subsystem init", str(e))
            raise e
        
        self.auto_selection = SendableChooser()
        self.auto_selection.setDefaultOption("Three L4 Right", autos.three_l4_right)
        self.auto_selection.addOption("Three L4 Left", autos.three_l4_left)
        self.auto_selection.addOption("Bump", autos.three_l4_left_bump)
        self.auto_selection.addOption("Leave", autos.leave)

        wpilib.SmartDashboard.putData("Auto", self.auto_selection)

        ctre.hardware.ParentDevice.optimize_bus_utilization_for_all()
        Field.update_field_table()

        self.log.complete("Robot initialized")

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
        Sensors.cam_controller.update_tables()
        ...

    # Initialize subsystems

    # Pneumatics

    def teleopInit(self):
        OI.init()
        OI.map_controls()
        self.scheduler.schedule(commands2.SequentialCommandGroup(
                # command.DrivetrainZero(Robot.drivetrain),
                command.DriveSwerveCustom(Robot.drivetrain)
        ))
        self.scheduler.schedule(commands2.SequentialCommandGroup(
            command.SetWrist(Robot.wrist, 0),
            # command.SetElevator(Robot.elevator, 0),
        ))
        self.log.info("Teleop initialized")

    def teleopPeriodic(self):
        pass

    def autonomousInit(self):
        auto: autos.AutoRoutine = self.auto_selection.getSelected()
        starting_pose: Pose2d = auto.blue_start_pose if DriverStation.getAlliance() == DriverStation.Alliance.kBlue else auto.red_start_pose
        Robot.drivetrain.reset_odometry_auto(starting_pose)
        self.scheduler.schedule(commands2.SequentialCommandGroup(
            command.DrivetrainZero(Robot.drivetrain, starting_pose.rotation().radians()),
            commands2.ParallelCommandGroup(
                auto.command,
                command.DeployClimb(Robot.climber, upper_bound=config.climb_initial_out)
            )
            
        ))

        self.log.info("Autonomous initialized")

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        Robot.drivetrain.set_driver_centric((0, 0), 0)
        self.scheduler.cancelAll()

    def disabledInit(self) -> None:
        self.log.info("Robot disabled")

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(_Robot)
