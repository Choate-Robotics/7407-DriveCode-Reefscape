import commands2
from toolkit.subsystem import Subsystem

# import phoenix6 as ctre
import ntcore
import wpilib
import command
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

# import sensors
# import subsystem
import utils
from oi.OI import OI


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.field_constants = utils.FieldConstants()

        self.log = utils.LocalLogger("Robot")
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.scheduler = commands2.CommandScheduler.getInstance()

    def robotInit(self):
        # self.log._robot_log_setup()
        # Initialize Operator Interface
        if config.DEBUG_MODE:
            self.log.setup("WARNING: DEBUG MODE IS ENABLED")
        OI.init()
        OI.map_controls()
        period = 0.03
        self.scheduler.setPeriod(period)
        self.log.info(f"Scheduler period set to {period} seconds")

        self.field_constants.update_tables()

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

        self.log.complete("Robot initialized")
        ...

    def robotPeriodic(self):
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

        Robot.drivetrain.update_tables()
        ...

    # Initialize subsystems

    # Pneumatics

    def teleopInit(self):
        self.log.info("Teleop initialized")
        self.scheduler.schedule(
            commands2.SequentialCommandGroup(
                command.DrivetrainZero(Robot.drivetrain),
                command.DriveSwerveCustom(Robot.drivetrain),
            )
        )

    def teleopPeriodic(self):
        pass

    def autonomousInit(self):
        self.log.info("Autonomous initialized")

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        self.log.info("Robot disabled")

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(_Robot)
