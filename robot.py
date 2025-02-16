import commands2
import ntcore
import phoenix6 as ctre
import wpilib

import command
import config
import constants
import sensors
import subsystem
import utils
from oi.OI import OI
from robot_systems import Field, LEDs, Pneumatics, PowerDistribution, Robot, Sensors
from toolkit.subsystem import Subsystem


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

        self.log = utils.LocalLogger("Robot")
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.scheduler = commands2.CommandScheduler.getInstance()

    def robotInit(self):
        # self.log._robot_log_setup()
        # Initialize Operator Interface
        if config.DEBUG_MODE == True:
            self.log.setup("WARNING: DEBUG MODE IS ENABLED")
        period = 0.03
        self.scheduler.setPeriod(period)
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
            #     {k: v for k, v in Sensors.__dict__.items() if isinstance(v, sensors.Sensor) and hasattr(v, 'init')}.values()
            # )

            for subsystem in subsystems:
                subsystem.init()

            # for sensor in sensors:
            #     sensor.init()

        if config.DEBUG_MODE == False:
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

        if config.DEBUG_MODE == False:
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

        # Robot.drivetrain.update_tables()
        ...

    # Initialize subsystems

    # Pneumatics

    def teleopInit(self):
        self.log.info("Teleop initialized")
        OI.init()
        OI.map_controls()
        # self.scheduler.schedule(commands2.SequentialCommandGroup(
        #     command.DrivetrainZero(Robot.drivetrain),
        #     command.DriveSwerveCustom(Robot.drivetrain)
        #     ))

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
