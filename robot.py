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
from robot_systems import (
    Field,
    LEDs,
    Pneumatics,
    PowerDistribution,
    Robot,
    Sensors,
    init_subsystems,
)


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
        OI.init()
        OI.map_controls()

        self.scheduler.setPeriod(constants.robot_period)
        self.log.info(f"Scheduler period set to {constants.robot_period} seconds")

        try:
            init_subsystems(Robot)
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable("errors").putString("subsystem init", str(e))

        self.log.complete("Robot initialized")

    def robotPeriodic(self):
        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        try:
            self.scheduler.run()
        except Exception as e:
            self.log.error(str(e))
            self.nt.getTable("errors").putString("command scheduler", str(e))
            raise e

        Robot.drivetrain.update_tables()

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
