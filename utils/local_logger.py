import os
from enum import IntEnum

from wpilib import DataLogManager, DriverStation, TimedRobot, Timer
from wpilib.deployinfo import getDeployData
from wpiutil.log import StringLogEntry

import config


class BColors:
    """
    ANSI escape codes for colors
    """

    TEST = "\u001b[41;1m"
    TIME = "\u001b[35;1m"
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"
    SETUP = "\u001b[46;1m"


class LocalLogger:
    """
    A logger that logs to the driver station and a file accessible from a USB

    This logger is intended to be instantiated wherever it is needed.

    This will keep messages organized and sorted for easy reading.

    :param name: The name of the thing being logged
    :type name: str"""

    class LogLevels(IntEnum):
        """
        Log levels for the logger

        Higher level = less information

        level 0 will log everything

        level 1 will log everything except debug

        levels:

        0 = DEBUG (ALL)

        1 = INFO

        2 = WARNING

        3 = ERROR

        4 = SETUP
        """

        DEBUG = 0

        INFO = 1

        WARNING = 2

        ERROR = 3

        SETUP = 4

        MATCH = 5

    log_data = None
    custom_entry = None

    def __init__(self, name: str):
        usb_path = "/u/logs"
        self.name: str = name
        self.dlm: DataLogManager = DataLogManager
        if config.LOGGING:
            # Check if the USB mount exists
            if os.path.exists(usb_path):
                self.dlm.start(usb_path)
                self.dlm.log("Logging to USB stick at /u/logs")
            else:
                self.dlm.start("logs/")  # Default internal storage
                self.dlm.log("USB not found, logging to internal storage.")

            # self.dlm.start("logs/")
            self.log_data = self.dlm.getLog()
            self.custom_entry = StringLogEntry(self.log_data, f"messages/{self.name}")

    def _robot_log_setup(self):
        """
        Sets up the general robot logging.

        Prints out deploy information and starts logging DriverStation data to the file.

        This should only be called in robotInit
        """
        self.get_log_levels()
        self.get_deploy_info()
        # deploy_msg = f"Deploy Info\n Branch: {branch}\n Deployment Date: {date}\n Deployed By: {by}"
        # self.setup(deploy_msg)
        self.log_driverstation(True)
        self.setup("Robot logging initialized")

    def get_deploy_info(self):
        """
        Reports the deploy information and logs it to the file.
        """
        data = getDeployData()
        if data:
            for key, value in data.items():
                self.setup(f"Deploying {str(key)}: {str(value)}")

    def get_log_levels(self):
        """
        Returns the current log levels.

        :returns: str"""

        if config.LOGGING:
            self.setup("Logging to file is enabled")
        else:
            self.warn("WARNING: Logging to file is disabled")

        self.setup(f"Log Out Level: {config.LOG_OUT_LEVEL}")

    def log_driverstation(self, joysticks: bool):
        """
        Enables logging of DriverStation data to the file.

        :param joysticks: Whether or not to log joystick data"""
        if not config.LOGGING:
            return
        DriverStation.startDataLog(self.log_data, joysticks)
        self.setup("DriverStation logging started")
        if joysticks:
            self.setup("Joystick logging started")

    def __pms(self, colors: bool = True):
        """
        Returns a string with the current match time, mode, and simulation status.

        This is intended to be used as a prefix for logging, and should not be used outside of this class.

        :param colors: Whether or not to use colors
        :type colors: bool"""
        sim_color = ""
        header_color = ""
        time_color = ""
        end_color = ""

        if colors:
            sim_color = BColors.TEST
            header_color = BColors.HEADER
            time_color = BColors.TIME
            end_color = BColors.ENDC

        mode = "DISABLED"
        is_sim = (
            f"{sim_color}SIMULATION{end_color}" if TimedRobot.isSimulation() else ""
        )
        if DriverStation.isEnabled():
            mode = "TELEOP" if DriverStation.isTeleopEnabled() else "AUTONOMOUS"

        mode = f"{header_color}{mode}{end_color}"

        combined = mode + "  " + is_sim

        time = Timer.getFPGATimestamp()
        time = f"{time:.3f}"
        if not TimedRobot.isSimulation():
            time = Timer.getMatchTime()
            time = f"{time:.3f}"

        return f"  {time}{time_color}  {combined}{end_color}"

    def __format_log_type(self, type: str):
        """
        Returns a formatted log type.

        This should not be used outside of this class.

        :param type: The type of log
        :type type: str
        """
        return f"  |  {type}  |  "

    def __format_std_out(self, color: BColors, type, message):
        """
        Returns a formatted string for printing to the console.

        This should not be used outside of this class.

        :param color: The color of the log
        :type color: BColors
        :param type: The type of log
        :type type: str
        :param message: The message to log
        :type message: str

        returns: str"""
        type = self.__format_log_type(type)
        return f"{self.__pms()}{color}{type}{self.name}: {message}{BColors.ENDC}"

    def __log(self, message, type, color, level: LogLevels, std_out: bool = True):
        """
        Logs a message to the file and prints it to the console.

        This should not be used outside of this class.
        """
        if std_out and config.LOG_OUT_LEVEL <= level:
            print(self.__format_std_out(color, type, message))
        if config.LOGGING and config.LOG_FILE_LEVEL <= level:
            self.custom_entry.append(f"{self.__pms(False)}{type}{self.name}: {message}")

    def message(self, message: str):
        """
        Logs a message to the file without printing it to the console.

        This does not log a type.

        :param message: The message to log
        """
        self.__log(message, "", "", self.LogLevels.INFO, False)

    def info(self, message: str, std_out: bool = True):
        """
        Logs an info message to the file and prints it to the console.

        :param message: The message to log
        """
        self.__log(message, "INFO", BColors.OKBLUE, self.LogLevels.INFO, std_out)

    def debug(self, message: str, std_out: bool = True):
        """
        Logs a debug message to the file and prints it to the console.

        :param message: The message to log"""
        self.__log(message, "DEBUG", BColors.OKCYAN, self.LogLevels.DEBUG, std_out)

    def complete(self, message: str, std_out: bool = True):
        """
        Logs a completion message to the file and prints it to the console.

        :param message: The message to log"""
        self.__log(message, "DONE", BColors.OKGREEN, self.LogLevels.INFO, std_out)

    def warn(self, message: str, std_out: bool = True):
        """
        Logs a warning message to the file and prints it to the console.

        :param message: The message to log"""
        self.__log(message, "WARN", BColors.WARNING, self.LogLevels.WARNING, std_out)

    def error(self, message: str, std_out: bool = True):
        """
        Logs an error message to the file and prints it to the console.

        This is typically used with a try/except block.

        If you are using this because of a breaking error, it may be worth considering patching the error instead.

        If you want to log an exception, use the traceback module.

        :param message: The message to log"""
        self.__log(message, "ERROR", BColors.FAIL, self.LogLevels.ERROR, std_out)

    def setup(self, message: str, std_out: bool = True):
        """
        Logs a setup message to the file and prints it to the console.

        :param message: The message to log"""
        self.__log(message, "SETUP", BColors.SETUP, self.LogLevels.SETUP, std_out)
