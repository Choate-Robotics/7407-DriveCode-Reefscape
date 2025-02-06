import wpilib
import commands2.button

from toolkit.oi import (
    XBoxController,
    LogitechController,
    JoystickAxis,
    Joysticks,
    DefaultButton,
)

controllerDRIVER = XBoxController
controllerOPERATOR = XBoxController


class Controllers:
    DRIVER: int = 0
    OPERATOR: int = 1

    DRIVER_CONTROLLER = wpilib.Joystick(0)
    OPERATOR_CONTROLLER = wpilib.Joystick(1)


class Keymap:
    class Drivetrain:
        DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[0])
        DRIVE_Y_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[1])
        DRIVE_ROTATION_AXIS = JoystickAxis(
            Controllers.DRIVER, controllerDRIVER.R_JOY[0]
        )
        RESET_GYRO = commands2.button.Trigger(
            lambda: Controllers.DRIVER_CONTROLLER.getPOV() == 180
        )
        X_MODE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.X
        )
        DRIVE_TO_RIGHT_POSE = commands2.button.Trigger(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.RT) > 0.4
        )
        DRIVE_TO_LEFT_POSE = commands2.button.Trigger(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.LT) > 0.4
        )
        RESET_POSE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.B
        )