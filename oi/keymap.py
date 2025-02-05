import wpilib
import commands2.button

import config

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
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.X)
    class Intake:
        INTAKE_CORAL = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.RT) > config.Trigger_Threshold)
        EXTAKE_CORAL = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.LT) > config.Trigger_Threshold)
        INTAKE_ALGAE = commands2.button.JoystickButton(Controllers.OPERATOR, controllerOPERATOR.RB)
        EXTAKE_ALGAE = commands2.button.JoystickButton(Controllers.OPERATOR, controllerOPERATOR.LB)
        REMOVE_ALGAE = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV == 0
        )
    class Scoring:
        SCORE_L1 = commands2.button.JoystickButton(Controllers.OPERATOR, controllerOPERATOR.B)
        SCORE_L2 = commands2.button.JoystickButton(Controllers.OPERATOR, controllerOPERATOR.A)
        SCORE_L3 = commands2.button.JoystickButton(Controllers.OPERATOR, controllerOPERATOR.X)
        SCORE_L4 = commands2.button.JoystickButton(Controllers.OPERATOR, controllerOPERATOR.Y)
    class climb:
        CLIMB_UNLOCK = commands2.button.JoystickButton(Controllers.OPERATOR, controllerOPERATOR.START)
        CLIMB = commands2.button.JoystickButton(Controllers.DRIVER, controllerOPERATOR.START)
    class Elevator:
        REZERO_ELEVATOR = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 180
        )