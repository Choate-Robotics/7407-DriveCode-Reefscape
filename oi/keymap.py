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
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.X
        )
        DRIVE_TO_RIGHT_POSE = commands2.button.Trigger(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.RT) > config.trigger_threshold
        )
        DRIVE_TO_LEFT_POSE = commands2.button.Trigger(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.LT) > config.trigger_threshold
        )
        ALGAE_ALIGN = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.RB
        )

    class Intake:
        INTAKE_CORAL = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.RT) > config.trigger_threshold
        )

        INTAKE_L1 = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.LT) > config.trigger_threshold
        )

        INTAKE_ALGAE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.RB
        )
        INTAKE_ALGAE_DRIVER = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.LB
        )

    class Scoring:
        SCORE_L1 = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.X
        )
        SCORE_L2 = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.A
        )
        SCORE_L3 = commands2.button.JoystickButton(Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.B
        )
        SCORE_L4 = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.Y
        )
        SCORE_BARGE = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 0
        )

    # to do: fix climb controllers: does not do what it's supposed to
    class Climb:
        CLIMB_UNLOCK = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.SELECT
        )
        CLIMB = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.START
        )
        MANUAL_CLIMB_DEPLOY = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 270
        )
        MANUAL_CLIMB = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 90
        )

    class Wrist:
        REMOVE_ALGAE = commands2.button.Trigger(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 180
        )
        EXTAKE_CORAL = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.A
        )
        EXTAKE_ALGAE_OPERATOR = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.LB
        )
        EXTAKE_ALGAE_DRIVER = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.B
        )
        EXTAKE_ALGAE_BARGE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.R_3
        )
    class Elevator:
        pass
        