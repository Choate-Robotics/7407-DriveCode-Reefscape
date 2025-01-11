import pytest
from toolkit.motors.rev_motors import SparkMax
from phoenix6 import TalonFX, ControlMode
from toolkit.subsystem import Subsystem as Subsystem_C

# to test a subsystem, you need to create a mechanism

# this will override the subsystem's motors and devices with mock versions of them

# this will allow you to test the subsystem without having to worry about the motors and devices, and streamline the process

# if you need to add more motors or devices, you can create a child class of the device and override the methods you need to test

# examples of this are the MockSparkMotor and MockTalonFX classes below

# you can also remove motors and devices by using the remove_parent_variables method in the Mechanism_C class


def mechanism(selected_subsystem: Subsystem_C):
    class MockSparkMotor(SparkMax):
        def __init__(self, motor: SparkMax):
            super().__init__(
                motor._can_id, motor._inverted, motor._brushless, motor._config
            )
            self._output = 0
            self._velocity = 0
            self._position = 0

        def set_target_velocity(self, vel):
            self._velocity = vel

        def get_sensor_velocity(self):
            return self._velocity

        def get_target_velocity(self):
            return self._velocity

        def set_target_position(self, target):
            self._position = target

        def get_sensor_position(self):
            return self._position

    class MockTalonFX(TalonFX):
        def __init__(self, talon: TalonFX):
            super().__init__(talon.getDeviceID())
            self._motor_output_percent = 0
            self._selected_sensor_velocity = 0
            self._selected_sensor_position = 0

        def getMotorOutputPercent(self):
            return self._motor_output_percent

        def set(self, mode, value):
            if mode == ControlMode.PercentOutput:
                self._motor_output_percent = value
            elif mode == ControlMode.Velocity:
                self._selected_sensor_velocity = value
            elif mode == ControlMode.Position:
                self._selected_sensor_position = value

        def getSelectedSensorVelocity(self):
            return self._selected_sensor_velocity

        def getSelectedSensorPosition(self, pidIdx: int = 0) -> float:
            return self._selected_sensor_position

    # add any extra motors or devices below

    # this is the class that will be returned
    # this creates a child class of the selected subsystem
    # Its identical to the parent, except any methods or variables that the class overrides
    class Mechanism_C(selected_subsystem):
        def __init__(self):
            super().__init__()

        def init(self):
            self.replace_parent_variables(SparkMax, MockSparkMotor)
            self.replace_parent_variables(TalonFX, MockTalonFX)
            # replace any motors or devices here

            super().init()

        def get_parent_variables_by_type(self, type):
            """
            Returns a list of all the variable names of the parent class that are of the specified type
            """
            return [
                attr
                for attr in dir(selected_subsystem)
                if not callable(getattr(selected_subsystem, attr))
                and not attr.startswith("__")
                and isinstance(getattr(selected_subsystem, attr), type)
            ]

        def replace_parent_variables(self, type, new_type):
            """
            Replaces all the variables of the parent class that are of the specified type with the new type
            """
            for attr in self.get_parent_variables_by_type(type):
                super().__setattr__(attr, new_type(getattr(selected_subsystem, attr)))

        def set_parent_variables(self, type, value):
            """
            Sets all the variables of the parent class that are of the specified type to the specified value

            NOTE: This is setting the value to a primitive type, so it will not work for objects
            """
            for attr in self.get_parent_variables_by_type(type):
                super().__setattr__(attr, value)

        def remove_parent_variables(self, type):
            """
            Removes all the variables of the parent class that are of the specified type
            """
            for attr in self.get_parent_variables_by_type(type):
                delattr(self, attr)

        def deconstruct(self):
            self.remove_parent_variables(SparkMax)
            self.remove_parent_variables(TalonFX)
            # remove any motors or devices here

    return Mechanism_C()
