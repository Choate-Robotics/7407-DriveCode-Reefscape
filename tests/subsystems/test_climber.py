from unittest.mock import MagicMock

import pytest
import rev
from pytest import MonkeyPatch

import config
import constants
from subsystem import Climber
from toolkit.motors.ctre_motors import TalonFX

@pytest.fixture()
def climber() -> Climber:
    # Create a climber, but it has mock
    # classes for its dependencies
    my_climber = Climber()
    my_climber.climber_motor = MagicMock()
    my_climber.climber_motor_follower = MagicMock()
    # my_climber.init()
    return my_climber

def climber_zero(climber: Climber):

    climber.init()
    climber.climber_motor.init.assert_called()
    climber.climber_motor_follower.init.assert_called()

@pytest.mark.parametrize(
    "test_input",
    [
        (1),
        (2),
        (3),
        (4)
    ]
)

def test_set_angle(test_input, climber: Climber):
    climber.set_angle(test_input)
    climber.climber_motor.set_target_position.assert_called_with(test_input * constants.climber_gear_ratio)

def test_get_angle(climber: Climber):
    climber.get_angle()
    climber.climber_motor.get_sensor_position.assert_called()

@pytest.mark.parametrize(
    "test_input",
    [
        (1),
        (2),
        (3),
        (4)
    ]
)

def test_set_raw_output(test_input, climber: Climber):
    climber.set_raw_output(test_input)
    climber.climber_motor.set_raw_output.assert_called_with(test_input)
    climber.climber_motor.set_raw_output.assert_called_with(test_input)

def test_get_raw_output(climber: Climber):
    climber.get_raw_output()
    climber.climber_motor.get_applied_output.assert_called()