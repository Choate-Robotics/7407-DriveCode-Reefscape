import pytest
from subsystem import Climber
from unittest.mock import MagicMock

@pytest.fixture
def climber() -> Climber:
    climber = Climber()
    climber.climber_motor = MagicMock()
    return climber

def test_climber_init(climber: Climber):
    climber.init()
    climber.climber_motor.init.assert_called()
    climber.climber_motor.set_sensor_position.assert_called_with(0)

def test_zero(climber: Climber):
    climber.zero()
    climber.climber_motor.set_sensor_position.assert_called_with(0)
    assert climber.zeroed == True

@pytest.mark.parametrize(
    "test_input",
    [
        (0.0),
        (0.5),
        (1.0),
        (-0.5)
    ]
)
def test_set_raw_output(test_input, climber: Climber):
    climber.set_raw_output(test_input)
    climber.climber_motor.set_raw_output.assert_called_with(test_input)

def test_get_motor_revolutions(climber: Climber):
    assert climber.get_motor_revolutions() == climber.climber_motor.get_sensor_position()

def test_update_table(climber: Climber):
    climber.update_table()
    climber.climber_motor.get_sensor_position.assert_called()
    climber.climber_motor.get_motor_current.assert_called()