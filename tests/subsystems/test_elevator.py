import pytest
from pytest import MonkeyPatch

import config
import constants 
from subsystem import Elevator 
from unittest.mock import MagicMock

@pytest.fixture
def elevator() -> Elevator: 
    elevator = Elevator()
    elevator.leader_motor = MagicMock()
    elevator.follower_motor = MagicMock()
    elevator.magsensor = MagicMock()
    return elevator 

def test_elevator_init(elevator: Elevator):
    elevator.init()
    elevator.leader_motor.init.assert_called()
    elevator.follower_motor.init.assert_called()
    elevator.leader_motor.set_sensor_position.assert_called()

@pytest.mark.parametrize(
    "test_input, test_output",
    [
        (0, 0),
        (constants.elevator_max_height/2, constants.elevator_max_height/2),
        (constants.elevator_max_height*2, constants.elevator_max_height),
        (-10, 0)
    ]
)
def test_limit_height(test_input, test_output, elevator: Elevator):
    assert elevator.limit_height(test_input) == test_output

@pytest.mark.parametrize(
    "test_input",
    [
        (0),
        (constants.elevator_max_height/2),
        (constants.elevator_max_height)
    ]
)
def test_set_position(test_input, elevator: Elevator):
    elevator.set_position(test_input)
    elevator.leader_motor.set_target_position.assert_called_with(
        (test_input * constants.elevator_gear_ratio) / constants.elevator_driver_gear_circumference
    )

def test_set_zero(elevator: Elevator):
    elevator.set_zero()
    elevator.leader_motor.set_target_position.assert_called_with(0)

@pytest.mark.parametrize(
    "test_input, test_output",
    [
        (0, 0),
        ((constants.elevator_max_height/2)
         /constants.elevator_driver_gear_circumference
         *constants.elevator_gear_ratio, constants.elevator_max_height/2),
         (constants.elevator_max_height
         /constants.elevator_driver_gear_circumference
         *constants.elevator_gear_ratio, constants.elevator_max_height)
    ]
)
def test_get_position(test_input, test_output, elevator: Elevator, monkeypatch: MonkeyPatch):
    monkeypatch.setattr(
        elevator.leader_motor, "get_sensor_position", lambda: test_input
    )
    assert(elevator.get_position() == test_output)