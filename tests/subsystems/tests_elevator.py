import pytest
from pytest import MonkeyPatch

import config
import constants 
from subsystem import Elevator 
from unittest.mock import MagicMock

@pytest.fixture
def elevator() -> Elevator: 
    elevator = Elevator()
    elevator.leader_motor = MagicMock
    elevator.follower_motor = MagicMock
    return elevator 

def test_elevator_init(elevator: Elevator):
    elevator.init()
    elevator.leader_motor.init.assert_called()
    elevator.follower_motor.init.assert_called()

@pytest.mark.parametrize(
    "test_input",
    [
        """
        insert whatever testing values
        """
    ]
)
def test_set_position(test_input, elevator: Elevator):
    elevator.set_position(test_input)
    elevator.leader_motor.set_target_position.assert_called_with(
        (test_input * constants.elevator_gear_ratio) / constants.elevator_driver_gear_circumference, 0
    )

def test_set_zero(elevator: Elevator):
    elevator.set_zero()
    pass

@pytest.mark.parametrize(
    "test_input",
    [
        """
        insert whatever testing values
        """
    ]
)
def test_get_position(test_input, elevator: Elevator, monkeypatch: MonkeyPatch):
    monkeypatch.setattr(
        elevator.leader_motor, "get_sensor_position", lambda: test_input
    )
    assert(
        elevator.get_position 
        == (test_input / constants.elevator_gear_ratio) 
        * constants.elevator_driver_gear_circumference
    )

def test_zero(elevator: Elevator):
    elevator.zero()
    pass