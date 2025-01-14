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
    
    
