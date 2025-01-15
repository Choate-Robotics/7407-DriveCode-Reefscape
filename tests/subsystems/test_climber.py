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