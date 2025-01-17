from unittest.mock import MagicMock

import pytest

import config
import constants
from subsystem import Wrist
from toolkit.utils.toolkit_math import bounded_angle_diff
from toolkit.motors.ctre_motors import TalonFX
from phoenix6.hardware import cancoder

@pytest.fixture()
def wrist() -> Wrist:

    my_wrist = Wrist()
    my_wrist.wrist_motor = MagicMock()
    my_wrist.feed_motor = MagicMock()
    return my_wrist
