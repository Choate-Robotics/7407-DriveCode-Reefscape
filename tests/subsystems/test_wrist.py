from unittest.mock import MagicMock

import pytest

import config
import constants
from subsystem import Wrist
from toolkit.utils.toolkit_math import bounded_angle_diff
from toolkit.motors.ctre_motors import TalonFX
from phoenix6.hardware import cancoder

import wpilib
from wpilib import Timer

@pytest.fixture()
def wrist() -> Wrist:

    my_wrist = Wrist()
    my_wrist.wrist_motor = MagicMock()
    my_wrist.feed_motor = MagicMock()
    return my_wrist


@pytest.mark.parametrize(
        "current_threshold, current, time_threshold, time, expected",
        [
            (5, 0.2, 0.5, 0.2, False)
        ]
)

def test_coral_detected(current_threshold, current, time_threshold, time, expected, wrist: Wrist):
    assert wrist.coral_in_feed == expected