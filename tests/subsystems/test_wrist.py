from unittest.mock import MagicMock

import pytest
from pytest import MonkeyPatch

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
        "current, time, expected",
        [
            (config.current_threshold+.5, config.current_time_threshold+.2, True),
            (config.current_threshold-.5, config.current_time_threshold+.2, False),
            (config.current_threshold+.5, config.current_time_threshold-.2, False),
            (config.current_threshold-.5, config.current_time_threshold-.2, False)

        ],
)
def test_coral_in_wrist(wrist, current, time, expected, monkeypatch: MonkeyPatch):
    def fake_has_Elapsed(self, x):
        return time>x
    monkeypatch.setattr(wpilib.Timer, "hasElapsed", fake_has_Elapsed)
    wrist.feed_motor.get_motor_current = lambda: current

    wrist.coral_in_wrist()
    
    assert wrist.coral_in_feed == expected