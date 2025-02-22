from unittest.mock import MagicMock

import pytest

import config

from subsystem import Intake


@pytest.fixture
def intake() -> Intake:
    intake = Intake()

    intake.horizontal_motor = MagicMock()
    intake.pivot_motor = MagicMock()

    return intake


# @pytest.fixture
# def intake() -> Intake:
#     intake = Intake()

#     intake.horizontal_motor = MagicMock()
#     intake.vertical_motor = MagicMock()
#     intake.pivot_motor = MagicMock()

#     return intake


# # def test_intake_init(intake: Intake):
# #     intake.init()


def test_roll_in(intake: Intake):
    intake.roll_in()
    intake.horizontal_motor.set_raw_output.assert_called_with(
        config.horizontal_intake_speed
    )
    assert intake.intake_running is True


def test_roll_out(intake: Intake):
    intake.roll_out()
    intake.horizontal_motor.set_raw_output.assert_called_with(
        -config.horizontal_intake_speed
    )
    assert intake.intake_running is True


def test_stop(intake: Intake):
    intake.stop()
    intake.horizontal_motor.set_raw_output.assert_called_with(0)
    assert intake.intake_running is False
