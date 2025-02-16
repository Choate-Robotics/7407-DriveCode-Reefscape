from unittest.mock import MagicMock

import pytest

import config

from subsystem import Intake


@pytest.fixture
def intake() -> Intake:
    intake = Intake()

    intake.horizontal_motor = MagicMock()
    intake.vertical_motor = MagicMock()
    intake.pivot_motor = MagicMock()

    return intake


def test_intake_init(intake: Intake):
    intake.init()

    intake.horizontal_motor.init.assert_called()
    intake.vertical_motor.init.assert_called()
    intake.pivot_motor.init.assert_called()


# @pytest.mark.parametrize(
#     "test_input",
#     [
#         """
#         add whatever testing values
#         """
#
#     ]
# )


def test_roll_in(intake: Intake):
    intake.roll_in()
    intake.horizontal_motor.set_raw_output.assert_called_with(
        config.horizontal_intake_speed
    )
    intake.vertical_motor.set_raw_output.assert_called_with(
        config.vertical_intake_speed
    )
    assert intake.intake_running is True


def test_roll_out(intake: Intake):
    intake.roll_out()
    intake.horizontal_motor.set_raw_output.assert_called_with(
        -config.horizontal_intake_speed
    )
    intake.vertical_motor.set_raw_output.assert_called_with(
        -config.vertical_intake_speed
    )
    assert intake.intake_running is True


def test_stop(intake: Intake):
    intake.stop()
    intake.vertical_motor.set_raw_output.assert_called_with(0)
    intake.horizontal_motor.set_raw_output.assert_called_with(0)
    assert intake.intake_running is False
