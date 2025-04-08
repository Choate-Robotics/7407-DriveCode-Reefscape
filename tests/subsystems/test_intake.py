from unittest.mock import MagicMock

import pytest
import math
import config
import constants
from subsystem import Intake

@pytest.fixture
def intake() -> Intake:
    intake = Intake()

    intake.horizontal_motor = MagicMock()
    intake.pivot_motor = MagicMock()

    return intake

def test_init(intake: Intake):
    intake.init()
    intake.horizontal_motor.init.assert_called()
    intake.pivot_motor.init.assert_called()

def test_roll_in(intake: Intake):
    intake.roll_in()
    intake.horizontal_motor.set_raw_output.assert_called_with(
        config.horizontal_intake_speed
    )
    assert intake.intake_running is True

def test_intake_algae(intake: Intake):
    intake.intake_algae()
    intake.horizontal_motor.set_raw_output.assert_called_with(
        -config.intake_algae_speed
    )
    assert intake.intake_running is True

def test_stop(intake: Intake):
    intake.stop()
    intake.horizontal_motor.set_raw_output.assert_called_with(0)
    assert intake.intake_running is False

def test_roll_out(intake: Intake):
    intake.roll_out()
    intake.horizontal_motor.set_raw_output.assert_called_with(
        -config.horizontal_intake_speed
    )
    assert intake.intake_running is True

def test_extake_algae(intake: Intake):
    intake.extake_algae()
    intake.horizontal_motor.set_raw_output.assert_called_with(
        config.extake_algae_speed
    )
    assert intake.intake_running is True

def test_get_horizontal_motor_current(intake: Intake):
    assert intake.get_horizontal_motor_current() == intake.horizontal_motor.get_motor_current()

def test_get_pivot_motor_current(intake: Intake):
    assert intake.get_pivot_motor_current() == intake.pivot_motor.get_motor_current()

# Since limit angle is a simple conditional, we can just test using a few angles

@pytest.mark.parametrize("angle", [
    config.intake_min_angle,
    config.intake_max_angle,
    config.intake_min_angle - 1,
    config.intake_max_angle + 1
])
def test_limit_angle(angle, intake: Intake):
    if angle <= config.intake_min_angle:
        assert intake.limit_angle(angle) == config.intake_min_angle
    elif angle > config.intake_max_angle:
        assert intake.limit_angle(angle) == config.intake_max_angle
    else:
        assert intake.limit_angle(angle) == angle

def test_zero_pivot(intake: Intake):
    intake.encoder = MagicMock()

    position_mock = MagicMock()
    position_mock.value = 0.7
    intake.encoder.get_absolute_position.return_value = position_mock
    
    intake.zero_pivot()

    expected_angle = ((position_mock.value - config.intake_encoder_zero) / constants.intake_encoder_gear_ratio * 2 * math.pi)
    
    assert intake.pivot_angle == expected_angle
    assert intake.pivot_zeroed is True
    
    expected_sensor_position = expected_angle * constants.intake_pivot_gear_ratio / (2 * math.pi)
    intake.pivot_motor.set_sensor_position.assert_called_with(expected_sensor_position)

@pytest.mark.parametrize("pivot_angle", [
    config.intake_min_angle,
    config.intake_max_angle,
    config.intake_min_angle - 1,
    config.intake_max_angle + 1
])
def test_get_and_set_pivot_angle(intake: Intake, pivot_angle: float):
    intake.pivot_motor.get_sensor_position.return_value = (pivot_angle / (2 * math.pi)) * constants.intake_pivot_gear_ratio
    intake.set_pivot_angle(pivot_angle)
    assert intake.get_pivot_angle() == pivot_angle
def test_stop_pivot(intake: Intake):
    intake.stop_pivot()
    intake.pivot_motor.set_raw_output.assert_called_with(0)

def test_update_table(intake: Intake):
    intake.update_table()
    intake.pivot_motor.get_motor_current.assert_called()
    intake.pivot_motor.get_applied_output.assert_called()
    intake.pivot_motor.get_sensor_velocity.assert_called()
    intake.pivot_motor.get_sensor_acceleration.assert_called()