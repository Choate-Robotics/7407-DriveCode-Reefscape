# from unittest.mock import MagicMock

# import pytest

# import config
# import constants
# from subsystem import Intake


# @pytest.fixture
# def intake() -> Intake:
#     intake = Intake()
#     intake.motor = MagicMock()
#     return intake


# def test_intake_init(intake: Intake):
#     intake.init()

#     intake.motor.init.assert_called()


# # @pytest.mark.parametrize(
# #     "test_input",
# #     [
# #         """
# #         add whatever testing values
# #         """
# #
# #     ]
# # )


# def test_roll_in(intake: Intake):
#     intake.roll_in()
#     intake.motor.set_raw_output.assert_called_with(
#         config.intake_speed * constants.intake_gear_ratio
#     )


# def test_roll_out(intake: Intake):
#     intake.roll_out()
#     intake.motor.set_raw_output.assert_called_with(
#         config.intake_eject_speed * constants.intake_gear_ratio
#     )


# def test_stop(intake: Intake):
#     intake.stop()
#     intake.motor.set_raw_output.assert_called_with(0)


# def detect_coral(intake: Intake):
#     # intake.sensor.getVoltage.return_value = 1
#     intake.detect_coral()
#     intake.sensor.getVoltage.assert_called()
