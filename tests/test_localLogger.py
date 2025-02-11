import subprocess
from unittest.mock import MagicMock

import pytest
from pytest import MonkeyPatch

from utils import LocalLogger


@pytest.fixture
def locallogger() -> LocalLogger:
    locallogger = LocalLogger("/logs")
    locallogger.custom_entry = MagicMock()
    return locallogger


def test_get_deploy_info(locallogger):
    git_branch = subprocess.run(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    git_branch = git_branch.stdout.strip()
    branch, data, by = locallogger.get_deploy_info()
    assert branch == "Simulation"


@pytest.mark.parametrize(
    "config_value_boolean, config_value_log_level, string",
    [
        (False, LocalLogger.LogLevels.INFO, "WARNING: DEBUG MODE IS ENABLED"),
        (True, "Robot logging initialized"),
        (constants.elevator_max_height),
    ],
)
def test_get_log_levels(locallogger):
    log_levels = locallogger.get_log_levels()
    assert log_levels == "WARNING: Logging to file is disabled"
