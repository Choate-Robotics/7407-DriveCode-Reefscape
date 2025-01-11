import pytest

from unittest import mock


# these test function will be run automatically by pytest


# this function tests a single assertion
def test_basic():
    assert True == True


# for multiple assertions, use a parametrized test
# this way, you can run the same test with different inputs
# and expected outputs, and see which ones fail
@pytest.mark.parametrize(
    "test_input,expected",
    [
        ("3+5", 8),
        ("2+4", 6),
        ("6*9", 54),
        ("6*7", 42),
    ],
)
# the test_input and expected variables are passed in as arguments
def test_math_multiple(test_input, expected):
    assert eval(test_input) == expected


# this test is to check for instances of the correct exception error
# use the pytest.raises context manager
# to check that the expected exception is raised.
# this is good if you want to make sure that your code is raising the correct exception
def test_check_exception():
    with pytest.raises(ZeroDivisionError):
        1 / 0


# If you want to skip a test, use the pytest.mark.skip decorator
@pytest.mark.skip(reason="this test is not implemented yet")
def test_not_implemented():
    pass


# If you want to mark a test as expected to fail, use the pytest.mark.xfail decorator
# this is good if you want to mark a test as expected to fail, but you still want to run it
# dont use this if you want to check for a specific exception, or if you want to skip the test
@pytest.mark.xfail(reason="this test is expected to fail")
def test_expected_failure():
    assert False == True
