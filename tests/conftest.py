import pytest
import wpilib
from unittest import mock
import phoenix6 as ctre
import commands2
import rev
from robot_systems import Robot, Sensors, Pneumatics

# this is a fixture, which is a function that runs before each test function
# this is good if you want to set up some data before each test
# and then clean up after the tests are done
# we will import our subsystems and other classes through fixtures

# This is an example of a fixture that starts the arm subsystem
# @pytest.fixture(scope="session")
# def arm():
#     print("arm setup")
#     Robot.arm.init()

#     yield Robot.arm()
#     print("arm teardown")


# # this is a fixture that will be used to mock the ctre library
@pytest.fixture(scope="session")
def ctre_mock():
    print("ctre_mock setup")
    # we will use the mock.patch context manager to mock the ctre library
    # this will replace the ctre library with a mock version of it
    # and then we will yield the mock library
    # and then when the context manager exits, it will unmock the ctre library
    with mock.patch("ctre"):
        yield ctre
    print("ctre_mock teardown")


# this is a fixture that will be used to mock the rev library
@pytest.fixture(scope="session")
def rev_mock():
    print("rev_mock setup")
    # we will use the mock.patch context manager to mock the rev library
    # this will replace the rev library with a mock version of it
    # and then we will yield the mock library
    # and then when the context manager exits, it will unmock the rev library
    with mock.patch("rev"):
        yield rev
    print("rev_mock teardown")


# this is a fixture that will be used to mock the commands2 library
@pytest.fixture(scope="session")
def commands2_mock():
    print("commands2_mock setup")
    # we will use the mock.patch context manager to mock the commands2 library
    # this will replace the commands2 library with a mock version of it
    # and then we will yield the mock library
    # and then when the context manager exits, it will unmock the commands2 library
    with mock.patch("commands2"):
        yield commands2
    print("commands2_mock teardown")


# this is a fixture that will be used to mock the wpilib library
@pytest.fixture(scope="session")
def wpilib_mock():
    print("wpilib_mock setup")
    # we will use the mock.patch context manager to mock the wpilib library
    # this will replace the wpilib library with a mock version of it
    # and then we will yield the mock library
    # and then when the context manager exits, it will unmock the wpilib library
    with mock.patch("wpilib"):
        yield wpilib
    print("wpilib_mock teardown")
