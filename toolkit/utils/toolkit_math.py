import math
from typing import Callable

import numpy as np


def bounded_angle_diff(theta_from: float, theta_too: float) -> float:
    """
    Finds the bounded (from -π to π) angle difference between two unbounded angles
    """
    res = math.fmod(theta_too - theta_from, 6.283185307179586)
    if res > math.pi:
        res -= 6.283185307179586
    if res < -math.pi:
        res += 6.283185307179586
    return res


def rotate_vector(x: float, y: float, theta: float) -> tuple[float, float]:
    return (
        x * math.cos(theta) - y * math.sin(theta),
        x * math.sin(theta) + y * math.cos(theta),
    )


def clamp(val: float, _min: float, _max: float):
    """
    Clamps a value between a min and max

    Args:
        val (float): value to clamp
        _min (float): min value
        _max (float): max value

    Returns:
        float: clamped value
    """

    if val < _min:
        return _min
    if val > _max:
        return _max
    return val


def ft_to_m(ft: float):
    """
    Converts feet to meters

    Args:
        ft (float): feet (float)

    Returns:
        float: meters (float)
    """
    return ft * 0.3048


def talon_sensor_units_to_inches(sensor_units: float, low_gear: bool) -> float:
    """
    Converts sensor units to inches

    Args:
        sensor_units (float): sensor units as a float
        low_gear (bool): low gear as a bool

    Returns:
        inches as a float
    """
    motor_rotations = sensor_units / 2048.0

    if low_gear:
        wheelbase_rotations = motor_rotations / 15.45  # Low gear
    else:
        wheelbase_rotations = motor_rotations / 8.21  # High gear

    inches = wheelbase_rotations * (6 * math.pi)

    return inches


def talon_sensor_units_to_meters(sensor_units: float, low_gear: bool) -> float:
    """
    Converts sensor units to meters

    Args:
        sensor_units (float):  sensor units as a float
        low_gear (bool): low gear as a bool

    Returns:
        meters as a float
    """
    return talon_sensor_units_to_inches(sensor_units, low_gear) * 0.0254


def meters_to_talon_sensor_units(meters: float, low_gear: bool) -> float:
    """
    Converts meters to sensor units

    Args:
        meters (float): meters as a float
        low_gear (float): low gear as a bool

    Returns:
        sensor units as a float
    """
    return inches_to_talon_sensor_units(meters / 0.0254, low_gear)


def inches_to_talon_sensor_units(inches: float, low_gear: bool) -> float:
    """
    Converts inches to sensor units

    Args:
        inches (float): inches as a float
        low_gear (float): low gear as a bool

    Returns:
        sensor units as a float
    """
    wheelbase_rotations = inches / (6 * math.pi)

    if low_gear:
        motor_rotations = wheelbase_rotations * 15.45  # Low gear
    else:
        motor_rotations = wheelbase_rotations * 8.21  # High gear

    sensor_units = motor_rotations * 2048.0

    return sensor_units


class NumericalIntegration:
    def rk4_step(self, func: Callable, y, t, h):
        """
        Perform a single RK4 step for a system of ODEs.
        :param func: Function representing the ODE system (dy/dt = f(t, y)).
        :param y: Current values of the dependent variables (as a vector).
        :param t: Current value of the independent variable.
        :param h: Step size.
        :return: Estimated values of y at t+h.
        """
        k1 = h * func(t, y)
        k2 = h * func(t + 0.5 * h, y + 0.5 * k1)
        k3 = h * func(t + 0.5 * h, y + 0.5 * k2)
        k4 = h * func(t + h, y + k3)

        return y + (k1 + 2 * k2 + 2 * k3 + k4) / 6

    def adaptive_rk4(self, func, y0, t0, tf, h0, tol, event=None):
        """
        RK4 with adaptive step size for a system of ODEs.
        :param func: Function representing the ODE system.
        :param y0: Initial values of the dependent variables (as a vector).
        :param t0: Initial value of the independent variable.
        :param tf: Final value of the independent variable.
        :param h0: Initial step size.
        :param tol: Tolerance for adaptive step size.
        :param event: Optional function that returns true when the integration should stop.
        :return: Arrays of t values and y values (as a matrix).
        """
        t_values = [t0]
        y_values = [y0]

        t = t0
        y = y0
        h = h0

        while t < tf:
            # Estimate one step with two half steps
            y_temp = self.rk4_step(func, y, t, h / 2)
            y_temp = self.rk4_step(func, y_temp, t + h / 2, h / 2)

            # Estimate one full step
            y_full = self.rk4_step(func, y, t, h)

            # Error estimate
            error = np.linalg.norm(y_temp - y_full)

            # Adjust step size
            if error < tol:
                t += h
                y = y_temp
                t_values.append(t)
                y_values.append(y)
                h = h * min(2, (tol / error) ** 0.25)  # Increase step size
            else:
                h = h * max(0.5, (tol / error) ** 0.25)  # Decrease step size
            # Check to see if end condition met
            if event is not None:
                if event(t, y):
                    break

        return np.array(t_values), np.array(y_values)


def extrapolate(x, x1, y1, x2, y2):
    # Calculate the slope (m)
    m = (y2 - y1) / (x2 - x1)

    # Calculate the y-intercept (c)
    c = y1 - m * x1

    # Use the line equation to extrapolate the y value
    y = m * x + c
    return y
