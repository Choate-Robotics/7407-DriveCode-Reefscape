from typing import List
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState
from wpimath.system.plant import DCMotor
from wpilib.simulation import DCMotorSim

from utils.simtalon import Talon
from toolkit.utils.toolkit_math import clamp

import constants


class SimTalon:
    gearing: float
    motortype: DCMotor
    sim: DCMotorSim
    simstate: TalonFXSimState
    inertia: float

    def __init__(
        self, gearing: float, motortype: DCMotor, sim: TalonFXSimState, inertia: float
    ) -> None:
        self.gearing = gearing
        self.motortype = motortype
        self.inertia = inertia
        self.sim = DCMotorSim(self.motortype, self.gearing, self.inertia)
        self.simstate = sim


class MotorSimulator:
    _talons: List[SimTalon]

    def __init__(self):
        self._talons = []

    def addFalcon(self, t: Talon, gearing: float, inertia: float):
        self._talons.append(
            SimTalon(gearing, DCMotor.falcon500(), t.getSimCollection(), inertia)
        )

    def addFalconFOC(self, t: Talon, gearing: float, inertia: float):
        self._talons.append(
            SimTalon(gearing, DCMotor.falcon500FOC(), t.getSimCollection(), inertia)
        )

    def addKraken(self, t: Talon, gearing: float, inertia: float):
        self._talons.append(
            SimTalon(gearing, DCMotor.krakenX60(), t.getSimCollection(), inertia)
        )

    def addKrakenFOC(self, t: Talon, gearing: float, inertia: float):
        self._talons.append(
            SimTalon(gearing, DCMotor.krakenX60FOC(), t.getSimCollection(), inertia)
        )

    def update(self, tm_diff: float, robotVoltage: float) -> None:
        for talon in self._talons:
            talon.sim.setInputVoltage(talon.simstate.motor_voltage)
            talon.sim.update(tm_diff)

            position_rot = (
                talon.sim.getAngularPosition()
                / constants.kRadiansPerRevolution
                * talon.gearing
            )
            velocity_rps = (
                talon.sim.getAngularVelocity()
                / constants.kRadiansPerRevolution
                * talon.gearing
            )

            talon.simstate.set_raw_rotor_position(position_rot)
            talon.simstate.set_rotor_velocity(velocity_rps)
            talon.simstate.set_supply_voltage(
                clamp(
                    robotVoltage
                    - talon.simstate.supply_current
                    * constants.kSimMotorResistance,  # simulate voltage drop due to resistance
                    0,
                    robotVoltage,
                )
            )