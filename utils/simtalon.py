from enum import Enum, auto
from phoenix6.configs.config_groups import CurrentLimitsConfigs, MotorOutputConfigs
from phoenix6.signals.spn_enums import (
    ForwardLimitValue,
    InvertedValue,
    NeutralModeValue,
    ReverseLimitValue,
)
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState

from phoenix6.controls.neutral_out import NeutralOut
from phoenix6.controls.velocity_voltage import VelocityVoltage
from phoenix6.controls.velocity_duty_cycle import VelocityDutyCycle
from phoenix6.controls.position_voltage import PositionVoltage
from phoenix6.controls.duty_cycle_out import DutyCycleOut
from phoenix6.controls.motion_magic_voltage import MotionMagicVoltage
from phoenix6.controls.follower import Follower
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode

from wpilib import RobotBase

#from util.logging import nt


class Talon:
    signals = {}

    class ControlMode(Enum):
        Position = auto()
        """rotations"""
        Velocity = auto()
        """rotations/s"""
        Percent = auto()
        Amps = auto()
        MotionMagic = auto()

    class NeutralMode(Enum):
        Brake = auto()
        Coast = auto()

    class LimitSwitch(Enum):
        Forwards = auto()
        Backwards = auto()

    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        canID: int,
        name: str,
        pGain: float = 1,
        iGain: float = 0,
        dGain: float = 0,
        isReversed: bool = False,
        canbus: str = "",
        kV: float = 0,
        moMagicAccel: float = 0,
        moMagicVel: float = 0,
    ) -> None:
        print(f"Init TalonFX with port {canID} on {canbus} with name {name}")
        self.id = canID
        self.name = name
        self.motor = TalonFX(canID, canbus)
        self.canbus = canbus

        self.isReversed = isReversed

        conf = TalonFXConfiguration()
        conf.slot0.k_p = pGain
        conf.slot0.k_i = iGain
        conf.slot0.k_d = dGain
        conf.slot0.k_v = kV
        conf.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if isReversed
            else InvertedValue.CLOCKWISE_POSITIVE
        )
        conf.motion_magic.motion_magic_acceleration = moMagicAccel
        conf.motion_magic.motion_magic_cruise_velocity = moMagicVel
        # self._nettableidentifier = f"motors/{self.name}({self.id})"
        # self._mainTable = nt.getTable(self._nettableidentifier)
        #
        # self._gainsTable = self._mainTable.getSubTable("gains")
        # self._gainsTable.putNumber("p", pGain)
        # self._gainsTable.putNumber("i", iGain)
        # self._gainsTable.putNumber("d", dGain)
        # self._gainsTable.putNumber("v", kV)
        #
        # self._mainTable.putBoolean("inverted", isReversed)
        # self._mainTable.putString("canbus", canbus)

        if canbus not in Talon.signals:
            Talon.signals[canbus] = []

        self.motor.configurator.apply(conf)

        self.velControl = VelocityVoltage(0, 0, False, 0, 0, False, False, False)
        self.posControl = PositionVoltage(0, 0, False, 0, 0, False, False, False)
        self.perControl = DutyCycleOut(0, False, False, False, False)
        self.moMagic = MotionMagicVoltage(0)

        self.velControl.slot = 0
        self.posControl.slot = 0

        self.positionSignal = self.motor.get_position()
        self.velocitySignal = self.motor.get_velocity()
        self.accelSignal = self.motor.get_acceleration()
        self.voltageSignal = self.motor.get_motor_voltage()
        self.supplyVoltageSignal = self.motor.get_supply_voltage()
        self.tempSignal = self.motor.get_device_temp()
        self.dutySignal = self.motor.get_duty_cycle()
        self.currentSignal = self.motor.get_torque_current()

        for signal in [
            self.positionSignal,
            self.voltageSignal,
            self.accelSignal,
            self.voltageSignal,
            self.supplyVoltageSignal,
            self.tempSignal,
            self.dutySignal,
            self.currentSignal,
        ]:
            Talon.signals[self.canbus].append(signal)
            signal.set_update_frequency(50)  # Hz

        if RobotBase.isSimulation():
            self.motor.get_position().set_update_frequency(100)
            self.motor.get_velocity().set_update_frequency(100)
        print("...Done")

    def set(
        self,
        controlMode: ControlMode,
        demand: float,
        ff: float = 0,
        duty_cycle: bool = True,
    ) -> None:
        # self.updateDashboard()

        #self._mainTable.putNumber("target", demand)
        c = None
        if controlMode == Talon.ControlMode.Position:
            c = self.motor.set_control(
                self.posControl.with_position(demand).with_feed_forward(ff)
            )
        elif controlMode == Talon.ControlMode.Velocity:
            c = self.motor.set_control(
                self.velControl.with_velocity(demand).with_feed_forward(ff)
                if not duty_cycle
                else VelocityDutyCycle(demand, feed_forward=ff)
            )
        elif controlMode == Talon.ControlMode.Percent:
            c = self.motor.set_control(self.perControl.with_output(demand + ff / 12))
        elif controlMode == Talon.ControlMode.MotionMagic:
            c = self.motor.set_control(self.moMagic.with_position(demand))
        elif controlMode == Talon.ControlMode.Amps:
            raise NotImplementedError("AMP control is currently not implemented")

        if c != StatusCode.OK:
            print(
                f"ERROR: {c} \n ({controlMode}, {demand}, {ff}, {self.motor.device_id})"
            )

    def follow(self, other, opp_direction: bool = False):
        c = self.motor.set_control(Follower(other.id, opp_direction))
        if c != StatusCode.OK:
            print(f"ERROR: {c} \n {self.motor.device_id})")

    def updateDashboard(self):
        pass
        # self._mainTable.putNumber("position", self.positionSignal.value)
        # self._mainTable.putNumber("velocity", self.velocitySignal.value)
        # self._mainTable.putNumber("acceleration", self.accelSignal.value)
        #
        # self._mainTable.putNumber("outVoltage", self.voltageSignal.value)
        # self._mainTable.putNumber("supplyVoltage", self.supplyVoltageSignal.value)
        # self._mainTable.putNumber("temp", self.tempSignal.value)
        # self._mainTable.putNumber("dutycycle", self.dutySignal.value)
        # self._mainTable.putNumber("current", self.currentSignal.value)

    def setCurrentLimit(self, lim: CurrentLimitsConfigs):
        self.motor.configurator.apply(lim)

    def neutralOutput(self):
        self.motor.set_control(NeutralOut())

    def setNeutralMode(self, mode: NeutralMode):
        conf = (
            MotorOutputConfigs()
            .with_neutral_mode(
                NeutralModeValue.COAST
                if mode == Talon.NeutralMode.Coast
                else NeutralModeValue.BRAKE
            )
            .with_inverted(
                InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                if self.isReversed
                else InvertedValue.CLOCKWISE_POSITIVE
            )
        )
        self.motor.configurator.apply(conf)

    def getLimitSwitch(self, switch: LimitSwitch) -> bool:
        if switch == Talon.LimitSwitch.Forwards:
            return (
                self.motor.get_forward_limit().value
                == ForwardLimitValue.CLOSED_TO_GROUND
            )
        elif switch == Talon.LimitSwitch.Backwards:
            return (
                self.motor.get_reverse_limit().value
                == ReverseLimitValue.CLOSED_TO_GROUND
            )
        return False

    def get(self, controlMode: ControlMode) -> float:
        # self.updateDashboard()

        if controlMode == Talon.ControlMode.Position:
            return self.positionSignal.value
        elif controlMode == Talon.ControlMode.Velocity:
            return self.velocitySignal.value
        elif controlMode == Talon.ControlMode.Percent:
            return self.voltageSignal.value / 12
        return 0

    def setEncoderPosition(self, rotations: float):
        self.motor.set_position(rotations)

    def getSimCollection(self) -> TalonFXSimState:
        return self.motor.sim_state