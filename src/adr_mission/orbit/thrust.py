from __future__ import annotations

"""Thrust model in RTN coordinates."""

from dataclasses import dataclass
import math
from typing import Callable, Optional

from .state_definitions import MEEState, OrbitalStateError


@dataclass(frozen=True)
class ControlInput:
    nr: float = 0.0
    nt: float = 0.0
    nh: float = 0.0

    @property
    def norm(self) -> float:
        return math.sqrt(self.nr**2 + self.nt**2 + self.nh**2)


ControlLaw = Callable[[float, MEEState, object], ControlInput]


def zero_control(_: float, __: MEEState, ___: object) -> ControlInput:
    return ControlInput()


def thrust_acceleration_rtn_km_s2(
    control: ControlInput,
    thrust_max_N: float,
    mass_kg: Optional[float],
    *,
    enabled: bool = True,
) -> tuple[float, float, float]:
    if not enabled:
        return (0.0, 0.0, 0.0)
    if mass_kg is None or mass_kg <= 0.0:
        raise OrbitalStateError("Positive spacecraft mass is required when thrust is enabled.")

    scale_km_s2 = (thrust_max_N / mass_kg) / 1000.0
    return (scale_km_s2 * control.nr, scale_km_s2 * control.nt, scale_km_s2 * control.nh)


def mass_flow_rate_kg_s(
    control: ControlInput,
    thrust_max_N: float,
    isp_s: float,
    g0_m_s2: float,
    *,
    enabled: bool = True,
) -> float:
    if not enabled:
        return 0.0
    return -(thrust_max_N * control.norm) / (isp_s * g0_m_s2)
