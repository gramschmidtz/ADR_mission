from __future__ import annotations

"""Orbital state definitions used by ADR_mission.

Internal propagation state is Modified Equinoctial Elements (MEE), consistent
with the paper's formulation. External inputs may be Cartesian, Keplerian, or
MEE.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from adr_mission.constants import ECC_EPS_DEFAULT, INC_EPS_DEFAULT


class AngleStatus(str, Enum):
    DEFINED = "defined"
    UNDEFINED = "undefined"
    SUBSTITUTED = "substituted"


@dataclass(frozen=True)
class ElementValidity:
    raan_status: AngleStatus = AngleStatus.UNDEFINED
    argp_status: AngleStatus = AngleStatus.UNDEFINED
    nu_status: AngleStatus = AngleStatus.UNDEFINED
    arglat_status: AngleStatus = AngleStatus.UNDEFINED
    lonper_status: AngleStatus = AngleStatus.UNDEFINED
    truelon_status: AngleStatus = AngleStatus.UNDEFINED
    is_circular: bool = False
    is_equatorial: bool = False


@dataclass(frozen=True)
class CartesianState:
    rx_km: float
    ry_km: float
    rz_km: float
    vx_km_s: float
    vy_km_s: float
    vz_km_s: float
    m_kg: Optional[float] = None


@dataclass(frozen=True)
class MEEState:
    p_km: float
    f: float
    g: float
    h: float
    k: float
    L_rad: float
    m_kg: Optional[float] = None


@dataclass(frozen=True)
class AuxiliaryAngles:
    arglat_rad: Optional[float] = None
    lonper_rad: Optional[float] = None
    truelon_rad: Optional[float] = None


@dataclass(frozen=True)
class KeplerianState:
    a_km: float
    e: float
    i_rad: float
    raan_rad: Optional[float] = None
    argp_rad: Optional[float] = None
    nu_rad: Optional[float] = None
    aux: AuxiliaryAngles = field(default_factory=AuxiliaryAngles)
    m_kg: Optional[float] = None
    validity: ElementValidity = field(default_factory=ElementValidity)


@dataclass(frozen=True)
class SingularityConfig:
    ecc_eps: float = ECC_EPS_DEFAULT
    inc_eps: float = INC_EPS_DEFAULT


class OrbitalStateError(ValueError):
    """Raised when an orbital state is inconsistent or unusable."""
