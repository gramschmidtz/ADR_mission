from __future__ import annotations

"""Atmospheric drag helpers for MEE dynamics."""

import math

from .state_definitions import MEEState, OrbitalStateError


def exponential_atmosphere_density_kg_m3(
    altitude_km: float,
    rho0_kg_m3: float,
    scale_height_km: float,
) -> float:
    altitude_km = max(0.0, altitude_km)
    return rho0_kg_m3 * math.exp(-altitude_km / scale_height_km)


def drag_acceleration_rtn_km_s2(
    mee: MEEState,
    *,
    radius_km: float,
    vr_km_s: float,
    vt_km_s: float,
    re_km: float,
    cd: float,
    area_m2: float,
    rho0_kg_m3: float,
    scale_height_km: float,
    enabled: bool = True,
) -> tuple[float, float, float]:
    if not enabled:
        return (0.0, 0.0, 0.0)
    if mee.m_kg is None or mee.m_kg <= 0.0:
        raise OrbitalStateError("Positive spacecraft mass is required when drag is enabled.")

    altitude_km = radius_km - re_km
    rho = exponential_atmosphere_density_kg_m3(altitude_km, rho0_kg_m3, scale_height_km)
    vr_m_s = vr_km_s * 1000.0
    vt_m_s = vt_km_s * 1000.0
    v_m_s = math.sqrt(vr_m_s**2 + vt_m_s**2)

    coeff_m_s2 = -0.5 * rho * area_m2 * cd / mee.m_kg
    a_r_m_s2 = coeff_m_s2 * v_m_s * vr_m_s
    a_t_m_s2 = coeff_m_s2 * v_m_s * vt_m_s
    return (a_r_m_s2 / 1000.0, a_t_m_s2 / 1000.0, 0.0)
