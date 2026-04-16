from __future__ import annotations

"""Conversions between Cartesian, Keplerian, and MEE states.

Key design choices:
- shared vector/angle helpers live in utils, so math is not duplicated across
  orbit modules.
- Keplerian singular cases are surfaced explicitly via metadata and auxiliary
  angles instead of being silently patched with arbitrary values.
"""

from dataclasses import replace
import math

from adr_mission.constants import MU_EARTH_KM3_S2
from adr_mission.utils.angle_utils import safe_acos, wrap_to_2pi
from adr_mission.utils.vec3 import cross, dot, norm
from .state_definitions import (
    AngleStatus,
    AuxiliaryAngles,
    CartesianState,
    ElementValidity,
    KeplerianState,
    MEEState,
    OrbitalStateError,
    SingularityConfig,
)


def classify_orbit(e: float, i_rad: float, cfg: SingularityConfig) -> tuple[bool, bool]:
    """Return (is_circular, is_equatorial) using consistent tolerances."""
    is_circular = abs(e) < cfg.ecc_eps
    is_equatorial = abs(math.sin(i_rad)) < cfg.inc_eps or abs(i_rad) < cfg.inc_eps
    return is_circular, is_equatorial


def _perifocal_to_eci(
    r_pf: tuple[float, float, float],
    v_pf: tuple[float, float, float],
    raan_rad: float,
    i_rad: float,
    argp_rad: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    cO, sO = math.cos(raan_rad), math.sin(raan_rad)
    ci, si = math.cos(i_rad), math.sin(i_rad)
    co, so = math.cos(argp_rad), math.sin(argp_rad)

    q11 = cO * co - sO * so * ci
    q12 = -cO * so + sO * co * ci
    q13 = sO * si
    q21 = sO * co + cO * so * ci
    q22 = -sO * so + cO * co * ci
    q23 = -cO * si
    q31 = so * si
    q32 = co * si
    q33 = ci

    r_eci = (
        q11 * r_pf[0] + q12 * r_pf[1] + q13 * r_pf[2],
        q21 * r_pf[0] + q22 * r_pf[1] + q23 * r_pf[2],
        q31 * r_pf[0] + q32 * r_pf[1] + q33 * r_pf[2],
    )
    v_eci = (
        q11 * v_pf[0] + q12 * v_pf[1] + q13 * v_pf[2],
        q21 * v_pf[0] + q22 * v_pf[1] + q23 * v_pf[2],
        q31 * v_pf[0] + q32 * v_pf[1] + q33 * v_pf[2],
    )
    return r_eci, v_eci


def validate_keplerian_state(
    state: KeplerianState,
    cfg: SingularityConfig = SingularityConfig(),
) -> None:
    if state.a_km <= 0.0:
        raise OrbitalStateError("Semi-major axis must be positive.")
    if state.e < 0.0:
        raise OrbitalStateError("Eccentricity cannot be negative.")
    if state.e >= 1.0:
        raise OrbitalStateError("Only elliptical orbits are supported here (e < 1).")

    is_circular, is_equatorial = classify_orbit(state.e, state.i_rad, cfg)

    if not is_equatorial and state.raan_rad is None:
        raise OrbitalStateError("RAAN is required for non-equatorial orbits.")

    if not is_circular and not is_equatorial:
        if state.argp_rad is None or state.nu_rad is None:
            raise OrbitalStateError("Non-circular, non-equatorial orbits need argp and nu.")

    if is_circular and not is_equatorial:
        has_u = state.aux.arglat_rad is not None
        has_sum = state.argp_rad is not None and state.nu_rad is not None
        if not (has_u or has_sum):
            raise OrbitalStateError("Circular inclined orbits need argument of latitude u.")

    if not is_circular and is_equatorial:
        has_lonper = state.aux.lonper_rad is not None or state.argp_rad is not None
        if not has_lonper or state.nu_rad is None:
            raise OrbitalStateError("Equatorial elliptical orbits need longitude of periapsis and nu.")

    if is_circular and is_equatorial:
        if state.aux.truelon_rad is None and state.nu_rad is None:
            raise OrbitalStateError("Circular equatorial orbits need true longitude.")


def cartesian_to_keplerian(
    state: CartesianState,
    mu_km3_s2: float = MU_EARTH_KM3_S2,
    cfg: SingularityConfig = SingularityConfig(),
) -> KeplerianState:
    r = (state.rx_km, state.ry_km, state.rz_km)
    v = (state.vx_km_s, state.vy_km_s, state.vz_km_s)

    r_norm = norm(r)
    v_norm = norm(v)
    if r_norm == 0.0:
        raise OrbitalStateError("Position norm must be non-zero.")

    h_vec = cross(r, v)
    h_norm = norm(h_vec)
    if h_norm == 0.0:
        raise OrbitalStateError("Angular momentum norm must be non-zero.")

    n_vec = (-h_vec[1], h_vec[0], 0.0)
    n_norm = norm(n_vec)

    e_cross = cross(v, h_vec)
    e_vec = (
        e_cross[0] / mu_km3_s2 - r[0] / r_norm,
        e_cross[1] / mu_km3_s2 - r[1] / r_norm,
        e_cross[2] / mu_km3_s2 - r[2] / r_norm,
    )
    e = norm(e_vec)

    energy = 0.5 * v_norm**2 - mu_km3_s2 / r_norm
    if abs(energy) < 1e-14:
        raise OrbitalStateError("Parabolic orbit is not supported in this converter.")
    a_km = -mu_km3_s2 / (2.0 * energy)

    i_rad = safe_acos(h_vec[2] / h_norm)
    is_circular, is_equatorial = classify_orbit(e, i_rad, cfg)
    validity = ElementValidity(is_circular=is_circular, is_equatorial=is_equatorial)

    raan_rad = None
    argp_rad = None
    nu_rad = None
    arglat_rad = None
    lonper_rad = None
    truelon_rad = None

    if not is_equatorial:
        raan_rad = wrap_to_2pi(math.atan2(n_vec[1], n_vec[0]))
        validity = replace(validity, raan_status=AngleStatus.DEFINED)

    if not is_circular:
        if not is_equatorial:
            argp_rad = safe_acos(dot(n_vec, e_vec) / (n_norm * e))
            if e_vec[2] < 0.0:
                argp_rad = 2.0 * math.pi - argp_rad
            argp_rad = wrap_to_2pi(argp_rad)
            validity = replace(validity, argp_status=AngleStatus.DEFINED)
        else:
            lonper_rad = wrap_to_2pi(math.atan2(e_vec[1], e_vec[0]))
            validity = replace(validity, lonper_status=AngleStatus.DEFINED)

        nu_rad = safe_acos(dot(e_vec, r) / (e * r_norm))
        if dot(r, v) < 0.0:
            nu_rad = 2.0 * math.pi - nu_rad
        nu_rad = wrap_to_2pi(nu_rad)
        validity = replace(validity, nu_status=AngleStatus.DEFINED)

    if is_circular and not is_equatorial:
        arglat_rad = safe_acos(dot(n_vec, r) / (n_norm * r_norm))
        if r[2] < 0.0:
            arglat_rad = 2.0 * math.pi - arglat_rad
        arglat_rad = wrap_to_2pi(arglat_rad)
        validity = replace(validity, arglat_status=AngleStatus.DEFINED)

    if is_circular and is_equatorial:
        truelon_rad = wrap_to_2pi(math.atan2(r[1], r[0]))
        validity = replace(validity, truelon_status=AngleStatus.DEFINED)

    return KeplerianState(
        a_km=a_km,
        e=e,
        i_rad=i_rad,
        raan_rad=raan_rad,
        argp_rad=argp_rad,
        nu_rad=nu_rad,
        aux=AuxiliaryAngles(
            arglat_rad=arglat_rad,
            lonper_rad=lonper_rad,
            truelon_rad=truelon_rad,
        ),
        m_kg=state.m_kg,
        validity=validity,
    )


def keplerian_to_cartesian(
    state: KeplerianState,
    mu_km3_s2: float = MU_EARTH_KM3_S2,
    cfg: SingularityConfig = SingularityConfig(),
) -> CartesianState:
    validate_keplerian_state(state, cfg)
    is_circular, is_equatorial = classify_orbit(state.e, state.i_rad, cfg)

    raan = 0.0 if is_equatorial else float(state.raan_rad)

    if not is_circular and not is_equatorial:
        argp = float(state.argp_rad)
        nu = float(state.nu_rad)
    elif is_circular and not is_equatorial:
        u = state.aux.arglat_rad
        if u is None:
            u = wrap_to_2pi(float(state.argp_rad) + float(state.nu_rad))
        argp = 0.0
        nu = wrap_to_2pi(u)
    elif not is_circular and is_equatorial:
        lonper = state.aux.lonper_rad if state.aux.lonper_rad is not None else float(state.argp_rad)
        argp = wrap_to_2pi(lonper)
        nu = float(state.nu_rad)
    else:
        truelon = state.aux.truelon_rad if state.aux.truelon_rad is not None else float(state.nu_rad)
        argp = 0.0
        nu = wrap_to_2pi(truelon)

    p_km = state.a_km * (1.0 - state.e**2)
    if p_km <= 0.0:
        raise OrbitalStateError("Semilatus rectum must be positive.")

    denom = 1.0 + state.e * math.cos(nu)
    r_mag = p_km / denom
    r_pf = (r_mag * math.cos(nu), r_mag * math.sin(nu), 0.0)
    sqrt_mu_over_p = math.sqrt(mu_km3_s2 / p_km)
    v_pf = (
        -sqrt_mu_over_p * math.sin(nu),
        sqrt_mu_over_p * (state.e + math.cos(nu)),
        0.0,
    )
    r_eci, v_eci = _perifocal_to_eci(r_pf, v_pf, raan, state.i_rad, argp)
    return CartesianState(*r_eci, *v_eci, m_kg=state.m_kg)


def keplerian_to_mee(
    state: KeplerianState,
    cfg: SingularityConfig = SingularityConfig(),
) -> MEEState:
    validate_keplerian_state(state, cfg)
    is_circular, is_equatorial = classify_orbit(state.e, state.i_rad, cfg)

    raan = 0.0 if is_equatorial else float(state.raan_rad)

    if is_circular and is_equatorial:
        truelon = state.aux.truelon_rad if state.aux.truelon_rad is not None else float(state.nu_rad)
        lonper = 0.0
        L_rad = wrap_to_2pi(truelon)
    elif is_circular and not is_equatorial:
        arglat = state.aux.arglat_rad
        if arglat is None:
            arglat = wrap_to_2pi(float(state.argp_rad) + float(state.nu_rad))
        lonper = raan
        L_rad = wrap_to_2pi(raan + arglat)
    elif not is_circular and is_equatorial:
        lonper = state.aux.lonper_rad if state.aux.lonper_rad is not None else float(state.argp_rad)
        L_rad = wrap_to_2pi(lonper + float(state.nu_rad))
    else:
        lonper = wrap_to_2pi(raan + float(state.argp_rad))
        L_rad = wrap_to_2pi(raan + float(state.argp_rad) + float(state.nu_rad))

    p_km = state.a_km * (1.0 - state.e**2)
    tan_half_i = math.tan(state.i_rad / 2.0)
    return MEEState(
        p_km=p_km,
        f=state.e * math.cos(lonper),
        g=state.e * math.sin(lonper),
        h=tan_half_i * math.cos(raan),
        k=tan_half_i * math.sin(raan),
        L_rad=L_rad,
        m_kg=state.m_kg,
    )


def mee_to_keplerian(
    state: MEEState,
    cfg: SingularityConfig = SingularityConfig(),
) -> KeplerianState:
    e = math.sqrt(state.f**2 + state.g**2)
    if e >= 1.0:
        raise OrbitalStateError("Only elliptical MEE states are supported here (e < 1).")
    if state.p_km <= 0.0:
        raise OrbitalStateError("MEE p must be positive.")

    a_km = state.p_km / (1.0 - e**2)
    hk_norm = math.sqrt(state.h**2 + state.k**2)
    i_rad = 2.0 * math.atan(hk_norm)
    is_circular, is_equatorial = classify_orbit(e, i_rad, cfg)
    validity = ElementValidity(is_circular=is_circular, is_equatorial=is_equatorial)

    raan_rad = None if is_equatorial else wrap_to_2pi(math.atan2(state.k, state.h))
    if raan_rad is not None:
        validity = replace(validity, raan_status=AngleStatus.DEFINED)

    lonper = None if is_circular else wrap_to_2pi(math.atan2(state.g, state.f))
    L = wrap_to_2pi(state.L_rad)

    argp = None
    nu = None
    arglat = None
    truelon = None

    if not is_circular and not is_equatorial:
        argp = wrap_to_2pi(lonper - raan_rad)
        nu = wrap_to_2pi(L - lonper)
        validity = replace(validity, argp_status=AngleStatus.DEFINED, nu_status=AngleStatus.DEFINED, lonper_status=AngleStatus.DEFINED)
    elif is_circular and not is_equatorial:
        arglat = wrap_to_2pi(L - raan_rad)
        validity = replace(validity, arglat_status=AngleStatus.DEFINED)
    elif not is_circular and is_equatorial:
        nu = wrap_to_2pi(L - lonper)
        validity = replace(validity, nu_status=AngleStatus.DEFINED, lonper_status=AngleStatus.DEFINED)
    else:
        truelon = L
        validity = replace(validity, truelon_status=AngleStatus.DEFINED)

    return KeplerianState(
        a_km=a_km,
        e=e,
        i_rad=i_rad,
        raan_rad=raan_rad,
        argp_rad=argp,
        nu_rad=nu,
        aux=AuxiliaryAngles(arglat_rad=arglat, lonper_rad=lonper, truelon_rad=truelon),
        m_kg=state.m_kg,
        validity=validity,
    )


def cartesian_to_mee(
    state: CartesianState,
    mu_km3_s2: float = MU_EARTH_KM3_S2,
    cfg: SingularityConfig = SingularityConfig(),
) -> MEEState:
    return keplerian_to_mee(cartesian_to_keplerian(state, mu_km3_s2=mu_km3_s2, cfg=cfg), cfg=cfg)


def mee_to_cartesian(
    state: MEEState,
    mu_km3_s2: float = MU_EARTH_KM3_S2,
    cfg: SingularityConfig = SingularityConfig(),
) -> CartesianState:
    return keplerian_to_cartesian(mee_to_keplerian(state, cfg=cfg), mu_km3_s2=mu_km3_s2, cfg=cfg)
