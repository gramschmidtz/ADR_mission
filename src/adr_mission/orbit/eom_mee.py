from __future__ import annotations

"""Modified equinoctial dynamics consistent with the ADR paper.

This version fixes two structural issues in the pasted implementation:
1. duplicated vector/math utilities were moved into shared utils modules.
2. the A(x) matrix now uses the paper's exact h/k rows:
   - row for h uses sin(L)
   - row for k uses cos(L)
"""

from dataclasses import dataclass
import math
from typing import Sequence

from adr_mission.constants import G0_M_S2, J2, J3, J4, MU_EARTH_KM3_S2, R_EARTH_KM
from .conversions import mee_to_cartesian
from .drag import drag_acceleration_rtn_km_s2
from .gravity import nonspherical_gravity_acceleration_eci_km_s2, project_eci_acceleration_to_rtn
from .state_definitions import MEEState, OrbitalStateError
from .thrust import ControlInput, ControlLaw, mass_flow_rate_kg_s, thrust_acceleration_rtn_km_s2, zero_control


@dataclass(frozen=True)
class MEEDynamicsConfig:
    mu_km3_s2: float = MU_EARTH_KM3_S2
    re_km: float = R_EARTH_KM
    ge_m_s2: float = G0_M_S2
    j2: float = J2
    j3: float = J3
    j4: float = J4
    thrust_max_N: float = 21e-3
    isp_s: float = 2000.0
    cd: float = 2.2
    area_m2: float = 8.0
    rho0_kg_m3: float = 1.225
    scale_height_km: float = 8.5
    include_thrust: bool = True
    include_nonspherical_gravity: bool = True
    include_drag: bool = True


def compute_q_magnitude(mee: MEEState) -> float:
    return 1.0 + mee.f * math.cos(mee.L_rad) + mee.g * math.sin(mee.L_rad)


def compute_s_squared(mee: MEEState) -> float:
    return 1.0 + mee.h**2 + mee.k**2


def mee_radius_km(mee: MEEState) -> float:
    q = compute_q_magnitude(mee)
    return mee.p_km / q


def mee_velocity_components_km_s(
    mee: MEEState,
    cfg: MEEDynamicsConfig,
) -> tuple[float, float]:
    sqrt_mu_over_p = math.sqrt(cfg.mu_km3_s2 / mee.p_km)
    vr = sqrt_mu_over_p * (mee.f * math.sin(mee.L_rad) - mee.g * math.cos(mee.L_rad))
    vt = sqrt_mu_over_p * (1.0 + mee.f * math.cos(mee.L_rad) + mee.g * math.sin(mee.L_rad))
    return vr, vt


def mee_kinematics_matrix(mee: MEEState, cfg: MEEDynamicsConfig) -> list[list[float]]:
    p = mee.p_km
    if p <= 0.0:
        raise OrbitalStateError("MEE semilatus rectum p must be positive.")

    q = compute_q_magnitude(mee)
    if q <= 0.0:
        raise OrbitalStateError("Invalid MEE state: q = 1 + f cos(L) + g sin(L) must stay positive.")

    L = mee.L_rad
    s2 = compute_s_squared(mee)
    sqrt_p_over_mu = math.sqrt(p / cfg.mu_km3_s2)
    hk_term = mee.h * math.sin(L) - mee.k * math.cos(L)

    return [
        [0.0, 2.0 * p / q * sqrt_p_over_mu, 0.0],
        [sqrt_p_over_mu * math.sin(L), sqrt_p_over_mu / q * ((q + 1.0) * math.cos(L) + mee.f), -sqrt_p_over_mu * mee.g / q * hk_term],
        [sqrt_p_over_mu * math.cos(L), sqrt_p_over_mu / q * ((q + 1.0) * math.sin(L) + mee.g), sqrt_p_over_mu * mee.f / q * hk_term],
        [0.0, 0.0, sqrt_p_over_mu * s2 / (2.0 * q) * math.sin(L)],
        [0.0, 0.0, sqrt_p_over_mu * s2 / (2.0 * q) * math.cos(L)],
        [0.0, 0.0, sqrt_p_over_mu * hk_term],
    ]


def mee_drift_vector(mee: MEEState, cfg: MEEDynamicsConfig) -> list[float]:
    p = mee.p_km
    q = compute_q_magnitude(mee)
    return [0.0, 0.0, 0.0, 0.0, 0.0, math.sqrt(cfg.mu_km3_s2 * p) * (q / p) ** 2]


def apply_mee_dynamics_matrix(
    mee: MEEState,
    accel_rtn_km_s2: Sequence[float],
    cfg: MEEDynamicsConfig,
) -> list[float]:
    A = mee_kinematics_matrix(mee, cfg)
    b = mee_drift_vector(mee, cfg)
    return [
        row[0] * accel_rtn_km_s2[0] + row[1] * accel_rtn_km_s2[1] + row[2] * accel_rtn_km_s2[2] + bias
        for row, bias in zip(A, b)
    ]


def total_perturbing_acceleration_rtn_km_s2(
    t_s: float,
    mee: MEEState,
    cfg: MEEDynamicsConfig,
    control_law: ControlLaw = zero_control,
) -> tuple[tuple[float, float, float], ControlInput]:
    control = control_law(t_s, mee, cfg)
    a_thrust = thrust_acceleration_rtn_km_s2(
        control,
        cfg.thrust_max_N,
        mee.m_kg,
        enabled=cfg.include_thrust,
    )

    vr_km_s, vt_km_s = mee_velocity_components_km_s(mee, cfg)
    a_drag = drag_acceleration_rtn_km_s2(
        mee,
        radius_km=mee_radius_km(mee),
        vr_km_s=vr_km_s,
        vt_km_s=vt_km_s,
        re_km=cfg.re_km,
        cd=cfg.cd,
        area_m2=cfg.area_m2,
        rho0_kg_m3=cfg.rho0_kg_m3,
        scale_height_km=cfg.scale_height_km,
        enabled=cfg.include_drag,
    )

    cart = mee_to_cartesian(mee, mu_km3_s2=cfg.mu_km3_s2)
    a_grav_eci = nonspherical_gravity_acceleration_eci_km_s2(
        mee,
        mu_km3_s2=cfg.mu_km3_s2,
        re_km=cfg.re_km,
        j2=cfg.j2,
        j3=cfg.j3,
        j4=cfg.j4,
        enabled=cfg.include_nonspherical_gravity,
    )
    a_grav = project_eci_acceleration_to_rtn(a_grav_eci, cart)

    return (
        (
            a_thrust[0] + a_drag[0] + a_grav[0],
            a_thrust[1] + a_drag[1] + a_grav[1],
            a_thrust[2] + a_drag[2] + a_grav[2],
        ),
        control,
    )


def mee_eom(
    t_s: float,
    state: Sequence[float],
    cfg: MEEDynamicsConfig,
    control_law: ControlLaw = zero_control,
) -> list[float]:
    if len(state) != 7:
        raise OrbitalStateError("MEE state must be [p, f, g, h, k, L, m].")

    mee = MEEState(
        p_km=float(state[0]),
        f=float(state[1]),
        g=float(state[2]),
        h=float(state[3]),
        k=float(state[4]),
        L_rad=float(state[5]),
        m_kg=float(state[6]),
    )
    accel_rtn, control = total_perturbing_acceleration_rtn_km_s2(t_s, mee, cfg, control_law)
    orbital_dot = apply_mee_dynamics_matrix(mee, accel_rtn, cfg)
    mdot = mass_flow_rate_kg_s(control, cfg.thrust_max_N, cfg.isp_s, cfg.ge_m_s2, enabled=cfg.include_thrust)
    return [*orbital_dot, mdot]


def zero_thrust_two_body_mee_eom(
    t_s: float,
    state: Sequence[float],
    mu_km3_s2: float = MU_EARTH_KM3_S2,
) -> list[float]:
    cfg = MEEDynamicsConfig(
        mu_km3_s2=mu_km3_s2,
        include_thrust=False,
        include_nonspherical_gravity=False,
        include_drag=False,
    )
    return mee_eom(t_s, state, cfg, zero_control)
