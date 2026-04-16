from __future__ import annotations

"""Non-spherical gravity perturbation helpers."""

import math

from adr_mission.utils.vec3 import dot, norm, scale, sub
from .conversions import mee_to_cartesian
from .frames import local_orbital_frame_from_cartesian
from .state_definitions import MEEState


def legendre_p2(x: float) -> float:
    return 0.5 * (3.0 * x**2 - 1.0)


def legendre_p3(x: float) -> float:
    return 0.5 * (5.0 * x**3 - 3.0 * x)


def legendre_p4(x: float) -> float:
    return (35.0 * x**4 - 30.0 * x**2 + 3.0) / 8.0


def legendre_dp2(x: float) -> float:
    return 3.0 * x


def legendre_dp3(x: float) -> float:
    return 0.5 * (15.0 * x**2 - 3.0)


def legendre_dp4(x: float) -> float:
    return 0.5 * (35.0 * x**3 - 15.0 * x)


def nonspherical_gravity_acceleration_eci_km_s2(
    mee: MEEState,
    *,
    mu_km3_s2: float,
    re_km: float,
    j2: float,
    j3: float,
    j4: float,
    enabled: bool = True,
) -> tuple[float, float, float]:
    if not enabled:
        return (0.0, 0.0, 0.0)

    cart = mee_to_cartesian(mee, mu_km3_s2=mu_km3_s2)
    r = (cart.rx_km, cart.ry_km, cart.rz_km)
    r_norm = norm(r)
    r_hat, _, _ = local_orbital_frame_from_cartesian(cart)

    e_north = (0.0, 0.0, 1.0)
    north_raw = sub(e_north, scale(r_hat, dot(e_north, r_hat)))
    north_norm = norm(north_raw)
    i_north = (0.0, 0.0, 0.0) if north_norm < 1e-14 else scale(north_raw, 1.0 / north_norm)

    sin_phi = r[2] / r_norm
    cos_phi = math.sqrt(max(0.0, 1.0 - sin_phi**2))
    re_over_r = re_km / r_norm
    mu_over_r2 = mu_km3_s2 / (r_norm**2)

    dg_n = -mu_over_r2 * cos_phi * (
        (re_over_r**2) * legendre_dp2(sin_phi) * j2
        + (re_over_r**3) * legendre_dp3(sin_phi) * j3
        + (re_over_r**4) * legendre_dp4(sin_phi) * j4
    )
    dg_r = -mu_over_r2 * (
        3.0 * (re_over_r**2) * legendre_p2(sin_phi) * j2
        + 4.0 * (re_over_r**3) * legendre_p3(sin_phi) * j3
        + 5.0 * (re_over_r**4) * legendre_p4(sin_phi) * j4
    )

    return (
        dg_n * i_north[0] - dg_r * r_hat[0],
        dg_n * i_north[1] - dg_r * r_hat[1],
        dg_n * i_north[2] - dg_r * r_hat[2],
    )


def project_eci_acceleration_to_rtn(
    accel_eci_km_s2: tuple[float, float, float],
    cart,
) -> tuple[float, float, float]:
    r_hat, t_hat, h_hat = local_orbital_frame_from_cartesian(cart)
    return (dot(accel_eci_km_s2, r_hat), dot(accel_eci_km_s2, t_hat), dot(accel_eci_km_s2, h_hat))
