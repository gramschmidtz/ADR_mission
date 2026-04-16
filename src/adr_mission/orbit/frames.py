from __future__ import annotations

"""Local orbital frame helpers."""

from adr_mission.utils.vec3 import cross, unit
from .state_definitions import CartesianState


def local_orbital_frame_from_cartesian(
    state: CartesianState,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Return (r_hat, t_hat, h_hat) unit vectors in ECI."""
    r = (state.rx_km, state.ry_km, state.rz_km)
    v = (state.vx_km_s, state.vy_km_s, state.vz_km_s)
    r_hat = unit(r)
    h_hat = unit(cross(r, v))
    t_hat = cross(h_hat, r_hat)
    return r_hat, t_hat, h_hat
