from __future__ import annotations

"""Project-wide physical constants used by ADR_mission.

Units
-----
- distance: km
- time: s
- mass: kg
- angles: rad
"""

MU_EARTH_KM3_S2 = 398600.4418
R_EARTH_KM = 6378.14
G0_M_S2 = 9.8066

J2 = 1082.639e-6
J3 = -2.565e-6
J4 = -1.608e-6

ECC_EPS_DEFAULT = 1e-10
INC_EPS_DEFAULT = 1e-10
TWOPI = 2.0 * 3.141592653589793