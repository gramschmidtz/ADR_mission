from __future__ import annotations

import math


def wrap_to_2pi(angle_rad: float) -> float:
    """Wrap an angle to [0, 2π)."""
    return angle_rad % (2.0 * math.pi)


def clamp(value: float, low: float, high: float) -> float:
    """Clamp a scalar into [low, high]."""
    return max(low, min(high, value))


def safe_acos(x: float) -> float:
    """acos with input clamped to [-1, 1]."""
    return math.acos(clamp(x, -1.0, 1.0))
