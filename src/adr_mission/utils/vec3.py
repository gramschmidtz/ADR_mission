from __future__ import annotations

import math
from typing import Sequence


Vector3 = tuple[float, float, float]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(v: Sequence[float]) -> float:
    return math.sqrt(dot(v, v))


def cross(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def scale(v: Sequence[float], s: float) -> Vector3:
    return (v[0] * s, v[1] * s, v[2] * s)


def add(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def unit(v: Sequence[float], *, eps: float = 1e-15) -> Vector3:
    n = norm(v)
    if n < eps:
        raise ValueError("Cannot normalize a near-zero vector.")
    return scale(v, 1.0 / n)
