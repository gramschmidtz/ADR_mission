# src/adr_mission/propagation.py
"""
고정스텝rk4로 적분
"""
from __future__ import annotations

import torch
from .states import MEEState

def rk4(dynamics_fn, state: MEEState, u, t, dt):
    """
    고정 스텝 rk4 적분기

    Args:
        dynamics_fn: x'=f(x,u,t)
        state: MEEState 객체 (현재 상태)
        u: 입력 벡터 (추력 가속도 등, 없을 경우 None)
        t: 현재시간
        dt = 적분 시간 간격 (Step size)
    """
    x = state.to_tensor()

    k1 = dynamics_fn(x, u, t)
    k2 = dynamics_fn(x + dt/2 * k1, u, t + dt/2)
    k3 = dynamics_fn(x + dt/2 * k2, u, t + dt/2)
    k4 = dynamics_fn(x + dt * k3, u, t + dt)

    next_x = x + (k1 + 2*k2 + 2*k3 + k4) * (dt/6.0)

    return MEEState.from_tensor(next_x)
