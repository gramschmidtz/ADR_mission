# src/adr_mission/utils.py

from __future__ import annotations

import torch
from .states import MEEState

def make_unit_vector(vector):
    return vector / torch.linalg.norm(vector, ord=2, dim=-1, keepdim=True)

def ke_to_mee(a, e, i, RAAN, AOP, nu, mass) -> MEEState:
    """
    a [km]
    e [무차원]
    i [degree]
    RAAN [degree]
    AOP [degree]
    nu [degree]
    mass [kg]
    """
    i_rad = torch.deg2rad(i)
    RAAN_rad = torch.deg2rad(RAAN)
    AOP_rad = torch.deg2rad(AOP)
    nu_rad = torch.deg2rad(nu)

    p = a * (1 - e**2)
    f = e * torch.cos(RAAN_rad + AOP_rad)
    g = e * torch.sin(RAAN_rad + AOP_rad)
    h = torch.tan(i_rad/2) * torch.cos(RAAN_rad)
    k = torch.tan(i_rad/2) * torch.sin(RAAN_rad)
    L = RAAN_rad + AOP_rad + nu_rad

    return MEEState(p,f,g,h,k,L,mass)


def mee_to_eci(p,f,g,h,k,L, mu) -> torch.Tensor:
    """
    MEE (p,f,g,h,k,L)에서
    ECI frame의 r (x,y,z), v (vx,vy,vz)으로 변환
    """
    q = 1 + f * torch.cos(L) + g * torch.sin(L)
    r = p / q
    chi = torch.sqrt(h**2 + k**2)
    s2 = 1 + chi**2

    sin_L = torch.sin(L)
    cos_L = torch.cos(L)

    x = (r/s2) * (1+h**2-k**2) * cos_L + 2*h*k*sin_L
    y = (r/s2) * (1-h**2+k**2) * sin_L + 2*h*k*cos_L
    z = (2*r/s2) * (h*sin_L - k*cos_L)

    position = torch.cat([x, y, z], dim=-1)

    vx = (-1/s2) * torch.sqrt(mu/p) * (sin_L + (h**2 - k**2)*sin_L - 2*h*k*cos_L + g - f*(h**2) + f*(k**2) + 2*g*h*k)
    vy = (-1/s2) * torch.sqrt(mu/p) * (-cos_L + (h**2 - k**2)*cos_L - 2*h*k*sin_L - f + g*(h**2) - g*(k**2) + 2*f*h*k)
    vz = (2/s2) * torch.sqrt(mu/p) * (h*cos_L + k*sin_L + f*h +g*k)

    velocity = torch.cat([vx, vy, vz], dim=-1)
    
    return position, velocity

def P2sinphi(r):
    sinphi = r[:,2:3] / torch.linalg.norm(r, ord=2, dim=-1, keepdim=True)
    p2 = 0.5 * (3 * (sinphi)**2 - 1)
    dp2 = 3 * sinphi
    return p2, dp2