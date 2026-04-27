# src/adr_mission/transfer_model.py
"""
"""
from __future__ import annotations

import torch
from .states import MEEState
from .utils import mee_to_ke

class RAANTransfer:
    def __init__(self, vehicle_cfg, constants_cfg):
        self.vehicle_cfg = vehicle_cfg
        self.constants_cfg = constants_cfg
        self.mu = constants_cfg['physics']['mu_m3_per_s2']
        self.J_2 = constants_cfg['physics']['J_2']
        self.R_e = constants_cfg['physics']['R_e_m']        

    def raan_transfer_model(
                self,
                x: torch.Tensor,
                u: torch.Tensor,  # (rk4와 인터페이스 맞춤)
                t: float          # (rk4와 인터페이스 맞춤)
        ) -> torch.Tensor:

        mee = MEEState.from_tensor(x)
        ke = mee_to_ke(mee)
        R2 = (ke.a*1000.0 - self.R_e)**2
        a72 = ke.a**3.5
        oneminusesqsq = (1 - ke.e**2)**2
        i = ke.i
        J2 = self.J_2
        sqrtmu = torch.sqrt(self.mu)

        RAAN_dot = -1.5 * ((J2*sqrtmu*R2)/(a72*oneminusesqsq)) * torch.cos(i)
        
        return RAAN_dot