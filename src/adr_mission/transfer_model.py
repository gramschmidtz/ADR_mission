# src/adr_mission/transfer_model.py
"""
"""
from __future__ import annotations

import torch
from .states import MEEState

class RAANTransfer:
    def __init__(self, vehicle_cfg, constants_cfg):
        self.vehicle_cfg = vehicle_cfg
        self.constants_cfg = constants_cfg
        self.mu = constants_cfg['physics']['mu_m3_per_s2']
        self.J_2 = constants_cfg['physics']['J_2']
        self.R_e = constants_cfg['physics']['R_e_m']
        self.h_0 = constants_cfg['atmosphere']['h_0']
        self.rho_0 = constants_cfg['atmosphere']['rho_0']
        self.H = constants_cfg['atmosphere']['H']

        self.S = vehicle_cfg['propulsion']['S_m2']
        self.C_D = vehicle_cfg['propulsion']['C_D']
        

    def _Ax(self, state: MEEState) -> torch.Tensor:
        p = state.p
        f = state.f
        g = state.g
        h = state.h
        k = state.k
        L = state.L

        q = 1 + f * torch.cos(L) + g * torch.sin(L)
        chi = torch.sqrt(h**2 + k**2)
        s2 = 1 + chi**2

        batch_size = p.shape[0]
        device = p.device

        A = torch.zeros((batch_size, 6, 3), device=device)

        sqrt_p_mu = torch.sqrt(p/self.mu)

        A[:, 0, 1] = ((2 * p / q) * sqrt_p_mu).squeeze()
        
        A[:, 1, 0] = (sqrt_p_mu * torch.sin(L)).squeeze()
        A[:, 1, 1] = (sqrt_p_mu / q * ((q + 1) * torch.cos(L) + f)).squeeze()
        A[:, 1, 2] = (-sqrt_p_mu * (g / q) * (h * torch.sin(L) - k * torch.cos(L))).squeeze()
        
        A[:, 2, 0] = (sqrt_p_mu * torch.cos(L)).squeeze()
        A[:, 2, 1] = (sqrt_p_mu / q * ((q + 1) * torch.sin(L) + g)).squeeze()
        A[:, 2, 2] = (sqrt_p_mu * (f / q) * (h * torch.sin(L) - k * torch.cos(L))).squeeze()
        
        A[:, 3, 2] = (sqrt_p_mu * s2 * torch.sin(L) / (2 * q)).squeeze()
        A[:, 4, 2] = (sqrt_p_mu * s2 * torch.cos(L) / (2 * q)).squeeze()
        A[:, 5, 2] = (sqrt_p_mu * (h * torch.sin(L) - k * torch.cos(L))).squeeze()

        return A

    def _bx(self, state: MEEState) -> torch.Tensor:
        p = state.p
        f = state.f
        g = state.g
        L = state.L

        q = 1 + f * torch.cos(L) + g * torch.sin(L)

        b = torch.zeros((p.shape[0], 6, 1), device=p.device)
        b[:, 5, 0] = (torch.sqrt(self.mu * p) * (q/p)**2).squeeze(-1)

        return b
    
    def _a_g(self, state: MEEState) -> torch.Tensor:
        p = state.p
        f = state.f
        g = state.g
        h = state.h
        k = state.k
        L = state.L

        q = 1 + f * torch.cos(L) + g * torch.sin(L)

        r_norm = p/q
        s2 = 1 + h**2 + k**2

        r, v = mee_to_eci(p,f,g,h,k,L, mu=self.mu)
        i_r = make_unit_vector(r)

        rcv = torch.cross(r, v, dim=-1)
        i_h = make_unit_vector(rcv)

        i_theta = torch.cross(i_h, i_r, dim=-1)

        Qr = torch.stack([i_r, i_theta, i_h], dim=-1)

        e_n = torch.tensor([0,0,1], dtype=torch.float32, device=p.device)

        enmentirir = e_n - torch.sum(e_n * i_r, dim=-1, keepdim=True) * i_r
        i_n = make_unit_vector(enmentirir)

        p2sinphi, dp2sinphi = P2sinphi(r)

        sinphi = r[:,2:3] / r_norm
        cosphi = torch.sqrt(1 - sinphi**2)
        
        delta_g_n = (-(self.mu*cosphi)/r_norm**2) * ((self.R_e/r_norm)**2) * dp2sinphi * self.J_2
        delta_g_r = (-self.mu/r_norm**2) * 3 * ((self.R_e/r_norm)**2) * p2sinphi * self.J_2

        delta_g = delta_g_n * i_n - delta_g_r * i_r

        a_g = torch.bmm(Qr.transpose(1,2), delta_g.unsqueeze(-1)).squeeze(-1)

        return a_g

    def _a_D(self, state: MEEState) -> torch.Tensor:
        p = state.p
        f = state.f
        g = state.g
        h = state.h
        k = state.k
        L = state.L

        SCD = self.S * self.C_D

        r, v = mee_to_eci(p,f,g,h,k,L, mu=self.mu)
        r_norm = torch.linalg.norm(r, ord=2, dim=-1, keepdim=True)
        height =  r_norm - self.R_e
        rho = make_rho(height, self.rho_0, self.h_0, self.H)
        v_norm = torch.linalg.norm(v, ord=2, dim=-1, keepdim=True)

        v_r = torch.sqrt(self.mu/p) * (f * torch.sin(L) - g * torch.cos(L))
        v_theta = torch.sqrt(self.mu/p) * (1 + f * torch.cos(L) + g * torch.sin(L))

        a_D_r = -0.5 * rho * SCD * v_norm * v_r
        a_D_theta = -0.5 * rho * SCD * v_norm * v_theta

        return torch.cat([a_D_r, a_D_theta, torch.zeros_like(a_D_r)], dim=-1)

    def dynamics_model(
                self,
                x: torch.Tensor,
                u: torch.Tensor,
                t: float  # t 인자 추가 (rk4와 인터페이스 맞춤)
        ) -> torch.Tensor:
        state = MEEState.from_tensor(x)

        A = self._Ax(state)
        b = self._bx(state)

        # 자연 환경 섭동만 적용 (입력 u는 무시)
        a_env = self._a_g(state) + self._a_D(state)
        x_dot = torch.bmm(A, a_env.unsqueeze(-1)) + b

        # 파편은 질량이 감소하지 않음
        mass_dot = torch.zeros_like(state.mass)
        
        return torch.cat([x_dot.squeeze(-1), mass_dot], dim=-1)