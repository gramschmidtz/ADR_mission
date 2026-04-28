# src/adr_mission/states.py
"""
MEE 상태 x=(p,f,g,h,k,L)
Trajectory
"""
from __future__ import annotations

import torch
from dataclasses import dataclass
from scipy.io import savemat

@dataclass
class OrbitState:
    p: torch.Tensor # (...,1)
    f: torch.Tensor # (...,1)
    g: torch.Tensor # (...,1)
    h: torch.Tensor # (...,1)
    k: torch.Tensor # (...,1)
    L: torch.Tensor # (...,1)
    mass: torch.Tensor # (...,1)

    def to_tensor(self):
        return torch.cat(
            [
                self.p,
                self.f,
                self.g,
                self.h,
                self.k,
                self.L,
                self.mass
            ],
            dim=-1
        )
    
    @classmethod
    def from_tensor(cls, tensor: torch.Tensor):
        return cls(
            p = tensor[:,0:1],
            f = tensor[:,1:2],
            g = tensor[:,2:3],
            h = tensor[:,3:4],
            k = tensor[:,4:5],
            L = tensor[:,5:6],
            mass = tensor[:,6:7]
        )
    
    def get_eci(self, mu: float):
        """
        MEE -> ECI Cartisian (r,v) 변환
        """
        q = 1 + self.f * torch.cos(self.L) + self.g * torch.sin(self.L)
        r_mag = self.p / q
        alpha2 = self.h**2 - self.k**2
        s2 = 1 + self.h**2 + self.k**2
        cosL, sinL = torch.cos(self.L), torch.sin(self.L)

        # Position (r) 
        x = (r_mag/s2) * (cosL + alpha2*cosL + 2*self.h*self.k*sinL)
        y = (r_mag/s2) * (sinL - alpha2*sinL + 2*self.h*self.k*cosL)
        z = (2*r_mag/s2) * (self.h*sinL - self.k*cosL)
        
        # Velocity (v)
        sqrt_mu_p = torch.sqrt(mu / self.p)
        vx = -(sqrt_mu_p/s2) * (sinL + alpha2*sinL - 2*self.h*self.k*cosL + self.g - self.f*alpha2 + 2*self.g*self.h*self.k)
        vy = -(sqrt_mu_p/s2) * (-cosL + alpha2*cosL + 2*self.h*self.k*sinL - self.f + self.g*alpha2 + 2*self.f*self.h*self.k)
        vz = (2*sqrt_mu_p/s2) * (self.h*cosL + self.k*sinL + self.f*self.h + self.g*self.k)
        
        return torch.cat([x, y, z], dim=-1), torch.cat([vx, vy, vz], dim=-1)

    def get_ke(self):
            """
            MEE -> KE 변환
            """
            ecc = torch.sqrt(self.f**2 + self.g**2)
            a = self.p / (1 - ecc**2)
            inc = 2 * torch.atan(torch.sqrt(self.h**2 + self.k**2))
            raan = torch.atan2(self.h, self.k)
            aop = torch.atan2(self.g,self.f) - raan
            nu = self.L - torch.atan2(self.g,self.f)
            return a, ecc, inc, raan, aop, nu

""""
class Trajectory:
    def __init__(self, num_steps: int, batch_size: int, device: str):
        self.data = torch.zeros((num_steps, batch_size, 7), device=device)
        self.times = torch.zeros(num_steps, device=device)
        self.curr = 0

    def log(self, t, state: MEEState):
        if self.curr < self.data.shape[0]:
            self.data[self.curr] = state.to_tensor()
            self.times[self.curr] = t
            self.curr += 1
    
    def to_mat(self, filename: str):
        savemat(filename, {
            'states': self.data.cpu().numpy(), # [Time, Batch, 7]
            'time': self.times.cpu().numpy()
        })

@dataclass
class KEState:
    a: torch.Tensor # (...,1)
    e: torch.Tensor # (...,1)
    i: torch.Tensor # (...,1)
    RAAN: torch.Tensor # (...,1)
    AOP: torch.Tensor # (...,1)
    nu: torch.Tensor # (...,1)
    mass: torch.Tensor # (...,1)

    def to_tensor(self):
        return torch.cat(
            [
                self.a,
                self.e,
                self.i,
                self.RAAN,
                self.AOP,
                self.nu,
                self.mass
            ],
            dim=-1
        )
    
    @classmethod
    def from_tensor(cls, tensor: torch.Tensor):
        return cls(
            a = tensor[:,0:1],
            e = tensor[:,1:2],
            i = tensor[:,2:3],
            RAAN = tensor[:,3:4],
            AOP = tensor[:,4:5],
            nu = tensor[:,5:6],
            mass = tensor[:,6:7]
        )
    
    def to_device(self, device: str):
        return KEState(**{k: v.to(device) for k,v in self.__dict__.items()})
"""