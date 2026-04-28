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
class KEState:
    a: torch.Tensor # (...,1)
    e: torch.Tensor # (...,1)
    i: torch.Tensor # (...,1)
    raan: torch.Tensor # (...,1)
    aop: torch.Tensor # (...,1)
    nu: torch.Tensor # (...,1)
    mass: torch.Tensor # (...,1)

    def to_tensor(self):
        return torch.cat(
            [
                self.a,
                self.e,
                self.i,
                self.raan,
                self.aop,
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
            raan = tensor[:,3:4],
            aop = tensor[:,4:5],
            nu = tensor[:,5:6],
            mass = tensor[:,6:7]
        )
    
    def get_mee(self) -> MEEState:
        p = self.a*(1-self.e**2)
        f = self.e*torch.cos(self.raan+self.aop)
        g = self.e*torch.sin(self.raan+self.aop)
        h = torch.tan(self.i/2)*torch.cos(self.raan)
        k = torch.tan(self.i/2)*torch.sin(self.raan)
        L = self.raan+self.aop+self.nu
        return MEEState(p,f,g,h,k,L,self.mass)
    
    def get_eci(self, mu : float) -> ECIState:
        mee = self.get_mee()
        return mee.get_eci(mu)

@dataclass
class ECIState:
    x: torch.Tensor # (...,1)
    y: torch.Tensor # (...,1)
    z: torch.Tensor # (...,1)
    vx: torch.Tensor # (...,1)
    vy: torch.Tensor # (...,1)
    vz: torch.Tensor # (...,1)
    mass: torch.Tensor # (...,1)

    def to_tensor(self):
        return torch.cat(
            [
                self.x,
                self.y,
                self.z,
                self.vx,
                self.vy,
                self.vz,
                self.mass
            ],
            dim=-1
        )

    @classmethod
    def from_tensor(cls, tensor: torch.Tensor):
        return cls(
            x = tensor[:,0:1],
            y = tensor[:,1:2],
            z = tensor[:,2:3],
            vx = tensor[:,3:4],
            vy = tensor[:,4:5],
            vz = tensor[:,5:6],
            mass = tensor[:,6:7]
        )
    
    def get_ke(self, mu : float) -> KEState:
        """
        ECI -> KE 변환 (Vectorial elements 활용)
        """
        r_vec = torch.stack([self.x, self.y, self.z], dim=-1).squeeze(1) # (B, 3)
        v_vec = torch.stack([self.vx, self.vy, self.vz], dim=-1).squeeze(1)
        
        r_mag = torch.norm(r_vec, dim=-1, keepdim=True)
        v_mag = torch.norm(v_vec, dim=-1, keepdim=True)
        
        # Specific Angular Momentum
        h_vec = torch.cross(r_vec, v_vec, dim=-1)
        h_mag = torch.norm(h_vec, dim=-1, keepdim=True)
        
        # Semi-major axis
        energy = (v_mag**2 / 2) - (mu / r_mag)
        a = -mu / (2 * energy)
        
        # Eccentricity vector
        e_vec = (torch.cross(v_vec, h_vec, dim=-1) / mu) - (r_vec / r_mag)
        e = torch.norm(e_vec, dim=-1, keepdim=True)
        
        # Inclination
        i = torch.acos(h_vec[:, 2:3] / h_mag)
        
        # RAAN (Node vector n)
        k_unit = torch.tensor([0, 0, 1.0], device=r_vec.device)
        n_vec = torch.cross(k_unit.expand_as(h_vec), h_vec, dim=-1)
        n_mag = torch.norm(n_vec, dim=-1, keepdim=True)
        raan = torch.where(n_mag > 1e-9, torch.atan2(n_vec[:, 1:2], n_vec[:, 0:1]), torch.zeros_like(i))
        
        # Argument of Periapsis
        aop = torch.where(n_mag > 1e-9, 
                          torch.acos(torch.sum(n_vec * e_vec, dim=-1, keepdim=True) / (n_mag * e)),
                          torch.atan2(e_vec[:, 1:2], e_vec[:, 0:1]))
        aop = torch.where(e_vec[:, 2:3] < 0, 2*torch.pi - aop, aop)
        
        # True Anomaly
        nu = torch.acos(torch.sum(e_vec * r_vec, dim=-1, keepdim=True) / (e * r_mag))
        nu = torch.where(torch.sum(r_vec * v_vec, dim=-1, keepdim=True) < 0, 2*torch.pi - nu, nu)
        
        return KEState(a, e, i, raan, aop, nu, self.mass)

    def get_mee(self, mu: float) -> MEEState:
        ke = self.get_ke(mu)
        return ke.get_mee()

@dataclass
class MEEState:
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
    
    def get_eci(self, mu: float) -> ECIState:
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
        
        return ECIState(x, y, z, vx, vy, vz, self.mass)

    def get_ke(self) -> KEState:
            """
            MEE -> KE 변환
            """
            e = torch.sqrt(self.f**2 + self.g**2)
            a = self.p / (1 - e**2)
            i = 2 * torch.atan(torch.sqrt(self.h**2 + self.k**2))
            raan = torch.atan2(self.h, self.k)
            aop = torch.atan2(self.g,self.f) - raan
            nu = self.L - torch.atan2(self.g,self.f)
            return KEState(a, e, i, raan, aop, nu, self.mass)

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