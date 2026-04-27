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
    
    def to_device(self, device: str):
        return MEEState(**{k: v.to(device) for k,v in self.__dict__.items()})

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