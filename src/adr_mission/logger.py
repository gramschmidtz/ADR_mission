# src/adr_mission/logger.py

from __future__ import annotations

import torch
from .states import MEEState

class MissionTrajectory:
    def __init__(self, num_steps: int, batch_size: int, device: str = "cpu"):
        # [Time, Batch, Features(7)] 형태의 텐서 미리 할당
        self.storage = torch.zeros((num_steps, batch_size, 7), device=device)
        self.time_storage = torch.zeros((num_steps,), device=device)
        self.step = 0

    def update(self, t: float, state: MEEState):
        """현재 시점의 상태를 텐서에 기록"""
        if self.step < self.storage.shape[0]:
            self.storage[self.step] = state.to_tensor()
            self.time_storage[self.step] = t
            self.step += 1

    def get_full_trajectory(self):
        """전체 궤적 텐서 반환 (딥러닝 모델 입력용으로 용이)"""
        return self.storage[:self.step]