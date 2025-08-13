from __future__ import annotations
import numpy as np
import pybullet as p
from dataclasses import dataclass
from typing import Tuple

@dataclass
class DiffDriveConfig:
    radius: float = 0.15  # base radius (for viz)
    wheel_r: float = 0.05
    wheel_base: float = 0.30
    height: float = 0.10

class DiffDrive:
    def __init__(self, env_cid: int, cfg: DiffDriveConfig) -> None:
        self.cfg = cfg
        self.body = self._spawn(env_cid)

    def _spawn(self, cid: int) -> int:
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=self.cfg.radius, height=self.cfg.height)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=self.cfg.radius, length=self.cfg.height, rgbaColor=[0.2,0.4,0.8,1])
        body = p.createMultiBody(baseMass=5.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                 basePosition=[-3, -3, self.cfg.height/2])
        return body

    def pose(self) -> Tuple[float, float, float]:
        pos, orn = p.getBasePositionAndOrientation(self.body)
        yaw = p.getEulerFromQuaternion(orn)[2]
        return pos[0], pos[1], yaw

    def apply_twist(self, v: float, w: float, dt: float) -> None:
        # Simple kinematic integration as external velocity command
        x, y, yaw = self.pose()
        nx = x + v * np.cos(yaw) * dt
        ny = y + v * np.sin(yaw) * dt
        nyaw = yaw + w * dt
        quat = p.getQuaternionFromEuler([0,0,nyaw])
        p.resetBasePositionAndOrientation(self.body, [nx, ny, self.cfg.height/2], quat)
