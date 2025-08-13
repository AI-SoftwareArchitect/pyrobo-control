from __future__ import annotations
import numpy as np
import pybullet as p
from dataclasses import dataclass
from typing import Tuple
from .env import BulletEnv

@dataclass
class LidarConfig:
    num_rays: int = 181
    fov: float = np.deg2rad(180)
    max_range: float = 8.0
    z: float = 0.2  # sensor height

class Lidar2D:
    def __init__(self, env: BulletEnv, robot_body: int, cfg: LidarConfig) -> None:
        self.env = env
        self.robot_body = robot_body
        self.cfg = cfg

    def scan(self) -> Tuple[np.ndarray, np.ndarray]:
        # returns angles, ranges
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_body)
        yaw = p.getEulerFromQuaternion(base_orn)[2]
        origin = np.array(base_pos)
        angles = np.linspace(-self.cfg.fov/2, self.cfg.fov/2, self.cfg.num_rays) + yaw
        from_pts = []
        to_pts = []
        for a in angles:
            d = self.cfg.max_range
            sx = origin[0]
            sy = origin[1]
            sz = origin[2] + self.cfg.z
            ex = sx + d * np.cos(a)
            ey = sy + d * np.sin(a)
            ez = sz
            from_pts.append([sx, sy, sz])
            to_pts.append([ex, ey, ez])
        results = p.rayTestBatch(from_pts, to_pts)
        ranges = np.array([
            r[2]*self.cfg.max_range if r[0] != -1 else self.cfg.max_range for r in results
        ])
        rel_angles = np.linspace(-self.cfg.fov/2, self.cfg.fov/2, self.cfg.num_rays)
        return rel_angles, ranges