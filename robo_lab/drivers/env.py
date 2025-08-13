from __future__ import annotations
import time
import numpy as np
import pybullet as p
import pybullet_data
from dataclasses import dataclass
from typing import Iterable

@dataclass
class Obstacle:
    pos: np.ndarray  # (x, y)
    size: np.ndarray  # (sx, sy, sz)

class BulletEnv:
    def __init__(self, gui: bool = True, dt: float = 1.0/120.0) -> None:
        self.gui = gui
        self.dt = dt
        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        p.setTimeStep(self.dt, physicsClientId=self.cid)
        p.setGravity(0, 0, -9.81, physicsClientId=self.cid)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = p.loadURDF("plane.urdf")
        self._bodies: list[int] = []

    def add_box(self, pos_xy: np.ndarray, size_xyz: np.ndarray, yaw: float = 0.0) -> int:
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size_xyz/2)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size_xyz/2)
        quat = p.getQuaternionFromEuler([0,0,yaw])
        body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[pos_xy[0], pos_xy[1], size_xyz[2]/2],
            baseOrientation=quat,
        )
        self._bodies.append(body)
        return body

    def step(self) -> None:
        p.stepSimulation(physicsClientId=self.cid)
        if self.gui:
            time.sleep(self.dt)

    def disconnect(self) -> None:
        p.disconnect(self.cid)