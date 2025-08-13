from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from ..core.types import Pose2D, Twist2D

@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float

class HeadingSpeedController:
    def __init__(self, v_ref: float, yaw_gains: PIDGains) -> None:
        self.v_ref = v_ref
        self.g = yaw_gains
        self._e_int = 0.0
        self._e_prev = 0.0

    def compute(self, pose: Pose2D, target: np.ndarray) -> Twist2D:
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        desired = np.arctan2(dy, dx)
        e = (desired - pose.yaw + np.pi) % (2*np.pi) - np.pi
        self._e_int += e
        de = e - self._e_prev
        self._e_prev = e
        w = self.g.kp*e + self.g.ki*self._e_int + self.g.kd*de
        return Twist2D(v=self.v_ref, w=float(w))