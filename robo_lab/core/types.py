from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol, Callable, Iterable, Optional
import numpy as np

Vec = np.ndarray  # shape (2,) or (3,)

@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float  # rad

@dataclass(frozen=True)
class Twist2D:
    v: float  # linear m/s
    w: float  # angular rad/s

@dataclass(frozen=True)
class ControlCommand:
    left_w: float
    right_w: float

class Planner(Protocol):
    def plan(self, start: Pose2D, goal: Pose2D, grid: "OccupancyGrid") -> list[Vec]:
        ...

class Controller(Protocol):
    def compute(self, pose: Pose2D, target: Vec) -> Twist2D:
        ...

@dataclass
class OccupancyGrid:
    resolution: float  # meters per cell
    width: int
    height: int
    origin: Vec  # world coords of grid (0,0) cell lower-left, shape (2,)
    data: np.ndarray  # shape (H, W), 0 free, 1 occupied

    def world_to_grid(self, pt: Vec) -> tuple[int, int]:
        gx = int((pt[0] - self.origin[0]) / self.resolution)
        gy = int((pt[1] - self.origin[1]) / self.resolution)
        return gx, gy

    def grid_to_world(self, ij: tuple[int, int]) -> Vec:
        i, j = ij
        return np.array([
            self.origin[0] + i * self.resolution + 0.5 * self.resolution,
            self.origin[1] + j * self.resolution + 0.5 * self.resolution,
        ])
