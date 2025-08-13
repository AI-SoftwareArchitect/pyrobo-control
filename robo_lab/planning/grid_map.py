from __future__ import annotations
import numpy as np
from ..core.types import OccupancyGrid

class GridBuilder:
    def __init__(self, res: float, width: int, height: int, origin: np.ndarray) -> None:
        self.grid = OccupancyGrid(resolution=res, width=width, height=height, origin=origin, data=np.zeros((height, width), dtype=np.uint8))

    def insert_scan(self, pose: np.ndarray, angles: np.ndarray, ranges: np.ndarray, max_range: float) -> None:
        # simple endpoint marking
        for a, r in zip(angles, ranges):
            if r >= max_range:
                continue
            end = np.array([pose[0] + r*np.cos(pose[2]+a), pose[1] + r*np.sin(pose[2]+a)])
            gx, gy = self.grid.world_to_grid(end)
            if 0 <= gx < self.grid.width and 0 <= gy < self.grid.height:
                self.grid.data[gy, gx] = 1

    def get(self) -> OccupancyGrid:
        return self.grid