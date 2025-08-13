from __future__ import annotations
import heapq
import numpy as np
from typing import List, Tuple
from ..core.types import OccupancyGrid, Pose2D

class AStarPlanner:
    def __init__(self, diag: bool = True) -> None:
        self.diag = diag
        self.moves = [
            (1,0,1.0),(-1,0,1.0),(0,1,1.0),(0,-1,1.0)
        ]
        if diag:
            self.moves += [(1,1,1.4),(1,-1,1.4),(-1,1,1.4),(-1,-1,1.4)]

    def plan(self, start: Pose2D, goal: Pose2D, grid: OccupancyGrid) -> List[np.ndarray]:
        s = grid.world_to_grid(np.array([start.x, start.y]))
        g = grid.world_to_grid(np.array([goal.x, goal.y]))
        W, H = grid.width, grid.height
        def h(i,j):
            return np.hypot(i-g[0], j-g[1])
        openq = []
        heapq.heappush(openq, (0+h(*s), 0, s, None))
        came = {}
        costs = {s:0.0}
        occ = grid.data
        visited = set()
        while openq:
            f, c, node, parent = heapq.heappop(openq)
            if node in visited:
                continue
            visited.add(node)
            came[node] = parent
            if node == g:
                break
            i,j = node
            for dx,dy,w in self.moves:
                ni, nj = i+dx, j+dy
                if not (0<=ni<W and 0<=nj<H):
                    continue
                if occ[nj, ni] == 1:
                    continue
                nc = c + w
                if (ni,nj) not in costs or nc < costs[(ni,nj)]:
                    costs[(ni,nj)] = nc
                    heapq.heappush(openq, (nc + h(ni,nj), nc, (ni,nj), node))
        # reconstruct
        path_cells: list[tuple[int,int]] = []
        cur = g
        if cur not in came:
            return []
        while cur is not None:
            path_cells.append(cur)
            cur = came[cur]
        path_cells.reverse()
        path = [grid.grid_to_world(c) for c in path_cells]
        return path
