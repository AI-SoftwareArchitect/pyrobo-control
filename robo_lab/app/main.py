from __future__ import annotations
import time
import numpy as np
from ..drivers.env import BulletEnv
from ..robot.diff_drive import DiffDrive, DiffDriveConfig
from ..drivers.lidar import Lidar2D, LidarConfig
from ..planning.grid_map import GridBuilder
from ..planning.a_star import AStarPlanner
from ..control.pid import HeadingSpeedController, PIDGains
from ..core.types import Pose2D
from .config import AppConfig

class Scenario:
    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg
        self.env = BulletEnv(gui=cfg.gui, dt=cfg.dt)
        # obstacles (basit labirent)
        self.env.add_box(np.array([0, 0]), np.array([0.2, 12, 1]))
        self.env.add_box(np.array([0, 0]), np.array([12, 0.2, 1]))
        self.env.add_box(np.array([2.0, 2.0]), np.array([1.0, 2.0, 1]))
        self.env.add_box(np.array([-1.0, 3.0]), np.array([2.0, 1.0, 1]))
        self.robot = DiffDrive(self.env.cid, DiffDriveConfig())
        self.lidar = Lidar2D(self.env, self.robot.body, LidarConfig(max_range=7.0))
        origin = np.array([-8.0, -8.0])
        self.grid = GridBuilder(self.cfg.map_res, self.cfg.map_w, self.cfg.map_h, origin)
        self.planner = AStarPlanner(diag=True)
        self.ctrl = HeadingSpeedController(v_ref=0.8, yaw_gains=PIDGains(1.8, 0.0, 0.15))
        self.path: list[np.ndarray] = []
        self.target_idx = 0
        self.goal = Pose2D(6.0, 6.0, 0.0)

    def sense_and_map(self) -> None:
        angles, ranges = self.lidar.scan()
        x, y, yaw = self.robot.pose()
        self.grid.insert_scan(np.array([x,y,yaw]), angles, ranges, self.lidar.cfg.max_range)

    def plan_once(self) -> None:
        x, y, yaw = self.robot.pose()
        start = Pose2D(x, y, yaw)
        self.path = self.planner.plan(start, self.goal, self.grid.get())
        self.target_idx = 0

    def follow_path(self) -> None:
        if not self.path:
            return
        x, y, yaw = self.robot.pose()
        pose = Pose2D(x, y, yaw)
        # ilerleme
        if self.target_idx < len(self.path)-1 and np.linalg.norm(self.path[self.target_idx] - np.array([x,y])) < 0.25:
            self.target_idx += 1
        target = self.path[self.target_idx]
        tw = self.ctrl.compute(pose, target)
        self.robot.apply_twist(tw.v, tw.w, self.cfg.dt)

    def run(self, max_sec: float = 60.0) -> None:
        t0 = time.time()
        # başlangıç haritası ve plan
        for _ in range(10):
            self.sense_and_map(); self.env.step()
        self.plan_once()
        while time.time() - t0 < max_sec:
            self.sense_and_map()
            # periyodik replan (opsiyonel)
            if int((time.time()-t0)*2) % 20 == 0:
                self.plan_once()
            self.follow_path()
            self.env.step()
            # hedefe ulaştı mı?
            x, y, _ = self.robot.pose()
            if np.linalg.norm(np.array([x,y]) - np.array([self.goal.x, self.goal.y])) < 0.5:
                print("Goal reached!")
                break

if __name__ == "__main__":
    Scenario(AppConfig(gui=True)).run()

