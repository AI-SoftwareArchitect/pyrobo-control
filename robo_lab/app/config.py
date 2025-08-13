from __future__ import annotations
from dataclasses import dataclass

dt = 1.0/60.0

@dataclass
class AppConfig:
    gui: bool = True
    dt: float = dt
    map_res: float = 0.1
    map_w: int = 160
    map_h: int = 160