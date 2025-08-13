from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Dict, List, Any

@dataclass
class Event:
    name: str
    payload: dict

class EventBus:
    """Thread-safe (coarse) pub/sub for sensors -> app."""
    def __init__(self) -> None:
        from threading import Lock
        self._subs: Dict[str, List[Callable[[Event], None]]] = {}
        self._lock = Lock()

    def subscribe(self, name: str, fn: Callable[[Event], None]) -> None:
        with self._lock:
            self._subs.setdefault(name, []).append(fn)

    def publish(self, event: Event) -> None:
        with self._lock:
            subs = list(self._subs.get(event.name, []))
        for fn in subs:
            fn(event)