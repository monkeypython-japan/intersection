import numpy as np
from .boundary import Boundary
from .crowd import Crowd
from .pedestrian import Pedestrian

DEFAULT_BOUNDARIES = [
    {"origin": [10, 10], "direction": [0, 1], "length": 20},
    {"origin": [40, 10], "direction": [0, 1], "length": 20},
]


class Intersection:
    WIDTH = 50.0
    HEIGHT = 50.0

    def __init__(self):
        self.boundaries: list[Boundary] = []
        self.crowds: list[Crowd] = []
        self._setup_done = False

    def setup(self, pedestrian_count: int) -> None:
        self.boundaries = [
            Boundary(**b) for b in DEFAULT_BOUNDARIES
        ]
        b1, b2 = self.boundaries
        crowd1 = Crowd(start_line=b1, goal_line=b2)
        crowd2 = Crowd(start_line=b2, goal_line=b1)
        crowd1.initialize(pedestrian_count)
        crowd2.initialize(pedestrian_count)
        self.crowds = [crowd1, crowd2]
        self._setup_done = True

    def step(self, dt: float = 0.1) -> None:
        if not self._setup_done:
            return
        all_pedestrians = self.all_pedestrians()
        for crowd in self.crowds:
            crowd.update(dt, all_pedestrians)

    def all_pedestrians(self) -> list[Pedestrian]:
        result = []
        for crowd in self.crowds:
            result.extend(crowd.pedestrians)
        return result

    def is_finished(self) -> bool:
        if not self._setup_done or not self.crowds:
            return False
        return all(c.all_reached() for c in self.crowds)
