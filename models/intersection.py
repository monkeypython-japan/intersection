import numpy as np
from .boundary import Boundary
from .crowd import Crowd
from .pedestrian import Pedestrian

# ペア間距離 30m、フィールド中央 (25, 25) を基準とした固定 X/Y 座標
_H_X_LEFT, _H_X_RIGHT = 10.0, 40.0   # 横軸ペアの X 位置
_V_Y_BOTTOM, _V_Y_TOP  = 10.0, 40.0  # 縦軸ペアの Y 位置
_CENTER = 25.0


def _make_boundaries(length: float, scramble: bool) -> list[dict]:
    half = length / 2
    h_left   = {"origin": [_H_X_LEFT,  _CENTER - half], "direction": [0, 1], "length": length}
    h_right  = {"origin": [_H_X_RIGHT, _CENTER - half], "direction": [0, 1], "length": length}
    if not scramble:
        return [h_left, h_right]
    v_bottom = {"origin": [_CENTER - half, _V_Y_BOTTOM], "direction": [1, 0], "length": length}
    v_top    = {"origin": [_CENTER - half, _V_Y_TOP],    "direction": [1, 0], "length": length}
    return [h_left, h_right, v_bottom, v_top]


class Intersection:
    WIDTH = 50.0
    HEIGHT = 50.0

    def __init__(self):
        self.boundaries: list[Boundary] = []
        self.crowds: list[Crowd] = []
        self._setup_done = False

    def setup(self, pedestrian_count: int, scramble: bool = False,
              line_length: float = 20.0) -> None:
        boundary_defs = _make_boundaries(line_length, scramble)
        self.boundaries = [Boundary(**b) for b in boundary_defs]

        b_h_left, b_h_right = self.boundaries[0], self.boundaries[1]
        crowd1 = Crowd(start_line=b_h_left,  goal_line=b_h_right)
        crowd2 = Crowd(start_line=b_h_right, goal_line=b_h_left)
        crowd1.initialize(pedestrian_count)
        crowd2.initialize(pedestrian_count)
        self.crowds = [crowd1, crowd2]

        if scramble:
            b_v_bottom, b_v_top = self.boundaries[2], self.boundaries[3]
            crowd3 = Crowd(start_line=b_v_bottom, goal_line=b_v_top)
            crowd4 = Crowd(start_line=b_v_top,    goal_line=b_v_bottom)
            crowd3.initialize(pedestrian_count)
            crowd4.initialize(pedestrian_count)
            self.crowds.extend([crowd3, crowd4])

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
