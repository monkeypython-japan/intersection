import numpy as np
from .boundary import Boundary
from .pedestrian import Pedestrian, RADIUS


class Crowd:
    def __init__(self, start_line: Boundary, goal_line: Boundary):
        self.start_line = start_line
        self.goal_line = goal_line
        self.pedestrians: list[Pedestrian] = []

    def initialize(self, count: int, goal_align_weight: float = 2.0) -> None:
        self.pedestrians = []
        # 開始線の中心からみたゴール線の接近側（符号）を求める
        start_center = self.start_line.origin + self.start_line.direction * (self.start_line.length / 2)
        approach_side = self.goal_line.side_of(start_center)
        positions = self._generate_positions(count)
        for pos in positions:
            self.pedestrians.append(
                Pedestrian(pos, self.goal_line, approach_side,
                           goal_align_weight=goal_align_weight)
            )

    def _generate_positions(self, count: int) -> list[np.ndarray]:
        """開始線のゴール線と逆側に、重ならないよう歩行者を配置"""
        # 開始線のゴール線と逆側へ配置するため、ゴール線の中心が
        # 開始線のどちら側にあるかを判定して逆方向を後方とする
        goal_center = self.goal_line.origin + self.goal_line.direction * (self.goal_line.length / 2)
        goal_side = self.start_line.side_of(goal_center)
        back_normal = -self.start_line.normal * goal_side

        positions = []
        diameter = RADIUS * 2 + 0.3  # 歩行者間の最小間隔

        # 開始線に沿った列と奥行き方向の行で格子配置（線の中央に寄せる）
        max_cols = max(1, int(self.start_line.length / diameter))
        cols = min(count, max_cols)
        rows = (count + cols - 1) // cols
        col_spacing = self.start_line.length / max_cols
        start_offset = (self.start_line.length - cols * col_spacing) / 2

        for i in range(count):
            row = i // cols
            col = i % cols
            along = self.start_line.origin + self.start_line.direction * (
                start_offset + (col + 0.5) * col_spacing
            )
            offset = back_normal * (diameter * (row + 1))
            pos = along + offset
            # 小さなランダムオフセットを加える
            pos += np.random.uniform(-0.15, 0.15, size=2)
            positions.append(pos)

        return positions

    def update(self, dt: float, all_pedestrians: list[Pedestrian]) -> None:
        for p in self.pedestrians:
            p.update(dt, all_pedestrians)

    def all_reached(self) -> bool:
        return all(p.reached_goal for p in self.pedestrians)
