import numpy as np
from .boundary import Boundary
from .pedestrian import Pedestrian, RADIUS


class Crowd:
    def __init__(self, start_line: Boundary, goal_line: Boundary):
        self.start_line = start_line
        self.goal_line = goal_line
        self.pedestrians: list[Pedestrian] = []

    def initialize(self, count: int) -> None:
        self.pedestrians = []
        positions = self._generate_positions(count)
        for pos in positions:
            self.pedestrians.append(Pedestrian(pos, self.goal_line))

    def _generate_positions(self, count: int) -> list[np.ndarray]:
        """開始線のゴール線と逆側に、重ならないよう歩行者を配置"""
        # 開始線のゴール線と逆側へ配置するため、ゴール線の中心が
        # 開始線のどちら側にあるかを判定して逆方向を後方とする
        goal_center = self.goal_line.origin + self.goal_line.direction * (self.goal_line.length / 2)
        goal_side = self.start_line.side_of(goal_center)
        back_normal = -self.start_line.normal * goal_side

        positions = []
        diameter = RADIUS * 2 + 0.3  # 歩行者間の最小間隔

        # 開始線に沿った列と奥行き方向の行で格子配置
        cols = max(1, int(self.start_line.length / diameter))
        rows = (count + cols - 1) // cols

        for i in range(count):
            row = i // cols
            col = i % cols
            along = self.start_line.origin + self.start_line.direction * (
                (col + 0.5) * self.start_line.length / cols
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
