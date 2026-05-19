import numpy as np
from dataclasses import dataclass, field


@dataclass
class Boundary:
    origin: np.ndarray      # 始点 (x, y) [m]
    direction: np.ndarray   # 単位方向ベクトル
    length: float           # 境界線の長さ [m]

    def __post_init__(self):
        self.origin = np.array(self.origin, dtype=float)
        self.direction = np.array(self.direction, dtype=float)
        norm = np.linalg.norm(self.direction)
        if norm > 0:
            self.direction = self.direction / norm

    @property
    def normal(self) -> np.ndarray:
        """境界線の法線ベクトル（左向き）"""
        return np.array([-self.direction[1], self.direction[0]])

    @property
    def end_point(self) -> np.ndarray:
        return self.origin + self.direction * self.length

    def signed_distance(self, point: np.ndarray) -> float:
        """点から境界線への符号付き距離（法線方向を正）"""
        return float(np.dot(point - self.origin, self.normal))

    def crossed_by(self, prev_pos: np.ndarray, curr_pos: np.ndarray) -> bool:
        """歩行者が前ステップから今ステップで境界線を通過したか判定"""
        prev_signed = self.signed_distance(prev_pos)
        curr_signed = self.signed_distance(curr_pos)
        if prev_signed * curr_signed > 0:
            return False  # 同じ側にいる

        # 境界線セグメント上の投影点が範囲内かチェック
        # 線分の交点パラメータを求める
        d = curr_pos - prev_pos
        denom = np.dot(d, self.normal)
        if abs(denom) < 1e-9:
            return False
        t = -prev_signed / denom
        intersect = prev_pos + t * d
        proj = np.dot(intersect - self.origin, self.direction)
        return 0.0 <= proj <= self.length

    def side_of(self, point: np.ndarray) -> int:
        """境界線のどちら側にいるか（+1 or -1）"""
        d = self.signed_distance(point)
        return 1 if d >= 0 else -1

    def closest_point_on_line(self, point: np.ndarray) -> np.ndarray:
        """境界線セグメント上の最近傍点"""
        t = np.dot(point - self.origin, self.direction)
        t = np.clip(t, 0.0, self.length)
        return self.origin + self.direction * t

    def goal_direction(self, position: np.ndarray) -> np.ndarray:
        """境界線へ向かう最短方向の単位ベクトル"""
        closest = self.closest_point_on_line(position)
        vec = closest - position
        dist = np.linalg.norm(vec)
        if dist < 1e-6:
            return self.normal
        return vec / dist
