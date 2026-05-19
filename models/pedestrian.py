import numpy as np
from .boundary import Boundary

RADIUS = 0.5            # 歩行者の占有半径 [m]
BASE_SPEED = 1.0        # 基本速度 [m/s]
SPEED_VARIATION = 0.2   # 速度ばらつき割合
SCAN_ANGLE = np.radians(45)   # 前方走査扇形の半角 [rad]
SECTOR_COUNT = 3
DECEL_DISTANCE = 2.0    # 減速を開始する距離 [m]
ACCEL_RATE = 0.1        # 回復加速度 [m/s²]
MAX_TURN = np.radians(30)       # 1ステップあたりの最大回転角 [rad]
ESCAPE_ANGLE = np.radians(100)  # 脱出回避角（直角 + 10° 後退）
ESCAPE_SPEED_RATIO = 0.5        # 脱出速度割合
STUCK_SPEED = 0.05              # これ以下の速度を「停止」とみなす [m/s]
STUCK_ESCAPE_STEPS = 5          # 停止がこのステップ数続いたら脱出（0.5秒）
ENDPOINT_INNER_MARGIN = 1.0     # 延長ゾーンでセグメント内側を目指すマージン [m]
PLAN_SECTORS = 7                # 方向計画のセクター数
PLAN_HALF_ANGLE = np.radians(60)  # ゴール方向基準で ±60° をスキャン [rad]
PLAN_DIST = 5.0                 # 方向計画の先読み距離 [m]
GOAL_ALIGN_WEIGHT = 2.0         # ゴール整合性スコアの重み（混雑ペナルティとのバランス）


def _rotate(vec: np.ndarray, angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]])


class Pedestrian:
    def __init__(self, position: np.ndarray, goal_line: Boundary,
                 goal_approach_side: int):
        self.position = np.array(position, dtype=float)
        self.goal_line = goal_line
        self.radius = RADIUS
        self.base_speed = BASE_SPEED * (1.0 + np.random.uniform(-SPEED_VARIATION, SPEED_VARIATION))
        self.reached_goal = False
        self._prev_position = self.position.copy()
        self._stuck_steps = 0
        self._goal_approach_side = goal_approach_side  # ゴール線の正規接近側の符号

        self.velocity = self._goal_dir_with_margin() * self.base_speed

    def update(self, dt: float, neighbors: list) -> None:
        if self.reached_goal:
            return

        self._prev_position = self.position.copy()

        # --- 端点への近接判定（最優先）---
        # 端点は境界線の一部。ゴール線面上または正規接近側から RADIUS 以内に到達 → ゴール
        for ep in (self.goal_line.origin, self.goal_line.end_point):
            signed_d = self.goal_line.signed_distance(self.position)
            if (np.linalg.norm(self.position - ep) <= self.radius and
                    signed_d >= -self.radius):
                self.reached_goal = True
                self.velocity = np.zeros(2)
                return

        # --- 安全策: すでにゴール線の背後にいる場合は最近傍端点へ誘導 ---
        if self.goal_line.side_of(self.position) != self._goal_approach_side:
            nearest = self.goal_line.nearest_endpoint(self.position)
            ep_dir = nearest - self.position
            ep_dist = np.linalg.norm(ep_dir)
            if ep_dist > 1e-6:
                self.velocity = (ep_dir / ep_dist) * self.base_speed * 0.5
            self.position = self.position + self.velocity * dt
            return

        nearby = [p for p in neighbors if p is not self and not p.reached_goal]
        self._adjust_velocity(nearby, dt)

        if np.linalg.norm(self.velocity) < STUCK_SPEED:
            self._stuck_steps += 1
        else:
            self._stuck_steps = 0

        new_pos = self.position + self.velocity * dt

        # --- ゴール線を正規方向（開始線側）から横断 → ゴール達成 ---
        if (self.goal_line.crossed_by(self._prev_position, new_pos) and
                self.goal_line.side_of(self._prev_position) == self._goal_approach_side):
            self.reached_goal = True
            self.velocity = np.zeros(2)
            self.position = new_pos
            return

        # --- ゴール線の延長を横断しようとした → 壁として扱いセグメント内側へ向かう ---
        if self.goal_line.side_of(new_pos) != self._goal_approach_side:
            signed_d = self.goal_line.signed_distance(new_pos)
            new_pos = new_pos - self.goal_line.normal * signed_d
            spd = np.linalg.norm(self.velocity)
            if spd > 1e-6:
                self.velocity = self._goal_dir_with_margin() * spd

        self.position = new_pos

    def _adjust_velocity(self, nearby: list, dt: float) -> None:
        speed = np.linalg.norm(self.velocity)
        if speed < 1e-6:
            direction = self.goal_line.goal_direction(self.position)
        else:
            direction = self.velocity / speed

        # 一定時間停止が続いた場合: ゴール方向を基準に垂直回避
        if self._stuck_steps >= STUCK_ESCAPE_STEPS:
            goal_dir = self._goal_dir_with_margin()
            blockers = [
                p for p in nearby
                if (np.linalg.norm(p.position - self.position) - self.radius - p.radius)
                   < DECEL_DISTANCE
                and np.dot(
                    (p.position - self.position)
                    / (np.linalg.norm(p.position - self.position) + 1e-6),
                    goal_dir
                ) > 0.3
            ]
            if blockers:
                escape_dir = self._perpendicular_escape(goal_dir, blockers)
                self.velocity = escape_dir * self.base_speed * ESCAPE_SPEED_RATIO
                self._stuck_steps = 0
                return

        # 前方の歩行者分布から最も空いている方向を選択
        turn_angle = self._plan_heading(direction, nearby)
        new_dir = _rotate(direction, turn_angle)

        # 速度調整（方向修正と同時適用）
        target_speed = self._collision_speed(nearby, new_dir)

        if target_speed < 1e-6:
            # 実際の重なり → 停止（stuck カウンターが進む）
            self.velocity = np.zeros(2)
        else:
            # 加速は緩やか、減速は即時
            if target_speed > speed:
                new_speed = min(speed + ACCEL_RATE * dt, target_speed)
            else:
                new_speed = target_speed
            goal_dir = self._goal_dir_with_margin()
            blended = new_dir * 0.9 + goal_dir * 0.1
            norm = np.linalg.norm(blended)
            new_dir = blended / norm if norm > 1e-6 else new_dir
            self.velocity = new_dir * new_speed

    def _plan_heading(self, forward: np.ndarray, nearby: list) -> float:
        """ゴール方向を中心に ±PLAN_HALF_ANGLE をスキャン。
        スコア = ゴール整合性（cos）- 混雑ペナルティ で最良セクターを選び、現在方向からのターン角を返す。"""
        goal_dir = self._goal_dir_with_margin()
        goal_angle = np.arctan2(goal_dir[1], goal_dir[0])
        fwd_angle  = np.arctan2(forward[1],  forward[0])

        sector_width   = 2 * PLAN_HALF_ANGLE / PLAN_SECTORS
        sector_offsets = [-PLAN_HALF_ANGLE + (i + 0.5) * sector_width
                          for i in range(PLAN_SECTORS)]

        # ゴール方向(offset=0)が最高、±60°端が最低のベーススコア
        scores = np.array([GOAL_ALIGN_WEIGHT * np.cos(c) for c in sector_offsets])

        for p in nearby:
            diff = p.position - self.position
            dist = np.linalg.norm(diff)
            if dist < 1e-6 or dist > PLAN_DIST:
                continue
            angle_to = np.arctan2(diff[1], diff[0]) - goal_angle
            angle_to = (angle_to + np.pi) % (2 * np.pi) - np.pi
            if abs(angle_to) > PLAN_HALF_ANGLE:
                continue
            p_vel_norm = p.velocity / (np.linalg.norm(p.velocity) + 1e-6)
            dot = float(np.dot(forward, p_vel_norm))
            w = (1.0 / (dist + 1e-3)) * (1.0 - dot * 0.5)
            for i, c in enumerate(sector_offsets):
                if abs(angle_to - c) <= sector_width / 2:
                    scores[i] -= w
                    break

        best = int(np.argmax(scores))
        target_angle = goal_angle + sector_offsets[best]
        turn = target_angle - fwd_angle
        turn = (turn + np.pi) % (2 * np.pi) - np.pi
        return float(np.clip(turn, -MAX_TURN, MAX_TURN))

    def _goal_dir_with_margin(self) -> np.ndarray:
        """延長ゾーンにいる場合は端点の ENDPOINT_INNER_MARGIN m 内側を目指す。
        セグメント内では通常の goal_direction と同じ。"""
        t = np.dot(self.position - self.goal_line.origin, self.goal_line.direction)
        if t < 0:
            target = self.goal_line.origin + self.goal_line.direction * ENDPOINT_INNER_MARGIN
        elif t > self.goal_line.length:
            target = self.goal_line.end_point - self.goal_line.direction * ENDPOINT_INNER_MARGIN
        else:
            return self.goal_line.goal_direction(self.position)
        vec = target - self.position
        dist = np.linalg.norm(vec)
        return vec / dist if dist > 1e-6 else self.goal_line.normal

    def _perpendicular_escape(self, forward: np.ndarray, blockers: list) -> np.ndarray:
        """前進方向の直角＋やや後退のうち、相手速度との内積が小さい側へ逃げる"""
        left = _rotate(forward, ESCAPE_ANGLE)
        right = _rotate(forward, -ESCAPE_ANGLE)

        def dot_score(d: np.ndarray) -> float:
            return sum(
                float(np.dot(d, p.velocity / (np.linalg.norm(p.velocity) + 1e-6)))
                for p in blockers
            )

        return left if dot_score(left) < dot_score(right) else right

    def _collision_speed(self, nearby: list, direction: np.ndarray) -> float:
        """前方の最近接距離に基づく目標速度を返す。
        base_speed 基準にすることで compound deceleration（速度が 0 に収束する問題）を防ぐ。
        実際の重なり（min_dist < 0）のみ 0 を返す。"""
        min_dist = float('inf')
        for p in nearby:
            diff = p.position - self.position
            dist_len = np.linalg.norm(diff)
            if dist_len < 1e-6:
                continue
            # 前方 45° 以内のみ対象
            if np.dot(diff / dist_len, direction) <= np.cos(SCAN_ANGLE):
                continue
            dist = dist_len - self.radius - p.radius
            if dist < min_dist:
                min_dist = dist

        if min_dist < 0:
            return 0.0                           # 実接触 → 停止
        if min_dist < self.radius * 2:
            return self.base_speed * 0.3         # 近接 → 30% に抑える
        if min_dist < DECEL_DISTANCE:
            ratio = min_dist / DECEL_DISTANCE
            return self.base_speed * (0.3 + 0.7 * ratio)

        return self.base_speed
