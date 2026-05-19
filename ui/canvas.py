import tkinter as tk
import math
import numpy as np
from models.intersection import Intersection
from models.pedestrian import DECEL_DISTANCE, STUCK_SPEED, SCAN_ANGLE, SECTOR_COUNT

MARGIN = 20         # キャンバス余白 [px]
DEBOUNCE_MS = 50    # リサイズデバウンス [ms]

CROWD_COLORS = ["mediumseagreen", "royalblue", "hotpink", "darkorange"]
SECTOR_COLORS = ["#C8E6C9", "#B3E5FC", "#F8BBD0"]   # 右・中央・左


class SimCanvas(tk.Canvas):
    def __init__(self, parent, intersection: Intersection, show_fans: tk.BooleanVar, **kwargs):
        init_size = int(Intersection.WIDTH * 10 + MARGIN * 2)
        super().__init__(parent, width=init_size, height=init_size, bg="white", **kwargs)
        self._intersection = intersection
        self._show_fans = show_fans
        self._scale = 10.0
        self._sim_size = init_size   # シミュレーション正方形のピクセルサイズ
        self._ox = 0                 # 正方形の左上 X オフセット
        self._oy = 0                 # 正方形の左上 Y オフセット
        self._width = init_size
        self._height = init_size
        self._elapsed = 0.0
        self._time_red = False       # 経過時間テキストを赤表示するフラグ
        self._resize_after = None

        self.bind("<Configure>", self._on_resize)

    # ------------------------------------------------------------------ resize
    def _on_resize(self, event) -> None:
        if self._resize_after is not None:
            self.after_cancel(self._resize_after)
        self._resize_after = self.after(
            DEBOUNCE_MS, lambda: self._apply_resize(event.width, event.height)
        )

    def _apply_resize(self, w: int, h: int) -> None:
        self._resize_after = None
        size = min(w, h)
        self._scale = (size - 2 * MARGIN) / Intersection.WIDTH
        self._sim_size = size
        self._ox = (w - size) // 2   # キャンバス内で正方形を中央寄せ
        self._oy = (h - size) // 2
        self._width = w
        self._height = h
        self.redraw(self._elapsed)

    # ---------------------------------------------------------------- coord
    def _m2px(self, x: float, y: float) -> tuple[float, float]:
        px = self._ox + MARGIN + x * self._scale
        py = self._oy + self._sim_size - MARGIN - y * self._scale
        return px, py

    # ---------------------------------------------------------------- drawing
    def redraw(self, elapsed: float = 0.0) -> None:
        self._elapsed = elapsed
        self.delete("all")
        self._draw_fans()
        self._draw_boundaries()
        self._draw_pedestrians()
        self._draw_overlay(elapsed)

    def _draw_overlay(self, elapsed: float) -> None:
        peds = self._intersection.all_pedestrians()
        total = len(peds)
        reached = sum(1 for p in peds if p.reached_goal)

        left_x  = self._ox + MARGIN
        right_x = self._ox + self._sim_size - MARGIN
        top_y   = self._oy + MARGIN // 2

        self.create_text(
            left_x, top_y,
            text=f"{reached} / {total}",
            anchor="nw", font=("Helvetica", 13, "bold"), fill="gray20"
        )
        self.create_text(
            right_x, top_y,
            text=f"{elapsed:.1f} s",
            anchor="ne", font=("Helvetica", 13, "bold"),
            fill="red" if self._time_red else "gray20"
        )

    def _boundary_color_map(self) -> dict[int, str]:
        """各境界線を目指す群衆の色を返す"""
        color_map = {}
        for ci, crowd in enumerate(self._intersection.crowds):
            color = CROWD_COLORS[ci % len(CROWD_COLORS)]
            for bi, b in enumerate(self._intersection.boundaries):
                if crowd.goal_line is b:
                    color_map[bi] = color
        return color_map

    def _draw_boundaries(self) -> None:
        color_map = self._boundary_color_map()
        line_width = max(2, int(self._scale * 0.3))
        for bi, b in enumerate(self._intersection.boundaries):
            x1, y1 = self._m2px(b.origin[0], b.origin[1])
            ep = b.end_point
            x2, y2 = self._m2px(ep[0], ep[1])
            color = color_map.get(bi, "gray50")
            self.create_line(x1, y1, x2, y2, fill=color, width=line_width)

    def _draw_fans(self) -> None:
        if not self._show_fans.get():
            return
        fan_radius_px = DECEL_DISTANCE * 2 * self._scale
        sector_half_deg = math.degrees(SCAN_ANGLE / SECTOR_COUNT)

        for crowd in self._intersection.crowds:
            for p in crowd.pedestrians:
                if p.reached_goal:
                    continue
                speed = np.linalg.norm(p.velocity)
                if speed < 1e-6:
                    continue

                cx, cy = self._m2px(p.position[0], p.position[1])
                vx, vy = p.velocity
                fwd_angle = math.degrees(math.atan2(vy, vx))

                for i in range(SECTOR_COUNT):
                    start_angle = fwd_angle - 45 + i * sector_half_deg * 2
                    self.create_arc(
                        cx - fan_radius_px, cy - fan_radius_px,
                        cx + fan_radius_px, cy + fan_radius_px,
                        start=start_angle,
                        extent=sector_half_deg * 2,
                        style=tk.PIESLICE,
                        fill=SECTOR_COLORS[i],
                        outline="",
                        stipple="gray25"
                    )

    def _draw_pedestrians(self) -> None:
        ped_r = max(3, int(self._scale * 0.5))
        arrow_shape = (
            max(4, int(self._scale * 0.6)),
            max(5, int(self._scale * 0.8)),
            max(2, int(self._scale * 0.3)),
        )
        for ci, crowd in enumerate(self._intersection.crowds):
            color = CROWD_COLORS[ci % len(CROWD_COLORS)]
            for p in crowd.pedestrians:
                cx, cy = self._m2px(p.position[0], p.position[1])
                self.create_oval(cx - ped_r, cy - ped_r, cx + ped_r, cy + ped_r,
                                 fill=color, outline="")
                speed = np.linalg.norm(p.velocity)
                if speed > STUCK_SPEED:
                    d = p.velocity / speed
                    arrow_px = speed * 1.5 * self._scale
                    ax = cx + d[0] * arrow_px
                    ay = cy - d[1] * arrow_px
                    self.create_line(cx, cy, ax, ay, fill="black",
                                     arrow=tk.LAST, arrowshape=arrow_shape)
