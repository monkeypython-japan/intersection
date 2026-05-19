import tkinter as tk
import math
import numpy as np
from models.intersection import Intersection
from models.pedestrian import DECEL_DISTANCE, STUCK_SPEED, SCAN_ANGLE, SECTOR_COUNT

MARGIN = 20         # キャンバス余白 [px]
PEDESTRIAN_R = 5    # 歩行者の描画半径 [px] (初期値、スケールに応じて変化)
DEBOUNCE_MS = 50    # リサイズデバウンス [ms]

CROWD_COLORS = ["mediumseagreen", "royalblue", "hotpink", "darkorange"]

# セクター色 (右・中央・左)
SECTOR_COLORS = ["#C8E6C9", "#B3E5FC", "#F8BBD0"]


class SimCanvas(tk.Canvas):
    def __init__(self, parent, intersection: Intersection, show_fans: tk.BooleanVar, **kwargs):
        init_size = int(Intersection.WIDTH * 10 + MARGIN * 2)
        super().__init__(parent, width=init_size, height=init_size, bg="white", **kwargs)
        self._intersection = intersection
        self._show_fans = show_fans
        self._height = init_size
        self._width = init_size
        self._scale = 10.0
        self._elapsed = 0.0
        self._resize_after = None

        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, event) -> None:
        if self._resize_after is not None:
            self.after_cancel(self._resize_after)
        self._resize_after = self.after(DEBOUNCE_MS, lambda: self._apply_resize(event.width, event.height))

    def _apply_resize(self, w: int, h: int) -> None:
        self._resize_after = None
        size = min(w, h)
        self._scale = (size - 2 * MARGIN) / Intersection.WIDTH
        self._width = w
        self._height = h
        self.redraw(self._elapsed)

    def _m2px(self, x: float, y: float) -> tuple[float, float]:
        px = MARGIN + x * self._scale
        py = self._height - MARGIN - y * self._scale
        return px, py

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

        self.create_text(
            MARGIN, MARGIN // 2,
            text=f"{reached} / {total}",
            anchor="nw", font=("Helvetica", 13, "bold"), fill="gray20"
        )
        self.create_text(
            self._width - MARGIN, MARGIN // 2,
            text=f"{elapsed:.1f} s",
            anchor="ne", font=("Helvetica", 13, "bold"), fill="gray20"
        )

    def _draw_boundaries(self) -> None:
        for b in self._intersection.boundaries:
            x1, y1 = self._m2px(b.origin[0], b.origin[1])
            ep = b.end_point
            x2, y2 = self._m2px(ep[0], ep[1])
            self.create_line(x1, y1, x2, y2, fill="royalblue", width=3)

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
                # tkinter arc角度: X軸右が0°、反時計回り正。Y軸は画面下方向なので反転済
                fwd_angle = math.degrees(math.atan2(-vy, vx))  # Y軸反転

                # 右(i=0)・中(i=1)・左(i=2): tkinter反時計回り正、Y軸反転済み
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
                    ay = cy - d[1] * arrow_px  # Y軸反転
                    self.create_line(cx, cy, ax, ay, fill="black",
                                     arrow=tk.LAST, arrowshape=(6, 8, 3))
