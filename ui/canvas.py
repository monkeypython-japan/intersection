import tkinter as tk
import numpy as np
from models.intersection import Intersection

SCALE = 10          # 1m = 10px
MARGIN = 20         # キャンバス余白 [px]
PEDESTRIAN_R = 5    # 歩行者の描画半径 [px]
ARROW_LEN = 12      # 速度矢印の長さ [px]


def _m2px(x: float, y: float, canvas_height: int) -> tuple[float, float]:
    """モデル座標(m) → キャンバス座標(px) 変換（Y軸反転）"""
    px = MARGIN + x * SCALE
    py = canvas_height - MARGIN - y * SCALE
    return px, py


class SimCanvas(tk.Canvas):
    def __init__(self, parent, intersection: Intersection, **kwargs):
        width = int(Intersection.WIDTH * SCALE + MARGIN * 2)
        height = int(Intersection.HEIGHT * SCALE + MARGIN * 2)
        super().__init__(parent, width=width, height=height, bg="white", **kwargs)
        self._intersection = intersection
        self._height = height
        self._width = width

    def redraw(self, elapsed: float = 0.0) -> None:
        self.delete("all")
        self._draw_boundaries()
        self._draw_pedestrians()
        self._draw_overlay(elapsed)

    def _draw_overlay(self, elapsed: float) -> None:
        peds = self._intersection.all_pedestrians()
        total = len(peds)
        reached = sum(1 for p in peds if p.reached_goal)

        # 左上：到達数 / 合計
        self.create_text(
            MARGIN, MARGIN // 2,
            text=f"{reached} / {total}",
            anchor="nw", font=("Helvetica", 13, "bold"), fill="gray20"
        )
        # 右上：経過時間
        self.create_text(
            self._width - MARGIN, MARGIN // 2,
            text=f"{elapsed:.1f} s",
            anchor="ne", font=("Helvetica", 13, "bold"), fill="gray20"
        )

    def _draw_boundaries(self) -> None:
        for b in self._intersection.boundaries:
            x1, y1 = _m2px(b.origin[0], b.origin[1], self._height)
            ep = b.end_point
            x2, y2 = _m2px(ep[0], ep[1], self._height)
            self.create_line(x1, y1, x2, y2, fill="royalblue", width=3)

    def _draw_pedestrians(self) -> None:
        colors = ["tomato", "mediumseagreen"]
        for ci, crowd in enumerate(self._intersection.crowds):
            color = colors[ci % len(colors)]
            for p in crowd.pedestrians:
                cx, cy = _m2px(p.position[0], p.position[1], self._height)
                r = PEDESTRIAN_R
                self.create_oval(cx - r, cy - r, cx + r, cy + r,
                                 fill=color, outline="")
                # 速度矢印
                speed = np.linalg.norm(p.velocity)
                if speed > 1e-6:
                    d = p.velocity / speed
                    ax = cx + d[0] * ARROW_LEN
                    ay = cy - d[1] * ARROW_LEN  # Y軸反転
                    self.create_line(cx, cy, ax, ay, fill="black",
                                     arrow=tk.LAST, arrowshape=(6, 8, 3))
