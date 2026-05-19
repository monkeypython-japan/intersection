import tkinter as tk
from tkinter import ttk
from models.intersection import Intersection
from ui.canvas import SimCanvas

INTERVAL_MS = 100   # シミュレーション更新間隔 [ms]
DT = 0.1            # 物理ステップ [s]


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Intersection Simulator")
        self.resizable(True, True)
        self.minsize(400, 450)

        self._intersection = Intersection()
        self._running = False
        self._after_id = None
        self._elapsed = 0.0
        self._show_fans = tk.BooleanVar(value=False)

        self._build_controls()
        self._canvas = SimCanvas(self, self._intersection, self._show_fans)
        self._canvas.pack(padx=10, pady=(0, 10), fill="both", expand=True)

    def _build_controls(self) -> None:
        frame = ttk.Frame(self)
        frame.pack(fill="x", padx=10, pady=10)

        ttk.Label(frame, text="歩行者数:").pack(side="left")
        self._ped_count = tk.IntVar(value=10)
        self._slider = ttk.Scale(
            frame, from_=1, to=50, orient="horizontal",
            variable=self._ped_count, length=200,
            command=lambda _: self._count_label.config(
                text=str(self._ped_count.get())
            )
        )
        self._slider.pack(side="left", padx=6)
        self._count_label = ttk.Label(frame, text="10", width=3)
        self._count_label.pack(side="left")

        ttk.Button(frame, text="設定", command=self._on_setup).pack(side="left", padx=(16, 4))
        self._start_btn = ttk.Button(frame, text="開始", command=self._on_start, state="disabled")
        self._start_btn.pack(side="left", padx=(0, 4))
        self._stop_btn = ttk.Button(frame, text="中断", command=self._on_interrupt, state="disabled")
        self._stop_btn.pack(side="left")

        ttk.Checkbutton(
            frame, text="扇形表示", variable=self._show_fans,
            command=lambda: self._canvas.redraw(self._elapsed)
        ).pack(side="left", padx=(12, 0))

        self._status = ttk.Label(frame, text="")
        self._status.pack(side="left", padx=12)

    def _on_setup(self) -> None:
        self._stop()
        self._elapsed = 0.0
        count = self._ped_count.get()
        self._intersection.setup(count)
        self._canvas.redraw(self._elapsed)
        self._start_btn.config(state="normal")
        self._stop_btn.config(state="disabled")
        self._status.config(text="準備完了")

    def _on_start(self) -> None:
        if self._running:
            return
        self._running = True
        self._start_btn.config(state="disabled")
        self._stop_btn.config(state="normal")
        self._slider.config(state="disabled")
        self._status.config(text="実行中...")
        self._loop()

    def _on_interrupt(self) -> None:
        self._stop()
        self._start_btn.config(state="normal")
        self._status.config(text="中断")

    def _loop(self) -> None:
        if not self._running:
            return
        self._intersection.step(DT)
        self._elapsed += DT
        self._canvas.redraw(self._elapsed)
        if self._intersection.is_finished():
            self._running = False
            self._stop_btn.config(state="disabled")
            self._status.config(text="完了")
            self._slider.config(state="normal")
            return
        self._after_id = self.after(INTERVAL_MS, self._loop)

    def _stop(self) -> None:
        self._running = False
        if self._after_id:
            self.after_cancel(self._after_id)
            self._after_id = None
        self._stop_btn.config(state="disabled")
        self._slider.config(state="normal")
