"""
real_time_plotter_pg.py
-----------------------

GPU-accelerated real-time dashboard for an inverted-pendulum simulation,
implemented with PyQtGraph.  Public API matches the original Matplotlib
class:

    rtp = RealTimePlotter(buffer_size=1000, update_frequency=5)
    ...
    rtp.update_data(t, x_cur, x_des, angle_deg, force)
    ...
    rtp.close()
"""

from __future__ import annotations
import sys
from collections import deque
from typing import Deque

import numpy as np

# ----------------------------------------------------------------------
#  Qt binding: try PyQt6 first, fall back to PySide6
# ----------------------------------------------------------------------
try:
    from PyQt6 import QtWidgets, QtCore
except ModuleNotFoundError:
    from PySide6 import QtWidgets, QtCore          # type: ignore

import pyqtgraph as pg

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')  

class RealTimePlotter:
    """
    Real-time inverted-pendulum visualiser using PyQtGraph.

    Parameters
    ----------
    buffer_size : int
        Maximum number of samples kept in the sliding window.
    update_frequency : int
        Redraw the dashboard every N calls to `update_data`.
    """

    # ----------------------------- init --------------------------------
    def __init__(self, buffer_size: int = 1000, update_frequency: int = 5):
        self.buffer_size: int      = buffer_size
        self.update_frequency: int = update_frequency
        self._update_counter: int  = 0

        # --- fixed-size FIFO buffers -----------------------------------
        self.t_buf:         Deque[float] = deque(maxlen=buffer_size)
        self.x_current_buf: Deque[float] = deque(maxlen=buffer_size)
        self.x_desired_buf: Deque[float] = deque(maxlen=buffer_size)
        self.angle_buf:     Deque[float] = deque(maxlen=buffer_size)
        self.force_buf:     Deque[float] = deque(maxlen=buffer_size)

        # --- Qt application --------------------------------------------
        self._app = (QtWidgets.QApplication.instance()
                     or QtWidgets.QApplication(sys.argv))
        pg.setConfigOptions(antialias=True)        # nicer lines

        # --- build 2 × 2 dashboard layout ------------------------------
        self.win = pg.GraphicsLayoutWidget(title="Real-Time Inverted Pendulum")
        self.win.resize(1200, 800)

        # 1) Position vs Time
        self.pos_plot = self.win.addPlot(title="Position vs Time")
        self.pos_plot.setLabel('bottom', 'Time', units='s')
        self.pos_plot.setLabel('left',   'Position', units='m')
        self.pos_plot.addLegend()
        self.curve_x_cur = self.pos_plot.plot(pen='b', name="x_current")
        self.curve_x_des = self.pos_plot.plot(
            pen=pg.mkPen('g', style=QtCore.Qt.PenStyle.DashLine),
            name="x_desired"
        )

        # 2) Angle vs Time
        self.win.nextColumn()
        self.angle_plot = self.win.addPlot(title="Angle vs Time")
        self.angle_plot.setLabel('bottom', 'Time', units='s')
        self.angle_plot.setLabel('left',   'Angle', units='deg')
        self.curve_angle = self.angle_plot.plot(pen='r')

        # 3) Force vs Time
        self.win.nextRow()
        self.force_plot = self.win.addPlot(title="Force vs Time")
        self.force_plot.setLabel('bottom', 'Time', units='s')
        self.force_plot.setLabel('left',   'Force')
        self.curve_force = self.force_plot.plot(pen='m')

        # 4) Pendulum animation
        self.anim_plot = self.win.addPlot(title="Inverted Pendulum")
        self.anim_plot.setLabel('bottom', 'Cart position', units='m')
        self.anim_plot.setYRange(-0.6, 0.6)
        self.anim_plot.disableAutoRange(axis=pg.ViewBox.YAxis)
        self.anim_cart = self.anim_plot.plot(
            pen=None, symbol='s', symbolBrush='k', symbolSize=14
        )
        self.anim_rod = self.anim_plot.plot(pen=pg.mkPen('r', width=3))

        # show non-blocking so the simulation keeps running
        self.win.show()
        self._app.processEvents()

    # ------------------------------------------------------------------
    #  Public API
    # ------------------------------------------------------------------
    def update_data(self,
                    t: float,
                    x_current: float,
                    x_desired: float,
                    angle_deg: float,
                    force: float) -> None:
        """Push a new sample; refresh every `update_frequency` calls."""
        self.t_buf.append(t)
        self.x_current_buf.append(x_current)
        self.x_desired_buf.append(x_desired)
        self.angle_buf.append(angle_deg)
        self.force_buf.append(force)

        self._update_counter += 1
        if self._update_counter >= self.update_frequency:
            self._update_counter = 0
            self._redraw()

    def close(self) -> None:
        """Cleanly close the dashboard window."""
        self.win.close()

    # ------------------------------------------------------------------
    #  Internal helpers
    # ------------------------------------------------------------------
    def _deque_to_ndarray(self, dq: Deque[float]) -> np.ndarray:
        """Convert deque → NumPy array (fast enough for ~1 k points)."""
        # np.fromiter avoids the intermediate list
        return np.fromiter(dq, dtype=float, count=len(dq))

    def _redraw(self) -> None:
        # convert the buffers
        t_arr     = self._deque_to_ndarray(self.t_buf)
        x_cur_arr = self._deque_to_ndarray(self.x_current_buf)
        x_des_arr = self._deque_to_ndarray(self.x_desired_buf)
        angle_arr = self._deque_to_ndarray(self.angle_buf)
        force_arr = self._deque_to_ndarray(self.force_buf)

        # update time-series curves
        self.curve_x_cur.setData(t_arr, x_cur_arr)
        self.curve_x_des.setData(t_arr, x_des_arr)
        self.curve_angle.setData(t_arr, angle_arr)
        self.curve_force.setData(t_arr, force_arr)

        # update pendulum animation (last sample only)
        if x_cur_arr.size:
            x = x_cur_arr[-1]
            theta = np.deg2rad(angle_arr[-1])
            L = 0.5
            self.anim_cart.setData([x], [0])
            self.anim_rod.setData(
                [x, x + np.sin(theta) * L],
                [0, np.cos(theta) * L]
            )

            # auto-expand X so cart never leaves view
            xmin, xmax = self.anim_plot.viewRange()[0]
            if x < xmin + 0.1 or x > xmax - 0.1:
                self.anim_plot.setXRange(x - 1, x + 1, padding=0)

        # let Qt paint
        self._app.processEvents()


# ----------------------------------------------------------------------
#  Example usage
# ----------------------------------------------------------------------
if __name__ == "__main__":
    import math
    import time
    rtp = RealTimePlotter(buffer_size=2000, update_frequency=1)

    t = 0.0
    dt = 0.005
    try:
        while t < 20:                          # 20-second demo
            # fake signals
            x_cur = 0.8 * math.sin(0.5 * t)
            x_des = 0.8 * math.sin(0.5 * (t - 0.2))
            ang   = 10 * math.sin(1.3 * t)
            force = 5 * math.sin(0.7 * t)

            rtp.update_data(t, x_cur, x_des, ang, force)
            t += dt
            time.sleep(dt)                     # mimic simulation step
    finally:
        rtp.close()
