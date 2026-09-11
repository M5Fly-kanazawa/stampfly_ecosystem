"""
StampFly Education Helper Library
StampFly 教育用ヘルパーライブラリ

Provides utilities for the StampFly control engineering curriculum:
制御工学カリキュラム用のユーティリティを提供:

- connect_or_simulate(): Connect to drone or fall back to simulator
- load_flight_log(): Load a flight-log v1 bundle (`.sflog.zip`/directory,
  see lib/sflog) or a plain CSV as a pandas DataFrame
- plot_trajectory(): Plot XY flight trajectory
- compare_logs(): Overlay two flight logs for comparison

Usage:
    from stampfly_edu import connect_or_simulate, load_flight_log, plot_trajectory

    drone = connect_or_simulate()
    log = load_flight_log("flight_data.sflog.zip")
    plot_trajectory(log)
"""

from .connect import connect_or_simulate
from .log_utils import load_flight_log, load_sample_data
from .plotting import plot_trajectory, compare_logs, plot_step_response

__all__ = [
    "connect_or_simulate",
    "load_flight_log",
    "load_sample_data",
    "plot_trajectory",
    "compare_logs",
    "plot_step_response",
]
