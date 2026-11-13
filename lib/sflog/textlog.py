"""
textlog.py - Read the vehicle's text log (the `text_log` stream) of a flight-log bundle.
textlog.py - フライトログ一式の機体のテキストログ（`text_log` ストリーム）を読む。

The `text_log` stream holds the ESP_LOG lines the vehicle's log sink delivered during a WiFi capture
(what the USB console would have shown), one row per line, in the vehicle's order, with the vehicle
time at which the line entered the sink (protocol/spec/flight_log.yaml). This module turns it into
printable lines, filtered by a substring and a time window, with the loss the vehicle reported made
visible. It is the implementation behind `sf log info --text`.
`text_log` ストリームは、WiFi 取得の間に機体のログシンクが届けた ESP_LOG の行（USB コンソールに出るはずだったもの）を、
1 行 1 行、機体の順序で、行がシンクに入った機体時刻つきで持つ（protocol/spec/flight_log.yaml）。本モジュールはそれを、
部分文字列と時間窓で絞った表示用の行にし、機体が報告した欠落を見えるようにする。`sf log info --text` の実体。

@design docs/commands/sf-log.md (sf log info --text); protocol/spec/flight_log.yaml stream text_log
"""

from __future__ import annotations

from typing import Optional

import pandas as pd

#: Microseconds per second. / 1 秒あたりのマイクロ秒。
_US_PER_S = 1_000_000


def has_text_log(log) -> bool:
    """True when the bundle carries a text_log stream with at least one line.
    一式が 1 行以上の text_log ストリームを持てば True。
    """
    return "text_log" in log.streams and len(log.streams["text_log"]) > 0


def format_text_log(
    log,
    grep: Optional[str] = None,
    ignore_case: bool = False,
    since_s: Optional[float] = None,
    until_s: Optional[float] = None,
) -> list:
    """Printable lines of the bundle's text log.

    Each line is `[<vehicle time> s] <text>`; a line the vehicle had to cut carries ` [truncated]` at the
    end; where the vehicle's log sink refused lines (ring full) a marker `*** N lines dropped here ***`
    comes BEFORE the line that followed the loss, whatever the filter (a loss is never hidden by `grep`).
    `grep` keeps the lines containing the substring (a plain substring, not a pattern); `since_s` / `until_s`
    keep the lines whose vehicle time (seconds since boot) lies in [since_s, until_s].
    Returns [] when the bundle has no text_log (the caller says so; this is not an error).

    一式のテキストログの表示用の行。各行は `[<機体時刻> s] <本文>`。機体が切り詰めた行は末尾に ` [truncated]` が付く。機体の
    ログシンクが行を断った（リングが満杯）位置では、欠落の直後の行の「前」に `*** N lines dropped here ***` の印を置く。絞り込みに
    関わらず出す（欠落は `grep` で隠れない）。`grep` は部分文字列（パターンではない）を含む行を残し、`since_s` / `until_s` は
    機体時刻（起動からの秒）が [since_s, until_s] にある行を残す。一式に text_log が無ければ [] を返す（呼び出し側が知らせる。
    エラーではない）。
    """
    if not has_text_log(log):
        return []
    df = log.streams["text_log"]
    needle = grep.lower() if (grep is not None and ignore_case) else grep
    lines = []
    for row in df.itertuples(index=False):
        text = "" if pd.isna(row.text) else str(row.text)
        seconds = row.timestamp_us / _US_PER_S
        keep = (
            (needle is None or needle in (text.lower() if ignore_case else text))
            and (since_s is None or seconds >= since_s)
            and (until_s is None or seconds <= until_s)
        )
        dropped = int(row.dropped_before) if not pd.isna(row.dropped_before) else 0
        if dropped > 0:
            lines.append(f"            *** {dropped} lines dropped here ***")
        if keep:
            suffix = " [truncated]" if (not pd.isna(row.truncated) and int(row.truncated) == 1) else ""
            lines.append(f"[{seconds:10.6f} s] {text}{suffix}")
    return lines


def diagnostics_summary(log) -> list:
    """Short report lines of the status v4 diagnostics streams (the LAST row of each, i.e. the totals at the
    end of the capture) and of the text log; [] when the bundle has none of them. The counts are since boot
    (loop timing, non-finite) or since the capture started (log paths), see the stream descriptions.
    ステータス v4 の診断値のストリーム（それぞれの「最後の」行、つまり取得の終わりの累計）とテキストログの短い報告の行。
    どれも持たない一式では []。数え方は起動以降（ループ周期・有限でない数）または取得の開始以降（ログ経路）。ストリームの説明を参照。
    """
    lines = []
    if "loop_timing" in log.streams and len(log.streams["loop_timing"]) > 0:
        last = log.streams["loop_timing"].iloc[-1]
        for task in ("imu", "control"):
            lines.append(
                f"{task:<8s} loop: cycles={int(last[task + '_cycles'])} late={int(last[task + '_late_count'])} "
                f"period_max={int(last[task + '_period_max_us'])}us busy_max={int(last[task + '_busy_max_us'])}us "
                f"over_budget={int(last[task + '_over_budget_count'])} | armed: "
                f"late={int(last[task + '_armed_late_count'])} period_max={int(last[task + '_armed_period_max_us'])}us "
                f"busy_max={int(last[task + '_armed_busy_max_us'])}us"
            )
    if "nonfinite" in log.streams and len(log.streams["nonfinite"]) > 0:
        last = log.streams["nonfinite"].iloc[-1]
        lines.append(
            f"nonfinite: detected {int(last['detected_cycles'])} of {int(last['checked_cycles'])} armed cycles "
            f"(first boundary {int(last['first_stage'])}, 255 = none)"
        )
    if "tof_diagnostics" in log.streams and len(log.streams["tof_diagnostics"]) > 0:
        last = log.streams["tof_diagnostics"].iloc[-1]
        for side in ("bottom", "front"):
            lines.append(
                f"ToF {side:<6s}: range_status={int(last[side + '_range_status'])} "
                f"signal={last[side + '_signal_rate']:.2f}Mcps ambient={last[side + '_ambient_rate']:.2f}Mcps "
                f"sigma={last[side + '_sigma_mm']:.1f}mm (the last 1 Hz sample)"
            )
    if "link_health" in log.streams and len(log.streams["link_health"]) > 0:
        last = log.streams["link_health"].iloc[-1]
        lines.append(
            f"log paths: log_stream_overflow={int(last['log_stream_overflow'])} send_drops={int(last['send_drops'])} "
            f"entry_drops={int(last['entry_drops'])} text_dropped_lines={int(last['text_dropped_lines'])}"
        )
    if has_text_log(log):
        lines.append(f"text log : {len(log.streams['text_log'])} lines (read them with `sf log info --text`)")
    return lines
