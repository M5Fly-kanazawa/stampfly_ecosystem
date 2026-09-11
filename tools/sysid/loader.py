"""
Flight-log bundle loader for the sysid toolkit
sysid ツールキット向けフライトログ一式ローダー

This replaces the two independent format detectors this file and
tools/sysid/plant_fit.py used to carry ("stream" from `sf log wifi
-o *.csv`, "convert"/"legacy" from `sf log convert`'s USB binary-log CSV)
with a single bundle-based loader. The standard flight log is now a
"StampFly flight-log v1" bundle (a `*.sflog.zip` file, or an extracted
directory with the same flat layout: `meta.json`, `schema.json`, and one
CSV per stream) -- see docs/plans/flight-log-format-plan.md section 2
(format) and section 3.2 (this loader's place in the read side). The
bundle read/write/align primitives themselves live in `lib/sflog`
(imported here as the `sflog` package -- this repo's pyproject.toml maps
`lib/` as the package root, so `import sflog` works with no sys.path
manipulation once the project is installed with `pip install -e .`).
これは、このファイルと tools/sysid/plant_fit.py がそれぞれ持っていた
2つの独立した形式判別（"stream" = `sf log wifi -o *.csv`、
"convert"/"legacy" = `sf log convert` の USB バイナリログ CSV）を、単一の
バンドル（一式）ベースのローダーに置き換える。標準のフライトログは今や
「StampFly フライトログ v1」一式（`*.sflog.zip` ファイル、または同じ平坦
レイアウト -- `meta.json`・`schema.json`・ストリームごとの CSV -- を持つ
展開済みフォルダ）である -- docs/plans/flight-log-format-plan.md の
2節（形式）・3.2節（読み込み側におけるこのローダーの位置づけ）参照。
一式の読み書き・整列そのものは `lib/sflog`（ここでは `sflog` パッケージ
として import する -- 本リポジトリの pyproject.toml が `lib/` をパッケージ
ルートに割り当てているため、`pip install -e .` 済みの環境なら sys.path
操作なしで `import sflog` できる）にある。

Old CSV-format API removed in this change (nothing in tools/sysid/ calls
these anymore): `SensorSample`, `LogData`, `detect_csv_format()`,
`load_csv()`. `tools/log_analyzer/rate_sysid.py` keeps its OWN, unrelated
`load_csv(path, axis)` for the SILS `rate_stream.csv` file emitted
directly by the emulator (simulator/sils/devices/emu_rate_stream.cpp) --
that is a different file this module does not touch, scheduled for
removal in flight-log-format-plan.md's Phase 3.
このファイルが持っていた旧 CSV 形式 API は本変更で削除した
（tools/sysid/ 配下はどこも呼んでいない）: `SensorSample`、`LogData`、
`detect_csv_format()`、`load_csv()`。`tools/log_analyzer/rate_sysid.py`
は、エミュレータが直接書き出す SILS の `rate_stream.csv`
（simulator/sils/devices/emu_rate_stream.cpp）向けに、これとは無関係の
**別の** `load_csv(path, axis)` を引き続き持つ -- 本モジュールが触る
ファイルではなく、flight-log-format-plan.md の Phase 3 で削除予定。
"""

from __future__ import annotations

import pandas as pd
import sflog


def load_aligned(path, base: str = "imu", method: str = "hold") -> pd.DataFrame:
    """Load a StampFly flight-log v1 bundle and return one aligned table.

    Thin wrapper over `sflog.FlightLog.load()` + `sflog.aligned()` (see
    lib/sflog/bundle.py and lib/sflog/align.py for the full merge
    semantics -- lockstep lockstep streams joined exactly on `seq`,
    slower streams held (forward-filled) with a `<stream>_timestamp_us`
    provenance column). `streams` is left at its default (every stream
    present in the bundle, merged in `schema.STREAMS` order) because the
    sysid backends need columns from several different streams (imu,
    attitude, posvel, rate_ref, motor, ctrl_output, pilot, ctrl_ref,
    baro, tof_bottom/front, flow, mag, status) and there is no benefit to
    hand-picking a subset here.

    StampFly フライトログ v1 一式を読み込み、1枚の整列表を返す。

    `sflog.FlightLog.load()` + `sflog.aligned()` の薄いラッパー（結合の
    完全な意味論は lib/sflog/bundle.py・lib/sflog/align.py 参照 --
    ロックステップ系ストリームは `seq` の完全一致、それ以外は保持
    （前方補完）し随伴列 `<stream>_timestamp_us` で出所を明示する）。
    `streams` は既定のまま（バンドルに実在する全ストリームを
    `schema.STREAMS` の順で結合）にしている -- sysid の各バックエンドは
    複数の異なるストリーム（imu, attitude, posvel, rate_ref, motor,
    ctrl_output, pilot, ctrl_ref, baro, tof_bottom/front, flow, mag,
    status）の列を必要とし、ここで部分集合に絞る利点が無いため。

    Args:
        path: a `.sflog.zip` file, or an extracted bundle directory.
        base: name of the stream whose `timestamp_us` becomes the
            result's time base (default "imu", the only REQUIRED
            stream -- every real vehicle/SILS bundle has it).
        method: "hold" (default, `merge_asof` backward -- the most
            recently observed value at or before the base time) or
            "nearest" -- see `sflog.aligned()`.

    Returns:
        A `pandas.DataFrame` at `base`'s native rate (400 Hz for `imu` on
        a real vehicle capture), columns depending on which streams the
        bundle actually contains (see the module-level column contract
        below). Two pieces of metadata are attached via `df.attrs`
        (read them IMMEDIATELY after this call returns -- `.attrs` is
        not guaranteed to survive every DataFrame operation, e.g. some
        copies/filters drop it):
          - `df.attrs["bundle_streams"]`: the `set` of stream names
            actually present in the bundle (`log.streams.keys()`). This
            is the RECOMMENDED way for a caller to check "is genuine
            400 Hz motor duty available" -- test
            `"motor" in df.attrs["bundle_streams"]` -- rather than the
            old code's stairstep/duplicate-row heuristic on the duty
            columns themselves, which is no longer needed now that the
            bundle format makes each stream's real origin explicit.
          - `df.attrs["bundle_path"]`: `str(path)`, for diagnostics/
            error messages.

    Column contract (present only when the corresponding source stream
    exists in the bundle -- `imu` is the only one guaranteed to exist):
      imu (always):        timestamp_us, seq, gyro_x/y/z [rad/s],
                            accel_x/y/z [m/s^2], gyro_raw_x/y/z,
                            accel_raw_x/y/z
      attitude:             quat_w/x/y/z, gyro_bias_x/y/z, accel_bias_x/y/z
      posvel:               pos_x/y/z [m], vel_x/y/z [m/s]
      rate_ref:             rate_ref_roll/pitch/yaw [rad/s]
      motor:                duty_FR/RR/RL/FL [0..1] at genuine 400 Hz
      ctrl_output:          thrust [N], torque_roll/pitch/yaw [N*m]
                            (pre-mixer commanded values, 400 Hz native)
      pilot (held):         throttle, roll, pitch, yaw
                            + pilot_timestamp_us
      ctrl_ref (held):      flight_mode, angle_ref_roll/pitch,
                            total_thrust, alt_setpoint, alt_vel_target,
                            climb_rate_cmd, pos_setpoint_x/y
                            + ctrl_ref_timestamp_us; ALSO duty_FR/RR/RL/FL
                            *only if `motor` is absent* -- when both are
                            present, ctrl_ref's 50 Hz duty collides with
                            motor's 400 Hz duty (motor merges first) and
                            is renamed ctrl_ref_duty_FR etc.
      baro (held):          altitude [m], pressure [Pa] + baro_timestamp_us
      tof_bottom (held):    distance [m], status + tof_bottom_timestamp_us
      tof_front (held):     tof_front_distance, tof_front_status
                            (renamed -- collides with tof_bottom's bare
                            names since tof_bottom merges first)
                            + tof_front_timestamp_us
      flow (held):          dx, dy, quality + flow_timestamp_us
      mag (held):           x, y, z [uT] + mag_timestamp_us
      status (held):        uptime_ms, voltage [V], current_ma,
                            flight_state, sensor_health, eskf_status,
                            reset_reason, pid_roll_kp/ti/td,
                            pid_pitch_kp/ti/td, pid_yaw_kp/ti/td
                            + status_timestamp_us

    Raises:
        ValueError: `base` is not present in the bundle, or `method` is
            not one of "hold"/"nearest" (see `sflog.aligned()`).
    """
    log = sflog.FlightLog.load(path)
    df = sflog.aligned(log, base=base, method=method)
    df.attrs["bundle_streams"] = set(log.streams.keys())
    df.attrs["bundle_path"] = str(path)
    return df


def sample_rate_hz(df: pd.DataFrame, fallback_hz: float = 400.0) -> float:
    """
    Estimate a bundle DataFrame's sample rate [Hz] from its `timestamp_us`
    column's span: `(n - 1) / duration_s`, where `duration_s` is the gap
    between the first and last timestamp. This is the SAME calculation the
    retired CSV-based `load_csv()`/`LogData.sample_rate_hz` used to make --
    kept here as one shared helper so `noise.py`/`motor.py`/`inertia.py`
    don't each duplicate it.

    Falls back to `fallback_hz` when there are fewer than 2 rows, or the
    timestamp span is zero or negative (e.g. every row shares one
    timestamp) -- both would otherwise divide by zero.

    バンドル DataFrame の `timestamp_us` 列の範囲からサンプルレート[Hz]を
    推定する: `(n-1) / 継続時間[s]`（継続時間は最初と最後のタイムスタンプの
    差）。廃止した CSV ベースの `load_csv()`/`LogData.sample_rate_hz` が
    行っていたのと同じ計算 -- `noise.py`/`motor.py`/`inertia.py` がそれぞれ
    重複させないよう、ここに共有ヘルパーとして置く。

    行数が2未満、またはタイムスタンプ範囲がゼロ以下（例: 全行が同じ
    タイムスタンプ）のときは `fallback_hz` を返す（そうしないとゼロ割りに
    なる）。

    Args:
        df: DataFrame with a `timestamp_us` column (e.g. from
            `load_aligned()`).
        fallback_hz: value to return when the rate cannot be computed from
            the data (default 400.0, StampFly's native control-cycle rate).

    Returns:
        Estimated sample rate [Hz].
    """
    n = len(df)
    if n < 2:
        return fallback_hz
    ts = df["timestamp_us"].to_numpy()
    duration_s = (ts[-1] - ts[0]) / 1e6
    if duration_s <= 0:
        return fallback_hz
    return (n - 1) / duration_s
