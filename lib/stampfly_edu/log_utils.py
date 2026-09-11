"""
Flight log loading and processing utilities
フライトログの読み込みと処理ユーティリティ

Provides standardized loading -- of either a plain CSV (legacy, still
supported for a student's own hand-authored data) or a StampFly flight-log
v1 bundle (`.sflog.zip` file or extracted directory; see `lib/sflog` and
docs/plans/flight-log-format-plan.md) -- with column name normalization for
educational use.

読み込みを標準化する -- 素の CSV（従来どおり。学生が自分で用意したデータの
ため引き続き対応）と、StampFly フライトログ v1 一式（`.sflog.zip` または
展開済みフォルダ。`lib/sflog` と計画書 docs/plans/flight-log-format-plan.md
参照）のどちらでも受け付け、列名を教育用に正規化する。
"""

from pathlib import Path
from typing import Union

import pandas as pd

import sflog


# Standard column name mapping for education use
# 教育用の標準列名マッピング
_COLUMN_ALIASES = {
    # Time / 時間
    "timestamp": "time",
    "time_ms": "time",
    "t": "time",
    # NOTE: a bundle's own "timestamp_us" (microseconds) is NOT handled
    # here -- it needs a UNIT CONVERSION (/1e6), not just a rename, so
    # load_flight_log() converts it to "time" [s] explicitly before this
    # table is applied. See that function.
    # 注: 一式自身の "timestamp_us"（マイクロ秒）はここでは扱わない --
    # 単なる改名ではなく単位変換（/1e6）が要るため、load_flight_log() が
    # この表を適用する前に明示的に "time"[s] へ変換する。同関数参照。
    #
    # Position / 位置 (also matches a bundle's posvel.csv columns directly)
    "pos_x": "x",
    "pos_y": "y",
    "pos_z": "z",
    "position_x": "x",
    "position_y": "y",
    "position_z": "z",
    # Velocity / 速度 (also matches a bundle's posvel.csv columns directly)
    "vel_x": "vx",
    "vel_y": "vy",
    "vel_z": "vz",
    "velocity_x": "vx",
    "velocity_y": "vy",
    "velocity_z": "vz",
    # Attitude / 姿勢
    "roll_deg": "roll",
    "pitch_deg": "pitch",
    "yaw_deg": "yaw",
    # A bundle's attitude.csv has NO roll/pitch/yaw -- only the quaternion
    # (quat_w/x/y/z). There is no short educational name for a quaternion
    # component, so quat_w/x/y/z are intentionally left UNALIASED ("kept
    # as-is") rather than mapped to something like "qw"/"qx".
    # 一式の attitude.csv に roll/pitch/yaw は無く、クォータニオン
    # （quat_w/x/y/z）のみ。クォータニオン成分に短い教育用名は無いため、
    # quat_w/x/y/z は意図的にエイリアスしない（そのまま残す）。
    #
    # Angular rate / 角速度 (gyro_x/y/z also matches a bundle's imu.csv
    # directly; imu.csv's gyro_raw_x/y/z are intentionally NOT aliased here
    # -- they are the pre-filter value, distinct from the corrected
    # gyro_x/y/z this maps to p/q/r, see protocol/spec/flight_log.yaml).
    "gyro_x": "p",
    "gyro_y": "q",
    "gyro_z": "r",
    "roll_rate": "p",
    "pitch_rate": "q",
    "yaw_rate": "r",
    # Acceleration / 加速度 (accel_x/y/z also matches a bundle's imu.csv
    # directly; accel_raw_x/y/z intentionally left unaliased, same reason
    # as gyro_raw_* above).
    "acc_x": "ax",
    "acc_y": "ay",
    "acc_z": "az",
    "accel_x": "ax",
    "accel_y": "ay",
    "accel_z": "az",
    # Altitude sensors / 高度センサ
    "altitude": "alt",           # bundle: baro.csv's "altitude" [m]
    "baro_alt": "baro",
    "tof_range": "tof",
    "distance": "tof",           # bundle: tof_bottom.csv's "distance" [m]
                                  # -- only correct when tof_front is NOT
                                  # ALSO present in the aligned table
                                  # (aligned()'s collision-rename rule would
                                  # then rename tof_front's bare "distance"
                                  # to "tof_front_distance", not this one,
                                  # since tof_bottom merges first -- see
                                  # lib/sflog/align.py). True for every
                                  # stampfly_edu sample bundle (none include
                                  # tof_front).
    # Motor / モーター
    "motor_1": "m1",
    "motor_2": "m2",
    "motor_3": "m3",
    "motor_4": "m4",
    # bundle: motor.csv's duty_FR/RR/RL/FL -- FR=M1, RR=M2, RL=M3, FL=M4 is
    # this repo's standing motor-numbering convention (see the ASCII
    # diagram in the repo root CLAUDE.md: "FL(M4) FR(M1) / RL(M3) RR(M2)").
    # Keys are lower-case because load_flight_log() lower-cases every
    # column name before this table is applied (duty_FR -> duty_fr).
    # キーは小文字 -- load_flight_log() はこの表を適用する前に全ての
    # 列名を小文字化するため（duty_FR -> duty_fr）。
    "duty_fr": "m1",
    "duty_rr": "m2",
    "duty_rl": "m3",
    "duty_fl": "m4",
    # Battery / バッテリー
    "battery_voltage": "vbat",
    "bat_v": "vbat",
    "voltage": "vbat",           # bundle: status.csv's "voltage" [V]
    # Setpoints / 目標値
    # Not part of the pre-migration alias table. The PACKAGED synthetic
    # samples (generate_samples.py) don't need this -- their rate_step/
    # altitude_step scenarios write a convenience "setpoint" column
    # directly into imu.csv/posvel.csv, so it already arrives bare, no
    # alias required. This entry instead helps a REAL vehicle-capture
    # bundle loaded through this same load_flight_log() (e.g. a student's
    # own `sf log wifi` capture, see 14_project_template/15_analysis_
    # toolkit): a real bundle has genuine rate_ref.csv (rate_ref_roll) and
    # ctrl_ref.csv (alt_setpoint) streams, which this surfaces under the
    # same friendly "setpoint" name. The two keys never co-occur in one
    # bundle, so aliasing both to "setpoint" is unambiguous.
    # 移行前の別名表には無い。同梱の合成サンプル（generate_samples.py）は
    # これを必要としない -- rate_step/altitude_step シナリオは便宜上の
    # "setpoint" 列を imu.csv/posvel.csv に直接書き込むため、別名無しで
    # そのまま届く。この項目はむしろ、同じ load_flight_log() で実機取得の
    # 一式を読む場合（例: 学生自身の `sf log wifi` 取得。
    # 14_project_template/15_analysis_toolkit 参照）に役立つ: 実機の一式は
    # 本物の rate_ref.csv（rate_ref_roll）・ctrl_ref.csv（alt_setpoint）
    # ストリームを持ち、これを同じ親しみやすい "setpoint" という名前で
    # 届ける。この2つのキーは同じ一式に同時には現れないため、両方を
    # "setpoint" に別名しても曖昧にならない。
    "rate_ref_roll": "setpoint",
    "alt_setpoint": "setpoint",
}

# Education sample dataset directory
# 教育用サンプルデータセットのディレクトリ
_DATASETS_DIR = Path(__file__).parent.parent.parent / "analysis" / "datasets" / "education"


def load_flight_log(
    path: Union[str, Path],
    normalize_columns: bool = True,
    time_zero: bool = True,
) -> pd.DataFrame:
    """Load a flight log as a pandas DataFrame.

    フライトログを pandas DataFrame として読み込む。

    `path` may be EITHER a StampFly flight-log v1 bundle (a `.sflog.zip`
    file or an extracted directory -- detected via `sflog.is_bundle()`; see
    `lib/sflog`) or a plain CSV file (the original, still-supported
    behavior, e.g. for a student's own hand-authored data). A bundle is
    loaded with `sflog.FlightLog.load()` and merged into one table aligned
    at the `imu` stream's 400Hz rate with `sflog.aligned(base="imu",
    method="hold")` -- see `lib/sflog/align.py` for the exact merge rules
    (a slower stream's columns are held/forward-filled onto every row, with
    a companion "<stream>_timestamp_us" column recording when that value
    was actually last updated).

    `path` は StampFly フライトログ v1 一式（`.sflog.zip` ファイルまたは
    展開済みフォルダ -- `sflog.is_bundle()` で判定。`lib/sflog` 参照）か、
    素の CSV ファイル（従来どおり。学生が自分で用意したデータ等）の
    どちらでもよい。一式は `sflog.FlightLog.load()` で読み込み、
    `sflog.aligned(base="imu", method="hold")` で `imu` ストリームの
    400Hz レートに整列した1枚の表へ結合する（結合の正確な規則は
    `lib/sflog/align.py` 参照 -- 低速ストリームの列は保持（前方補完）され、
    随伴列 "<stream>_timestamp_us" にその値が実際に最後に更新された時刻が
    記録される）。

    Args:
        path: Path to a CSV file, or a flight-log v1 bundle (`.sflog.zip`
            or an extracted directory) / CSV ファイル、または一式へのパス
        normalize_columns: Apply column name aliases / 列名エイリアスを適用
        time_zero: Shift time column to start from 0 / 時間列を 0 開始にシフト

    Returns:
        DataFrame with normalized column names

    Usage:
        >>> import tempfile, os
        >>> f = tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False)
        >>> _ = f.write("time,pos_x,pos_y\\n0.0,1.0,2.0\\n0.1,1.1,2.1\\n")
        >>> f.close()
        >>> df = load_flight_log(f.name)
        >>> list(df.columns)
        ['time', 'x', 'y']
        >>> os.unlink(f.name)
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Log file not found: {path}")

    if sflog.is_bundle(path):
        log = sflog.FlightLog.load(path)
        df = sflog.aligned(log, base="imu", method="hold")
    else:
        df = pd.read_csv(path)

    if normalize_columns:
        # Apply column aliases (lowercase first)
        # 列エイリアスを適用（まず小文字化）
        df.columns = [c.strip().lower() for c in df.columns]
        if "timestamp_us" in df.columns:
            # A bundle's timestamp_us is integer MICROSECONDS since boot --
            # this needs an explicit unit conversion (not just a rename),
            # so it is handled here rather than via the plain _COLUMN_ALIASES
            # rename table.
            # 一式の timestamp_us は起動基準の整数マイクロ秒 -- 単なる改名
            # ではなく明示的な単位変換が要るため、単純な rename 表
            # _COLUMN_ALIASES ではなくここで扱う。
            df = df.rename(columns={"timestamp_us": "time"})
            df["time"] = df["time"] / 1e6
        df.rename(columns=_COLUMN_ALIASES, inplace=True)

    if time_zero and "time" in df.columns:
        df["time"] = df["time"] - df["time"].iloc[0]

    return df


def load_sample_data(name: str) -> pd.DataFrame:
    """Load a bundled education sample dataset.

    バンドルされた教育用サンプルデータセットを読み込む。

    Available datasets / 利用可能なデータセット:
        - "static_noise": 60s stationary sensor noise / 静止センサノイズ
        - "rate_step": Rate PID step response / レート PID ステップ応答
        - "hover": 30s hover flight / 30 秒ホバリング
        - "altitude_step": Altitude step response / 高度ステップ応答
        - "square_path": Square path flight / 矩形パス飛行

    Args:
        name: Dataset name (without extension) / データセット名（拡張子なし）

    Returns:
        DataFrame with normalized column names

    Usage:
        >>> df = load_sample_data("hover")
        >>> "time" in df.columns
        True
    """
    # Map short names to filenames -- StampFly flight-log v1 bundles
    # (generate_samples.py writes these as `.sflog.zip`, not flat CSVs, so
    # that the notebooks see real multi-rate structure; see lib/sflog).
    # 短縮名をファイル名にマッピング -- StampFly フライトログ v1 一式
    # （generate_samples.py はフラット CSV ではなく `.sflog.zip` として書く
    # -- ノートブックが実際の多レート構造を見られるように。lib/sflog 参照）。
    name_map = {
        "static_noise": "static_noise_60s.sflog.zip",
        "rate_step": "rate_step_response.sflog.zip",
        "hover": "hover_30s.sflog.zip",
        "altitude_step": "altitude_step.sflog.zip",
        "square_path": "square_path.sflog.zip",
    }

    filename = name_map.get(name)
    if filename is None:
        available = ", ".join(sorted(name_map.keys()))
        raise ValueError(
            f"Unknown dataset '{name}'. Available: {available}"
        )

    path = _DATASETS_DIR / filename
    if not path.exists():
        raise FileNotFoundError(
            f"Sample data not found: {path}\n"
            f"Run 'python -m stampfly_edu.generate_samples' to create sample data."
        )

    return load_flight_log(path)
