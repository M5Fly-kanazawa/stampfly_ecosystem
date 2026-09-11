"""
Generate synthetic sample datasets for education
教育用の合成サンプルデータセットを生成

Generates realistic-looking flight data based on StampFly physical parameters
so that notebooks work without a real drone.

Each dataset is written as a StampFly flight-log v1 BUNDLE (`.sflog.zip`,
see `lib/sflog` and docs/plans/flight-log-format-plan.md) instead of one
flat CSV: several per-signal streams at their own native rate (e.g. imu at
400Hz, baro at 50Hz, status at 1Hz), so students see the same multi-rate
bundle structure a real flight log has -- `stampfly_edu.load_flight_log()`
merges it back into one aligned table transparently (see log_utils.py).
These are purely synthetic teaching data (hand-written noise/step-response
models below, not derived from any real flight), so a stream's DataFrame is
free to carry a convenience column beyond what `lib/sflog/schema.py`
declares for that stream (e.g. an explicit "setpoint" column) -- neither
`sflog.FlightLog.save()`/`.load()` nor `sflog.aligned()` validates a
stream's columns against the schema, they simply read/write/merge whatever
is there.

各データセットは、1枚のフラット CSV ではなく StampFly フライトログ v1
一式（`.sflog.zip`、`lib/sflog` と計画書参照）として書き出す: 信号ごとに
その原レート（imu=400Hz, baro=50Hz, status=1Hz 等）の複数ストリームに分け、
実際のフライトログと同じ多レート構造を学習者に見せる --
`stampfly_edu.load_flight_log()` が1枚の整列表へ透過的に結合し直す
（log_utils.py 参照）。これらは純粋に合成した教育用データ（下記の手書き
ノイズ／ステップ応答モデル、実飛行由来ではない）なので、ストリームの
DataFrame は `lib/sflog/schema.py` がそのストリームに定める列を超えて、
便宜上の列（例: 明示的な "setpoint" 列）を自由に持たせてよい --
`sflog.FlightLog.save()`/`.load()` も `sflog.aligned()` も、ストリームの
列をスキーマと照合しない。あるものをそのまま読み書き・結合するだけ。

Usage:
    python -m stampfly_edu.generate_samples
"""

from pathlib import Path

import numpy as np
import pandas as pd

import sflog

# Output directory
# 出力ディレクトリ
OUTPUT_DIR = Path(__file__).parent.parent.parent / "analysis" / "datasets" / "education"

# StampFly parameters (from tools/sysid/defaults.py)
# StampFly パラメータ
GYRO_NOISE = 1.22e-4      # rad/s/√Hz
ACCEL_NOISE = 1.18e-3      # m/s²/√Hz
BARO_NOISE = 0.11          # m
TOF_NOISE = 0.01           # m
SAMPLE_RATE = 400           # Hz (imu native rate / imu の原レート)
DT = 1.0 / SAMPLE_RATE
GRAVITY = 9.80665           # m/s^2


# =============================================================================
# Small helpers shared by several generators
# 複数の生成関数が共有する小さなヘルパー
# =============================================================================

def _lockstep_base(n: int, dt: float) -> tuple:
    """timestamp_us + seq for a LOCKSTEP stream (imu/attitude/posvel/motor
    -- see lib/sflog/align.py's LOCKSTEP_STREAMS): `seq` is what aligned()
    actually merges lockstep streams on, so every lockstep stream in the
    SAME bundle sharing the same `n`/`dt` here merges row-for-row correctly.
    ロックステップ系ストリーム（imu/attitude/posvel/motor -- lib/sflog/
    align.py の LOCKSTEP_STREAMS 参照）用の timestamp_us + seq。aligned() は
    ロックステップ系ストリームを実際には `seq` で結合するため、同じ一式内で
    同じ n/dt を使う全ロックステップ系ストリームは行ごとに正しく結合される。
    """
    t = np.arange(n) * dt
    timestamp_us = np.round(t * 1e6).astype(np.int64)
    seq = np.arange(n)
    return t, timestamp_us, seq


def _held_timestamps(duration_s: float, rate_hz: float) -> tuple:
    """timestamp_us for a non-lockstep (held/forward-filled) stream, e.g.
    baro/tof/status -- these merge onto the base via merge_asof on
    timestamp_us alone (no `seq` needed; see lib/sflog/align.py).
    非ロックステップ系（保持/前方補完）ストリーム用の timestamp_us（baro/
    tof/status 等） -- これらは timestamp_us だけの merge_asof で基準へ
    結合される（`seq` は不要。lib/sflog/align.py 参照）。
    """
    n = max(1, int(duration_s * rate_hz))
    t = np.arange(n) / rate_hz
    return t, np.round(t * 1e6).astype(np.int64)


def _step_response(t: np.ndarray, t_start: float, base: float, step: float,
                   wn: float, zeta: float) -> np.ndarray:
    """Analytic underdamped 2nd-order step response, sampled at arbitrary
    times `t` -- used to generate the SAME step trajectory at several
    different rates (posvel @400Hz, baro @50Hz, tof @30Hz) without
    resampling/interpolating one array into another.
    任意の時刻配列 `t` でサンプリングする、減衰振動2次系のステップ応答の
    解析解 -- 同じステップ軌道を複数のレート（posvel@400Hz, baro@50Hz,
    tof@30Hz）で、配列の再サンプリング・補間なしに個別生成するために使う。

    Args:
        t: time array [s]
        t_start: step onset time [s] (output is `base` before this)
        base: pre-step value
        step: step size (post-step steady value = base + step)
        wn: natural frequency [rad/s]
        zeta: damping ratio (0 < zeta < 1)
    """
    y = np.full_like(t, base, dtype=float)
    active = t >= t_start
    tau = t[active] - t_start
    wd = wn * np.sqrt(1.0 - zeta**2)
    envelope = np.exp(-zeta * wn * tau)
    phi = np.arctan2(np.sqrt(1.0 - zeta**2), zeta)
    y[active] = base + step * (
        1.0 - envelope * np.sin(wd * tau + phi) / np.sqrt(1.0 - zeta**2)
    )
    return y


def _euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
    """Small-angle-safe ZYX Euler (degrees) -> quaternion [w, x, y, z].
    Standard aerospace (Tait-Bryan ZYX) convention, matching the body-frame
    attitude quaternion lib/sflog/schema.py's `attitude` stream declares.
    ZYX オイラー角（度）->クォータニオン [w,x,y,z]。標準的な航空工学の
    ZYX 規約、lib/sflog/schema.py の `attitude` ストリームが定める機体
    姿勢クォータニオンと同じ規約。
    """
    r = np.radians(roll_deg) / 2.0
    p = np.radians(pitch_deg) / 2.0
    y = np.radians(yaw_deg) / 2.0
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


# =============================================================================
# Dataset generators -- each returns {stream_name: DataFrame}
# データセット生成関数 -- 各々 {ストリーム名: DataFrame} を返す
# =============================================================================

def generate_static_noise(duration: float = 60.0) -> dict:
    """Generate 60s of stationary sensor noise data.

    60 秒間の静止センサノイズデータを生成する。

    Streams: imu (400Hz), baro (50Hz), tof_bottom (30Hz) -- three different
    native rates, a clear first example of a bundle's multi-rate structure.
    ストリーム: imu(400Hz)・baro(50Hz)・tof_bottom(30Hz) -- 3つの異なる原
    レート。一式の多レート構造の分かりやすい最初の例。
    """
    n = int(duration * SAMPLE_RATE)
    t, ts_us, seq = _lockstep_base(n, DT)
    rng = np.random.default_rng(42)

    # Gyroscope noise + slow bias drift
    # ジャイロノイズ + 低速バイアスドリフト
    gyro_sigma = GYRO_NOISE * np.sqrt(SAMPLE_RATE)
    bias_drift = 1.75e-3  # rad/s bias instability
    gx = rng.normal(0, gyro_sigma, n) + bias_drift * np.sin(2 * np.pi * 0.01 * t)
    gy = rng.normal(0, gyro_sigma, n) + bias_drift * np.cos(2 * np.pi * 0.008 * t)
    gz = rng.normal(0, gyro_sigma, n) + bias_drift * np.sin(2 * np.pi * 0.012 * t)

    # Accelerometer noise (gravity on z-axis)
    # 加速度計ノイズ（z 軸に重力）
    accel_sigma = ACCEL_NOISE * np.sqrt(SAMPLE_RATE)
    ax = rng.normal(0, accel_sigma, n)
    ay = rng.normal(0, accel_sigma, n)
    az = rng.normal(-GRAVITY, accel_sigma, n)

    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": gx, "gyro_y": gy, "gyro_z": gz,
        "accel_x": ax, "accel_y": ay, "accel_z": az,
        "gyro_raw_x": gx, "gyro_raw_y": gy, "gyro_raw_z": gz,
        "accel_raw_x": ax, "accel_raw_y": ay, "accel_raw_z": az,
    })

    t_baro, ts_us_baro = _held_timestamps(duration, 50.0)
    baro = pd.DataFrame({
        "timestamp_us": ts_us_baro,
        "altitude": rng.normal(0, BARO_NOISE, len(t_baro)),
        "pressure": np.full(len(t_baro), 101325.0),
    })

    t_tof, ts_us_tof = _held_timestamps(duration, 30.0)
    tof_bottom = pd.DataFrame({
        "timestamp_us": ts_us_tof,
        "distance": rng.normal(0, TOF_NOISE, len(t_tof)),
        "status": np.zeros(len(t_tof), dtype=int),
    })

    return {"imu": imu, "baro": baro, "tof_bottom": tof_bottom}


def generate_rate_step_response() -> dict:
    """Generate a rate PID step response (roll axis).

    レート PID ステップ応答を生成する（ロール軸）。

    Streams: imu (400Hz, gyro_x = measured roll rate, plus a convenience
    "setpoint" column -- not a real IMU field, kept here rather than in a
    separate rate_ref stream so the step-response notebook/example can read
    "time"/"p"/"setpoint" from one place), motor (400Hz, the 4 duties).
    ストリーム: imu(400Hz、gyro_x=計測ロールレート、加えて便宜上の
    "setpoint" 列 -- 本物のIMUフィールドではないが、別の rate_ref
    ストリームに分けず imu に同居させることで、ステップ応答の
    ノートブック/サンプルが "time"/"p"/"setpoint" を1か所から読める)、
    motor(400Hz、4つのduty)。
    """
    duration = 3.0
    n = int(duration * SAMPLE_RATE)
    t, ts_us, seq = _lockstep_base(n, DT)

    # Step input: 0 for first 0.5s, then 1.0 rad/s
    # ステップ入力: 最初 0.5 秒は 0、その後 1.0 rad/s
    setpoint = np.zeros(n)
    setpoint[int(0.5 * SAMPLE_RATE):] = 1.0

    # Simulate second-order response (typical PID tuned system)
    # 2 次応答をシミュレート（典型的な PID 調整済みシステム）
    response = _step_response(t, t_start=0.5, base=0.0, step=1.0, wn=25.0, zeta=0.7)

    rng = np.random.default_rng(123)
    noise = rng.normal(0, GYRO_NOISE * np.sqrt(SAMPLE_RATE), n)
    measured = response + noise

    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": measured, "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.full(n, -GRAVITY),
        "gyro_raw_x": measured, "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.full(n, -GRAVITY),
        "setpoint": setpoint,
    })
    motor = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "duty_FR": 0.5 + 0.1 * response,
        "duty_RR": 0.5 - 0.1 * response,
        "duty_RL": 0.5 - 0.1 * response,
        "duty_FL": 0.5 + 0.1 * response,
    })

    return {"imu": imu, "motor": motor}


def generate_hover(duration: float = 30.0) -> dict:
    """Generate 30s hover flight data.

    30 秒ホバリングデータを生成する。

    Streams: posvel (400Hz), attitude (400Hz, quaternion), imu (400Hz,
    gyro only), motor (400Hz), status (1Hz, voltage) -- status's 1Hz rate
    against everything else's 400Hz is this dataset's clearest multi-rate
    moment.
    ストリーム: posvel(400Hz)・attitude(400Hz、クォータニオン)・
    imu(400Hz、ジャイロのみ)・motor(400Hz)・status(1Hz、電圧) -- status の
    1Hz と他全ての400Hzの対比が、このデータセットで最も分かりやすい
    多レートの例。
    """
    n = int(duration * SAMPLE_RATE)
    t, ts_us, seq = _lockstep_base(n, DT)
    rng = np.random.default_rng(456)

    # Position with small drift
    # 小さなドリフト付きの位置
    x = np.cumsum(rng.normal(0, 0.0001, n))
    y = np.cumsum(rng.normal(0, 0.0001, n))
    z = 0.5 + rng.normal(0, 0.005, n)  # Hover at 0.5m

    # Attitude oscillation (small), in degrees -- converted to quaternion below
    # 小さな姿勢振動（度）-- 下でクォータニオンへ変換
    roll_deg = rng.normal(0, 0.5, n)
    pitch_deg = rng.normal(0, 0.5, n)
    yaw_deg = np.cumsum(rng.normal(0, 0.01, n))
    qw, qx, qy, qz = _euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg)

    # Angular rates
    # 角速度
    p = rng.normal(0, 0.02, n)  # rad/s
    q = rng.normal(0, 0.02, n)
    r = rng.normal(0, 0.01, n)

    # Motors near hover thrust
    # ホバー推力付近のモーター
    hover_duty = 0.55
    m1 = hover_duty + rng.normal(0, 0.02, n)
    m2 = hover_duty + rng.normal(0, 0.02, n)
    m3 = hover_duty + rng.normal(0, 0.02, n)
    m4 = hover_duty + rng.normal(0, 0.02, n)

    posvel = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "pos_x": x, "pos_y": y, "pos_z": z,
        "vel_x": np.gradient(x, DT), "vel_y": np.gradient(y, DT), "vel_z": np.gradient(z, DT),
    })
    attitude = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "quat_w": qw, "quat_x": qx, "quat_y": qy, "quat_z": qz,
        "gyro_bias_x": np.zeros(n), "gyro_bias_y": np.zeros(n), "gyro_bias_z": np.zeros(n),
        "accel_bias_x": np.zeros(n), "accel_bias_y": np.zeros(n), "accel_bias_z": np.zeros(n),
    })
    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": p, "gyro_y": q, "gyro_z": r,
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.full(n, -GRAVITY),
        "gyro_raw_x": p, "gyro_raw_y": q, "gyro_raw_z": r,
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.full(n, -GRAVITY),
    })
    motor = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "duty_FR": m1, "duty_RR": m2, "duty_RL": m3, "duty_FL": m4,
    })

    t_status, ts_us_status = _held_timestamps(duration, 1.0)
    status = pd.DataFrame({
        "timestamp_us": ts_us_status,
        "voltage": 3.7 - 0.3 * t_status / duration,
    })

    return {"posvel": posvel, "attitude": attitude, "imu": imu, "motor": motor, "status": status}


def generate_altitude_step() -> dict:
    """Generate altitude step response (0.3m -> 0.6m).

    高度ステップ応答を生成する（0.3m → 0.6m）。

    Streams: imu (400Hz, all-zero gyro/accel except gravity on z -- this
    dataset's focus is altitude, not attitude; included ONLY because a
    bundle's alignment base defaults to `imu`, see log_utils.py
    load_flight_log() and lib/sflog/schema.py REQUIRED_STREAMS), posvel
    (400Hz, pos_z/vel_z + a convenience "setpoint" column, same rationale
    as generate_rate_step_response()'s imu.setpoint), baro (50Hz),
    tof_bottom (30Hz) -- the SAME analytic step trajectory
    (_step_response()) is sampled independently at each stream's own rate,
    so posvel/baro/tof each show a genuinely native-rate version of the
    same physical event instead of one array resampled into another.
    ストリーム: imu(400Hz、重力を除きジャイロ/加速度は全ゼロ -- この
    データセットの主眼は高度でなく姿勢ではないため。含める理由は一式の
    整列基準が既定で `imu` になるからのみ、log_utils.py の
    load_flight_log() と lib/sflog/schema.py の REQUIRED_STREAMS 参照)、
    posvel(400Hz、pos_z/vel_z ＋便宜上の "setpoint" 列。
    generate_rate_step_response() の imu.setpoint と同じ理由)、
    baro(50Hz)、tof_bottom(30Hz) -- 同じ解析的ステップ軌道
    （_step_response()）を各ストリーム自身のレートで個別にサンプリング
    するため、posvel/baro/tof はそれぞれ他方から再サンプリングしたのでは
    ない、正真正銘その原レートの同一物理事象を示す。
    """
    duration = 10.0
    t_step_start, base_alt, step_size = 2.0, 0.3, 0.3
    wn, zeta = 5.0, 0.9

    n = int(duration * SAMPLE_RATE)
    t, ts_us, seq = _lockstep_base(n, DT)
    setpoint = np.where(t < t_step_start, base_alt, base_alt + step_size)
    z_clean = _step_response(t, t_step_start, base_alt, step_size, wn, zeta)
    rng = np.random.default_rng(789)
    z = z_clean + rng.normal(0, 0.005, n)

    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.full(n, -GRAVITY),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.full(n, -GRAVITY),
    })
    posvel = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "pos_x": np.zeros(n), "pos_y": np.zeros(n), "pos_z": z,
        "vel_x": np.zeros(n), "vel_y": np.zeros(n), "vel_z": np.gradient(z, DT),
        "setpoint": setpoint,
    })

    t_baro, ts_us_baro = _held_timestamps(duration, 50.0)
    z_baro = _step_response(t_baro, t_step_start, base_alt, step_size, wn, zeta)
    baro = pd.DataFrame({
        "timestamp_us": ts_us_baro,
        "altitude": z_baro + np.random.default_rng(790).normal(0, BARO_NOISE, len(t_baro)),
        "pressure": np.full(len(t_baro), 101325.0),
    })

    t_tof, ts_us_tof = _held_timestamps(duration, 30.0)
    z_tof = _step_response(t_tof, t_step_start, base_alt, step_size, wn, zeta)
    tof_bottom = pd.DataFrame({
        "timestamp_us": ts_us_tof,
        "distance": z_tof + np.random.default_rng(791).normal(0, TOF_NOISE, len(t_tof)),
        "status": np.zeros(len(t_tof), dtype=int),
    })

    return {"imu": imu, "posvel": posvel, "baro": baro, "tof_bottom": tof_bottom}


def generate_square_path() -> dict:
    """Generate square path flight data (1m x 1m).

    矩形パス飛行データを生成する（1m x 1m）。

    Streams: posvel AND a minimal imu, both at 50Hz -- a deliberate
    deviation from a real bundle (where imu is always 400Hz; nothing in
    lib/sflog validates a stream's actual rate against schema.py's declared
    nominal_rate_hz, so this is fine for a synthetic teaching bundle). The
    50Hz rate matches the original flat CSV's already-downsampled position
    data -- there is no velocity measurement in the source data this
    dataset was modeled on, so vel_x/y/z are derived here via a simple
    gradient rather than invented from nothing. `imu` is included only
    because a bundle's alignment base defaults to it (see log_utils.py
    load_flight_log()) -- this dataset's focus is the XY trajectory, so its
    gyro/accel are all-zero except gravity.
    ストリーム: posvel と最小限の imu の両方、ともに50Hz -- 実際の一式
    （imu は常に400Hz）からの意図的な逸脱（lib/sflog はストリームの実際の
    レートを schema.py の宣言 nominal_rate_hz と照合しないため、合成教育用
    一式としては問題ない）。50Hz は元のフラット CSV が既に50Hzへ
    ダウンサンプリングしていた位置データに合わせたもの -- このデータセットの
    元になったソースに速度計測は無いため、vel_x/y/z は何もないところから
    捏造するのではなく単純な勾配から導出する。`imu` を含めるのは一式の
    整列基準が既定でそれになるからのみ（log_utils.py の load_flight_log()
    参照）-- このデータセットの主眼は XY 軌跡なので、ジャイロ/加速度は
    重力を除き全ゼロ。
    """
    # Build waypoints: takeoff -> square -> land
    # ウェイポイント構築: 離陸 → 矩形 → 着陸
    speed = 0.3  # m/s
    side = 1.0   # m
    rate_hz = 50.0

    segments = [
        # (dx, dy, dz, description)
        (0, 0, 0.5, "takeoff"),       # Hover at 0.5m
        (side, 0, 0, "forward"),      # Forward 1m
        (0, side, 0, "left"),         # Left 1m
        (-side, 0, 0, "backward"),    # Backward 1m
        (0, -side, 0, "right"),       # Right 1m
        (0, 0, -0.5, "land"),         # Land
    ]

    all_t, all_x, all_y, all_z = [], [], [], []
    cx, cy, cz = 0.0, 0.0, 0.0
    ct = 0.0

    rng = np.random.default_rng(101)

    for dx, dy, dz, desc in segments:
        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        seg_time = dist / speed if dist > 0 else 1.0
        n_pts = int(seg_time * rate_hz)
        if n_pts < 10:
            n_pts = 10

        t_seg = np.linspace(0, seg_time, n_pts, endpoint=False)
        frac = t_seg / seg_time

        # Smooth trajectory (S-curve)
        # 滑らかな軌跡（S カーブ）
        smooth_frac = 3 * frac**2 - 2 * frac**3

        x_seg = cx + dx * smooth_frac + rng.normal(0, 0.005, n_pts)
        y_seg = cy + dy * smooth_frac + rng.normal(0, 0.005, n_pts)
        z_seg = cz + dz * smooth_frac + rng.normal(0, 0.003, n_pts)

        all_t.extend((ct + t_seg).tolist())
        all_x.extend(x_seg.tolist())
        all_y.extend(y_seg.tolist())
        all_z.extend(z_seg.tolist())

        cx += dx
        cy += dy
        cz += dz
        ct += seg_time

    t_arr = np.array(all_t)
    x_arr = np.array(all_x)
    y_arr = np.array(all_y)
    z_arr = np.array(all_z)
    dt_nominal = 1.0 / rate_hz

    ts_us = np.round(t_arr * 1e6).astype(np.int64)
    seq = np.arange(len(t_arr))
    n = len(t_arr)

    posvel = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "pos_x": x_arr, "pos_y": y_arr, "pos_z": z_arr,
        "vel_x": np.gradient(x_arr, dt_nominal),
        "vel_y": np.gradient(y_arr, dt_nominal),
        "vel_z": np.gradient(z_arr, dt_nominal),
    })
    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.full(n, -GRAVITY),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.full(n, -GRAVITY),
    })

    return {"imu": imu, "posvel": posvel}


def main():
    """Generate all sample datasets as flight-log v1 bundles.

    全サンプルデータセットをフライトログ v1 一式として生成する。
    """
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    datasets = {
        "static_noise_60s.sflog.zip": generate_static_noise,
        "rate_step_response.sflog.zip": generate_rate_step_response,
        "hover_30s.sflog.zip": generate_hover,
        "altitude_step.sflog.zip": generate_altitude_step,
        "square_path.sflog.zip": generate_square_path,
    }

    for filename, generator in datasets.items():
        path = OUTPUT_DIR / filename
        streams = generator()
        meta = sflog.make_meta(
            source="sim",
            tool_name="stampfly_edu.generate_samples",
            tool_version="0.1.0",
            notes="Synthetic education dataset -- hand-written noise/step-"
                  "response models, not derived from a real flight.",
            streams=streams,
        )
        # Self-describing schema.json for the streams THIS bundle actually
        # carries (schema.schema_for() -- see lib/sflog/schema.py), same as
        # a real capture's bundle.
        # このバンドルが実際に持つストリームだけの自己記述 schema.json
        # （schema.schema_for() -- lib/sflog/schema.py 参照）。実機取得の
        # 一式と同じ。
        schema_json = sflog.schema.schema_for(streams.keys())
        log = sflog.FlightLog(meta=meta, schema=schema_json, streams=streams)
        log.save(path)
        n_rows = {name: len(df) for name, df in streams.items()}
        print(f"Generated {path} (streams: {n_rows})")

    print(f"\nAll datasets saved to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
