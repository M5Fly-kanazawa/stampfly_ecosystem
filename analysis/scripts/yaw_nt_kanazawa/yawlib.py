"""
yawlib.py
=========
StampFly UDP JSONL ログを解析するための再利用ライブラリ。

1行1レコードのJSONL（id, ts[マイクロ秒]付き）を id ごとの numpy 配列時系列に
パースし、クォータニオンからヨー角を計算・アンラップし、モータ駆動状態から
飛行区間（離陸〜着陸）を検出するユーティリティ関数群を提供する。

後続の解析スクリプト（yaw_event_scan.py 等）から import して使う。

主な関数:
    load_jsonl(path)            -> dict[str, dict[str, np.ndarray]]
        id別にフィールド名をキーとした numpy 配列の辞書を返す。
        各 id の 'ts' は秒単位（float, ts_us/1e6）。

    quat_to_yaw(quat)           -> np.ndarray
        [w,x,y,z] 配列(N,4) からヨー角 [rad] を計算（アンラップなし、atan2の主値）。

    unwrap_deg(psi_rad)         -> np.ndarray
        ラジアン角配列を連続化（unwrap）して度に変換する。

    detect_flight_segments(t_ctrl_ref, motor_duty, min_duration_s=2.0, merge_gap_s=1.0)
        -> list[(t0, t1)]
        ctrl_ref.motor_duty（4基平均）が閾値超の区間を飛行区間として検出。
        2秒未満の継続は除外し、1秒未満の途切れは前後の区間を結合する。

    mode_str(mode_int) -> str
        vehicle の flight mode 整数値を文字列名に変換（不明値はその数値文字列）。
"""

import json
import numpy as np

# FlightMode enum of firmware/vehicle (flight_state.hpp:74-84) — verified against source
# firmware/vehicle の FlightMode 列挙（flight_state.hpp:74-84 でソース照合済み）
# NOTE: 初版は1つズレた誤対応（0:MANUAL,...,3:ALT_HOLD）で、第1弾解析のモード表記を
#       誤らせた（例: mode=3 を ALT_HOLD と表示。正しくは POS_HOLD）。2026-07-13 修正。
_MODE_NAMES = {
    0: "ACRO",
    1: "STABILIZE",
    2: "ALT_HOLD",
    3: "POS_HOLD",
}


def mode_str(mode_int):
    """Flight mode 整数値を文字列名へ変換する。不明な値は 'MODE_<n>' を返す。"""
    try:
        m = int(mode_int)
    except (TypeError, ValueError):
        return str(mode_int)
    return _MODE_NAMES.get(m, f"MODE_{m}")


def load_jsonl(path):
    """
    JSONL ログファイルを読み込み、id ごとにフィールド名->numpy配列の辞書を返す。

    戻り値: { id: { field_name: np.ndarray, ... , 'ts': np.ndarray(秒) }, ... }
    'ts' は元の ts (マイクロ秒, 整数) を 1e6 で割った秒単位の float。
    元の生 ts (マイクロ秒, ラップ検査用) は 'ts_us_raw' に整数のまま保持する。
    """
    records_by_id = {}

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            rid = rec.get("id")
            if rid is None:
                continue
            records_by_id.setdefault(rid, []).append(rec)

    out = {}
    for rid, recs in records_by_id.items():
        # 全レコードに共通するフィールド名の和集合を取り、欠損はNaN/0で埋める
        field_names = set()
        for r in recs:
            field_names.update(r.keys())
        field_names.discard("id")

        n = len(recs)
        field_data = {}
        for fname in field_names:
            sample = None
            for r in recs:
                if fname in r:
                    sample = r[fname]
                    break
            if isinstance(sample, list):
                arr = np.full((n, len(sample)), np.nan, dtype=float)
                for i, r in enumerate(recs):
                    if fname in r:
                        arr[i, :] = r[fname]
            else:
                arr = np.full(n, np.nan, dtype=float)
                for i, r in enumerate(recs):
                    if fname in r:
                        arr[i] = r[fname]
            field_data[fname] = arr

        ts_us_raw = np.array([r["ts"] for r in recs], dtype=np.int64)
        field_data["ts_us_raw"] = ts_us_raw
        field_data["ts"] = ts_us_raw.astype(np.float64) / 1e6
        out[rid] = field_data

    return out


def quat_to_yaw(quat):
    """
    クォータニオン配列 [w,x,y,z] (shape (N,4)) からヨー角 [rad] を計算する。
    atan2 の主値（-pi, pi] を返す（アンラップは unwrap_deg で行う）。

    psi = atan2( 2*(w*z + x*y), 1 - 2*(y^2 + z^2) )
    """
    quat = np.asarray(quat, dtype=float)
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    psi = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return psi


def unwrap_deg(psi_rad):
    """ラジアン角配列を unwrap して連続化し、度単位に変換する。"""
    return np.degrees(np.unwrap(psi_rad))


def detect_flight_segments(t_ctrl_ref, motor_duty, min_duration_s=2.0,
                            merge_gap_s=1.0, duty_threshold=0.05):
    """
    ctrl_ref の motor_duty (shape (N,4)) からモータ平均デューティが
    duty_threshold を超える区間を飛行区間として検出する。

    - 1秒(merge_gap_s)未満の途切れ（デューティ低下）は前後の区間を結合する
    - 結合後、継続時間が min_duration_s 未満の区間は除外する

    戻り値: [(t0, t1), ...] 秒単位のタプルのリスト（時系列順）
    """
    t_ctrl_ref = np.asarray(t_ctrl_ref, dtype=float)
    motor_duty = np.asarray(motor_duty, dtype=float)
    duty_mean = np.nanmean(motor_duty, axis=1)

    active = duty_mean > duty_threshold
    if not np.any(active):
        return []

    # active な連続区間のインデックスを抽出
    idx = np.where(active)[0]
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))

    raw_segments = [(t_ctrl_ref[s], t_ctrl_ref[e]) for s, e in zip(starts, ends)]

    # gap が merge_gap_s 未満なら結合
    merged = [raw_segments[0]]
    for t0, t1 in raw_segments[1:]:
        prev_t0, prev_t1 = merged[-1]
        if t0 - prev_t1 < merge_gap_s:
            merged[-1] = (prev_t0, t1)
        else:
            merged.append((t0, t1))

    # 最小継続時間でフィルタ
    result = [(t0, t1) for t0, t1 in merged if (t1 - t0) >= min_duration_s]
    return result
