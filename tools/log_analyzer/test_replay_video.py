"""
test_replay_video.py - tests for `sf log video` (tools/log_analyzer/replay_video/)
`sf log video` の試験

Covers the data extractor (frame grid, window, quaternion continuity, altitude
sign, motor order, voltage hold), the default output-file naming and Chrome
discovery with a fake file system.
NOT covered here: the end-to-end recording (headless Chrome + ffmpeg). That is
a MANUAL check (`sf log video <log>`), not run in CI.
データ抽出（コマ格子・窓・四元数の連続性・高度の符号・モータの並び・電圧の保持）、
既定の出力ファイル名、偽のファイル構成での Chrome 探索を検証する。
ここでは録画全体（ヘッドレス Chrome + ffmpeg）は扱わない。それは手動確認
（`sf log video <ログ>`）であり、CI では実行しない。
"""

import math

import numpy as np
import pandas as pd
import pytest

import sflog
from replay_video import output_path, record, replay_data

T0_US = 5_000_000
RATE = 400
DURATION_S = 10
FLY_START_S, FLY_END_S = 3.0, 7.0


def _make_log():
    n = RATE * DURATION_S
    ts = T0_US + np.arange(n) * (1_000_000 // RATE)
    sec = np.arange(n) / RATE
    seq = np.arange(n)
    # Yaw turns 0.5 rad/s; every 40th sample carries the sign-flipped quaternion
    # (the same rotation), which a naive interpolation would break on.
    # ヨーは 0.5 rad/s で回る。40 サンプルごとに符号反転した（同じ回転の）四元数を
    # 入れ、素朴な補間だと壊れる状況にする。
    yaw = 0.5 * sec
    quat = np.stack([np.cos(yaw / 2), 0 * yaw, 0 * yaw, np.sin(yaw / 2)], axis=1)
    quat[::40] *= -1
    attitude = pd.DataFrame({
        "timestamp_us": ts, "seq": seq,
        "quat_w": quat[:, 0], "quat_x": quat[:, 1], "quat_y": quat[:, 2], "quat_z": quat[:, 3]})
    posvel = pd.DataFrame({
        "timestamp_us": ts, "seq": seq,
        "pos_x": 0.01 * sec, "pos_y": -0.02 * sec, "pos_z": -(0.2 + 0.05 * sec)})
    motor = pd.DataFrame({
        "timestamp_us": ts, "seq": seq,
        "duty_FR": 0.1, "duty_RR": 0.2, "duty_RL": 0.3, "duty_FL": 0.4})
    thrust = np.where((sec >= FLY_START_S) & (sec <= FLY_END_S), 0.4, 0.0)
    ctrl_output = pd.DataFrame({
        "timestamp_us": ts, "seq": seq, "thrust": thrust,
        "torque_roll": 0.0, "torque_pitch": 0.0, "torque_yaw": 0.0})
    ts50 = ts[::8]
    ctrl_ref = pd.DataFrame({
        "timestamp_us": ts50, "flight_mode": 3, "angle_ref_roll": 0.0, "angle_ref_pitch": 0.0,
        "total_thrust": 0.4, "duty_FR": 0.9, "duty_RR": 0.9, "duty_RL": 0.9, "duty_FL": 0.9,
        "alt_setpoint": 0.6, "alt_vel_target": 0.0, "climb_rate_cmd": 0.0,
        "pos_setpoint_x": 0.1, "pos_setpoint_y": 0.2})
    ts1 = ts[::RATE]
    volt = np.where(np.arange(len(ts1)) < 5, 4.0, 3.9)       # steps down at 5 s / 5 s で階段状に下がる
    status = pd.DataFrame({
        "timestamp_us": ts1, "voltage": volt, "flight_state": 5})
    streams = {"attitude": attitude, "posvel": posvel, "motor": motor,
               "ctrl_output": ctrl_output, "ctrl_ref": ctrl_ref, "status": status}
    return sflog.FlightLog(meta={}, schema={}, streams=streams)


@pytest.fixture(scope="module")
def data():
    return replay_data.build_replay_data(_make_log(), fps=30, speed=2.0, source_name="synthetic.sflog.zip")


def test_frame_grid_is_exact(data):
    meta = data["meta"]
    t = np.array(data["t"])
    expected = meta["start"] + np.arange(len(t)) * meta["speed"] / meta["fps"]
    assert np.allclose(t, expected, atol=1e-5)
    assert meta["n_frames"] == len(t)
    assert t[-1] <= meta["end"] + 1e-9


def test_default_window_is_thrust_segment_padded(data):
    meta = data["meta"]
    assert meta["start"] == pytest.approx(FLY_START_S - 1.0, abs=0.01)
    assert meta["end"] >= FLY_END_S + 1.0 - 2 * meta["speed"] / meta["fps"]
    assert meta["source"] == "synthetic.sflog.zip"


def test_time_range_overrides_window():
    d = replay_data.build_replay_data(_make_log(), fps=10, speed=1.0, time_range=(1.0, 2.0))
    assert d["meta"]["start"] == pytest.approx(1.0)
    assert d["t"][0] == pytest.approx(1.0) and d["t"][-1] == pytest.approx(2.0)


def test_quaternion_continuity_and_unit_norm(data):
    q = np.array(data["quat"])
    assert np.allclose(np.linalg.norm(q, axis=1), 1.0, atol=1e-5)
    assert np.all(np.einsum("ij,ij->i", q[1:], q[:-1]) > 0.99)
    # The yaw read back from the quaternion follows the 0.5 rad/s turn.
    # 四元数から読み戻したヨーは 0.5 rad/s の回転に従う。
    t = np.array(data["t"])
    wrapped = np.angle(np.exp(1j * (np.radians(data["yaw"]) - 0.5 * t)))   # difference modulo 2 pi / 2 pi を法とした差
    assert np.allclose(wrapped, 0.0, atol=1e-3)
    assert np.allclose(data["roll"], 0.0, atol=1e-3)


def test_altitude_is_minus_pos_z(data):
    pos, h, t = np.array(data["pos"]), np.array(data["h"]), np.array(data["t"])
    assert np.allclose(h, -pos[:, 2], atol=1e-5)
    assert np.allclose(h, 0.2 + 0.05 * t, atol=1e-4)
    assert np.allclose(pos[:, 0], 0.01 * t, atol=1e-4)      # N
    assert np.allclose(pos[:, 1], -0.02 * t, atol=1e-4)     # E


def test_motor_order_is_fr_rr_rl_fl_from_motor_stream(data):
    # `motor` (0.1, 0.2, 0.3, 0.4) wins over the ctrl_ref duties (0.9).
    # `motor` が ctrl_ref の duty (0.9) より優先される。
    assert np.allclose(np.array(data["motor"])[0], [0.1, 0.2, 0.3, 0.4])
    assert data["meta"]["motor_order"] == ["FR", "RR", "RL", "FL"]


def test_motor_falls_back_to_ctrl_ref():
    log = _make_log()
    del log.streams["motor"]
    d = replay_data.build_replay_data(log, fps=10, speed=1.0, time_range=(1.0, 2.0))
    assert np.allclose(np.array(d["motor"])[0], 0.9)


def test_voltage_is_held_not_interpolated(data):
    t, v = np.array(data["t"]), np.array(data["voltage"], dtype=float)
    assert set(np.unique(v)) <= {4.0, 3.9}
    assert np.all(v[t < 4.99] == 4.0) and np.all(v[t >= 5.0] == 3.9)


def test_voltage_is_none_before_first_sample_and_without_status():
    log = _make_log()
    log.streams["status"] = log.streams["status"].iloc[3:]
    d = replay_data.build_replay_data(log, fps=10, speed=1.0, time_range=(0.5, 1.0))
    assert d["voltage"][0] is None
    del log.streams["status"]
    d = replay_data.build_replay_data(log, fps=10, speed=1.0, time_range=(0.5, 1.0))
    assert all(v is None for v in d["voltage"])


def test_mode_state_and_setpoint(data):
    assert set(data["mode"]) == {3}
    assert set(data["flight_state"]) == {5}
    sp = data["setpoint"]
    assert sp["source"] == "ctrl_ref" and sp["ned"] == [0.1, 0.2] and sp["h"] == pytest.approx(0.6)


def test_setpoint_falls_back_to_takeoff_point():
    log = _make_log()
    log.streams["ctrl_ref"] = log.streams["ctrl_ref"].assign(flight_mode=1)
    d = replay_data.build_replay_data(log, fps=10, speed=1.0, time_range=(1.0, 2.0))
    assert d["setpoint"]["source"] == "takeoff_point"
    assert d["setpoint"]["ned"] == pytest.approx([0.01, -0.02], abs=1e-3)


def test_missing_pose_streams_raise():
    log = _make_log()
    del log.streams["attitude"]
    with pytest.raises(ValueError):
        replay_data.build_replay_data(log)


def test_hemisphere_continuous_flips_back():
    q = np.array([[1, 0, 0, 0], [-1, 0, 0, 0.01], [1, 0, 0, 0.02]], dtype=float)
    out = replay_data.hemisphere_continuous(q)
    assert np.all(np.einsum("ij,ij->i", out[1:], out[:-1]) > 0)


# ---------------------------------------------------------- output file name
def test_default_name_and_time_range_name():
    assert output_path.default_video_name("flight_x") == "flight_x_3d.mp4"
    assert output_path.default_video_name("flight_x", (50.0, 80.0)) == "flight_x_3d_50-80s.mp4"
    assert output_path.default_video_name("flight_x", (12.5, 30)) == "flight_x_3d_12.5-30s.mp4"


def test_existing_default_gets_numbered(tmp_path):
    videos = tmp_path / "videos"            # does not exist yet: it is created / まだ無い: 作られる
    first = output_path.resolve_output_path("flight_x", videos)
    assert first == (videos / "flight_x_3d.mp4").absolute() and videos.is_dir()
    first.write_bytes(b"x")
    second = output_path.resolve_output_path("flight_x", videos)
    assert second.name == "flight_x_3d_2.mp4"
    second.write_bytes(b"x")
    assert output_path.resolve_output_path("flight_x", videos).name == "flight_x_3d_3.mp4"
    # --force reuses the default name / --force は既定名をそのまま使う
    assert output_path.resolve_output_path("flight_x", videos, force=True).name == "flight_x_3d.mp4"


def test_explicit_out_is_not_overwritten_without_force(tmp_path):
    out = tmp_path / "mine.mp4"
    assert output_path.resolve_output_path("s", tmp_path, out=str(out)) == out.absolute()
    out.write_bytes(b"x")
    with pytest.raises(FileExistsError):
        output_path.resolve_output_path("s", tmp_path, out=str(out))
    assert output_path.resolve_output_path("s", tmp_path, out=str(out), force=True) == out.absolute()


# ----------------------------------------------------------- Chrome discovery
def test_chrome_env_override_wins():
    found = record.find_chrome(env={"CHROME_PATH": "/x/chrome"}, exists=lambda p: p == "/x/chrome",
                               which=lambda n: None, platform="linux")
    assert found == "/x/chrome"


def test_chrome_mac_app_path():
    app = record.MAC_APPS[0]
    assert record.find_chrome(env={}, exists=lambda p: p == app, which=lambda n: None, platform="darwin") == app


def test_chrome_windows_program_files_and_local_appdata():
    env = {"PROGRAMFILES": r"C:\Program Files", "LOCALAPPDATA": r"C:\Users\u\AppData\Local"}
    pf = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
    la = r"C:\Users\u\AppData\Local\Microsoft\Edge\Application\msedge.exe"
    assert record.find_chrome(env=env, exists=lambda p: p == pf, which=lambda n: None, platform="win32") == pf
    assert record.find_chrome(env=env, exists=lambda p: p == la, which=lambda n: None, platform="win32") == la


def test_chrome_linux_path_lookup_order():
    have = {"chromium": "/usr/bin/chromium", "chromium-browser": "/usr/bin/chromium-browser"}
    assert record.find_chrome(env={}, exists=lambda p: False, which=have.get, platform="linux") == "/usr/bin/chromium"


def test_chrome_absent_returns_none():
    assert record.find_chrome(env={}, exists=lambda p: False, which=lambda n: None, platform="linux") is None
    assert record.find_chrome(env={}, exists=lambda p: False, which=lambda n: None, platform="darwin") is None
