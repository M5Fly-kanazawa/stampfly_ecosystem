"""
test_visualize_interactive.py - unit tests for the flight-log v1 bundle
dashboard backend (visualize_interactive.py, the backend of `sf log viz -i`).
test_visualize_interactive.py - フライトログ v1 一式ダッシュボード
バックエンド（visualize_interactive.py、`sf log viz -i` のバックエンド）の
単体テスト。

Three things are checked:
  (a) load_bundle() flattens a full-stream synthetic bundle correctly,
      including the explicit signal->time-axis map (`_signal_axis`) that
      keeps same-rate streams (pilot / ctrl_ref, both 50 Hz) from being
      confused by length-matching alone.
  (b) generate_html() carries that same explicit map into the page's
      SIGNAL_TIME_MAP JS constant.
  (c) the real-vehicle reference bundle (which lacks motor/ctrl_output/
      tof_front -- see analysis/datasets/flightlog/README.md) still loads
      and visualize() produces a non-trivial HTML file end to end.

3点を検証する:
  (a) load_bundle() が全ストリームを持つ合成一式を正しく平坦化すること。
      同じレートのストリーム（pilot と ctrl_ref、どちらも50Hz）を長さ一致
      だけで取り違えないようにする、明示的な信号→時間軸マップ
      （`_signal_axis`）を含めて確認する。
  (b) generate_html() がその同じ明示マップをページの SIGNAL_TIME_MAP JS
      定数へ引き継ぐこと。
  (c) 実機由来の基準一式（motor/ctrl_output/tof_front を持たない --
      analysis/datasets/flightlog/README.md 参照）が読み込め、
      visualize() が中身のある HTML を最後まで生成できること。
"""

import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import conftest  # noqa: E402  (provides REFERENCE_BUNDLE path constant)
import visualize_interactive  # noqa: E402


def test_load_bundle_flattens_synthetic_bundle(synthetic_bundle):
    """load_bundle() on the full-stream synthetic bundle exposes every
    signal family (raw stream columns + derived signals) under its
    documented key, with the correct per-stream time axis.
    合成一式（全ストリーム）に対する load_bundle() が、各信号ファミリー
    （ストリーム列そのもの + 派生信号）を規定どおりのキーで、正しい
    ストリーム別時間軸付きで公開すること。
    """
    data = visualize_interactive.load_bundle(synthetic_bundle)

    assert len(data['_time_imu']) == 2000
    assert 'time_s' in data
    assert data['time_s'] == data['_time_imu']

    for key in (
        'attitude_roll_deg', 'duty_FR', 'ctrl_thrust', 'pilot_throttle',
        'ctrl_ref_duty_FR', 'baro_pressure', 'imu_interval_us', 'total_duty',
    ):
        assert key in data, f"missing signal key: {key}"

    # pilot and ctrl_ref are both nominally 50 Hz (same row count in the
    # synthetic bundle) -- the explicit _signal_axis map must still keep
    # them apart, which length-matching alone could not do.
    # pilot と ctrl_ref はどちらも公称50Hz（合成一式では行数も同じ）--
    # 明示的な _signal_axis マップはそれでも両者を区別できなければならない
    # （長さ一致だけでは不可能）。
    axis = data['_signal_axis']
    assert axis['pilot_throttle'] == '_time_pilot'
    assert axis['ctrl_ref_duty_FR'] == '_time_ctrl_ref'


def test_generate_html_carries_explicit_signal_time_map(synthetic_bundle):
    """generate_html() must embed the SIGNAL_TIME_MAP JS constant, and the
    embedded map must keep pilot_throttle on pilot's own time axis (not
    ctrl_ref's, despite the matching row count).
    generate_html() は SIGNAL_TIME_MAP という JS 定数を埋め込み、その
    マップは pilot_throttle を（行数が一致する ctrl_ref ではなく）pilot
    自身の時間軸に対応付けていなければならない。
    """
    data = visualize_interactive.load_bundle(synthetic_bundle)
    html = visualize_interactive.generate_html(data, 'x', plotly_js='')

    assert 'SIGNAL_TIME_MAP' in html

    # Extract the embedded JSON object robustly (don't depend on
    # json.dumps' exact separator spacing) by matching the assignment
    # statement and parsing its right-hand side.
    # 埋め込まれた JSON オブジェクトを堅牢に取り出す（json.dumps の区切り
    # 文字の空白幅に依存しない）。代入文を見つけてその右辺をパースする。
    m = re.search(r'const SIGNAL_TIME_MAP = (\{.*?\});', html, re.DOTALL)
    assert m, "SIGNAL_TIME_MAP assignment not found in generated HTML"
    signal_time_map = json.loads(m.group(1))

    assert signal_time_map['pilot_throttle']['time'] == '_time_pilot'
    assert signal_time_map['ctrl_ref_duty_FR']['time'] == '_time_ctrl_ref'


def test_reference_bundle_loads_and_visualizes(reference_bundle, tmp_path):
    """The real-vehicle reference bundle (no motor/ctrl_output/tof_front
    streams) loads without error, exposes the 50 Hz ctrl_ref duty echo
    instead of the (absent) 400 Hz motor duty, and visualize() writes a
    substantial end-to-end HTML dashboard.
    実機由来の基準一式（motor/ctrl_output/tof_front ストリームを持たない）
    がエラーなく読み込め、（存在しない）400Hz モータ duty の代わりに 50Hz
    の ctrl_ref duty エコーを公開し、visualize() が中身のある HTML
    ダッシュボードを最後まで生成できること。
    """
    data = visualize_interactive.load_bundle(reference_bundle)

    assert 'duty_FR' not in data
    assert 'ctrl_ref_duty_FR' in data

    output = tmp_path / 'reference.html'
    vendor_plotly = visualize_interactive.PLOTLY_CACHE_FILE

    # `get_plotly_js()` reads tools/log_analyzer/vendor/plotly.min.js when
    # present (offline, ~4.5 MB) or else hits the network -- monkeypatch it
    # away and lower the size bound when the vendor cache is missing so
    # this test stays offline-safe.
    # `get_plotly_js()` は tools/log_analyzer/vendor/plotly.min.js があれば
    # それを読む（オフライン、約4.5MB）。無ければネットワークへ出るため、
    # キャッシュが無い場合はモンキーパッチしてサイズ下限を下げ、このテスト
    # をオフラインでも安全に保つ。
    if vendor_plotly.exists():
        min_size = 100_000
        visualize_interactive.visualize(str(reference_bundle), output=str(output))
    else:
        min_size = 10_000
        original = visualize_interactive.get_plotly_js
        visualize_interactive.get_plotly_js = lambda: ''
        try:
            visualize_interactive.visualize(str(reference_bundle), output=str(output))
        finally:
            visualize_interactive.get_plotly_js = original

    assert output.stat().st_size > min_size
