"""
lib/sflog - StampFly flight-log v1 bundle: read, write, align, check, convert.
lib/sflog - StampFly フライトログ v1 一式: 読み書き・整列・検査・変換。

The on-disk format's Single Source of Truth is
`protocol/spec/flight_log.yaml`; `lib/sflog/schema.py` is generated from it
by `protocol/tools/gen_flight_log.py` (never hand-edited).

Dependencies: standard library, numpy, pandas, PyYAML only (see
docs/plans/flight-log-format-plan.md section 2.5).

ディスク上形式の正本は `protocol/spec/flight_log.yaml`。
`lib/sflog/schema.py` はそこから `protocol/tools/gen_flight_log.py` が
生成する（手編集しない）。

依存はいずれも標準ライブラリ・numpy・pandas・PyYAML のみ
（計画書 2.5節参照）。
"""

from .align import aligned
from .bundle import FlightLog, is_bundle, make_meta
from .check import Finding, check_bundle, is_ok
from .convert import aligned_to_csv, bundle_to_jsonl, jsonl_to_bundle

__version__ = "0.1.0"


def load(path) -> FlightLog:
    """Convenience alias for `FlightLog.load(path)`.
    `FlightLog.load(path)` の簡易エイリアス。
    """
    return FlightLog.load(path)


__all__ = [
    "FlightLog",
    "load",
    "is_bundle",
    "aligned",
    "check_bundle",
    "jsonl_to_bundle",
    "bundle_to_jsonl",
    "aligned_to_csv",
    "__version__",
    # not in the Phase 0 deliverable's required export list, but small and
    # useful enough to expose directly rather than forcing a submodule
    # import for such common needs.
    # Phase 0 成果物の必須エクスポート一覧には無いが、これだけのために
    # サブモジュール import を強いるほどでもない小さく便利な関数。
    "make_meta",
    "is_ok",
    "Finding",
]
