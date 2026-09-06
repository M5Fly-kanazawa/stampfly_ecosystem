# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
# Part of StampFly Ecosystem (SILS GUI backend).
"""Parse the firmware parameter table (the SSOT) into metadata for the GUI.

ファームのパラメータテーブル（SSOT）を GUI 用メタデータにパースする。

The single source of truth for parameters is the hand-written ``table[]`` in
``firmware/vehicle/components/sf_core/params.cpp`` (see auto-memory
``reference_params_ssot``). Each row looks like::

    {"rate.roll.kp", ParamType::FLOAT, &rate_roll_kp, 1.83e-4f, 0.0f, 0.01f, nullptr},
    { name           type              ptr            default    min    max   cb }

We regex these rows so the GUI's parameter panel always reflects the firmware
without a second list to keep in sync. 二重管理を避けるため正規表現で読む。
"""

import re
from pathlib import Path

# {"name", ParamType::TYPE, &var, default, min, max, ...}
# Numbers may be C float literals (1.83e-4f, 0.0f, 800.0f).
# 数値は C の float リテラル（末尾 f、指数表記）を許す。
_ROW = re.compile(
    r'\{\s*"([^"]+)"\s*,\s*ParamType::(FLOAT|BOOL|INT)\s*,\s*&\w+\s*,'
    r'\s*([-\d.eE+]+)f?\s*,\s*([-\d.eE+]+)f?\s*,\s*([-\d.eE+]+)f?\s*,'
)

# Human-friendly group titles by name prefix (display order matters).
# 名前プレフィックスごとの表示グループ（表示順）。
GROUPS = [
    ("rate",        "レート制御 / Rate"),
    ("attitude",    "姿勢制御 / Attitude"),
    ("altitude",    "高度制御 / Altitude"),
    ("position",    "位置制御 / Position"),
    ("eskf",        "推定器 ESKF / Estimator"),
    ("safety",      "安全 / Safety"),
    ("calibration", "校正 / Calibration"),
    ("estimator",   "推定器選択 / Estimator select"),
]


def _to_float(tok: str) -> float:
    """Parse a C float literal token ('1.83e-4f', '0.0f', '800.0f')."""
    return float(tok.rstrip("fF"))


def parse_params(params_cpp: Path):
    """Return a list of param dicts {name, type, default, min, max, group}.

    params.cpp の table[] を読み、各 param を辞書で返す。見つからなければ空リスト。
    """
    text = Path(params_cpp).read_text(encoding="utf-8")
    out = []
    for m in _ROW.finditer(text):
        name, ptype, dflt, lo, hi = m.groups()
        group = next((title for pfx, title in GROUPS if name.startswith(pfx)),
                     "その他 / Other")
        out.append({
            "name": name,
            "type": ptype,                  # FLOAT | BOOL | INT
            "default": _to_float(dflt),
            "min": _to_float(lo),
            "max": _to_float(hi),
            "group": group,
        })
    return out


if __name__ == "__main__":  # quick manual check / 手動確認
    import json
    import sys
    root = Path(__file__).resolve().parents[3]
    p = root / "firmware/vehicle/components/sf_core/params.cpp"
    params = parse_params(p)
    print(f"{len(params)} params", file=sys.stderr)
    print(json.dumps(params[:5], indent=2, ensure_ascii=False))
