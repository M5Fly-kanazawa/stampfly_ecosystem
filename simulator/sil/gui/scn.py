# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
# Part of StampFly Ecosystem (SIL GUI backend).
"""Parse / generate SIL scenario (*.scn) files for the GUI scenario builder.

GUI のシナリオビルダー用に SIL シナリオ (*.scn) を双方向変換する。

The .scn grammar is owned by ``simulator/sil/devices/scenario.cpp`` (the parser the
firmware actually runs). This module mirrors it on the Python side so the builder can
LOAD an existing scenario into editable events and GENERATE a .scn back. The server is
the single owner of the format; the JS builder only edits the structured event list.

.scn 文法の実体は scenario.cpp（ファームが実際に走らせるパーサ）。本モジュールはそれを
Python 側で鏡写しにし、ビルダーが既存シナリオを編集可能イベントに読み込み・.scn に書き戻す。

Event dict shape (one per timeline row):
  {at, ch, comment, <channel-specific fields...>}
  at: time token — "0"/"5000" (absolute ms) or "+"/"+1000" (relative to prev event end)
"""

# Order of the optional trailing rc tokens (positional, so to set `pos` you must also
# emit `alt` and `acro`). rc の任意末尾トークンは位置依存。
_RC_TAIL = ["alt", "acro", "pos"]


def _num(tok, cast=float):
    try:
        return cast(tok)
    except (ValueError, TypeError):
        return cast(0)


def parse_scn(text: str):
    """Parse .scn text → list of event dicts. Unknown channels are kept as {ch:'raw'}."""
    events = []
    for line in text.splitlines():
        # split off the trailing comment (a '#' inside a quoted key string is rare; the
        # builder does not need to preserve those, so a plain split is fine here).
        comment = ""
        hidx = line.find("#")
        if hidx != -1:
            comment = line[hidx + 1:].strip()
            line = line[:hidx]
        toks = line.split()
        if not toks:
            continue
        at = toks[0]
        if len(toks) < 2:
            continue
        ch = toks[1]
        a = toks[2:]
        e = {"at": at, "ch": ch, "comment": comment}
        if ch == "rc":
            keys = ["thr", "roll", "pitch", "yaw", "arm", "hold_ms", "rate_hz",
                    "alt", "acro", "pos"]
            for i, k in enumerate(keys):
                e[k] = _num(a[i], int) if i < len(a) else (20 if k == "rate_hz" else 0)
        elif ch == "rc_ramp":
            for i, k in enumerate(["field", "from", "to", "step", "rate_hz", "arm",
                                   "alt", "acro"]):
                if k == "field":
                    e[k] = a[i] if i < len(a) else "throttle"
                else:
                    e[k] = _num(a[i], int) if i < len(a) else 0
        elif ch == "wind":
            for i, k in enumerate(["fx", "fy", "fz"]):
                e[k] = _num(a[i]) if i < len(a) else 0.0
            e["dur_ms"] = _num(a[3], int) if len(a) > 3 else 0
        elif ch == "fault":
            e["motor"] = _num(a[0], int) if a else 0
            e["gain"] = _num(a[1]) if len(a) > 1 else 1.0
        elif ch == "bias":
            for i, k in enumerate(["ax", "ay", "az", "gx", "gy", "gz"]):
                e[k] = _num(a[i]) if i < len(a) else 0.0
        elif ch == "handle":
            e["carry_alt"] = _num(a[0]) if a else 0.40
            e["px"] = _num(a[1]) if len(a) > 1 else 0.0
            e["py"] = _num(a[2]) if len(a) > 2 else 0.0
            for i, k in enumerate(["lift_ms", "carry_ms", "place_ms"], start=3):
                e[k] = _num(a[i], int) if len(a) > i else 1200
        elif ch == "key":
            # everything after 'key' is the quoted string (keep raw)
            e["text"] = " ".join(a)
        else:
            e["raw"] = " ".join(a)
        events.append(e)
    return events


def _rc_tokens(e):
    vals = ["thr", "roll", "pitch", "yaw", "arm", "hold_ms", "rate_hz"]
    out = [str(int(e.get(k, 0))) for k in vals]
    # Append trailing optionals only up to the highest one that is set (positional).
    last = -1
    for i, k in enumerate(_RC_TAIL):
        if int(e.get(k, 0)):
            last = i
    for i in range(last + 1):
        out.append(str(int(e.get(_RC_TAIL[i], 0))))
    return out


def _fmt_num(v):
    """Format a float without a trailing '.0' clutter when it is integral-ish."""
    f = float(v)
    return str(int(f)) if f == int(f) else repr(round(f, 6))


def generate_scn(events, header: str = "") -> str:
    """Generate .scn text from a list of event dicts."""
    lines = []
    if header:
        for hl in header.splitlines():
            lines.append("# " + hl if hl and not hl.startswith("#") else hl)
    for e in events:
        ch = e.get("ch", "rc")
        at = str(e.get("at", "+"))
        if ch == "rc":
            toks = _rc_tokens(e)
        elif ch == "rc_ramp":
            toks = [str(e.get("field", "throttle"))] + [
                str(int(e.get(k, 0))) for k in ["from", "to", "step", "rate_hz", "arm"]]
            last = -1
            for i, k in enumerate(["alt", "acro"]):
                if int(e.get(k, 0)):
                    last = i
            for i in range(last + 1):
                toks.append(str(int(e.get(["alt", "acro"][i], 0))))
        elif ch == "wind":
            toks = [_fmt_num(e.get(k, 0.0)) for k in ["fx", "fy", "fz"]]
            if int(e.get("dur_ms", 0)):
                toks.append(str(int(e["dur_ms"])))
        elif ch == "fault":
            toks = [str(int(e.get("motor", 0))), _fmt_num(e.get("gain", 1.0))]
        elif ch == "bias":
            toks = [_fmt_num(e.get(k, 0.0)) for k in ["ax", "ay", "az", "gx", "gy", "gz"]]
        elif ch == "handle":
            toks = [_fmt_num(e.get("carry_alt", 0.40)),
                    _fmt_num(e.get("px", 0.0)), _fmt_num(e.get("py", 0.0)),
                    str(int(e.get("lift_ms", 1200))),
                    str(int(e.get("carry_ms", 1200))),
                    str(int(e.get("place_ms", 1200)))]
        elif ch == "key":
            toks = [e.get("text", '""')]
        else:
            toks = [e.get("raw", "")]
        row = f"{at:<5} {ch:<6} " + " ".join(toks)
        if e.get("comment"):
            row += f"   # {e['comment']}"
        lines.append(row.rstrip())
    return "\n".join(lines) + "\n"


def estimate_duration_us(events) -> int:
    """Rough wall-clock the scenario occupies, for a sensible default --duration.

    rc/rc_ramp hold + handle phases + wind gust durations, plus a 3 s margin. Mirrors
    scenario.cpp's event_duration_us closely enough for a default (the user can override).
    シナリオが占める概算時間（既定 --duration 用）。3秒マージン込み。
    """
    total_ms = 0
    for e in events:
        ch = e.get("ch")
        # '+<ms>' relative gaps add to the timeline; absolute jumps are ignored here.
        at = str(e.get("at", "+"))
        if at.startswith("+") and len(at) > 1:
            total_ms += _num(at[1:], int)
        if ch == "rc":
            total_ms += max(int(e.get("hold_ms", 0)), 0)
        elif ch == "rc_ramp":
            span = abs(int(e.get("to", 0)) - int(e.get("from", 0)))
            step = max(abs(int(e.get("step", 1))), 1)
            rate = max(int(e.get("rate_hz", 20)), 1)
            total_ms += int((span / step + 1) * (1000 / rate))
        elif ch == "wind":
            total_ms += max(int(e.get("dur_ms", 0)), 0)
        elif ch == "handle":
            total_ms += (int(e.get("lift_ms", 1200)) + int(e.get("carry_ms", 1200))
                         + int(e.get("place_ms", 1200)))
    return (total_ms + 3000) * 1000  # +3 s margin, ms → us


if __name__ == "__main__":  # round-trip check on a real scenario / 実シナリオで往復確認
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[3]
    scn = (root / "simulator/sil/scenarios/modeswitch.scn").read_text()
    evs = parse_scn(scn)
    print(f"{len(evs)} events parsed", file=sys.stderr)
    print(generate_scn(evs))
