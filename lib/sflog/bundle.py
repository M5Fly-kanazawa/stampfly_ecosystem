"""
bundle.py - FlightLog bundle read/write for the StampFly flight-log v1 format.
bundle.py - StampFly フライトログ v1 形式の一式（バンドル）読み書き。

A "bundle" is either a `.sflog.zip` file or a directory with the same flat
layout: `meta.json`, `schema.json`, and one CSV per stream (see
protocol/spec/flight_log.yaml, the format's Single Source of Truth).
「一式（バンドル）」は `.sflog.zip` ファイルか、同じ平坦レイアウトの
フォルダ（`meta.json`・`schema.json`・ストリームごとの CSV）のどちらか
（形式の正本 protocol/spec/flight_log.yaml を参照）。

@design docs/plans/flight-log-format-plan.md section 2 (Phase 0)
"""

from __future__ import annotations

import io
import json
import subprocess
import zipfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import pandas as pd

from . import schema


# =============================================================================
# git hash helper (used by make_meta())
# git ハッシュ取得（make_meta() が使う）
# =============================================================================


def _find_repo_root(start: Path) -> Optional[Path]:
    """Walk upward from `start` looking for a `.git` directory.
    `start` から上位へ辿って `.git` ディレクトリを探す。
    """
    for parent in (start, *start.parents):
        if (parent / ".git").exists():
            return parent
    return None


def _tool_git_hash() -> Optional[str]:
    """Best-effort short git hash of the repository this package lives in.

    Returns None (never raises) when git is unavailable, the file tree is
    not a git checkout, or the command fails for any reason -- meta.json's
    `tool.git_hash` is informational, not load-bearing.
    このパッケージが属すリポジトリの短い git ハッシュを返す（取れなければ
    None、例外は投げない）。meta.json の `tool.git_hash` は参考情報であり、
    取得できなくても動作に支障はない。
    """
    repo_root = _find_repo_root(Path(__file__).resolve())
    if repo_root is None:
        return None
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=repo_root,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip() or None


# =============================================================================
# is_bundle()
# =============================================================================


def is_bundle(path) -> bool:
    """True if `path` looks like a v1 flight-log bundle (zip file or
    extracted directory) -- i.e. it has a `meta.json` whose `format` field
    equals `schema.FORMAT`. Never raises; any read error is treated as "not
    a bundle".
    `path` が v1 フライトログ一式（zip かフォルダ）に見えるか判定する --
    `meta.json` があり `format` フィールドが `schema.FORMAT` と一致すること。
    例外は投げず、読めなければ「一式ではない」とみなす。
    """
    path = Path(path)
    try:
        if path.is_dir():
            meta_path = path / "meta.json"
            if not meta_path.exists():
                return False
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        elif path.is_file():
            with zipfile.ZipFile(path) as zf:
                if "meta.json" not in zf.namelist():
                    return False
                meta = json.loads(zf.read("meta.json").decode("utf-8"))
        else:
            return False
    except (OSError, json.JSONDecodeError, zipfile.BadZipFile):
        return False
    return meta.get("format") == schema.FORMAT


# =============================================================================
# FlightLog
# =============================================================================


@dataclass
class FlightLog:
    """In-memory representation of one flight-log v1 bundle.
    フライトログ v1 一式のメモリ上表現。

    Attributes:
        meta: parsed meta.json (dict).
        schema: parsed schema.json (dict).
        streams: stream name -> pandas.DataFrame, e.g. streams["imu"].
            Only streams actually present in the bundle are keyed here --
            a missing stream is simply absent from the dict, never a
            DataFrame with all-empty rows.
        meta: パース済み meta.json。
        schema: パース済み schema.json。
        streams: ストリーム名 -> pandas.DataFrame（例: streams["imu"]）。
            バンドルに実在するストリームだけがキーとして存在する -- 無い
            ストリームは辞書に含まれない（空行だけの DataFrame にはしない）。
    """

    meta: dict = field(default_factory=dict)
    schema: dict = field(default_factory=dict)
    streams: dict = field(default_factory=dict)

    # ---- loading -----------------------------------------------------

    @classmethod
    def load(cls, path) -> "FlightLog":
        """Load a bundle from a `.sflog.zip` file or an extracted directory.

        Reads `meta.json` and `schema.json`, then every stream CSV listed
        in `schema.py`'s STREAMS that is actually present. Files not named
        after a known stream (e.g. SILS's `results.json`) are ignored;
        streams that are absent are tolerated (simply missing from
        `.streams`, not an error).
        `.sflog.zip` ファイルまたは展開済みフォルダから一式を読み込む。

        `meta.json`・`schema.json` を読み、`schema.py` の STREAMS に列挙
        された中で実際に存在するストリーム CSV を全て読む。既知のストリーム
        名に該当しないファイル（SILS の `results.json` 等）は無視する。
        無いストリームは許容する（`.streams` に単に含まれないだけでエラー
        にしない）。
        """
        path = Path(path)
        if path.is_dir():
            return cls._load_from_dir(path)
        return cls._load_from_zip(path)

    @classmethod
    def _load_from_dir(cls, path: Path) -> "FlightLog":
        names = {p.name for p in path.iterdir() if p.is_file()}
        meta = _read_json_file(path / "meta.json") if "meta.json" in names else {}
        schema_json = _read_json_file(path / "schema.json") if "schema.json" in names else {}
        streams = {}
        for stream_name, info in schema.STREAMS.items():
            file_name = info["file"]
            if file_name in names:
                streams[stream_name] = pd.read_csv(path / file_name)
        return cls(meta=meta, schema=schema_json, streams=streams)

    @classmethod
    def _load_from_zip(cls, path: Path) -> "FlightLog":
        with zipfile.ZipFile(path) as zf:
            names = set(zf.namelist())
            meta = json.loads(zf.read("meta.json").decode("utf-8")) if "meta.json" in names else {}
            schema_json = (
                json.loads(zf.read("schema.json").decode("utf-8")) if "schema.json" in names else {}
            )
            streams = {}
            for stream_name, info in schema.STREAMS.items():
                file_name = info["file"]
                if file_name in names:
                    streams[stream_name] = pd.read_csv(io.BytesIO(zf.read(file_name)))
        return cls(meta=meta, schema=schema_json, streams=streams)

    # ---- saving --------------------------------------------------------

    def save(self, path) -> None:
        """Write the bundle to `path`.

        A path ending in `.zip` (including the `.sflog.zip` convention,
        since `Path.suffix` reads the last extension) is written as a
        deflate zip with a flat member layout. Any other path is treated
        as a directory and populated with plain files.
        `path` へ一式を書き出す。`.zip` で終わるパス（`Path.suffix` は
        末尾の拡張子だけを見るため `.sflog.zip` も該当）は平坦なメンバー
        構成の deflate zip として書く。それ以外はディレクトリとして扱い、
        素のファイル群を書き込む。
        """
        path = Path(path)
        meta_bytes = json.dumps(self.meta, ensure_ascii=False, indent=2).encode("utf-8")
        schema_bytes = json.dumps(self.schema, ensure_ascii=False, indent=2).encode("utf-8")

        if path.suffix == ".zip":
            path.parent.mkdir(parents=True, exist_ok=True)
            with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zf:
                zf.writestr("meta.json", meta_bytes)
                zf.writestr("schema.json", schema_bytes)
                for name, df in self.streams.items():
                    zf.writestr(self._file_name_for(name), _dataframe_to_csv_text(df))
        else:
            path.mkdir(parents=True, exist_ok=True)
            (path / "meta.json").write_bytes(meta_bytes)
            (path / "schema.json").write_bytes(schema_bytes)
            for name, df in self.streams.items():
                (path / self._file_name_for(name)).write_text(
                    _dataframe_to_csv_text(df), encoding="utf-8"
                )

    @staticmethod
    def _file_name_for(stream_name: str) -> str:
        """CSV file name for a stream: schema.py's declared name, or
        "<name>.csv" for a stream this schema version does not know about
        (kept permissive rather than raising, per the container rule that
        unknown files are simply ignored by readers).
        ストリームの CSV ファイル名: schema.py 記載の名前、もしくは本
        スキーマ版が知らないストリームなら "<name>.csv"（未知ファイルは
        読み込み側が単に無視するという容器の規約に合わせ、例外にはしない）。
        """
        info = schema.STREAMS.get(stream_name)
        return info["file"] if info else f"{stream_name}.csv"


def _read_json_file(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _dataframe_to_csv_text(df: pd.DataFrame) -> str:
    """Render a stream DataFrame to CSV text per csv_rules (utf-8, header
    row, no index column, `%.7g` floats, integer `timestamp_us`).
    csv_rules に従い DataFrame を CSV テキストへ変換する（utf-8、ヘッダ行、
    index 列なし、`%.7g` の浮動小数点、整数の `timestamp_us`）。
    """
    if "timestamp_us" in df.columns:
        df = df.copy()
        df["timestamp_us"] = df["timestamp_us"].astype("int64")
    return df.to_csv(index=False, float_format="%.7g")


# =============================================================================
# make_meta()
# =============================================================================


def make_meta(
    source: str,
    tool_name: str,
    tool_version: str,
    capture: Optional[dict] = None,
    firmware: Optional[dict] = None,
    notes: Optional[str] = None,
    streams: Optional[dict] = None,
) -> dict:
    """Build a meta.json-compatible dict (protocol/spec/flight_log.yaml
    `meta_fields`).

    Args:
        source: "vehicle" | "sils" | "sim".
        tool_name: generating tool's name (e.g. "sf log convert").
        tool_version: generating tool's version string.
        capture: optional {"ip", "port", "duration_s"} for a vehicle
            capture session; None for SILS/sim.
        firmware: optional {"version", "git_hash"} when known.
        notes: optional free-form string.
        streams: stream name -> DataFrame, used to compute each stream's
            summary stats (rows, first/last timestamp_us, nominal_rate_hz
            from schema.py, measured_rate_hz derived from row count and
            the first/last timestamps).

    Returns:
        A dict ready to be written as meta.json (`derived` is always
        False here -- callers building a derived product such as an
        aligned table set that flag themselves, see convert.aligned_to_csv).

    meta.json 相当の dict を作る（protocol/spec/flight_log.yaml の
    `meta_fields` 参照）。引数の意味は英語側を参照。

    `derived` は常に False（整列表などの派生物を作る側は自分で立てる --
    convert.aligned_to_csv 参照）。
    """
    streams = streams or {}
    stream_stats = {name: _stream_stats(name, df) for name, df in streams.items()}

    return {
        "format": schema.FORMAT,
        "version": schema.VERSION,
        "source": source,
        "created_at": datetime.now().astimezone().isoformat(),
        "tool": {
            "name": tool_name,
            "version": tool_version,
            "git_hash": _tool_git_hash(),
        },
        "firmware": firmware,
        "capture": capture,
        "streams": stream_stats,
        "derived": False,
        "notes": notes,
    }


def _stream_stats(name: str, df: pd.DataFrame) -> dict:
    """One stream's summary-stats block for meta.json's `streams` field.
    meta.json の `streams` フィールド用に、1ストリーム分の要約統計を作る。
    """
    info = schema.STREAMS.get(name, {})
    nominal_hz = info.get("nominal_rate_hz")
    rows = len(df)

    first_ts = last_ts = measured_hz = None
    if rows > 0 and "timestamp_us" in df.columns:
        first_ts = int(df["timestamp_us"].iloc[0])
        last_ts = int(df["timestamp_us"].iloc[-1])
        duration_s = (last_ts - first_ts) / 1e6
        if rows > 1 and duration_s > 0:
            measured_hz = (rows - 1) / duration_s

    return {
        "rows": rows,
        "first_timestamp_us": first_ts,
        "last_timestamp_us": last_ts,
        "nominal_rate_hz": nominal_hz,
        "measured_rate_hz": measured_hz,
    }
