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
# resolve_bundle_path()
# =============================================================================

#: Suffixes tried, in order, once the argument as given does not itself
#: resolve to a bundle (see resolve_bundle_path()). Concatenated onto the
#: argument as plain strings, never via Path.with_suffix() -- that method
#: replaces an existing suffix instead of appending, which would mangle a
#: bare name that happens to contain a dot.
#: 引数そのままでは一式に解決しない場合に順に試す拡張子
#: （resolve_bundle_path() 参照）。引数へ単純な文字列連結で付け足す
#: （Path.with_suffix() は既存の拡張子を置き換えてしまうため使わない --
#: ドットを含む裸の名前を壊しかねない）。
_RESOLVE_SUFFIXES = (".sflog.zip", ".zip", ".sflog")


def resolve_bundle_path(arg, search_dirs=(), notify=None) -> Path:
    """Resolve a bundle argument that may omit its trailing extension.

    Every reader (`sf log check/info/viz/analyze`, `sf sysid fit/rate-fit/
    noise/motor/drag/inertia`, `sf trim analyze`, `sf cal plot`) accepts a
    bundle name with the extension left off, and -- for a bare name --
    also looks in the caller's extra search directories (typically the
    project's `logs/`, see `paths.logs()`), so e.g. `sf log viz
    flight_20260912T093015` works from any directory. Candidates are tried
    in this order; the FIRST one `is_bundle()` accepts wins:

      1. `arg` as given (a `.sflog.zip` file, or an extracted directory);
      2. `<arg>.sflog.zip`;
      3. `<arg>.zip`, then `<arg>.sflog`;
      4. steps 1-3 again, with `arg` joined onto each of `search_dirs` in
         turn (a no-op when `arg` is itself absolute, since joining a
         directory onto an absolute path just yields that absolute path
         back -- see `pathlib`'s `/` operator).

    No fuzzy matching -- only these exact names are ever tried.

    When step 1 resolves AND `<arg>.sflog.zip` is ALSO a bundle (e.g. a
    directory bundle shadowing an identically-stemmed zip), step 1's match
    still wins, but `notify` -- if given -- receives one message noting
    both exist, so a caller with a console can tell the user rather than
    resolve the ambiguity silently.

    Args:
        arg: the bundle argument as typed by the user (bare name, relative
            path, or absolute path).
        search_dirs: extra directories tried, in the given order, after
            `arg`'s own location.
        notify: optional `str -> None` callback for the "both exist"
            notice above. Kept as a callback (rather than importing a
            console module here) so `lib/sflog` stays free of printing
            code -- callers wire it to e.g. `console.info`.

    Returns:
        The resolved `Path`, exactly as one of the tried candidates (not
        further normalized/resolved).

    Raises:
        FileNotFoundError: no candidate is a bundle. The message lists
            every path tried, in the order above, so a CLI can print it
            directly (e.g. via `console.error`).

    拡張子を省略できるバンドル引数を解決する。

    一式を読み込む側（`sf log check/info/viz/analyze`、`sf sysid fit/
    rate-fit/noise/motor/drag/inertia`、`sf trim analyze`、`sf cal plot`）は
    全て、拡張子を省いたバンドル名を受け付ける。さらに裸の名前であれば
    呼び出し側の追加検索ディレクトリ（通常はプロジェクトの `logs/`、
    `paths.logs()` 参照）も探すため、例えば `sf log viz
    flight_20260912T093015` がどのディレクトリからでも動く。候補は次の順に
    試し、`is_bundle()` が最初に真を返したものが勝つ:

      1. `arg` そのまま（`.sflog.zip` ファイルまたは展開済みフォルダ）;
      2. `<arg>.sflog.zip`;
      3. `<arg>.zip`、次に `<arg>.sflog`;
      4. 1〜3 を、`arg` を `search_dirs` の各ディレクトリへ結合した上で
         繰り返す（`arg` が絶対パスなら、ディレクトリとの結合は絶対パス
         そのものを返すだけなので実質的に無効 -- pathlib の `/` 演算子の
         挙動参照）。

    あいまい一致はしない -- これらの厳密な名前だけを試す。

    ステップ1が解決し、かつ `<arg>.sflog.zip` も一式であるとき（例:
    同じ幹の zip を覆い隠すフォルダ一式）、ステップ1の一致を採用しつつ、
    `notify`（指定時）へ両方存在する旨のメッセージを1件渡す -- 解決を
    黙って行わず、コンソールを持つ呼び出し側が利用者に知らせられるように
    するため。

    Args (日本語):
        arg: 利用者が入力したバンドル引数（裸の名前・相対パス・絶対パス）。
        search_dirs: `arg` 自身の場所の後に、指定順で試す追加ディレクトリ。
        notify: 上記「両方存在」通知用の任意の `str -> None` コールバック。
            `lib/sflog` を表示コードから独立させるためコールバックとした
            （ここでコンソールモジュールを import しない）-- 呼び出し側が
            例えば `console.info` へ配線する。

    Returns:
        解決した `Path`（試した候補のいずれかそのまま。これ以上の正規化は
        しない）。

    Raises:
        FileNotFoundError: どの候補も一式でなかった場合。メッセージに
            試した全パスを上記の順で列挙するため、CLI 側はそのまま
            （例えば `console.error` で）表示できる。
    """
    arg_str = str(arg)
    arg_path = Path(arg_str)

    # A bare/relative arg is also tried under each search dir; an absolute
    # arg already fully specifies its own location, so joining a search
    # dir onto it would just reproduce the same path (pathlib's `/`
    # discards the left side when the right side is absolute) -- skipped
    # here purely to keep the "tried" list free of exact duplicates.
    # 裸/相対の引数は各検索ディレクトリでも試す。絶対パスの引数は既に
    # 自身の場所を完全に指定しているため、検索ディレクトリと結合しても
    # 同じパスに戻るだけ（pathlib の `/` は右辺が絶対パスなら左辺を捨てる）
    # -- 「試した」一覧に厳密な重複を残さないためだけにここで省く。
    roots = [arg_path]
    if not arg_path.is_absolute():
        roots.extend(Path(d) / arg_str for d in search_dirs)

    tried = []
    match = None
    match_root = None
    for root in roots:
        candidates = (root, *(Path(str(root) + suffix) for suffix in _RESOLVE_SUFFIXES))
        for candidate in candidates:
            tried.append(candidate)
            if is_bundle(candidate):
                match = candidate
                match_root = root
                break
        if match is not None:
            break

    if match is None:
        listing = "\n".join(f"  {p}" for p in tried)
        raise FileNotFoundError(
            f"No flight-log bundle found for '{arg_str}'. Tried:\n{listing}"
        )

    # The "both exist" notice only applies when the WINNING candidate is
    # the bare one (step 1 of whichever root won) -- that is the only case
    # where a same-stemmed .sflog.zip could be silently shadowed instead
    # of simply being step 2's own match.
    # 「両方存在」通知は、勝った候補がどのルートのステップ1（そのまま）で
    # あるときにだけ当てはまる -- 同じ幹の .sflog.zip が黙って隠れうるのは
    # その場合だけで、そうでなければそれは単にステップ2自身の一致である。
    if notify is not None and match == match_root:
        sibling = Path(str(match_root) + ".sflog.zip")
        if sibling != match and is_bundle(sibling):
            notify(f"Both '{match}' and '{sibling}' exist; using '{match}'.")

    return match


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
