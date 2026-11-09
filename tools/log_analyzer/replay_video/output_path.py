"""
output_path.py - where `sf log video` writes its mp4
`sf log video` が mp4 を書く場所

Default: <videos_dir>/<log stem>_3d.mp4 (with --time-range S E:
<log stem>_3d_<S>-<E>s.mp4). An existing file is never overwritten: _2, _3, ...
is appended before ".mp4" unless force is set. An explicit --out wins over the
default name, and is refused when it exists unless force is set.
既定: <videos_dir>/<ログ名>_3d.mp4（--time-range S E のときは
<ログ名>_3d_<S>-<E>s.mp4）。既存のファイルは上書きせず、force が無ければ
".mp4" の前に _2, _3, ... を付ける。--out を明示すれば既定名より優先され、
既存ならば force が無い限り拒否する。
"""

from pathlib import Path
from typing import Optional, Sequence


def compact_number(x: float) -> str:
    """Compact decimal text: 50.0 -> "50", 12.5 -> "12.5".
    短い 10 進表記: 50.0 -> "50"、12.5 -> "12.5"。"""
    return format(round(float(x), 2), "g")


def default_video_name(stem: str, time_range: Optional[Sequence[float]] = None) -> str:
    """File name for the default output.
    既定の出力のファイル名。"""
    if time_range is None:
        return f"{stem}_3d.mp4"
    return f"{stem}_3d_{compact_number(time_range[0])}-{compact_number(time_range[1])}s.mp4"


def numbered_free_path(path: Path) -> Path:
    """`path` if it does not exist, else the first free <stem>_2.mp4, _3.mp4, ...
    `path` が無ければそれ、あれば最初に空いている <名前>_2.mp4, _3.mp4, ...。"""
    if not path.exists():
        return path
    n = 2
    while True:
        candidate = path.with_name(f"{path.stem}_{n}{path.suffix}")
        if not candidate.exists():
            return candidate
        n += 1


def resolve_output_path(
    stem: str,
    videos_dir: Path,
    out: Optional[str] = None,
    time_range: Optional[Sequence[float]] = None,
    force: bool = False,
) -> Path:
    """Final output path (see module doc). Raises FileExistsError for an
    existing explicit --out without force. Creates the parent directory.
    最終的な出力パス（モジュール説明を参照）。--out が既存で force が無ければ
    FileExistsError。親フォルダは作る。"""
    if out:
        path = Path(out).expanduser()
        if path.suffix == "":
            path = path.with_suffix(".mp4")
        if path.exists() and not force:
            raise FileExistsError(f"{path} already exists (use --force to overwrite)")
    else:
        path = Path(videos_dir) / default_video_name(stem, time_range)
        if not force:
            path = numbered_free_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.absolute()
