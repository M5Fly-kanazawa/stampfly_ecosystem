"""
test_resolve.py - bundle.resolve_bundle_path(): extension-omitted bundle
name resolution shared by every `sf log`/`sf sysid`/`sf trim`/`sf cal`
reader.
test_resolve.py - bundle.resolve_bundle_path() の試験: `sf log`/`sf sysid`/
`sf trim`/`sf cal` の読み込み側全てが共有する、拡張子省略バンドル名の
解決。
"""

from __future__ import annotations

import pandas as pd
import pytest

from sflog.bundle import FlightLog, make_meta, resolve_bundle_path


def _tiny_streams() -> dict:
    """The minimal schema-valid stream set ("imu" is the only required
    stream) -- enough to make `is_bundle()` accept the saved bundle.
    最小限のスキーマ有効なストリーム集合（必須ストリームは "imu" のみ）--
    保存した一式を `is_bundle()` が受け付けるのに十分な量。
    """
    return {
        "imu": pd.DataFrame(
            {
                "timestamp_us": [1000, 2500],
                "gyro_x": [0.0, 0.0], "gyro_y": [0.0, 0.0], "gyro_z": [0.0, 0.0],
                "accel_x": [0.0, 0.0], "accel_y": [0.0, 0.0], "accel_z": [-9.8, -9.8],
                "gyro_raw_x": [0.0, 0.0], "gyro_raw_y": [0.0, 0.0], "gyro_raw_z": [0.0, 0.0],
                "accel_raw_x": [0.0, 0.0], "accel_raw_y": [0.0, 0.0], "accel_raw_z": [-9.8, -9.8],
            }
        )
    }


def _save_bundle(path) -> None:
    """Write a tiny synthetic bundle to `path` (zip if it ends in .zip
    variants Path.suffix recognizes, a directory otherwise -- see
    FlightLog.save()).
    合成一式を `path` へ書く（`Path.suffix` が認識する .zip 系ならzip、
    それ以外はディレクトリ -- FlightLog.save() 参照）。
    """
    streams = _tiny_streams()
    meta = make_meta(source="sim", tool_name="test_resolve.py", tool_version="0", streams=streams)
    FlightLog(meta=meta, schema={}, streams=streams).save(path)


def test_bare_name_resolves_to_sflog_zip(tmp_path):
    """`<arg>` with no extension at all resolves to `<arg>.sflog.zip`
    (resolve_bundle_path() step 2).
    拡張子なしの `<arg>` は `<arg>.sflog.zip` に解決する（ステップ2）。
    """
    _save_bundle(tmp_path / "hover.sflog.zip")

    resolved = resolve_bundle_path(str(tmp_path / "hover"))

    assert resolved == tmp_path / "hover.sflog.zip"


def test_directory_bundle_resolves_as_given(tmp_path):
    """A bare name that is itself an extracted directory bundle resolves
    at step 1, with no extension guessing needed.
    それ自体が展開済みのフォルダ一式である裸の名前は、拡張子の推測なしに
    ステップ1で解決する。
    """
    _save_bundle(tmp_path / "hover_dir")

    resolved = resolve_bundle_path(str(tmp_path / "hover_dir"))

    assert resolved == tmp_path / "hover_dir"


def test_search_dir_fallback(tmp_path):
    """A bare name not found relative to the current directory is looked
    up inside each `search_dirs` entry (the CLI passes `paths.logs()`/
    `get_log_dir()` here).
    カレントディレクトリ相対で見つからない裸の名前は、`search_dirs` の
    各ディレクトリ内でも探す（CLI はここに `paths.logs()`/
    `get_log_dir()` を渡す）。
    """
    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    bare_name = "sflog_resolve_test_bare_name_9f3c1"
    _save_bundle(log_dir / f"{bare_name}.sflog.zip")

    # A name unlikely to exist relative to the actual working directory,
    # so step 1 (without search_dirs) is exercised as a genuine miss.
    # 実際のカレントディレクトリ相対にはまず存在しない名前にし、
    # 検索ディレクトリ無しのステップ1が本当に外れることを確認する。
    resolved = resolve_bundle_path(bare_name, search_dirs=(log_dir,))

    assert resolved == log_dir / f"{bare_name}.sflog.zip"


def test_both_exist_notice(tmp_path):
    """Step 1 (a directory bundle) wins over a same-stemmed `.sflog.zip`,
    but `notify` is told both exist.
    ステップ1（フォルダ一式）が同じ幹の `.sflog.zip` に優先するが、
    `notify` へは両方存在する旨が伝わる。
    """
    _save_bundle(tmp_path / "dup")
    _save_bundle(tmp_path / "dup.sflog.zip")

    messages = []
    resolved = resolve_bundle_path(str(tmp_path / "dup"), notify=messages.append)

    assert resolved == tmp_path / "dup"
    assert len(messages) == 1
    assert str(tmp_path / "dup") in messages[0]
    assert str(tmp_path / "dup.sflog.zip") in messages[0]


def test_non_bundle_file_skipped_for_sflog_zip_sibling(tmp_path):
    """A plain (non-bundle) file at the bare name is skipped in favour of
    `<arg>.sflog.zip` -- `is_bundle()`, not mere existence, gates step 1.
    裸の名前にある一式でない普通のファイルは無視し、`<arg>.sflog.zip` を
    採用する -- ステップ1の合否を決めるのは存在ではなく `is_bundle()`。
    """
    (tmp_path / "x").write_text("not a bundle", encoding="utf-8")
    _save_bundle(tmp_path / "x.sflog.zip")

    resolved = resolve_bundle_path(str(tmp_path / "x"))

    assert resolved == tmp_path / "x.sflog.zip"


def test_not_found_lists_every_candidate_tried(tmp_path):
    """No candidate resolving raises FileNotFoundError whose message lists
    every path tried (arg as given, plus the 3 suffixed forms).
    どの候補も解決しなければ FileNotFoundError を送出し、メッセージに
    試した全パス（引数そのまま + 3通りの拡張子付き）を列挙する。
    """
    missing = tmp_path / "nope"

    with pytest.raises(FileNotFoundError) as exc_info:
        resolve_bundle_path(str(missing))

    message = str(exc_info.value)
    assert str(missing) in message
    assert str(missing) + ".sflog.zip" in message
    assert str(missing) + ".zip" in message
    assert str(missing) + ".sflog" in message
