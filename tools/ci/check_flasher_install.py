#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/ci/check_flasher_install.py

Dependency-free automated test for lib/sfcli/utils/flasher_install's
pure logic (no network, no touching a real install location).
lib/sfcli/utils/flasher_install のうち依存の無い純粋なロジックを検証する
（ネットワークアクセス無し、実インストール先には一切触れない）。

Covers / 対象:
    1. detect_platform_key() for all four supported platforms, plus its
       error path for unsupported OS/architecture.
       detect_platform_key() の対応4プラットフォーム全分岐 + 非対応
       OS/アーキテクチャでのエラー経路。
    2. select_assets() asset-name matching, using the real asset names
       from release v2026.07.1 plus a hypothetical post-Linux-support
       release, including the "asset missing" error message.
       select_assets() の資産名マッチング。実在の v2026.07.1 資産名と、
       Linux対応後を想定した仮想リリースの両方、および「資産が無い」
       エラーメッセージを検証する。
    3. SHA256 verification via download_and_verify(), using file:// URLs
       so no network access is needed; both the matching and mismatching
       checksum cases, and that no partial files are left behind on
       failure.
       download_and_verify() 経由の SHA256 検証。file:// URL を使い
       ネットワーク不要にする。一致/不一致の両ケースと、失敗時に部分
       ファイルを残さないことを検証する。
    4. Install manifest write/read round-trip (the data that backends
       use to enumerate uninstall targets).
       インストールマニフェストの書き込み/読み出しの往復
       （バックエンドがアンインストール対象を列挙する際に使うデータ）。

Usage / 使い方:
    python3 tools/ci/check_flasher_install.py

Exit code 0 = all checks passed, 1 = at least one failed (a summary of
failures is printed either way; no external test framework required).
終了コード 0=全チェック合格、1=1つ以上失敗（結果に関わらずサマリを
表示する。外部テストフレームワーク不要）。
"""

import hashlib
import sys
import tempfile
from pathlib import Path
from unittest import mock

# Make lib/ importable without installing the package (this script is
# meant to run standalone in CI, before/without `pip install -e .`).
# lib/ をパッケージ未インストールでもimport可能にする(このスクリプトは
# `pip install -e .` 無しでもCI単体で動くことを意図している)。
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "lib"))

from sfcli.utils import flasher_install  # noqa: E402  (see sys.path.insert above)


# ---------------------------------------------------------------------------
# Test data: real asset names copied from the v2026.07.1 GitHub release
# (verified via `gh release view v2026.07.1 --json assets`), plus a
# hypothetical future release with a Linux asset added.
# テストデータ: v2026.07.1 の実在資産名(`gh release view v2026.07.1
# --json assets` で確認済み)と、Linux資産を追加した仮想の将来リリース。
# ---------------------------------------------------------------------------

V2026_07_1_ASSETS = [
    {"name": "SHA256SUMS.txt", "browser_download_url": "https://example.invalid/SHA256SUMS.txt"},
    {
        "name": "StampFlyFlasher_v2026.07.1_macos-arm64.zip",
        "browser_download_url": "https://example.invalid/macos-arm64.zip",
    },
    {
        "name": "StampFlyFlasher_v2026.07.1_macos-x64.zip",
        "browser_download_url": "https://example.invalid/macos-x64.zip",
    },
    {
        "name": "StampFlyFlasher_v2026.07.1_windows-x64.exe",
        "browser_download_url": "https://example.invalid/windows-x64.exe",
    },
    {"name": "stampfly_controller_v2026.07.1_app.bin", "browser_download_url": "https://example.invalid/a"},
    {"name": "stampfly_controller_v2026.07.1_full.bin", "browser_download_url": "https://example.invalid/b"},
    {"name": "stampfly_vehicle_v2026.07.1_app.bin", "browser_download_url": "https://example.invalid/c"},
    {"name": "stampfly_vehicle_v2026.07.1_full.bin", "browser_download_url": "https://example.invalid/d"},
]
V2026_07_1_RELEASE = {"tag_name": "v2026.07.1", "assets": V2026_07_1_ASSETS}

# A later, hypothetical release that adds the Linux asset (per spec 4-4,
# release.yml's Linux packaging step publishes a bare, extension-less
# binary named "..._linux-x64").
# Linux資産を追加した将来の仮想リリース(仕様4-4のとおり、release.yml の
# Linuxパッケージングステップは拡張子無しの "..._linux-x64" を公開する)。
V2026_07_2_ASSETS = [
    {"name": "SHA256SUMS.txt", "browser_download_url": "https://example.invalid/SHA256SUMS.txt"},
    {
        "name": "StampFlyFlasher_v2026.07.2_macos-arm64.zip",
        "browser_download_url": "https://example.invalid/macos-arm64.zip",
    },
    {
        "name": "StampFlyFlasher_v2026.07.2_macos-x64.zip",
        "browser_download_url": "https://example.invalid/macos-x64.zip",
    },
    {
        "name": "StampFlyFlasher_v2026.07.2_windows-x64.exe",
        "browser_download_url": "https://example.invalid/windows-x64.exe",
    },
    {
        "name": "StampFlyFlasher_v2026.07.2_linux-x64",
        "browser_download_url": "https://example.invalid/linux-x64",
    },
]
V2026_07_2_RELEASE = {"tag_name": "v2026.07.2", "assets": V2026_07_2_ASSETS}


# ---------------------------------------------------------------------------
# Checks
# 各チェック
# ---------------------------------------------------------------------------


def check_detect_platform_key() -> None:
    """detect_platform_key() for every supported (sys.platform, machine)
    pair, plus its error path for unsupported combinations."""
    supported_cases = [
        ("win32", "AMD64", "windows-x64"),
        ("win32", "x86_64", "windows-x64"),
        ("darwin", "arm64", "macos-arm64"),
        ("darwin", "aarch64", "macos-arm64"),
        ("darwin", "x86_64", "macos-x64"),
        ("darwin", "amd64", "macos-x64"),
        ("linux", "x86_64", "linux-x64"),
        ("linux", "amd64", "linux-x64"),
    ]
    for sys_platform, machine, expected in supported_cases:
        with mock.patch.object(flasher_install.sys, "platform", sys_platform), mock.patch.object(
            flasher_install.platform, "machine", return_value=machine
        ):
            actual = flasher_install.detect_platform_key()
            assert actual == expected, (
                f"detect_platform_key() for ({sys_platform!r}, {machine!r}): "
                f"expected {expected!r}, got {actual!r}"
            )

    unsupported_cases = [
        ("darwin", "i386"),  # 32-bit Intel Mac, unsupported
        ("linux", "armv7l"),  # 32-bit ARM Linux, unsupported
        ("freebsd12", "x86_64"),  # unsupported OS entirely
    ]
    for sys_platform, machine in unsupported_cases:
        with mock.patch.object(flasher_install.sys, "platform", sys_platform), mock.patch.object(
            flasher_install.platform, "machine", return_value=machine
        ):
            try:
                flasher_install.detect_platform_key()
            except flasher_install.FlasherInstallError:
                pass
            else:
                raise AssertionError(
                    f"detect_platform_key() should have raised for ({sys_platform!r}, {machine!r})"
                )


def check_select_assets() -> None:
    """select_assets() picks the right asset per platform key and raises
    a specific, actionable error when one is missing."""
    expected_names = {
        "windows-x64": "StampFlyFlasher_v2026.07.1_windows-x64.exe",
        "macos-arm64": "StampFlyFlasher_v2026.07.1_macos-arm64.zip",
        "macos-x64": "StampFlyFlasher_v2026.07.1_macos-x64.zip",
    }
    for platform_key, expected_name in expected_names.items():
        asset, sums_asset = flasher_install.select_assets(V2026_07_1_RELEASE, platform_key)
        assert asset["name"] == expected_name, (
            f"select_assets(v2026.07.1, {platform_key!r}): "
            f"expected {expected_name!r}, got {asset['name']!r}"
        )
        assert sums_asset["name"] == flasher_install.SHA256SUMS_ASSET_NAME

    # v2026.07.1 predates Linux support: must raise with a hint mentioning
    # the release that adds it, not a bare "not found".
    # v2026.07.1 はLinux対応前: 単なる「見つからない」ではなく、対応する
    # リリースを示すヒント付きで例外を出すこと。
    try:
        flasher_install.select_assets(V2026_07_1_RELEASE, "linux-x64")
    except flasher_install.FlasherInstallError as exc:
        assert flasher_install.FIRST_RELEASE_WITH_LINUX_ASSET in str(exc), (
            f"linux-x64 error message should mention "
            f"{flasher_install.FIRST_RELEASE_WITH_LINUX_ASSET!r}: {exc}"
        )
    else:
        raise AssertionError("select_assets(v2026.07.1, 'linux-x64') should have raised")

    # v2026.07.2 (hypothetical) has all four platforms, including a Linux
    # asset with no file extension.
    # v2026.07.2(仮想)は拡張子無しのLinux資産を含む全4プラットフォームを
    # 持つ。
    linux_asset, linux_sums = flasher_install.select_assets(V2026_07_2_RELEASE, "linux-x64")
    assert linux_asset["name"] == "StampFlyFlasher_v2026.07.2_linux-x64", linux_asset["name"]
    assert linux_sums["name"] == flasher_install.SHA256SUMS_ASSET_NAME

    for platform_key in ("windows-x64", "macos-arm64", "macos-x64"):
        asset, _ = flasher_install.select_assets(V2026_07_2_RELEASE, platform_key)
        assert asset["name"].endswith(f"_{platform_key}" + (
            ".exe" if platform_key == "windows-x64" else ".zip"
        )), asset["name"]

    # Missing SHA256SUMS.txt entirely: also a FlasherInstallError.
    # SHA256SUMS.txt 自体が無い場合も FlasherInstallError。
    release_without_sums = {
        "tag_name": "v9.9.9",
        "assets": [a for a in V2026_07_1_ASSETS if a["name"] != "SHA256SUMS.txt"],
    }
    try:
        flasher_install.select_assets(release_without_sums, "windows-x64")
    except flasher_install.FlasherInstallError:
        pass
    else:
        raise AssertionError("select_assets() should raise when SHA256SUMS.txt is missing")


def check_download_and_verify_sha256() -> None:
    """download_and_verify() accepts a matching digest and rejects a
    mismatching one, cleaning up partial files either way. Uses file://
    URLs so this needs no real network access."""
    with tempfile.TemporaryDirectory(prefix="check_flasher_install_src_") as src_dir_str, \
            tempfile.TemporaryDirectory(prefix="check_flasher_install_dst_") as dst_dir_str:
        src_dir = Path(src_dir_str)
        dst_dir = Path(dst_dir_str)

        firmware_name = "StampFlyFlasher_vtest_macos-arm64.zip"
        firmware_content = b"fake-frozen-app-bytes-for-testing"
        firmware_path = src_dir / firmware_name
        firmware_path.write_bytes(firmware_content)
        correct_digest = hashlib.sha256(firmware_content).hexdigest()

        asset = {"name": firmware_name, "browser_download_url": firmware_path.as_uri()}

        # --- matching digest: succeeds, returns the downloaded path ---
        good_sums_path = src_dir / "SHA256SUMS_good.txt"
        good_sums_path.write_text(f"{correct_digest}  {firmware_name}\n", encoding="utf-8")
        good_sums_asset = {"name": "SHA256SUMS.txt", "browser_download_url": good_sums_path.as_uri()}

        result_path = flasher_install.download_and_verify(asset, good_sums_asset, dst_dir / "ok")
        assert result_path.exists(), "verified download should exist"
        assert result_path.read_bytes() == firmware_content

        # --- mismatching digest: raises, leaves no partial files ---
        wrong_digest = "0" * 64
        bad_sums_path = src_dir / "SHA256SUMS_bad.txt"
        bad_sums_path.write_text(f"{wrong_digest}  {firmware_name}\n", encoding="utf-8")
        bad_sums_asset = {"name": "SHA256SUMS.txt", "browser_download_url": bad_sums_path.as_uri()}

        mismatch_dest = dst_dir / "mismatch"
        try:
            flasher_install.download_and_verify(asset, bad_sums_asset, mismatch_dest)
        except flasher_install.FlasherInstallError:
            pass
        else:
            raise AssertionError("download_and_verify() should reject a mismatching checksum")

        leftover = list(mismatch_dest.iterdir()) if mismatch_dest.exists() else []
        assert not leftover, f"partial files left behind after a failed verify: {leftover}"

        # --- checksum entry missing for this filename entirely ---
        empty_sums_path = src_dir / "SHA256SUMS_empty.txt"
        empty_sums_path.write_text("", encoding="utf-8")
        empty_sums_asset = {"name": "SHA256SUMS.txt", "browser_download_url": empty_sums_path.as_uri()}
        try:
            flasher_install.download_and_verify(asset, empty_sums_asset, dst_dir / "no_entry")
        except flasher_install.FlasherInstallError:
            pass
        else:
            raise AssertionError("download_and_verify() should reject a missing checksum entry")


def check_manifest_roundtrip() -> None:
    """The install manifest (version + files list) survives a
    write/read round-trip exactly -- this is the data backends use to
    enumerate uninstall targets (see backend contract in __init__.py)."""
    with tempfile.TemporaryDirectory(prefix="check_flasher_install_manifest_") as tmp_dir_str:
        manifest_dir = Path(tmp_dir_str) / "nested" / "install_dir"
        manifest_path = manifest_dir / flasher_install.MANIFEST_FILENAME

        version = "v2026.07.9"
        files = [
            str(manifest_dir / "StampFlyFlasher.app"),
            str(manifest_dir / "shortcut.lnk"),
        ]

        # Before writing, there is nothing to read (this is the state
        # uninstall() falls back from -- "not installed").
        # 書き込み前は読み出せるものが無い(uninstall()がこの状態から
        # 「未導入」としてフォールバックする)。
        assert flasher_install._read_manifest(manifest_path) is None

        flasher_install._write_manifest(manifest_path, version, files)
        assert manifest_path.exists(), "manifest file should have been created"

        loaded = flasher_install._read_manifest(manifest_path)
        assert loaded is not None, "manifest should be readable immediately after writing"
        assert loaded["version"] == version
        assert loaded["files"] == files, "uninstall target enumeration relies on this exact list"
        assert "installed_at" in loaded and loaded["installed_at"], "installed_at must be recorded"

        # Corrupt manifest -> treated as absent, not a crash.
        # 壊れたマニフェスト -> クラッシュではなく「無い」扱い。
        manifest_path.write_text("{not valid json", encoding="utf-8")
        assert flasher_install._read_manifest(manifest_path) is None


# ---------------------------------------------------------------------------
# Runner
# 実行部
# ---------------------------------------------------------------------------

CHECKS = [
    ("detect_platform_key", check_detect_platform_key),
    ("select_assets", check_select_assets),
    ("download_and_verify (SHA256)", check_download_and_verify_sha256),
    ("manifest round-trip", check_manifest_roundtrip),
]


def main() -> int:
    failures = []
    for name, check_fn in CHECKS:
        try:
            check_fn()
        except AssertionError as exc:
            failures.append((name, str(exc)))
            print(f"FAIL {name}: {exc}")
        except Exception as exc:  # noqa: BLE001 - report any unexpected error, not just AssertionError
            failures.append((name, f"{type(exc).__name__}: {exc}"))
            print(f"ERROR {name}: {type(exc).__name__}: {exc}")
        else:
            print(f"PASS {name}")

    print()
    if failures:
        print(f"{len(failures)}/{len(CHECKS)} check(s) FAILED")
        return 1
    print(f"All {len(CHECKS)} checks PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
