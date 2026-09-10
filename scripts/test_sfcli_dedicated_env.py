#!/usr/bin/env python3
"""
test_sfcli_dedicated_env.py - Tests for sf CLI's dedicated-environment
support (Phase D of docs/plans/dedicated-environment-plan.md)
sf CLI の専用環境対応のテスト（docs/plans/dedicated-environment-plan.md
の Phase D）

Phase B (scripts/installer.py) already provisions a self-contained
private Python + ESP-IDF under SF_HOME and writes it into `.sf/config.toml`
as a `[env]` section (kind = "dedicated" | "legacy"). This file covers the
sf CLI's *reading* side of that contract:

  - lib/sfcli/utils/paths.py: read_config() / read_config_value() /
    dedicated_env() / esp_idf()'s config-aware lookup.
  - lib/sfcli/commands/doctor.py: the "Checking environment" section
    (_check_environment()).
  - lib/sfcli/commands/upgrade.py: the migration offer
    (_offer_dedicated_migration()).

Phase B（scripts/installer.py）は既に、SF_HOME 配下に自己完結した専用
Python + ESP-IDF を用意し、`.sf/config.toml` の `[env]` 節（kind =
"dedicated" | "legacy"）へ書き込む。本ファイルはその契約の sf CLI 側
「読み取り」を検証する:

  - lib/sfcli/utils/paths.py: read_config() / read_config_value() /
    dedicated_env() / esp_idf() の設定認識ロジック
  - lib/sfcli/commands/doctor.py: 「Checking environment」節
    (_check_environment())
  - lib/sfcli/commands/upgrade.py: 移行提案 (_offer_dedicated_migration())

Usage:
    python3 scripts/test_sfcli_dedicated_env.py
    pytest scripts/test_sfcli_dedicated_env.py
"""

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "lib"))  # for sfcli

from sfcli.commands import doctor  # noqa: E402
from sfcli.commands import upgrade  # noqa: E402
from sfcli.utils import paths as sf_paths  # noqa: E402


# ---------------------------------------------------------------------------
# Shared fixtures / helpers
# 共有フィクスチャ・ヘルパー
# ---------------------------------------------------------------------------


def _make_upgrade_args(migrate: bool = False, no_migrate: bool = False) -> argparse.Namespace:
    """Minimal argparse.Namespace matching lib/sfcli/commands/upgrade.py's
    `register()` flags, for calling _offer_dedicated_migration() directly.
    lib/sfcli/commands/upgrade.py の `register()` が定義するフラグを
    模した最小限の argparse.Namespace（_offer_dedicated_migration() の
    直接呼び出し用）。
    """
    return argparse.Namespace(
        yes=False,
        discard_local=False,
        no_flasher=False,
        skip_deps=False,
        migrate=migrate,
        no_migrate=no_migrate,
    )


def _build_fake_dedicated_root(tmp_path: Path) -> Path:
    """Build a minimal on-disk layout matching a real dedicated
    environment (see docs/plans/dedicated-environment-plan.md's folder
    layout table): python/, espressif/, esp-idf/tools/idf.py,
    manifest.json.
    実際の専用環境の配置（docs/plans/dedicated-environment-plan.md の
    フォルダ配置表参照）を模した最小限のディスク上構成を作る:
    python/, espressif/, esp-idf/tools/idf.py, manifest.json
    """
    root = tmp_path / "sf_home"
    (root / "python").mkdir(parents=True)
    (root / "espressif").mkdir(parents=True)
    (root / "esp-idf" / "tools").mkdir(parents=True)
    (root / "esp-idf" / "tools" / "idf.py").write_text("# fake idf.py\n", encoding="utf-8")
    (root / "manifest.json").write_text(
        json.dumps({"python": {"version": "3.12.14", "release": "20260901"}}),
        encoding="utf-8",
    )
    return root


def _write_dedicated_config(config_path: Path, root: Path) -> None:
    """Write a v2 `[env] kind = "dedicated"` config.toml pointing at
    `root` (mirrors scripts/installer.py's Installer._save_config()).
    `root` を指すv2の `[env] kind = "dedicated"` config.toml を書く
    （scripts/installer.py の Installer._save_config() を模す）。
    """
    config_path.write_text(
        "[esp_idf]\n"
        f'path = "{root / "esp-idf"}"\n'
        'version = "v5.5.2"\n'
        "\n"
        "[env]\n"
        'kind = "dedicated"\n'
        f'root = "{root}"\n'
        f'python = "{root / "python" / "bin" / "python3"}"\n'
        f'python_dir = "{root / "python" / "bin"}"\n'
        f'tools_path = "{root / "espressif"}"\n',
        encoding="utf-8",
    )


# ---------------------------------------------------------------------------
# paths.read_config() / read_config_value() / dedicated_env()
# ---------------------------------------------------------------------------


def test_read_config_dedicated(tmp_path, monkeypatch):
    """A v2 dedicated config.toml parses into nested {section: {key: value}}
    and dedicated_env() returns its [env] section."""
    config_path = tmp_path / "config.toml"
    root = tmp_path / "sf_home"
    _write_dedicated_config(config_path, root)
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    config = sf_paths.read_config()
    assert config["env"]["kind"] == "dedicated"
    assert config["env"]["root"] == str(root)
    assert config["esp_idf"]["version"] == "v5.5.2"
    assert sf_paths.read_config_value("esp_idf", "path") == str(root / "esp-idf")
    assert sf_paths.dedicated_env() == config["env"]


def test_read_config_legacy(tmp_path, monkeypatch):
    """A v2 legacy config.toml parses correctly and dedicated_env() is
    None (this is not a dedicated environment)."""
    config_path = tmp_path / "config.toml"
    config_path.write_text(
        '[esp_idf]\npath = "/opt/esp-idf"\nversion = "v5.5.2"\n'
        '\n[env]\nkind = "legacy"\n\n[project]\ndefault_target = "vehicle"\n',
        encoding="utf-8",
    )
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    config = sf_paths.read_config()
    assert config["env"]["kind"] == "legacy"
    assert sf_paths.dedicated_env() is None


def test_read_config_missing(tmp_path, monkeypatch):
    """A missing config.toml reads as {} (never raises), and both helper
    readers degrade to None/empty accordingly."""
    config_path = tmp_path / "does_not_exist" / "config.toml"
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    assert sf_paths.read_config() == {}
    assert sf_paths.read_config_value("esp_idf", "path") is None
    assert sf_paths.dedicated_env() is None


# ---------------------------------------------------------------------------
# paths.esp_idf()'s config-aware lookup
# ---------------------------------------------------------------------------


def test_esp_idf_uses_config_path_when_it_exists(tmp_path, monkeypatch):
    """esp_idf() returns the configured [esp_idf] path when it exists on
    disk, even though it is neither the .esp-idf symlink nor a home-dir
    default location."""
    monkeypatch.delenv("IDF_PATH", raising=False)
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "fake_home")

    fake_root = tmp_path / "repo_root"
    fake_root.mkdir()
    monkeypatch.setattr(sf_paths, "root", lambda: fake_root)

    idf_dir = tmp_path / "configured_idf"
    idf_dir.mkdir()
    config_path = tmp_path / "config.toml"
    config_path.write_text(f'[esp_idf]\npath = "{idf_dir}"\n', encoding="utf-8")
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    assert sf_paths.esp_idf() == idf_dir


def test_esp_idf_falls_through_when_config_path_is_missing(tmp_path, monkeypatch):
    """esp_idf() ignores a configured path that does not exist on disk
    (a stale/hand-edited entry) and falls through to the remaining
    searches -- which, with no real ESP-IDF anywhere reachable, resolve
    to None."""
    monkeypatch.delenv("IDF_PATH", raising=False)
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "fake_home")

    fake_root = tmp_path / "repo_root2"
    fake_root.mkdir()
    monkeypatch.setattr(sf_paths, "root", lambda: fake_root)

    config_path = tmp_path / "config.toml"
    config_path.write_text('[esp_idf]\npath = "/nonexistent/idf/path/xyz"\n', encoding="utf-8")
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    assert sf_paths.esp_idf() is None


# ---------------------------------------------------------------------------
# doctor._check_environment()
# ---------------------------------------------------------------------------


def test_check_environment_dedicated_healthy(tmp_path, monkeypatch, capsys):
    """A dedicated config whose running interpreter/IDF_TOOLS_PATH/ESP-IDF
    all check out produces zero warnings."""
    root = _build_fake_dedicated_root(tmp_path)
    config_path = tmp_path / "config.toml"
    _write_dedicated_config(config_path, root)
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)
    monkeypatch.setattr(sys, "base_prefix", str(root / "python"))
    monkeypatch.setenv("IDF_TOOLS_PATH", str(root / "espressif"))

    warnings = []
    doctor._check_environment(warnings)
    captured = capsys.readouterr()

    assert warnings == []
    assert f"Dedicated environment: {root}" in captured.out
    assert "Private Python 3.12.14 (release 20260901)" in captured.out
    assert "NOT FOUND" not in captured.out
    assert captured.err == ""  # no warnings -> nothing on stderr / 警告なし->stderrは空


def test_check_environment_dedicated_wrong_python(tmp_path, monkeypatch, capsys):
    """A dedicated config, but the CURRENTLY RUNNING interpreter is not
    under <root>/python -- warns about a stale `sf` on PATH.

    console.warning() prints to stderr (see lib/sfcli/utils/console.py),
    so the warning text is asserted against `captured.err`, not `.out`.
    console.warning() は stderr に出力する
    （lib/sfcli/utils/console.py 参照）ため、警告文言は `captured.out`
    ではなく `captured.err` に対して検証する。
    """
    root = _build_fake_dedicated_root(tmp_path)
    config_path = tmp_path / "config.toml"
    _write_dedicated_config(config_path, root)
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)
    monkeypatch.setattr(sys, "base_prefix", str(tmp_path / "somewhere_else"))
    monkeypatch.setenv("IDF_TOOLS_PATH", str(root / "espressif"))

    warnings = []
    doctor._check_environment(warnings)
    captured = capsys.readouterr()

    assert any("different Python" in w for w in warnings)
    assert "different Python than the dedicated one" in captured.err


def test_check_environment_legacy_prints_migrate_hint(tmp_path, monkeypatch, capsys):
    """A legacy (v2) config prints the migration hint and raises no
    warnings."""
    config_path = tmp_path / "config.toml"
    config_path.write_text(
        '[esp_idf]\npath = "/opt/esp-idf"\nversion = "v5.5.2"\n\n[env]\nkind = "legacy"\n',
        encoding="utf-8",
    )
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    warnings = []
    doctor._check_environment(warnings)
    captured = capsys.readouterr()

    assert warnings == []
    assert "sf upgrade --migrate" in captured.out


def test_check_environment_no_config(tmp_path, monkeypatch, capsys):
    """No .sf/config.toml at all -- reported plainly, no warnings."""
    config_path = tmp_path / "missing" / "config.toml"
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    warnings = []
    doctor._check_environment(warnings)
    captured = capsys.readouterr()

    assert warnings == []
    assert "No .sf/config.toml" in captured.out


# ---------------------------------------------------------------------------
# upgrade._offer_dedicated_migration() decision matrix
# ---------------------------------------------------------------------------


def test_offer_migration_no_config_asks_and_declines(tmp_path, monkeypatch):
    """No config at all (pre-v1, never installed) still offers to
    migrate when interactive; declining writes nothing (no config file
    to annotate)."""
    config_path = tmp_path / "config.toml"  # never created
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: True))

    confirm_calls = []
    monkeypatch.setattr(
        upgrade, "_confirm",
        lambda prompt, default_yes: (confirm_calls.append(prompt), False)[1],
    )
    installer_calls = []
    monkeypatch.setattr(
        upgrade, "_run_dedicated_installer",
        lambda root: (installer_calls.append(root), 0)[1],
    )

    result = upgrade._offer_dedicated_migration(tmp_path, _make_upgrade_args())

    assert result is None
    assert confirm_calls, "should have asked via _confirm"
    assert installer_calls == []
    assert not config_path.exists()


def test_offer_migration_pre_v2_confirm_accepts(tmp_path, monkeypatch):
    """A pre-v2 config (no [env] section) + interactive accept -> the
    dedicated installer runs and its exit code is returned directly."""
    config_path = tmp_path / "config.toml"
    config_path.write_text('[esp_idf]\npath = "/opt/esp-idf"\nversion = "v5.5.2"\n', encoding="utf-8")
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr(upgrade, "_confirm", lambda prompt, default_yes: True)

    installer_calls = []
    monkeypatch.setattr(
        upgrade, "_run_dedicated_installer",
        lambda root: (installer_calls.append(root), 0)[1],
    )

    result = upgrade._offer_dedicated_migration(tmp_path, _make_upgrade_args())

    assert result == upgrade.EXIT_OK
    assert installer_calls == [tmp_path]


def test_offer_migration_pre_v2_noninteractive_skips(tmp_path, monkeypatch):
    """A pre-v2 config, but stdin is not a tty (CI/non-interactive) --
    the offer is skipped (info line only), installer never runs."""
    config_path = tmp_path / "config.toml"
    config_path.write_text('[esp_idf]\npath = "/opt/esp-idf"\n', encoding="utf-8")
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: False))

    installer_calls = []
    monkeypatch.setattr(
        upgrade, "_run_dedicated_installer",
        lambda root: (installer_calls.append(root), 0)[1],
    )

    result = upgrade._offer_dedicated_migration(tmp_path, _make_upgrade_args())

    assert result is None
    assert installer_calls == []
    assert not config_path.read_text(encoding="utf-8").count("[env]")


def test_offer_migration_legacy_without_migrate_flag_skips(tmp_path, monkeypatch):
    """kind == "legacy" without --migrate: the user already chose
    legacy -- never asked, installer never runs."""
    config_path = tmp_path / "config.toml"
    config_path.write_text('[env]\nkind = "legacy"\n', encoding="utf-8")
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    installer_calls = []
    monkeypatch.setattr(
        upgrade, "_run_dedicated_installer",
        lambda root: (installer_calls.append(root), 0)[1],
    )

    result = upgrade._offer_dedicated_migration(tmp_path, _make_upgrade_args())

    assert result is None
    assert installer_calls == []


def test_offer_migration_legacy_with_migrate_flag_runs(tmp_path, monkeypatch):
    """kind == "legacy" WITH --migrate: --migrate overrides the earlier
    legacy choice and runs the installer unconditionally (no prompt)."""
    config_path = tmp_path / "config.toml"
    config_path.write_text('[env]\nkind = "legacy"\n', encoding="utf-8")
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    installer_calls = []
    monkeypatch.setattr(
        upgrade, "_run_dedicated_installer",
        lambda root: (installer_calls.append(root), 0)[1],
    )

    result = upgrade._offer_dedicated_migration(tmp_path, _make_upgrade_args(migrate=True))

    assert result == upgrade.EXIT_OK
    assert installer_calls == [tmp_path]


def test_offer_migration_already_dedicated_never_asks(tmp_path, monkeypatch):
    """kind == "dedicated": nothing to do, never even asks."""
    config_path = tmp_path / "config.toml"
    config_path.write_text('[env]\nkind = "dedicated"\nroot = "/whatever"\n', encoding="utf-8")
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    confirm_calls = []
    monkeypatch.setattr(upgrade, "_confirm", lambda prompt, default_yes: confirm_calls.append(1) or False)
    installer_calls = []
    monkeypatch.setattr(
        upgrade, "_run_dedicated_installer",
        lambda root: (installer_calls.append(root), 0)[1],
    )

    result = upgrade._offer_dedicated_migration(tmp_path, _make_upgrade_args())

    assert result is None
    assert confirm_calls == []
    assert installer_calls == []


def test_offer_migration_no_migrate_flag_never_asks(tmp_path, monkeypatch):
    """--no-migrate short-circuits before even reading the config."""
    config_path = tmp_path / "config.toml"  # never created; must not matter
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)

    confirm_calls = []
    monkeypatch.setattr(upgrade, "_confirm", lambda prompt, default_yes: confirm_calls.append(1) or False)

    result = upgrade._offer_dedicated_migration(tmp_path, _make_upgrade_args(no_migrate=True))

    assert result is None
    assert confirm_calls == []


def test_offer_migration_declined_writes_legacy_kind(tmp_path, monkeypatch):
    """Declining an interactive offer records kind = "legacy" into the
    existing config.toml (appending an [env] section), so future runs
    stop asking."""
    config_path = tmp_path / "config.toml"
    config_path.write_text(
        '[esp_idf]\npath = "/opt/esp-idf"\nversion = "v5.5.2"\n\n'
        '[project]\ndefault_target = "vehicle"\n',
        encoding="utf-8",
    )
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr(upgrade, "_confirm", lambda prompt, default_yes: False)

    result = upgrade._offer_dedicated_migration(tmp_path, _make_upgrade_args())

    assert result is None
    updated_config = sf_paths.read_config()
    assert updated_config["env"]["kind"] == "legacy"
    # The pre-existing [esp_idf] section must survive untouched.
    # 既存の [esp_idf] 節はそのまま残らなければならない。
    assert updated_config["esp_idf"]["path"] == "/opt/esp-idf"


def test_offer_migration_installer_failure_returns_none(tmp_path, monkeypatch):
    """A failed installer run (non-zero exit) does not abort `sf
    upgrade` -- it warns and returns None so normal flow continues."""
    config_path = tmp_path / "config.toml"
    config_path.write_text('[esp_idf]\npath = "/opt/esp-idf"\n', encoding="utf-8")
    monkeypatch.setattr(sf_paths, "config_file", lambda: config_path)
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr(upgrade, "_confirm", lambda prompt, default_yes: True)
    monkeypatch.setattr(upgrade, "_run_dedicated_installer", lambda root: 1)

    result = upgrade._offer_dedicated_migration(tmp_path, _make_upgrade_args())

    assert result is None


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
