r"""
test_installer_dedicated.py - Tests for the dedicated environment support
in scripts/installer.py (docs/plans/dedicated-environment-plan.md Phase B).

専用環境(docs/plans/dedicated-environment-plan.md Phase B)関連の
scripts/installer.py テスト。

Loads scripts/installer.py the same way scripts/test_gui_installer_parity.py
does (importlib, not a package import, since scripts/ is not a package).
No real network access: every test that would otherwise download the
python-build-standalone (PBS) archive instead builds a small local fake
tar.gz and monkeypatches `_download_file`/`private_python_asset` to point
at it. Tests that spawn the fake PBS interpreter as a subprocess build it
as a Unix shell script and are skipped on Windows -- see the per-test
`skipif` markers.

scripts/test_gui_installer_parity.py と同じ方法(importlib。scripts/ は
パッケージではないため)で scripts/installer.py を読み込む。実ネットワーク
アクセスは一切行わない: python-build-standalone(PBS)アーカイブを
ダウンロードするはずの全テストは、代わりに小さなローカル偽tar.gzを組み立て
`_download_file`/`private_python_asset` をそちらへ向くようmonkeypatchする。
偽PBSインタプリタをsubprocessとして起動するテストはUnixシェルスクリプトとして
組み立てるため、Windowsではスキップする(各テストの`skipif`マーカー参照)。

Run: pytest scripts/test_installer_dedicated.py -v
"""

import importlib.util
import os
import shutil
import sys
import tarfile
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
INSTALLER_PATH = REPO_ROOT / "scripts" / "installer.py"

# Tests that spawn the fake PBS python3 as a subprocess build it as a
# `#!/bin/sh` script, which only runs directly on a Unix-like OS.
# 偽PBSのpython3をsubprocessとして起動するテストは `#!/bin/sh` スクリプト
# として組み立てるため、Unix系OSでのみ直接実行できる。
UNIX_SHELL_SCRIPT_ONLY = pytest.mark.skipif(
    sys.platform == "win32",
    reason="fake PBS fixture is a Unix shell script (#!/bin/sh)",
)


def _load_module(module_name: str, path: Path) -> ModuleType:
    """Load `path` as a standalone module without adding it to sys.modules
    under its real package name (mirrors scripts/test_gui_installer_parity.py's
    own copy of this helper -- duplicated rather than shared since these
    test files have no common import target).
    `path` を、実パッケージ名で sys.modules に登録せず単独モジュールとして
    ロードする(scripts/test_gui_installer_parity.py 自身の同名ヘルパーを
    模す -- 共有先が無いため複製する)。"""
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def installer() -> ModuleType:
    return _load_module("_test_installer_dedicated", INSTALLER_PATH)


@pytest.fixture(autouse=True)
def _clear_dedicated_context_between_tests(installer: ModuleType):
    """Guard against _DEDICATED state leaking between tests (e.g. a test
    that fails before its own cleanup runs).
    テスト間で _DEDICATED の状態が漏れないようにする(独自の後片付けが
    走る前に失敗したテスト等への保険)。"""
    installer.clear_dedicated_context()
    yield
    installer.clear_dedicated_context()


def _build_fake_pbs_archive(build_dir: Path, version_output: str, name: str = "fake-pbs.tar.gz") -> Path:
    """Build a minimal fake python-build-standalone (PBS) tar.gz: just
    `python/bin/python3`, a `#!/bin/sh` script that ignores all arguments
    and prints `version_output` -- enough for
    _extract_private_python()/provision_private_python() to exercise real
    tarfile extraction and real subprocess verification, with no network
    access and no real Python interpreter involved.
    最小限の偽python-build-standalone(PBS)tar.gzを組み立てる:
    `python/bin/python3` だけを持ち、全引数を無視して `version_output` を
    出力する `#!/bin/sh` スクリプト -- ネットワークアクセスも実Python
    インタプリタも使わずに、_extract_private_python()/
    provision_private_python() に実tarfile展開・実subprocess検証を
    行わせるのに十分。
    """
    src_root = build_dir / "_fake_pbs_src"
    if src_root.exists():
        shutil.rmtree(src_root)
    bin_dir = src_root / "python" / "bin"
    bin_dir.mkdir(parents=True)
    python3_script = bin_dir / "python3"
    python3_script.write_text(f'#!/bin/sh\necho "{version_output}"\n')
    python3_script.chmod(0o755)

    archive_path = build_dir / name
    with tarfile.open(archive_path, "w:gz") as tar:
        tar.add(src_root / "python", arcname="python")
    return archive_path


# =============================================================================
# private_python_asset() / _normalize_machine() / private_python_asset_url()
# =============================================================================

@pytest.mark.parametrize(
    "plat, machine, expected_name_substr",
    [
        ("win32", "AMD64", "x86_64-pc-windows-msvc"),
        ("win32", "ARM64", "aarch64-pc-windows-msvc"),
        ("darwin", "arm64", "aarch64-apple-darwin"),
        ("darwin", "x86_64", "x86_64-apple-darwin"),
        ("linux", "x86_64", "x86_64-unknown-linux-gnu"),
        ("linux", "aarch64", "aarch64-unknown-linux-gnu"),
    ],
)
def test_private_python_asset_selection(installer, monkeypatch, plat, machine, expected_name_substr):
    monkeypatch.setattr(installer.sys, "platform", plat)
    monkeypatch.setattr(installer.platform, "machine", lambda: machine)

    name, sha256 = installer.private_python_asset()

    assert expected_name_substr in name
    assert len(sha256) == 64  # sha256 hex digest length / sha256の16進表現の長さ


def test_normalize_machine_handles_windows_and_unix_spellings(installer):
    assert installer._normalize_machine("AMD64") == "x86_64"
    assert installer._normalize_machine("x86_64") == "x86_64"
    assert installer._normalize_machine("x64") == "x86_64"
    assert installer._normalize_machine("ARM64") == "arm64"
    assert installer._normalize_machine("aarch64") == "arm64"
    # Unrecognized spellings are lowercased, not guessed at.
    # 認識できない表記は推測せず小文字化するのみ。
    assert installer._normalize_machine("RISCV64") == "riscv64"


def test_private_python_asset_unsupported_target_raises(installer, monkeypatch):
    monkeypatch.setattr(installer.sys, "platform", "freebsd")
    monkeypatch.setattr(installer.platform, "machine", lambda: "riscv64")

    with pytest.raises(RuntimeError, match="No dedicated Python build"):
        installer.private_python_asset()


def test_private_python_asset_url_encodes_plus_as_percent_2b(installer, monkeypatch):
    monkeypatch.setattr(
        installer, "private_python_asset",
        lambda: ("cpython-3.12.14+20260901-x86_64-apple-darwin-install_only.tar.gz", "deadbeef"),
    )

    url = installer.private_python_asset_url()

    assert "%2B" in url
    assert "+" not in url


# =============================================================================
# sf_home_default()
# =============================================================================

def test_sf_home_default_env_var_wins(installer, monkeypatch, tmp_path):
    custom_home = tmp_path / "custom_sf_home"
    monkeypatch.setenv(installer.SF_HOME_ENV, str(custom_home))

    assert installer.sf_home_default() == custom_home


def test_sf_home_default_unix_uses_dotstampfly_under_home(installer, monkeypatch, tmp_path):
    monkeypatch.delenv(installer.SF_HOME_ENV, raising=False)
    monkeypatch.setattr(installer.sys, "platform", "linux")
    monkeypatch.setattr(installer.Path, "home", lambda: tmp_path)

    assert installer.sf_home_default() == tmp_path / ".stampfly"


def test_sf_home_default_windows_prefers_c_stampfly(installer, monkeypatch, tmp_path):
    monkeypatch.delenv(installer.SF_HOME_ENV, raising=False)
    monkeypatch.setattr(installer.sys, "platform", "win32")
    monkeypatch.setattr(installer, "SF_HOME_WINDOWS_DEFAULT", tmp_path / "StampFly")

    result = installer.sf_home_default()

    assert result == tmp_path / "StampFly"
    assert result.is_dir()  # sf_home_default() creates it / sf_home_default()が作成する


def test_sf_home_default_windows_falls_back_to_localappdata_on_mkdir_failure(installer, monkeypatch, tmp_path):
    monkeypatch.delenv(installer.SF_HOME_ENV, raising=False)
    monkeypatch.setattr(installer.sys, "platform", "win32")
    local_app_data = tmp_path / "LocalAppData"
    monkeypatch.setenv("LOCALAPPDATA", str(local_app_data))

    class _UnwritableDefault:
        """Stands in for SF_HOME_WINDOWS_DEFAULT: its mkdir() always fails,
        simulating a locked-down machine where C:\\ is not writable.
        SF_HOME_WINDOWS_DEFAULTの代役: mkdir()が常に失敗し、C:\\へ書き込め
        ないロックダウンされたマシンを模す。"""

        def mkdir(self, parents=True, exist_ok=True):
            raise OSError("permission denied (simulated)")

    monkeypatch.setattr(installer, "SF_HOME_WINDOWS_DEFAULT", _UnwritableDefault())

    result = installer.sf_home_default()

    assert result == local_app_data / "StampFly"


# =============================================================================
# path_is_idf_safe() / warn_if_path_unsafe()
# =============================================================================

def test_path_is_idf_safe_accepts_plain_ascii(installer):
    assert installer.path_is_idf_safe(Path("/Users/test/StampFly")) is True


def test_path_is_idf_safe_rejects_space(installer):
    assert installer.path_is_idf_safe(Path("/Users/test user/StampFly")) is False


def test_path_is_idf_safe_rejects_non_ascii(installer):
    assert installer.path_is_idf_safe(Path("/Users/\u4f0a\u85e4/StampFly")) is False


# =============================================================================
# read_config() round-trip with Installer._save_config()
# =============================================================================

def _installer_with_tmp_config(installer: ModuleType, tmp_path: Path):
    inst = installer.Installer()
    inst.config_dir = tmp_path / ".sf"
    inst.config_file = inst.config_dir / "config.toml"
    return inst


def test_read_config_round_trip_dedicated(installer, tmp_path):
    inst = _installer_with_tmp_config(installer, tmp_path)
    idf_path = tmp_path / "esp-idf"
    idf_path.mkdir()
    (idf_path / "version.txt").write_text("v5.5.2")
    dedicated_root = tmp_path / "sf_home"

    inst._save_config(idf_path, dedicated_root=dedicated_root)
    config = installer.read_config(inst.config_file)

    assert config["esp_idf"]["path"] == str(idf_path)
    assert config["esp_idf"]["version"] == "v5.5.2"
    assert config["env"]["kind"] == "dedicated"
    assert config["env"]["root"] == str(dedicated_root)
    assert config["env"]["python"] == str(installer.dedicated_python_exe(dedicated_root))
    assert config["env"]["python_dir"] == str(installer.dedicated_python_dir(dedicated_root))
    assert config["env"]["tools_path"] == str(installer.dedicated_tools_dir(dedicated_root))
    assert config["project"]["default_target"] == "vehicle"


def test_read_config_round_trip_legacy(installer, tmp_path):
    inst = _installer_with_tmp_config(installer, tmp_path)
    idf_path = tmp_path / "esp-idf"
    idf_path.mkdir()
    (idf_path / "version.txt").write_text("v5.5.2")

    inst._save_config(idf_path)
    config = installer.read_config(inst.config_file)

    assert config["env"]["kind"] == "legacy"
    assert "root" not in config["env"]


def test_read_config_missing_file_returns_empty_dict(installer, tmp_path):
    assert installer.read_config(tmp_path / "does_not_exist.toml") == {}


# =============================================================================
# provision_private_python()
# =============================================================================

@UNIX_SHELL_SCRIPT_ONLY
def test_provision_private_python_downloads_extracts_and_is_idempotent(installer, monkeypatch, tmp_path):
    root = tmp_path / "sf_home"
    archive_path = _build_fake_pbs_archive(tmp_path, installer.PRIVATE_PYTHON_VERSION)
    fake_sha256 = installer._sha256_of(archive_path)
    fake_asset_name = archive_path.name

    monkeypatch.setattr(installer, "private_python_asset", lambda: (fake_asset_name, fake_sha256))
    download_calls = []

    def _fake_download(url, dest):
        download_calls.append((url, dest))
        shutil.copyfile(archive_path, dest)

    monkeypatch.setattr(installer, "_download_file", _fake_download)

    python_exe = installer.provision_private_python(root)

    assert python_exe == installer.dedicated_python_exe(root)
    assert python_exe.is_file()
    assert len(download_calls) == 1
    manifest = installer.read_manifest(root)
    assert manifest["python"]["version"] == installer.PRIVATE_PYTHON_VERSION
    assert manifest["python"]["release"] == installer.PRIVATE_PYTHON_RELEASE
    assert manifest["python"]["asset"] == fake_asset_name
    assert manifest["python"]["sha256"] == fake_sha256

    # Second call: manifest already matches and the interpreter still
    # verifies -- must short-circuit without downloading again.
    # 2回目の呼び出し: manifest は既に一致し、インタプリタも検証に成功する
    # -- 再ダウンロードせず短絡すること。
    python_exe_again = installer.provision_private_python(root)

    assert python_exe_again == python_exe
    assert len(download_calls) == 1  # unchanged / 変化なし


@UNIX_SHELL_SCRIPT_ONLY
def test_provision_private_python_adopts_existing_install_without_manifest(installer, monkeypatch, tmp_path):
    """install.sh/install.bat (Phase C) may bootstrap the same PBS archive
    via curl+tar before installer.py ever runs, without writing
    manifest.json. provision_private_python() must adopt that existing,
    already-valid interpreter instead of re-downloading -- only backfilling
    the manifest.
    install.sh/install.bat(Phase C)は installer.py の実行前に curl+tar で
    同じPBSアーカイブを既にブートストラップしていることがあり、
    manifest.json は書かない。provision_private_python() は再ダウンロード
    せず、その既存の(検証済みの)インタプリタを採用し、manifest だけを
    補完しなければならない。
    """
    root = tmp_path / "sf_home"
    python_exe = installer.dedicated_python_exe(root)
    python_exe.parent.mkdir(parents=True)
    python_exe.write_text(f'#!/bin/sh\necho "{installer.PRIVATE_PYTHON_VERSION}"\n')
    python_exe.chmod(0o755)
    assert not installer.dedicated_manifest_path(root).exists()

    monkeypatch.setattr(installer, "private_python_asset", lambda: ("fake-asset.tar.gz", "deadbeef"))
    download_calls = []
    monkeypatch.setattr(
        installer, "_download_file",
        lambda url, dest: download_calls.append((url, dest)),
    )

    result = installer.provision_private_python(root)

    assert result == python_exe
    assert download_calls == []
    manifest = installer.read_manifest(root)
    assert manifest["python"]["version"] == installer.PRIVATE_PYTHON_VERSION
    assert manifest["python"]["asset"] == "fake-asset.tar.gz"
    assert manifest["python"]["sha256"] == "deadbeef"


@UNIX_SHELL_SCRIPT_ONLY
def test_provision_private_python_sha_mismatch_raises_and_deletes_file(installer, monkeypatch, tmp_path):
    root = tmp_path / "sf_home"
    archive_path = _build_fake_pbs_archive(tmp_path, installer.PRIVATE_PYTHON_VERSION)
    fake_asset_name = archive_path.name
    wrong_sha256 = "0" * 64

    monkeypatch.setattr(installer, "private_python_asset", lambda: (fake_asset_name, wrong_sha256))
    monkeypatch.setattr(
        installer, "_download_file",
        lambda url, dest: shutil.copyfile(archive_path, dest),
    )

    with pytest.raises(RuntimeError, match="SHA-256 mismatch"):
        installer.provision_private_python(root)

    downloaded_path = installer.dedicated_downloads_dir(root) / fake_asset_name
    assert not downloaded_path.exists()


# =============================================================================
# Dedicated-context env steering: _clean_env_for_cmd()/_env_with_python3_steering()
# =============================================================================

def test_clean_env_for_cmd_uses_dedicated_context_without_system_python_discovery(installer, monkeypatch, tmp_path):
    python_dir = tmp_path / "python"
    tools_path = tmp_path / "espressif"
    installer.set_dedicated_context(python_dir, tools_path)

    def _must_not_be_called():
        raise AssertionError("_find_system_python_dir() must not be called in dedicated mode")

    monkeypatch.setattr(installer, "_find_system_python_dir", _must_not_be_called)

    env = installer._clean_env_for_cmd()

    assert env["PATH"].split(os.pathsep)[0] == str(python_dir)
    assert env["IDF_TOOLS_PATH"] == str(tools_path)
    assert "IDF_PYTHON_ENV_PATH" not in env


def test_env_with_python3_steering_uses_dedicated_context_without_system_python_discovery(
    installer, monkeypatch, tmp_path
):
    python_dir = tmp_path / "python" / "bin"
    tools_path = tmp_path / "espressif"
    installer.set_dedicated_context(python_dir, tools_path)

    def _must_not_be_called():
        raise AssertionError("_find_system_python_dir() must not be called in dedicated mode")

    monkeypatch.setattr(installer, "_find_system_python_dir", _must_not_be_called)

    env = installer._env_with_python3_steering()

    assert env["PATH"].split(os.pathsep)[0] == str(python_dir)
    assert env["IDF_TOOLS_PATH"] == str(tools_path)
    assert "IDF_PYTHON_ENV_PATH" not in env


def test_clean_env_for_cmd_without_dedicated_context_is_unaffected(installer, monkeypatch, tmp_path):
    """Sanity check: with no dedicated context set, _clean_env_for_cmd()
    falls back to its pre-existing legacy behavior (system Python
    discovery may run; IDF_TOOLS_PATH is not force-set by this function).
    健全性チェック: 専用コンテキストが未設定なら、_clean_env_for_cmd() は
    既存の旧来動作にフォールバックする(システムPython発見が走りうる。
    IDF_TOOLS_PATH はこの関数によって強制設定されない)。
    """
    installer.clear_dedicated_context()

    env = installer._clean_env_for_cmd()

    # No dedicated IDF_TOOLS_PATH is injected by this function itself.
    # この関数自体は専用の IDF_TOOLS_PATH を注入しない。
    assert env.get("IDF_TOOLS_PATH") == os.environ.get("IDF_TOOLS_PATH")


# =============================================================================
# _find_idf_python(idf_path, tools_path=...)
# =============================================================================

def test_find_idf_python_with_explicit_tools_path_override(installer, tmp_path):
    idf_path = tmp_path / "esp-idf"
    idf_path.mkdir()
    (idf_path / "version.txt").write_text("v5.5.2")

    tools_path = tmp_path / "tools"
    bin_subdir = "Scripts" if sys.platform == "win32" else "bin"
    python_name = "python.exe" if sys.platform == "win32" else "python"
    venv_bin_dir = tools_path / "python_env" / "idf5.5_py3.12_env" / bin_subdir
    venv_bin_dir.mkdir(parents=True)
    python_exe = venv_bin_dir / python_name
    python_exe.write_text("#!/bin/sh\n")
    python_exe.chmod(0o755)

    found = installer._find_idf_python(idf_path, tools_path=tools_path)

    assert found == python_exe


def test_find_idf_python_explicit_tools_path_ignores_env_var_default(installer, monkeypatch, tmp_path):
    """An explicit `tools_path` argument must win over whatever
    IDF_TOOLS_PATH happens to be set to -- otherwise the override would be
    pointless.
    明示的な `tools_path` 引数は、IDF_TOOLS_PATH が何に設定されていようと
    それに優先しなければならない -- そうでなければこの引数を用意する意味が
    無い。
    """
    idf_path = tmp_path / "esp-idf"
    idf_path.mkdir()
    (idf_path / "version.txt").write_text("v5.5.2")

    decoy_tools_path = tmp_path / "decoy_tools"
    monkeypatch.setenv("IDF_TOOLS_PATH", str(decoy_tools_path))

    real_tools_path = tmp_path / "real_tools"
    bin_subdir = "Scripts" if sys.platform == "win32" else "bin"
    python_name = "python.exe" if sys.platform == "win32" else "python"
    venv_bin_dir = real_tools_path / "python_env" / "idf5.5_py3.12_env" / bin_subdir
    venv_bin_dir.mkdir(parents=True)
    python_exe = venv_bin_dir / python_name
    python_exe.write_text("#!/bin/sh\n")
    python_exe.chmod(0o755)

    found = installer._find_idf_python(idf_path, tools_path=real_tools_path)

    assert found == python_exe


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
