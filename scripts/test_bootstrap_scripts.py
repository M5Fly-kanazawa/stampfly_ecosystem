"""Tests for the dedicated-environment bootstrap in install.sh / install.bat /
setup_env.sh / setup_env.bat.

See docs/plans/dedicated-environment-plan.md (Phase C). These scripts fetch a
pinned python-build-standalone (PBS) Python build into $SF_HOME/python and
teach setup_env.* to read the resulting .sf/config.toml v2 [env] section
without probing the system. install.py/setup_env.bat themselves cannot be
executed on this (macOS) CI machine, so they get static checks only
(ASCII/CRLF, asset table parity); install.sh and setup_env.sh get full
subprocess execution tests.

Run:
    source setup_env.sh && python -m pytest scripts/test_bootstrap_scripts.py -q
"""

import os
import platform
import re
import shutil
import stat
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
INSTALL_SH = REPO_ROOT / "install.sh"
INSTALL_BAT = REPO_ROOT / "install.bat"
SETUP_ENV_SH = REPO_ROOT / "setup_env.sh"
SETUP_ENV_BAT = REPO_ROOT / "setup_env.bat"

# The six PBS assets pinned in docs/plans/dedicated-environment-plan.md
# section 2. Hardcoded here (independent of scripts/installer.py, which is
# being written concurrently in Phase B) so this is a real cross-check, not
# a tautology against the scripts themselves.
EXPECTED_PBS_ASSETS = {
    "cpython-3.12.14+20260901-x86_64-pc-windows-msvc-install_only.tar.gz":
        "e90c1b6419da3bd812dd73bb3de40287a21abf153438147639ec5e20375ea93f",
    "cpython-3.12.14+20260901-aarch64-pc-windows-msvc-install_only.tar.gz":
        "4e852236277eb8f7105cbe0f5adf45592f521af238bc0f700c351856e2c2e41a",
    "cpython-3.12.14+20260901-aarch64-apple-darwin-install_only.tar.gz":
        "3ee3ee547cedfeb7c2b16b2b7156039f7b470bb8f857e226fd3d2eb11db83c76",
    "cpython-3.12.14+20260901-x86_64-apple-darwin-install_only.tar.gz":
        "2e31b23f3f1319f707d0e620b48847a0046577541d357276821f9f1b5492e0ba",
    "cpython-3.12.14+20260901-x86_64-unknown-linux-gnu-install_only.tar.gz":
        "936c246dfdbbfa7cb22dd01814a21f582a892689fae96b06071a5e433baffa22",
    "cpython-3.12.14+20260901-aarch64-unknown-linux-gnu-install_only.tar.gz":
        "b61b856c3e1a4fc65b8f6e6b0495ef975dd0924f90c59f3ea61b38a079173b84",
}

# Pre-staged archive for the offline bootstrap test (see the Phase C task
# briefing). Only present on this development machine, not in general CI --
# tests that need it skip cleanly when it is absent.
_SCRATCH_DIR = Path(
    "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem"
    "/2c354218-2c90-4187-99aa-4d97559421ed/scratchpad"
)
PBS_MAC_ARM64_ASSET = "cpython-3.12.14+20260901-aarch64-apple-darwin-install_only.tar.gz"
PBS_MAC_ARM64_ARCHIVE = _SCRATCH_DIR / "pbs_mac.tar.gz"


# ---------------------------------------------------------------------------
# Syntax checks
# ---------------------------------------------------------------------------


def test_install_sh_syntax():
    result = subprocess.run(["bash", "-n", str(INSTALL_SH)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_setup_env_sh_syntax():
    result = subprocess.run(["bash", "-n", str(SETUP_ENV_SH)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


# ---------------------------------------------------------------------------
# PBS asset table parity: install.sh (Darwin/Linux) + install.bat (Windows)
# together must equal the plan's six pinned (asset, sha256) pairs exactly.
# ---------------------------------------------------------------------------


def _extract_pairs_sh(text):
    """asset="..." immediately followed by sha256="..." in install.sh."""
    pattern = re.compile(
        r'asset="(cpython-3\.12\.14\+20260901-[A-Za-z0-9_.\-]+\.tar\.gz)"'
        r'\s*\n\s*sha256="([0-9a-f]{64})"'
    )
    return dict(pattern.findall(text))


def _extract_pairs_bat(text):
    """set "SF_PBS_ASSET=...it" immediately followed by the SHA256 line."""
    pattern = re.compile(
        r'SF_PBS_ASSET=(cpython-3\.12\.14\+20260901-[A-Za-z0-9_.\-]+\.tar\.gz)"'
        r'\s*\r?\n\s*set "SF_PBS_SHA256=([0-9a-f]{64})"'
    )
    return dict(pattern.findall(text))


def test_pbs_asset_table_matches_plan():
    sh_pairs = _extract_pairs_sh(INSTALL_SH.read_text())
    bat_pairs = _extract_pairs_bat(INSTALL_BAT.read_text(encoding="ascii"))

    assert sh_pairs, "no PBS (asset, sha256) pairs found in install.sh"
    assert bat_pairs, "no PBS (asset, sha256) pairs found in install.bat"

    # install.sh (uname-based) only ever targets Darwin/Linux; install.bat
    # only ever targets Windows -- they must not overlap, and their union
    # must be exactly the plan's six pinned assets (no drift either way).
    assert set(sh_pairs) & set(bat_pairs) == set(), "install.sh and install.bat both list the same asset"
    assert all("windows" not in name for name in sh_pairs), "install.sh should not carry Windows assets"
    assert all("windows" in name for name in bat_pairs), "install.bat should only carry Windows assets"

    combined = {**sh_pairs, **bat_pairs}
    assert combined == EXPECTED_PBS_ASSETS


def test_percent_2b_urls_correspond_to_known_assets():
    # install.sh: literal runtime "%2B". install.bat: "%%2B" in source,
    # which cmd.exe collapses to a literal "%2B" only at execution time.
    sh_text = INSTALL_SH.read_text()
    bat_text = INSTALL_BAT.read_text(encoding="ascii")

    assert "%2B" in sh_text, "install.sh does not URL-encode '+' as %2B anywhere"
    assert "%%2B" in bat_text, "install.bat does not URL-encode '+' as %%2B (runtime %2B) anywhere"

    for url in re.findall(r'https://[^\s"]*%2B[^\s"]*', sh_text):
        decoded = url.replace("%2B", "+")
        assert any(asset in decoded for asset in EXPECTED_PBS_ASSETS), (
            f"install.sh: %2B URL does not decode to a known asset name: {url}"
        )

    for url in re.findall(r'https://[^\s"]*%%2B[^\s"]*', bat_text):
        decoded = url.replace("%%2B", "+")
        assert any(asset in decoded for asset in EXPECTED_PBS_ASSETS), (
            f"install.bat: %%2B URL does not decode to a known asset name: {url}"
        )


# ---------------------------------------------------------------------------
# .bat files: CRLF line endings, ASCII-only bytes (enforced by .gitattributes
# and required because cp932 consoles misinterpret non-ASCII bytes as shell
# metacharacters -- see the header comments in both files).
# ---------------------------------------------------------------------------


def _assert_crlf_and_ascii(path):
    data = path.read_bytes()
    assert data, f"{path.name} is empty"
    assert b"\r\n" in data, f"{path.name} has no CRLF line endings"
    lone_lf = data.count(b"\n") - data.count(b"\r\n")
    assert lone_lf == 0, f"{path.name} has {lone_lf} LF-only line ending(s)"
    non_ascii = sorted({b for b in data if b > 127})
    assert not non_ascii, f"{path.name} contains non-ASCII byte(s): {non_ascii}"


def test_install_bat_is_crlf_and_ascii():
    _assert_crlf_and_ascii(INSTALL_BAT)


def test_setup_env_bat_is_crlf_and_ascii():
    _assert_crlf_and_ascii(SETUP_ENV_BAT)


# ---------------------------------------------------------------------------
# setup_env.sh: dedicated mode reads .sf/config.toml v2 and skips all Python
# discovery/probing.
# ---------------------------------------------------------------------------


def _make_executable(path: Path, content: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    mode = path.stat().st_mode
    path.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def test_setup_env_sh_dedicated_mode(tmp_path):
    home = tmp_path / "home"
    repo = tmp_path / "repo"
    repo.mkdir()

    python_dir = home / "python" / "bin"
    _make_executable(python_dir / "python3", "#!/bin/sh\necho 3.12.14\n")

    venv_dir = home / "espressif" / "python_env" / "idf5.5_py3.12_env"
    _make_executable(venv_dir / "bin" / "python", "#!/bin/sh\necho fake-venv-python\n")

    idf_dir = home / "esp-idf"
    _make_executable(
        idf_dir / "export.sh",
        f'#!/bin/bash\nexport IDF_PYTHON_ENV_PATH="{venv_dir}"\n',
    )

    (repo / "setup_env.sh").write_text(SETUP_ENV_SH.read_text())

    config = repo / ".sf" / "config.toml"
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(
        f'[esp_idf]\n'
        f'path = "{idf_dir}"\n'
        f'version = "v5.5.2"\n'
        f'\n'
        f'[env]\n'
        f'kind = "dedicated"\n'
        f'root = "{home}"\n'
        f'python = "{python_dir / "python3"}"\n'
        f'python_dir = "{python_dir}"\n'
        f'tools_path = "{home / "espressif"}"\n'
        f'\n'
        f'[project]\n'
        f'default_target = "vehicle"\n'
    )

    result = subprocess.run(
        ["bash", "-c", "source ./setup_env.sh && echo PATH=$PATH && echo IDF=$IDF_PATH && echo TOOLS=$IDF_TOOLS_PATH"],
        cwd=repo,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr

    path_line = re.search(r"^PATH=(.*)$", result.stdout, re.MULTILINE)
    idf_line = re.search(r"^IDF=(.*)$", result.stdout, re.MULTILINE)
    tools_line = re.search(r"^TOOLS=(.*)$", result.stdout, re.MULTILINE)
    assert path_line and idf_line and tools_line, result.stdout

    assert path_line.group(1).startswith(str(python_dir) + ":") or path_line.group(1) == str(python_dir)
    assert idf_line.group(1) == str(idf_dir)
    assert tools_line.group(1) == str(home / "espressif")

    assert "[OK]" in result.stdout
    assert "StampFly development environment ready" in result.stdout
    # No Python discovery/probing output should appear in dedicated mode.
    assert "shell default did not match" not in result.stdout


def test_setup_env_sh_legacy_config_skips_dedicated_branch(tmp_path):
    """A v1 config (no [env] section) must not export IDF_TOOLS_PATH from
    the dedicated branch -- it must go down the legacy (probing) path
    instead. We do not require the legacy path to succeed on this machine
    (it depends on this machine's own system Python happening to match some
    installed ESP-IDF venv); we only require the dedicated branch's
    `export IDF_TOOLS_PATH=...` line never ran.
    """
    home = tmp_path / "home"
    repo = tmp_path / "repo"
    repo.mkdir()

    idf_dir = home / "esp-idf"
    _make_executable(idf_dir / "export.sh", "#!/bin/bash\ntrue\n")

    (repo / "setup_env.sh").write_text(SETUP_ENV_SH.read_text())

    config = repo / ".sf" / "config.toml"
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(
        f'[esp_idf]\n'
        f'path = "{idf_dir}"\n'
        f'version = "v5.5.2"\n'
        f'\n'
        f'[project]\n'
        f'default_target = "vehicle"\n'
    )

    env = dict(os.environ)
    env["IDF_TOOLS_PATH"] = "/sentinel-should-not-change"
    result = subprocess.run(
        ["bash", "-c", "source ./setup_env.sh >/dev/null 2>&1; echo TOOLS=$IDF_TOOLS_PATH"],
        cwd=repo,
        capture_output=True,
        text=True,
        env=env,
    )
    assert "TOOLS=/sentinel-should-not-change" in result.stdout, result.stdout


# ---------------------------------------------------------------------------
# install.sh: offline private-Python bootstrap using the pre-downloaded
# archive (no network access exercised).
# ---------------------------------------------------------------------------


_IS_DARWIN_ARM64 = sys.platform == "darwin" and platform.machine() == "arm64"


@pytest.mark.skipif(not _IS_DARWIN_ARM64, reason="pre-staged archive matches only Darwin arm64")
@pytest.mark.skipif(not PBS_MAC_ARM64_ARCHIVE.exists(), reason=f"pre-downloaded archive missing: {PBS_MAC_ARM64_ARCHIVE}")
def test_install_sh_bootstraps_private_python_offline(tmp_path):
    sf_home = tmp_path / "home"
    downloads = sf_home / "downloads"
    downloads.mkdir(parents=True)
    shutil.copyfile(PBS_MAC_ARM64_ARCHIVE, downloads / PBS_MAC_ARM64_ASSET)

    # Build a minimal PATH with no python/python3 on it (macOS always has
    # /usr/bin/python3, so install.sh's "--help" fast path would otherwise
    # take the "any python3 on PATH" shortcut and never reach
    # bootstrap_private_python at all -- defeating the point of this test).
    # Only the specific external tools install.sh's dedicated/bootstrap path
    # needs are included.
    min_bin = tmp_path / "minbin"
    min_bin.mkdir()
    needed_tools = ("dirname", "grep", "uname", "mkdir", "shasum", "awk", "rm", "tar", "mv", "du", "curl")
    for tool in needed_tools:
        found = shutil.which(tool)
        assert found, f"required tool not found on this machine: {tool}"
        (min_bin / tool).symlink_to(found)

    env = {"HOME": str(tmp_path), "PATH": str(min_bin), "SF_HOME": str(sf_home)}

    result = subprocess.run(
        ["/bin/bash", str(INSTALL_SH), "--help"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "usage: installer.py" in result.stdout, result.stdout
    assert "Reusing downloaded" in result.stdout or "Downloaded and verified" in result.stdout, result.stdout

    private_python = sf_home / "python" / "bin" / "python3"
    assert private_python.exists()

    # Re-run: must reuse the already-provisioned private Python without
    # re-extracting (the fast path's first check, "-x $SF_HOME/python/bin/
    # python3", should short-circuit before calling bootstrap_private_python
    # again). A sentinel file inside python/ surviving proves no re-extract.
    sentinel = sf_home / "python" / "SENTINEL"
    sentinel.write_text("still here")

    result2 = subprocess.run(
        ["/bin/bash", str(INSTALL_SH), "--help"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        env=env,
    )
    assert result2.returncode == 0, result2.stdout + result2.stderr
    assert "usage: installer.py" in result2.stdout, result2.stdout
    assert sentinel.exists(), "private Python was re-extracted on the second run"
    assert "already installed" in result2.stdout or "Using existing private Python" in result2.stdout, result2.stdout


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
