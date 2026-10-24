"""
ESP-IDF utilities for StampFly CLI

Handles ESP-IDF environment setup and command execution.
ESP-IDF環境のセットアップとコマンド実行を処理
"""

import os
import shutil
import sys
import subprocess
from pathlib import Path
from typing import Optional, List
from .platform import platform
from .console import console


def find_idf_path() -> Optional[Path]:
    """Find ESP-IDF installation path"""
    return platform.esp_idf_path()


def idf_command(args: List[str]) -> List[str]:
    """Build command list to invoke idf.py on every platform via
    sys.executable — the ESP-IDF venv Python that sf itself runs from.
    idf.pyを全プラットフォームで sys.executable（sf 自身が動いている
    ESP-IDF venv の Python）経由で起動するコマンドリストを構築

    Windows: idf.py cannot be executed directly (WinError 193).
    Unix: running bare "idf.py" resolves its `#!/usr/bin/env python`
    shebang against PATH, and anything prepended to PATH AFTER
    setup_env.sh ran — e.g. pyenv shims added by .zshrc in the StampFly
    Terminal launcher's interactive handoff shell — shadows the venv
    python with a bare interpreter and idf.py dies with a confusing
    "No module named 'click'" (observed 2026-07-22, macOS). Invoking
    idf.py with sys.executable's absolute path removes that PATH-order
    dependency entirely.
    Windows: idf.py は直接実行できない（WinError 193）。
    Unix: 素の "idf.py" 起動は `#!/usr/bin/env python` shebang を PATH で
    解決するため、setup_env.sh 実行**後**に PATH 先頭へ割り込んだもの
    — 例: StampFly Terminal ランチャーの対話シェル引き継ぎで .zshrc が
    追加する pyenv shims — が venv python を覆い隠し、idf.py が
    「No module named 'click'」という分かりにくいエラーでクラッシュする
    （2026-07-22, macOS で観測）。sys.executable の絶対パスで起動すれば
    この PATH 順序依存を構造的に排除できる。

    Args:
        args: Arguments to pass to idf.py (e.g., ["build"], ["-p", "COM3", "flash"])

    Returns:
        Command list suitable for subprocess.run()
        ['build'] → [sys.executable, '<IDF_PATH>/tools/idf.py', 'build']
    """
    idf_path = find_idf_path()
    if idf_path:
        idf_py = idf_path / "tools" / "idf.py"
        return [sys.executable, str(idf_py)] + args
    # Fallback: try python + idf.py on PATH
    # フォールバック: PATH上のidf.pyをpython経由で実行
    return [sys.executable, "idf.py"] + args


def prepare_idf_env(idf_path: Optional[Path] = None) -> dict:
    """Prepare environment with ESP-IDF settings

    Since sfcli is now installed in ESP-IDF's Python environment,
    we assume the user has already sourced export.sh before running sf commands.
    We simply inherit the current environment.

    Args:
        idf_path: Path to ESP-IDF, or None to auto-detect (unused, kept for compatibility)

    Returns:
        Environment dictionary (copy of current environment)
    """
    # Just use the current environment - user should have sourced export.sh
    return os.environ.copy()


def verify_idf_env(env: dict) -> Optional[str]:
    """Verify that the ESP-IDF environment carried in `env` is loaded and fresh.
    `env` に含まれるESP-IDF環境がロード済みかつ有効(陳腐化していない)か検証する

    prepare_idf_env() just inherits whatever the shell already has. If the
    caller's terminal never sourced setup_env.sh, idf.py is simply missing
    from PATH ("[Errno 2] No such file or directory: 'idf.py'"). If the
    terminal sourced it a long time ago and the referenced ESP-IDF Python
    venv was since deleted/recreated (e.g. by a reinstall), idf.py silently
    falls back to a bare system Python and fails with an unrelated-looking
    "No module named 'click'". Both are confusing to debug from the error
    alone, so this check catches them up front with actionable guidance.

    prepare_idf_env() は現在のシェル環境をそのまま継承するだけである。呼び出し元の
    ターミナルで setup_env.sh が一度も source されていない場合、idf.py が単に
    PATH に無く「[Errno 2] No such file or directory: 'idf.py'」となる。以前に
    source した後、参照先の ESP-IDF Python venv が削除・再作成された場合
    （再インストール等）は、idf.py が素の system Python にフォールバックし、
    一見無関係な「No module named 'click'」で失敗する。どちらもエラーメッセージ
    単体では原因が分かりにくいため、実行前にこの検証で検出し対処法を提示する。

    Args:
        env: Environment dict to verify (as returned by prepare_idf_env()).

    Returns:
        None if the environment looks usable. Otherwise a bilingual,
        actionable error message ready to hand to console.error().
    """
    venv_path_str = env.get("IDF_PYTHON_ENV_PATH")
    if not venv_path_str:
        return (
            "ESP-IDF environment is not loaded in this terminal.\n"
            "ESP-IDF環境がこのターミナルにロードされていません。\n"
            "  -> Run 'source setup_env.sh' in this terminal, then retry.\n"
            "  -> このターミナルで 'source setup_env.sh' を実行してから再試行してください。"
        )

    venv_dir = Path(venv_path_str)
    bin_subdir = "Scripts" if platform.is_windows() else "bin"
    python_name = "python.exe" if platform.is_windows() else "python"
    venv_python = venv_dir / bin_subdir / python_name

    if not venv_dir.is_dir() or not venv_python.is_file():
        return (
            "This terminal references a deleted/stale ESP-IDF Python "
            f"environment ({venv_path_str}).\n"
            "このターミナルは削除済みまたは古いESP-IDF Python環境を参照しています "
            f"({venv_path_str})。\n"
            "  -> Open a NEW terminal and run 'source setup_env.sh' again.\n"
            "  -> 新しいターミナルを開いて 'source setup_env.sh' をやり直してください。"
        )

    # idf_command() invokes 'idf.py' via PATH on Unix (via sys.executable on
    # Windows, so PATH is irrelevant there).
    # idf_command() はUnixではPATH経由でidf.pyを起動する
    # （Windowsではsys.executable経由のためPATHは無関係）
    if not platform.is_windows():
        if shutil.which("idf.py", path=env.get("PATH")) is None:
            return (
                "idf.py was not found on PATH.\n"
                "idf.py がPATH上に見つかりません。\n"
                "  -> Run 'source setup_env.sh' in this terminal, then retry.\n"
                "  -> このターミナルで 'source setup_env.sh' を実行してから再試行してください。"
            )

    return None


def run_idf_command(
    cmd: list,
    cwd: Path,
    idf_path: Optional[Path] = None,
) -> subprocess.CompletedProcess:
    """Run an idf.py command with proper environment

    Args:
        cmd: Command and arguments (e.g., ["idf.py", "build"])
        cwd: Working directory
        idf_path: Path to ESP-IDF, or None to auto-detect

    Returns:
        CompletedProcess from subprocess.run
    """
    if idf_path is None:
        idf_path = find_idf_path()

    env = prepare_idf_env(idf_path)

    # Use idf_command() for cross-platform compatibility
    # クロスプラットフォーム対応のため idf_command() を使用
    if cmd and cmd[0] == "idf.py":
        cmd = idf_command(cmd[1:])

    return subprocess.run(cmd, cwd=cwd, env=env)


# Maximum number of mismatched keys listed in the warning.
# 警告に列挙する食い違いキーの最大数。
MAX_SDKCONFIG_MISMATCHES_SHOWN = 10

# Value used for "not set" after normalization.
# 正規化後の「未設定」を表す値。
_SDKCONFIG_NOT_SET = "n"


def _parse_sdkconfig_file(path: Path) -> dict:
    """Parse a sdkconfig / sdkconfig.defaults file into {key: value}.

    `CONFIG_X=value` -> value (strings keep their quotes);
    `# CONFIG_X is not set` and `CONFIG_X=n` -> "n". Other lines are ignored.
    sdkconfig 系ファイルを {キー: 値} に変換する。`CONFIG_X=値` は値そのまま
    （文字列は引用符付き）、`# CONFIG_X is not set` と `CONFIG_X=n` は "n"。
    他の行は無視する。
    """
    result = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if line.startswith("# CONFIG_") and line.endswith(" is not set"):
            key = line[len("# "):-len(" is not set")].strip()
            result[key] = _SDKCONFIG_NOT_SET
        elif line.startswith("CONFIG_") and "=" in line:
            key, value = line.split("=", 1)
            result[key.strip()] = value.strip()
    return result


def find_sdkconfig_mismatches(project_dir: Path) -> list:
    """Return [(key, defaults_value, sdkconfig_value)] for every key set in
    sdkconfig.defaults (and sdkconfig.defaults.<target>, which overrides it)
    whose value differs in project_dir/sdkconfig. Returns [] when either file
    is missing or unreadable: this check must never break a build.
    sdkconfig.defaults（と、それを上書きする sdkconfig.defaults.<target>）で
    設定されたキーのうち、project_dir/sdkconfig で値が異なるものを
    [(キー, defaults の値, sdkconfig の値)] で返す。どちらかのファイルが無い・
    読めない場合は [] を返す（この検査でビルドを壊さないため）。
    """
    sdkconfig_path = project_dir / "sdkconfig"
    defaults_path = project_dir / "sdkconfig.defaults"
    if not sdkconfig_path.is_file() or not defaults_path.is_file():
        return []
    try:
        current = _parse_sdkconfig_file(sdkconfig_path)
        defaults = _parse_sdkconfig_file(defaults_path)
        # ESP-IDF also loads sdkconfig.defaults.<target>; it overrides the base.
        # ESP-IDF は sdkconfig.defaults.<target> も読み、基本ファイルを上書きする。
        target = current.get("CONFIG_IDF_TARGET", "").strip('"')
        target_path = project_dir / f"sdkconfig.defaults.{target}"
        if target and target_path.is_file():
            defaults.update(_parse_sdkconfig_file(target_path))
    except (OSError, UnicodeDecodeError):
        return []

    mismatches = []
    for key, default_value in defaults.items():
        # A key absent from sdkconfig is skipped: it stays absent even in a
        # freshly generated sdkconfig (unmet Kconfig dependency, or a symbol
        # this IDF version does not know), so the warning would have no remedy.
        # sdkconfig に無いキーは報告しない。新規生成した sdkconfig でも無いまま
        # （Kconfig の依存条件が不成立、またはこの IDF が知らないシンボル）で、
        # 警告しても対処のしようがないため。
        if key not in current:
            continue
        if current[key] != default_value:
            mismatches.append((key, default_value, current[key]))
    return mismatches


def _describe_sdkconfig_value(value: str) -> str:
    """Format a normalized value for the warning ("n" -> "n (not set)").
    警告用に値を整形する（"n" は "n (not set)" と表示）。
    """
    return f"{value} (not set)" if value == _SDKCONFIG_NOT_SET else value


def warn_sdkconfig_mismatches(project_dir: Path, mismatches: list) -> None:
    """Print the stale-sdkconfig warning. Does nothing if mismatches is empty.
    古い sdkconfig の警告を表示する。mismatches が空なら何もしない。
    """
    if not mismatches:
        return
    sdkconfig_path = Path(project_dir) / "sdkconfig"
    # The heading goes to stderr and the body to stdout. Flush stdout on both
    # sides so the lines stay in order and ahead of idf.py's output even when
    # piped (stdout is block-buffered there).
    # 見出しは stderr、本文は stdout に出る。パイプ時は stdout がまとめて
    # 書き出されるため、前後で stdout を書き出して行の順序を保ち、idf.py の
    # 出力より前に表示されるようにする。
    sys.stdout.flush()
    console.warning(
        f"{sdkconfig_path} does not match sdkconfig.defaults:", prefix="WARNING"
    )
    for key, default_value, current_value in mismatches[:MAX_SDKCONFIG_MISMATCHES_SHOWN]:
        console.print(
            f"    {key}: defaults={_describe_sdkconfig_value(default_value)}, "
            f"sdkconfig={_describe_sdkconfig_value(current_value)}"
        )
    hidden = len(mismatches) - MAX_SDKCONFIG_MISMATCHES_SHOWN
    if hidden > 0:
        console.print(f"    ... and {hidden} more")
    console.print("  ESP-IDF reads sdkconfig.defaults only when sdkconfig does not exist, so an")
    console.print("  sdkconfig created before the defaults changed keeps the old settings.")
    console.print("  `sf build --clean` does not regenerate it.")
    console.print("  To regenerate it, rename or delete this file and build again")
    console.print("  (keep the renamed copy if you edited it by hand):")
    console.print(f"    {sdkconfig_path}")
    console.print("  If you changed these settings on purpose (menuconfig), you can ignore this.")
    sys.stdout.flush()
