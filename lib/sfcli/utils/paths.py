"""
Path utilities for StampFly CLI

Provides consistent path resolution across the project.
プロジェクト全体で一貫したパス解決を提供
"""

import os
from pathlib import Path
from typing import Optional

# Env var an externally re-exec'd copy of `sf` can set to force root() to a
# specific checkout instead of walking up from this file's location. See
# root()'s docstring for why this exists. The string value must be kept in
# sync by hand with lib/sfcli/commands/upgrade.py's ROOT_OVERRIDE_ENV --
# paths.py must not import a command module (that would invert the
# dependency direction), so the two constants cannot share a definition.
# 外部から再実行された `sf` のコピーが、__file__ から上へ辿る通常のロジック
# の代わりにルートを特定のチェックアウトへ強制するための環境変数。存在理由は
# root() の docstring を参照。値は lib/sfcli/commands/upgrade.py の
# ROOT_OVERRIDE_ENV と手動で一致させ続ける必要がある -- paths.py がコマンド
# モジュールを import することは依存方向が逆転するため許されず、2つの定数は
# 定義を共有できない。
_ROOT_OVERRIDE_ENV_VAR = "SF_ROOT_OVERRIDE"


class Paths:
    """Path manager for StampFly Ecosystem"""

    def __init__(self):
        self._root: Optional[Path] = None

    def _find_root(self) -> Path:
        """Find the repository root directory"""
        if self._root is not None:
            return self._root

        # Start from this file's location
        current = Path(__file__).resolve()

        # Walk up looking for markers
        markers = [".git", "CLAUDE.md", "PROJECT_PLAN.md"]

        for parent in [current] + list(current.parents):
            for marker in markers:
                if (parent / marker).exists():
                    self._root = parent
                    return self._root

        # Fallback: use environment variable or current directory
        if "STAMPFLY_ROOT" in os.environ:
            self._root = Path(os.environ["STAMPFLY_ROOT"])
        else:
            self._root = Path.cwd()

        return self._root

    def root(self) -> Path:
        """Get repository root directory.
        リポジトリルートディレクトリを取得する

        Internal mechanism: if the SF_ROOT_OVERRIDE environment variable is
        set to a path that exists and contains a `.git` entry, that path is
        returned directly (resolved) instead of walking up from this
        file's location. Any other value (unset, missing path, no `.git`)
        is ignored silently and falls through to the normal logic below.

        This exists for `sf upgrade`'s self-bootstrap
        (lib/sfcli/commands/upgrade.py): when it detects that `sf` itself
        changed upstream, it re-execs the freshly fetched code from a
        temporary extraction directory that has no `.git` of its own, so
        it cannot self-locate the real checkout by walking up from
        __file__ -- the re-exec sets SF_ROOT_OVERRIDE so it can still find
        it.
        内部機構: 環境変数 SF_ROOT_OVERRIDE が、存在し `.git` を含むパスに
        設定されている場合、__file__ から上へ辿る通常のロジックの代わりに
        そのパスをそのまま（resolve済みで）返す。それ以外の値（未設定・
        パス不在・`.git` 無し）は黙って無視し、下記の通常ロジックへ
        フォールバックする。

        これは `sf upgrade` の自己ブートストラップ
        （lib/sfcli/commands/upgrade.py）のために存在する: `sf` 自身が
        上流で更新されたと検知すると、`.git` を持たない一時展開
        ディレクトリから取得したばかりのコードを再実行するため、
        __file__ から上へ辿って本来のチェックアウトを自力で特定できない
        -- 再実行時に SF_ROOT_OVERRIDE を設定することで、それでも実体を
        見つけられるようにする。
        """
        override = os.environ.get(_ROOT_OVERRIDE_ENV_VAR)
        if override:
            override_path = Path(override)
            if override_path.exists() and (override_path / ".git").exists():
                return override_path.resolve()
        return self._find_root()

    def lib(self) -> Path:
        """Get lib/ directory"""
        return self.root() / "lib"

    def bin(self) -> Path:
        """Get bin/ directory"""
        return self.root() / "bin"

    def scripts(self) -> Path:
        """Get scripts/ directory"""
        return self.root() / "scripts"

    def firmware(self) -> Path:
        """Get firmware/ directory"""
        return self.root() / "firmware"

    def vehicle(self) -> Path:
        """Get firmware/vehicle/ directory"""
        return self.firmware() / "vehicle"

    def vehicle_old(self) -> Path:
        """Get firmware/vehicle_old/ directory (legacy firmware)"""
        return self.firmware() / "vehicle_old"

    def controller(self) -> Path:
        """Get firmware/controller/ directory"""
        return self.firmware() / "controller"

    def workshop(self) -> Path:
        """Get firmware/workshop/ directory"""
        return self.firmware() / "workshop"

    def apps(self) -> Path:
        """Get firmware/apps/ directory (user-created projects, cloned from
        firmware/vehicle/examples/ via `sf app new`).
        firmware/apps/ ディレクトリ（`sf app new` で
        firmware/vehicle/examples/ から複製したユーザー自作プロジェクト）"""
        return self.firmware() / "apps"

    def simulator(self) -> Path:
        """Get simulator/ directory"""
        return self.root() / "simulator"

    def sils_build(self) -> Path:
        """Get simulator/sils/build/ directory (host SILS build output)."""
        return self.simulator() / "sils" / "build"

    def tools(self) -> Path:
        """Get tools/ directory"""
        return self.root() / "tools"

    def docs(self) -> Path:
        """Get docs/ directory"""
        return self.root() / "docs"

    def logs(self) -> Path:
        """Get log storage directory (created if not exists)"""
        log_dir = self.root() / "logs"
        log_dir.mkdir(exist_ok=True)
        return log_dir

    def config_dir(self) -> Path:
        """Get .sf/ configuration directory"""
        return self.root() / ".sf"

    def esp_idf(self) -> Optional[Path]:
        """Get ESP-IDF directory (searches multiple locations)"""
        # Check local installation first
        local_idf = self.root() / ".esp-idf"
        if local_idf.exists():
            return local_idf

        # Check symlink
        if local_idf.is_symlink():
            return local_idf.resolve()

        # Check .sf/config.toml -- scripts/installer.py records the
        # ESP-IDF path there for both the legacy and dedicated flows
        # (see docs/plans/dedicated-environment-plan.md). Must exist on
        # disk to count: a stale/hand-edited config entry must not shadow
        # a real ESP-IDF the fallback searches below would have found.
        # .sf/config.toml を確認する -- scripts/installer.py は旧来・専用
        # どちらのフローでもESP-IDFパスをここに記録する
        # (docs/plans/dedicated-environment-plan.md 参照)。実在するパスの
        # みを採用する -- 古い/手編集された設定項目が、下のフォールバック
        # 探索で見つかったはずの実在するESP-IDFを覆い隠さないようにする。
        config_idf_path = self.read_config_value("esp_idf", "path")
        if config_idf_path:
            config_idf = Path(config_idf_path)
            if config_idf.exists():
                return config_idf

        # Check common locations
        common_paths = [
            Path.home() / "esp" / "esp-idf",
            Path.home() / ".espressif" / "esp-idf",
            Path("/opt/esp-idf"),
        ]

        for p in common_paths:
            if p.exists():
                return p

        # Check environment variable
        if "IDF_PATH" in os.environ:
            return Path(os.environ["IDF_PATH"])

        return None

    def config_file(self) -> Path:
        """Get CLI configuration file path"""
        return self.config_dir() / "config.toml"

    def read_config(self) -> dict:
        """Minimal `.sf/config.toml` reader: parses `[section]` headers
        and `key = "value"` lines into {section: {key: value}}, ignoring
        comments (#) and blank lines. Returns {} if the file is missing
        or unreadable.

        Deliberately reimplements scripts/installer.py's identically
        -behaved `read_config()` rather than importing it: sfcli must not
        import that standalone top-level script (see this module's own
        `_ROOT_OVERRIDE_ENV_VAR` comment on why paths.py cannot depend on
        code outside the sfcli package), and installer.py explicitly
        keeps its own copy stdlib-only and independent for the same
        reason in reverse. Not a full TOML parser (no arrays/tables/
        multi-line strings) -- config.toml only ever holds flat quoted
        -string values (see installer.py's `_save_config()`), so this is
        deliberately just enough for that shape.
        最小限の `.sf/config.toml` リーダー: `[section]` 見出しと
        `key = "value"` 行を {section: {key: value}} に解析する。コメント
        (#)と空行は無視する。ファイルが無い/読めない場合は {} を返す。

        scripts/installer.py の同じ振る舞いの read_config() を意図的に
        import せず再実装する: sfcli はそのスタンドアロンなトップレベル
        スクリプトに依存してはならず(理由はこのモジュール自身の
        `_ROOT_OVERRIDE_ENV_VAR` のコメント参照)、installer.py 側も同じ
        理由の裏返しで自身のコピーを標準ライブラリのみ・独立に保っている。
        完全なTOMLパーサーではない(配列/テーブル/複数行文字列非対応) --
        config.toml は常にフラットな引用符付き文字列値のみを持つ
        (installer.py の `_save_config()` 参照)ため、意図的にその形に
        限定している。
        """
        sections: dict = {}
        try:
            text = self.config_file().read_text(encoding="utf-8")
        except OSError:
            return sections

        current: Optional[str] = None
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("[") and line.endswith("]"):
                current = line[1:-1].strip()
                sections.setdefault(current, {})
                continue
            if current is None or "=" not in line:
                continue
            key, _, value = line.partition("=")
            key = key.strip()
            value = value.strip()
            if len(value) >= 2 and value[0] == '"' and value[-1] == '"':
                value = value[1:-1]
            sections[current][key] = value
        return sections

    def read_config_value(self, section: str, key: str) -> Optional[str]:
        """Shortcut for `read_config()[section][key]`, or None if either
        the section or the key is absent.
        `read_config()[section][key]` の簡略形。sectionまたはkeyが
        無ければNone。
        """
        return self.read_config().get(section, {}).get(key)

    def dedicated_env(self) -> Optional[dict]:
        """The `[env]` section of `.sf/config.toml` if this checkout uses
        the dedicated environment (`kind == "dedicated"`), else None --
        covers both a legacy environment (`kind == "legacy"`) and a
        pre-v2 config with no `[env]` section at all.
        `.sf/config.toml` の `[env]` セクションを返す。このチェックアウトが
        専用環境(`kind == "dedicated"`)を使っている場合のみ内容を返し、
        それ以外(旧来環境 `kind == "legacy"`、または `[env]` 節自体が無い
        v1以前の設定)は None を返す。
        """
        env_section = self.read_config().get("env", {})
        if env_section.get("kind") == "dedicated":
            return env_section
        return None

    def templates(self) -> Path:
        """Get templates directory"""
        return self.lib() / "sfcli" / "templates"

    def get_firmware_targets(self) -> list[str]:
        """Get all firmware targets (directories with CMakeLists.txt)"""
        fw_dir = self.firmware()
        targets = []
        for d in sorted(fw_dir.iterdir()):
            if d.is_dir() and (d / "CMakeLists.txt").exists():
                targets.append(d.name)
        return targets

    def firmware_target_dir(self, target: str) -> Path:
        """Get directory for a firmware target by name"""
        return self.firmware() / target

    def ensure_dir(self, path: Path) -> Path:
        """Ensure directory exists, create if not"""
        path.mkdir(parents=True, exist_ok=True)
        return path


# Global paths instance
paths = Paths()
