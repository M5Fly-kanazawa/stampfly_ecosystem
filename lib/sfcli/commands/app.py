"""
sf app - Manage custom firmware applications

Create and manage custom firmware projects in the firmware/ directory.
firmware/ディレクトリ内のカスタムファームウェアプロジェクトを管理します。
"""

import argparse
import shutil
from ..utils import console, paths

COMMAND_NAME = "app"
COMMAND_HELP = "Manage custom firmware applications"

# Reserved names that cannot be used for custom apps
# カスタムアプリに使用できない予約名
RESERVED_NAMES = {"vehicle", "vehicle_old", "controller", "workshop", "common"}


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
    )
    sub = parser.add_subparsers(dest="action", help="Action to perform")

    # sf app new <name>
    new_parser = sub.add_parser("new", help="Create a new firmware project")
    new_parser.add_argument("name", help="Project name (e.g. my_drone)")

    # sf app list
    sub.add_parser("list", help="List custom firmware projects")

    parser.set_defaults(func=run)


def run(args: argparse.Namespace) -> int:
    """Execute app command"""
    if args.action == "new":
        return _app_new(args.name)
    elif args.action == "list":
        return _app_list()
    else:
        console.error("Usage: sf app new <name> | sf app list")
        return 1


def _app_new(name: str) -> int:
    """Create a new firmware project from template"""
    # Validate name / 名前を検証
    if not name.isidentifier():
        console.error(f"Invalid project name: '{name}' (must be a valid C identifier)")
        return 1

    if name in RESERVED_NAMES:
        console.error(f"Reserved name: '{name}' (cannot use {', '.join(sorted(RESERVED_NAMES))})")
        return 1

    project_dir = paths.firmware() / name
    if project_dir.exists():
        console.error(f"Project already exists: {project_dir}")
        return 1

    template_dir = paths.templates() / "custom_firmware"
    if not template_dir.exists():
        console.error(f"Template not found: {template_dir}")
        return 1

    console.info(f"Creating new firmware project: {name}")

    # Copy template / テンプレートをコピー
    shutil.copytree(template_dir, project_dir)

    # Replace {{PROJECT_NAME}} placeholder / プレースホルダを置換
    for file_path in project_dir.rglob("*"):
        if file_path.is_file():
            try:
                content = file_path.read_text(encoding="utf-8")
                if "{{PROJECT_NAME}}" in content:
                    content = content.replace("{{PROJECT_NAME}}", name)
                    file_path.write_text(content, encoding="utf-8")
            except UnicodeDecodeError:
                pass  # Skip binary files / バイナリファイルはスキップ

    main_dir = project_dir / "main"

    # Copy build config files from vehicle
    # vehicleからビルド設定ファイルをコピー
    vehicle_dir = paths.vehicle()
    for filename in ["sdkconfig.defaults", "partitions.csv"]:
        src = vehicle_dir / filename
        if src.exists():
            shutil.copy2(src, project_dir / filename)
            console.print(f"  Copied {filename} from vehicle/")

    # Copy idf_component.yml from vehicle/main (for led_strip dependency)
    # vehicleのidf_component.ymlをコピー（led_strip依存解決用）
    idf_yml_src = vehicle_dir / "main" / "idf_component.yml"
    if idf_yml_src.exists():
        shutil.copy2(idf_yml_src, main_dir / "idf_component.yml")
        console.print(f"  Copied idf_component.yml from vehicle/main/")

    console.success(f"Project created: {project_dir}")
    console.print()
    console.print("Next steps:")
    console.print(f"  1. Edit {project_dir / 'main' / 'main.cpp'}")
    console.print(f"  2. sf build {name}")
    console.print(f"  3. sf flash {name} -m")

    return 0


def _app_list() -> int:
    """List custom firmware projects"""
    fw_dir = paths.firmware()
    if not fw_dir.exists():
        console.error(f"Firmware directory not found: {fw_dir}")
        return 1

    console.info("Firmware projects:")
    found = False
    for d in sorted(fw_dir.iterdir()):
        if d.is_dir() and (d / "CMakeLists.txt").exists():
            marker = " (built-in)" if d.name in RESERVED_NAMES else " (custom)"
            console.print(f"  {d.name}{marker}")
            found = True

    if not found:
        console.print("  (no projects found)")

    return 0
