"""
sf params - Physical parameter consistency audit
sf params - 物理パラメータ整合検査

Thin wrapper around tools/params_audit (see that directory's README.md).
Checks that hand-copied vehicle physical parameters (C_T, C_Q, kappa,
inertia, ...) agree across firmware/SIL/simulator source files.
tools/params_audit の薄いラッパー（詳細は同ディレクトリの README.md）。
ファーム/SIL/シミュレータの各ソースに手動複製された機体物理パラメータ
（C_T, C_Q, kappa, 慣性 等）が一致しているかを検査する。

Subcommands:
    check      - Run the parameter consistency audit
    generate   - Regenerate physical-parameter code from control/models/stampfly_physical.yaml
"""

import argparse
import sys

from ..utils import console, paths

COMMAND_NAME = "params"
COMMAND_HELP = "Physical parameter consistency audit (C_T, C_Q, kappa, inertia, ...)"


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="params_command", title="subcommands", metavar="<subcommand>")

    check_parser = sub.add_parser(
        "check",
        help="Audit hand-copied physical parameters for consistency",
        description="Re-read every manifested parameter copy (tools/params_audit/"
                     "params_manifest.py) and report OK/MISMATCH/UNRESOLVED/EXEMPT/ERROR "
                     "per location.",
    )
    check_parser.add_argument(
        "--json", action="store_true",
        help="emit machine-readable JSON instead of a text table",
    )
    check_parser.add_argument(
        "--strict", action="store_true",
        help="also fail (exit 1) when any parameter is UNRESOLVED",
    )
    check_parser.set_defaults(func=run_check)

    generate_parser = sub.add_parser(
        "generate",
        help="Regenerate physical-parameter code from the SSOT YAML",
        description="Read control/models/stampfly_physical.yaml (the Single Source of "
                     "Truth) and regenerate tools/sysid/_generated_params.py, "
                     "simulator/sil/plant/generated_params.hpp, and the "
                     "<!-- AUTO-GENERATED:params --> marker table in "
                     "docs/architecture/stampfly-parameters.md.",
    )
    generate_parser.add_argument(
        "--check", action="store_true",
        help="do not write anything; exit 1 if generated files are stale "
             "(the YAML changed without regenerating)",
    )
    generate_parser.set_defaults(func=run_generate)

    parser.set_defaults(func=lambda a: (parser.print_help(), 0)[1])


def run_check(args: argparse.Namespace) -> int:
    """Run the parameter consistency audit (tools/params_audit/check_params.py).
    パラメータ整合検査を実行する（tools/params_audit/check_params.py）。

    Import happens with tools/ temporarily on sys.path — same pattern as
    sf sysid / sf sil (see lib/sfcli/commands/sysid.py) — so a missing or
    broken params_audit module only disables `sf params`, never `sf` itself.
    tools/ を一時的に sys.path に載せて import する（sf sysid / sf sil と
    同じ流儀、lib/sfcli/commands/sysid.py 参照）— params_audit が壊れていても
    `sf params` だけが無効化され、`sf` 全体は道連れにならない。
    """
    tools_dir = str(paths.root() / "tools")
    sys.path.insert(0, tools_dir)
    try:
        from params_audit import check_params
    except ImportError as e:
        console.error(f"Failed to import params_audit.check_params: {e}")
        return 1
    finally:
        if tools_dir in sys.path:
            sys.path.remove(tools_dir)

    argv = []
    if args.json:
        argv.append("--json")
    if args.strict:
        argv.append("--strict")
    return check_params.main(argv)


def run_generate(args: argparse.Namespace) -> int:
    """Run the code generator (tools/params_audit/generate.py).
    コード生成器を実行する（tools/params_audit/generate.py）。

    Same tools/-on-sys.path import pattern as run_check() above -- a missing
    or broken params_audit/generate.py (or a missing PyYAML) only disables
    `sf params generate`, never `sf` itself.
    run_check() と同じ tools/ を sys.path に載せる import パターン --
    params_audit/generate.py の欠落・破損（または PyYAML 未導入）があっても
    `sf params generate` だけが無効化され、`sf` 全体は道連れにならない。
    """
    tools_dir = str(paths.root() / "tools")
    sys.path.insert(0, tools_dir)
    try:
        from params_audit import generate
    except ImportError as e:
        console.error(f"Failed to import params_audit.generate: {e}")
        return 1
    finally:
        if tools_dir in sys.path:
            sys.path.remove(tools_dir)

    argv = []
    if args.check:
        argv.append("--check")
    return generate.main(argv)
