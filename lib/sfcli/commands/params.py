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
    check   - Run the parameter consistency audit
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
