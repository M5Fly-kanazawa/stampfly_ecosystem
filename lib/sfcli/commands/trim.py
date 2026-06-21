"""
sf trim - Equilibrium attitude trim identification from hover logs
sf trim - ホバリングログからの平衡姿勢トリム同定

The true "level" that lets the craft hover in place cannot be measured on the
ground: it depends on CG offset, thrust/airframe asymmetry, and the estimator's
gravity reference — none observable at rest, and the floor is never perfectly
level. This tool identifies it from a STABILIZE/ALT_HOLD hover FLIGHT log, so the
pilot can iterate trim.roll/trim.pitch to zero out steady horizontal drift
("bita-hover" — a rock-steady hover that does not creep). In a tight room you
need NOT stay hands-off: pilot to keep off the walls and the estimate folds your
average correction into the trim (it uses the MEAN HELD ATTITUDE, not just drift).

機体がその場でホバリングする真の「水平」は地上で測れない: CG オフセット・推力/機体の
非対称・推定器の重力基準に依存し、いずれも静止では観測できず、床も完全水平ではない。
本ツールは STABILIZE/ALT_HOLD のホバリング「飛行」ログから同定し、trim.roll/trim.pitch
を反復調整して定常水平ドリフト（ビタホバ）を消せるようにする。狭い室内では手放しを保つ
必要はない: 壁を避けるため操縦してよく、推定は機体が保った「平均姿勢」を使うのでパイロット
の平均修正をトリムに織り込む（ドリフトだけに頼らない）。

Subcommands:
    analyze - Compute the trim correction from a hover log
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from ..utils import console, paths

COMMAND_NAME = "trim"
COMMAND_HELP = "Identify equilibrium attitude trim from hover logs"

G = 9.80665  # gravity [m/s^2] / 重力加速度


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register the trim command / trim コマンドを登録"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(
        dest="trim_command", title="subcommands", metavar="<subcommand>"
    )

    analyze = sub.add_parser(
        "analyze",
        help="Compute trim correction from a hover log",
        description="Analyze a STABILIZE/ALT_HOLD hover log and suggest "
                    "attitude.roll.trim / attitude.pitch.trim.",
    )
    analyze.add_argument(
        "input", nargs="?",
        help="JSON-Lines hover log (.jsonl). Omit to use the latest log in logs/",
    )
    analyze.add_argument("-o", "--output", help="Save the JSON report to this file")
    analyze.add_argument(
        "--hover-start", type=float, default=6.0,
        help="Seconds to skip at the start (takeoff transient). Default 6.0",
    )
    analyze.add_argument(
        "--hover-duration", type=float,
        help="Analysis window length [s]. Default: to end-0.5s",
    )
    analyze.add_argument(
        "--current-roll", type=float, default=0.0,
        help="Currently-applied attitude.roll.trim [rad] (iteration: new = current + delta)",
    )
    analyze.add_argument(
        "--current-pitch", type=float, default=0.0,
        help="Currently-applied attitude.pitch.trim [rad]",
    )
    analyze.set_defaults(func=run_analyze)

    parser.set_defaults(func=run_help)


def run_help(args: argparse.Namespace) -> int:
    """Show help when no subcommand is given / サブコマンド無しでヘルプ表示"""
    console.print("Usage: sf trim <subcommand> [options]")
    console.print()
    console.print("Subcommands:")
    console.print("  analyze   Compute the trim correction from a hover log")
    console.print()
    console.print("Pilot a hover (you may correct to stay off the walls), then:")
    console.print("  sf trim analyze            # uses the latest log in logs/")
    console.print()
    console.print("Iteration (pass the trim you already applied):")
    console.print("  sf trim analyze hover2.jsonl --current-roll 0.012 --current-pitch -0.004")
    return 0


def run_analyze(args: argparse.Namespace) -> int:
    """Run the trim analysis and print the report / トリム解析を実行し報告"""
    # No file given -> use the most recent log in logs/ (like sf cal / sf log).
    # ファイル無指定なら logs/ の最新ログを使う（sf cal / sf log と同様）。
    log_path = args.input or _find_latest_log()
    if not log_path:
        console.error("No log given and no .jsonl found in logs/ - specify a file.")
        return 1
    if not args.input:
        console.info(f"Using latest log: {Path(log_path).name}")
    try:
        result = analyze_trim(
            log_path,
            hover_start_s=args.hover_start,
            hover_duration_s=args.hover_duration,
            current_roll=args.current_roll,
            current_pitch=args.current_pitch,
        )
    except FileNotFoundError:
        console.error(f"Log not found: {log_path}")
        return 1
    except Exception as e:  # noqa: BLE001 - surface any analysis error to the user
        console.error(f"Trim analysis failed: {e}")
        return 1

    _print_report(result)

    if args.output:
        Path(args.output).write_text(json.dumps(result, indent=2))
        console.success(f"Report saved: {args.output}")
    return 0


# ---------------------------------------------------------------------------
# Core analysis
# ---------------------------------------------------------------------------

def _find_latest_log():
    """The most-recent *.jsonl in the repo logs/ dir, or None.
    リポジトリ logs/ の最新 *.jsonl（無ければ None）。"""
    logs_dir = paths.root() / "logs"
    if not logs_dir.exists():
        return None
    files = sorted(logs_dir.glob("*.jsonl"),
                   key=lambda f: f.stat().st_mtime, reverse=True)
    return str(files[0]) if files else None


def _load_jsonl(path: str) -> dict:
    """Load a JSON-Lines log grouped by record id / レコード id 別に読み込み"""
    grouped: dict = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            grouped.setdefault(d["id"], []).append(d)
    return grouped


def _q2eul(q):
    """Quaternion [w,x,y,z] -> (roll, pitch, yaw) [rad]. Same convention as
    altlog_sysid_eskf.py. / クォータニオン→オイラー角（altlog と同一規約）。"""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def analyze_trim(log_path, hover_start_s=6.0, hover_duration_s=None,
                 current_roll=0.0, current_pitch=0.0) -> dict:
    """Identify the equilibrium attitude trim from a hover log.
    ホバリングログから平衡姿勢トリムを同定する。"""
    S = _load_jsonl(log_path)
    if "imu" not in S or "posvel" not in S:
        raise ValueError("log is missing 'imu' or 'posvel' records")

    imu = S["imu"]
    t0 = imu[0]["ts"]
    t_imu = np.array([(d["ts"] - t0) / 1e6 for d in imu])
    quat = np.array([d["quat"] for d in imu])

    posvel = S["posvel"]
    t_pv = np.array([(d["ts"] - t0) / 1e6 for d in posvel])
    pos = np.array([d["pos"] for d in posvel])
    vel = np.array([d["vel"] for d in posvel])

    # Hover window: skip the takeoff transient and a short landing tail (mirrors
    # altlog_sysid_eskf.py). / ホバー区間: 離陸過渡と着陸直前を除外（altlog に倣う）。
    t_end = t_imu[-1] - 0.5
    if hover_duration_s:
        t_end = min(t_end, hover_start_s + hover_duration_s)
    duration = t_end - hover_start_s
    if duration < 10.0:
        raise ValueError(f"hover window too short: {duration:.1f}s (need >=10s)")

    sel_pv = (t_pv >= hover_start_s) & (t_pv <= t_end)
    sel_imu = (t_imu >= hover_start_s) & (t_imu <= t_end)
    t_f = t_pv[sel_pv]
    pos_f = pos[sel_pv]
    vel_f = vel[sel_pv]
    quat_f = quat[sel_imu]
    if t_f.size < 10:
        raise ValueError("not enough posvel samples in the hover window")

    # Mean horizontal acceleration = slope of a linear fit to the ESKF NED velocity
    # over the window (robust to endpoint noise; the ESKF velocity is flow-aided,
    # std ~0.11 m/s). NED order is [north, east, down].
    # 平均水平加速度 = 区間の ESKF NED 速度を線形フィットした傾き（端点ノイズに強い・
    # フロー併用で std~0.11）。NED は [北, 東, 下]。
    a_north = float(np.polyfit(t_f, vel_f[:, 0], 1)[0])
    a_east = float(np.polyfit(t_f, vel_f[:, 1], 1)[0])

    # Circular mean yaw; mean roll/pitch = the attitude the craft actually held
    # (the pilot's hand corrections are baked in) — used by the trim estimate below.
    # ヨーの円周平均。roll/pitch 平均 = 機体が実際に保った姿勢（パイロットの手修正が
    # 織り込まれる）— 下のトリム推定で使う。
    eul = np.array([_q2eul(q) for q in quat_f])
    yaw_mean = math.atan2(float(np.mean(np.sin(eul[:, 2]))),
                          float(np.mean(np.cos(eul[:, 2]))))
    roll_mean = float(np.mean(eul[:, 0]))
    pitch_mean = float(np.mean(eul[:, 1]))

    # Rotate the OBSERVED NED drift acceleration into the body FRD frame (yaw only;
    # roll/pitch ~0 at hover). Identical rotation to pid_controller.cpp
    # computePositionHold (L691-693), so body axes match the firmware exactly.
    # 観測 NED ドリフト加速度を機体 FRD へ回転（ヨーのみ・ホバーで roll/pitch~0）。
    # pid_controller の computePositionHold と同一回転ゆえ機体軸はファームと一致。
    cy, sy = math.cos(yaw_mean), math.sin(yaw_mean)
    ax_body = cy * a_north + sy * a_east     # forward (FRD X) / 前方
    ay_body = -sy * a_north + cy * a_east    # right   (FRD Y) / 右

    # --- Equilibrium trim (works hands-off OR while actively piloting) ------
    # The trim that makes the craft bita-hover is the ATTITUDE it must hold at
    # stick-neutral, so:
    #     trim = (mean attitude the craft held) - (residual drift accel)/g
    # In a tight room you cannot stay hands-off; you nudge the sticks to keep off
    # the walls. Those corrections are baked into the held attitude (euler), so
    # averaging the attitude already captures the trim the pilot supplied by hand.
    # Two limits, both correct:
    #   - hands-off: euler ~= current trim, drift large -> trim = euler - drift/g
    #     (reduces to the old accel-only formula: new = current + (-a_y/g))
    #   - active piloting that holds position: euler ~= equilibrium, drift ~= 0
    #     -> trim = mean(euler)  (the pilot's average correction IS the trim)
    # Signs: roll a_y=+g*phi -> roll_trim = mean_roll - a_y/g; pitch a_x=-g*theta
    # -> pitch_trim = mean_pitch + a_x/g. (Accel term verified on the SIL wind
    # bench attitude_trim_test.scn: trim>0 cuts downwind drift ~87%.)
    # 平衡トリム（手放しでも、操縦中でも成立）。ビタホバさせるトリム = スティック中立で
    # 機体が保つべき姿勢。よって trim =（機体が保った平均姿勢）−（残留ドリフト加速度）/g。
    # 狭い室内では手放しを続けられず壁を避けるため微修正する。その修正は保った姿勢(euler)
    # に織り込まれるので、姿勢を平均すればパイロットが手で与えたトリム分を捉えられる。両極限:
    #   手放し: euler≈現トリム・ドリフト大 → trim=euler-drift/g（旧加速度式 new=current-a_y/g に一致）
    #   位置保持の操縦: euler≈平衡・ドリフト≈0 → trim=mean(euler)（平均修正がトリムそのもの）
    new_roll = roll_mean - ay_body / G
    new_pitch = pitch_mean + ax_body / G
    delta_roll = new_roll - current_roll
    delta_pitch = new_pitch - current_pitch

    # Drift summary for the report. / 報告用ドリフト要約。
    horiz_disp = pos_f[-1, :2] - pos_f[0, :2]
    horiz_drift = float(np.linalg.norm(horiz_disp))
    vh_std = float(np.std(np.linalg.norm(vel_f[:, :2], axis=1)))

    squal = None
    if "flow" in S:
        fl = S["flow"]
        t_fl = np.array([(d["ts"] - t0) / 1e6 for d in fl])
        q_fl = np.array([d.get("quality", d.get("squal", 0)) for d in fl])
        m = (t_fl >= hover_start_s) & (t_fl <= t_end)
        if np.any(m):
            squal = float(np.mean(q_fl[m]))

    return {
        "log": Path(log_path).name,
        "window": {"start_s": float(hover_start_s), "end_s": float(t_end),
                   "duration_s": float(duration)},
        "attitude_mean_deg": {"roll": math.degrees(roll_mean),
                              "pitch": math.degrees(pitch_mean),
                              "yaw": math.degrees(yaw_mean)},
        "drift": {
            "horiz_displacement_m": horiz_drift,
            "horiz_displacement_ned_m": [float(horiz_disp[0]), float(horiz_disp[1])],
            "velocity_std_mps": vh_std,
            "flow_squal_mean": squal,
            "accel_ned_mps2": [a_north, a_east],
            "accel_body_mps2": [ax_body, ay_body],
        },
        "delta_trim_rad": {"roll": delta_roll, "pitch": delta_pitch},
        "current_trim_rad": {"roll": current_roll, "pitch": current_pitch},
        "suggested_trim_rad": {"roll": new_roll, "pitch": new_pitch},
    }


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def _print_report(r: dict) -> None:
    """Print a human-readable report / 人間可読の報告を出力"""
    w = r["window"]
    d = r["drift"]
    dr = r["delta_trim_rad"]
    ct = r["current_trim_rad"]
    st = r["suggested_trim_rad"]

    console.print()
    console.print("=" * 66)
    console.print("  TRIM IDENTIFICATION  (STABILIZE / ALT_HOLD hover)")
    console.print("=" * 66)
    console.print(f"  Log:     {r['log']}")
    console.print(f"  Window:  {w['start_s']:.1f}-{w['end_s']:.1f} s  "
                  f"({w['duration_s']:.1f} s clean hover)")
    console.print(f"  Yaw:     {r['attitude_mean_deg']['yaw']:+.1f} deg (mean)")
    console.print("-" * 66)
    console.print("  MEASURED  (held attitude + residual drift)")
    am = r["attitude_mean_deg"]
    console.print(f"    Mean attitude held      : roll {am['roll']:+.2f}, "
                  f"pitch {am['pitch']:+.2f}  deg  (your corrections are in here)")
    console.print(f"    Horizontal displacement : {d['horiz_displacement_m']:.2f} m  "
                  f"({d['horiz_displacement_ned_m'][0]:+.2f} N, "
                  f"{d['horiz_displacement_ned_m'][1]:+.2f} E)")
    squal_str = (f"   (flow SQUAL {d['flow_squal_mean']:.0f})"
                 if d["flow_squal_mean"] else "")
    console.print(f"    Velocity std            : {d['velocity_std_mps']:.3f} m/s{squal_str}")
    console.print(f"    Residual accel (body)   : fwd {d['accel_body_mps2'][0]:+.3f}, "
                  f"right {d['accel_body_mps2'][1]:+.3f}  m/s^2")
    console.print("-" * 66)
    console.print("  TRIM CORRECTION")
    console.print(f"    delta roll  : {dr['roll']:+.4f} rad  ({math.degrees(dr['roll']):+.2f} deg)")
    console.print(f"    delta pitch : {dr['pitch']:+.4f} rad  ({math.degrees(dr['pitch']):+.2f} deg)")
    console.print(f"    current     : roll {ct['roll']:+.4f}   pitch {ct['pitch']:+.4f}   rad")
    console.print(f"    SUGGESTED   : roll {st['roll']:+.4f}   pitch {st['pitch']:+.4f}   rad")
    console.print("-" * 66)
    console.print("  APPLY & ITERATE")
    console.print(f"    param set attitude.roll.trim  {st['roll']:.5f}")
    console.print(f"    param set attitude.pitch.trim {st['pitch']:.5f}")
    console.print("    param save")
    console.print("    then re-fly ~30 s and:")
    console.print(f"    sf trim analyze --current-roll {st['roll']:.5f} "
                  f"--current-pitch {st['pitch']:.5f}")
    console.print("    repeat until |delta| < 0.001 rad.")
    console.print("-" * 66)
    console.print("  CAVEATS")
    console.print("    - You MAY pilot to stay off the walls: the trim uses the average")
    console.print("      attitude you held, so your corrections are captured. Just keep")
    console.print("      TRYING to hold position - do not deliberately translate or turn.")
    console.print("    - Assumes NO WIND (indoor). Wind adds to the drift; if unsure,")
    console.print("      apply HALF the delta first and re-test.")
    console.print("    - Sign verified on the SIL wind bench (attitude_trim_test.scn).")
    console.print("=" * 66)
    console.print()
