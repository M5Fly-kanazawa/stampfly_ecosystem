"""
sf hub - Control USB hub port power (development tool)

Turns the power of a supported USB hub on/off/cycle with uhubctl, so a
StampFly can be reset or put into download mode without unplugging it.
対応USBハブのポート電源を uhubctl で ON/OFF/サイクルし、StampFly を
抜き差しせずにリセット・書き込みモードへ入れられるようにします。
"""

import argparse
import re
import shutil
import sys
import subprocess
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from ..utils import console

COMMAND_NAME = "hub"
COMMAND_HELP = "Control USB hub port power (uhubctl)"


# ---------------------------------------------------------------------------
# Hub model definitions (add another hub model here)
# ハブ機種の定義（別機種を追加するときはここだけ編集する）
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class HubModel:
    """Hub model description / ハブ機種の記述"""
    name: str
    usb2_id: str          # VID:PID of the USB2 half / USB2側の VID:PID
    usb3_id: str          # VID:PID of the USB3 half / USB3側の VID:PID
    control_port: int     # Port whose power gates all USB-A ports / 全USB-Aポートの電源を握るポート
    note: str             # User-facing note / 利用者向け注記


SUPPORTED_HUBS: Tuple[HubModel, ...] = (
    HubModel(
        name="StarTech HB31C3A1CPD3",
        usb2_id="14b0:013a",
        usb3_id="14b0:0131",
        control_port=2,
        note="Power of all 3 USB-A ports is switched; the USB-C port is always powered "
             "and cannot be switched. / USB-A 3ポートの電源が切れる。USB-Cポートは常時給電で切れない。",
    ),
)

ESPRESSIF_VID = "303a"
DOWNLOAD_MODE_ID = "303a:1001"   # ROM download mode / USB Serial/JTAG
DEFAULT_OFF_TIME = 2.0           # seconds / 秒
ENUMERATE_TIMEOUT = 10.0         # seconds / 秒


# ---------------------------------------------------------------------------
# Parsing uhubctl output
# uhubctl 出力の解析
# ---------------------------------------------------------------------------

_HUB_RE = re.compile(r"^Current status for hub (\S+) \[([0-9a-fA-F]{4}:[0-9a-fA-F]{4})\b")
_PORT_RE = re.compile(r"^\s*Port (\d+): ([0-9a-fA-F]{4})\s*(.*)$")
_DEV_RE = re.compile(r"\[([0-9a-fA-F]{4}:[0-9a-fA-F]{4})\s*([^\]]*)\]")


@dataclass
class PortInfo:
    number: int
    powered: bool
    device_id: Optional[str] = None
    description: str = ""


@dataclass
class HubStatus:
    location: str
    vid_pid: str
    ports: List[PortInfo] = field(default_factory=list)

    def port(self, number: int) -> Optional[PortInfo]:
        for p in self.ports:
            if p.number == number:
                return p
        return None


def parse_uhubctl(text: str) -> List[HubStatus]:
    """Parse `uhubctl` (no -a) output into hubs and ports.
    `uhubctl`（-aなし）の出力をハブとポートに分解する。

    A port is powered when its status text contains the word "power";
    "off" means unpowered (judged from uhubctl, not from the OS device list).
    ポート行に "power" があれば給電中、"off" なら停止中
    （OSのデバイス一覧ではなく uhubctl の表示で判定する）。
    """
    hubs: List[HubStatus] = []
    current: Optional[HubStatus] = None
    for line in text.splitlines():
        m = _HUB_RE.match(line)
        if m:
            current = HubStatus(location=m.group(1), vid_pid=m.group(2).lower())
            hubs.append(current)
            continue
        m = _PORT_RE.match(line)
        if m and current is not None:
            rest = m.group(3)
            words = re.sub(r"\[.*?\]", "", rest).split()
            powered = "power" in words
            dev = _DEV_RE.search(rest)
            current.ports.append(PortInfo(
                number=int(m.group(1)),
                powered=powered,
                device_id=dev.group(1).lower() if dev else None,
                description=dev.group(2).strip() if dev else "",
            ))
    return hubs


def find_hub(hubs: List[HubStatus]) -> Optional[Tuple[HubModel, HubStatus, HubStatus]]:
    """Find a supported hub (USB2 and USB3 halves) by VID:PID.
    VID:PID で対応ハブ（USB2側とUSB3側）を探す。
    """
    for model in SUPPORTED_HUBS:
        usb2 = next((h for h in hubs if h.vid_pid == model.usb2_id), None)
        usb3 = next((h for h in hubs if h.vid_pid == model.usb3_id), None)
        if usb2 and usb3:
            return model, usb2, usb3
    return None


def espressif_devices(*hubs: HubStatus) -> List[Tuple[str, PortInfo]]:
    """List Espressif devices (VID 303a) as (hub location, port).
    Espressif デバイス（VID 303a）を (ハブ位置, ポート) で列挙する。
    """
    found = []
    for h in hubs:
        for p in h.ports:
            if p.device_id and p.device_id.startswith(ESPRESSIF_VID + ":"):
                found.append((h.location, p))
    return found


# ---------------------------------------------------------------------------
# uhubctl invocation
# uhubctl の呼び出し
# ---------------------------------------------------------------------------

class HubError(Exception):
    """Hub control error / ハブ制御エラー"""


def uhubctl_available() -> bool:
    return shutil.which("uhubctl") is not None


def run_uhubctl(args: List[str]) -> str:
    """Run uhubctl and return stdout. / uhubctl を実行して標準出力を返す。"""
    try:
        r = subprocess.run(["uhubctl", *args], capture_output=True, text=True, timeout=30)
    except FileNotFoundError:
        raise HubError("uhubctl not found")
    except subprocess.TimeoutExpired:
        raise HubError("uhubctl timed out")
    if r.returncode != 0:
        raise HubError(f"uhubctl {' '.join(args)} failed: {(r.stderr or r.stdout).strip()}")
    return r.stdout


def read_status() -> Tuple[HubModel, HubStatus, HubStatus]:
    """Detect the hub and read its current status. / ハブを検出し現在状態を読む。"""
    found = find_hub(parse_uhubctl(run_uhubctl([])))
    if found is None:
        raise HubError("No supported USB hub found. / 対応するUSBハブが見つかりません。")
    return found


def _switch(location: str, port: int, action: str) -> None:
    # -e (exact) is required: uhubctl's USB3 pairing fails when port counts differ.
    # -e（厳密指定）が必須: ポート数が異なるとuhubctlのUSB3自動対応付けが失敗する。
    run_uhubctl(["-e", "-l", location, "-p", str(port), "-a", action])


def power_off(model: HubModel, usb2: HubStatus, usb3: HubStatus) -> None:
    """OFF order: USB3 half first, then USB2. / OFF順: USB3側 → USB2側。"""
    _switch(usb3.location, model.control_port, "off")
    _switch(usb2.location, model.control_port, "off")


def power_on(model: HubModel, usb2: HubStatus, usb3: HubStatus) -> None:
    """ON order: USB2 half first, then USB3. / ON順: USB2側 → USB3側。"""
    _switch(usb2.location, model.control_port, "on")
    _switch(usb3.location, model.control_port, "on")


def is_powered(model: HubModel, usb2: HubStatus, usb3: HubStatus) -> Optional[bool]:
    """True/False if both halves agree on the control port, else None.
    両側の制御ポートが一致すれば True/False、不一致なら None。
    """
    p2, p3 = usb2.port(model.control_port), usb3.port(model.control_port)
    if p2 is None or p3 is None or p2.powered != p3.powered:
        return None
    return p2.powered


def wait_for_device(
    timeout: float = ENUMERATE_TIMEOUT,
    wanted_id: Optional[str] = None,
    sleep: Callable[[float], None] = time.sleep,
    interval: float = 0.5,
) -> Optional[Tuple[str, PortInfo]]:
    """Poll until an Espressif device (optionally a given VID:PID) appears.
    Espressif デバイス（任意でVID:PID指定）が現れるまでポーリングする。
    """
    waited = 0.0
    while True:
        _, usb2, usb3 = read_status()
        for loc, p in espressif_devices(usb2, usb3):
            if wanted_id is None or p.device_id == wanted_id:
                return loc, p
        if waited >= timeout:
            return None
        sleep(interval)
        waited += interval


# ---------------------------------------------------------------------------
# Command implementations
# サブコマンドの実装
# ---------------------------------------------------------------------------

def _print_install_hint() -> None:
    console.error("uhubctl not found. / uhubctl が見つかりません。")
    console.print("  macOS: brew install uhubctl")
    console.print("  Linux: install the 'uhubctl' package (apt/dnf), and set udev rules so "
                  "sudo is not needed (see https://github.com/mvp/uhubctl).")


def _print_status(model: HubModel, usb2: HubStatus, usb3: HubStatus) -> None:
    console.print(f"Hub: {model.name}")
    console.print(f"  USB2 half: {usb2.location} [{usb2.vid_pid}]")
    console.print(f"  USB3 half: {usb3.location} [{usb3.vid_pid}]")
    state = is_powered(model, usb2, usb3)
    label = {True: "ON (power)", False: "OFF", None: "MIXED (halves disagree)"}[state]
    console.print(f"  Port {model.control_port} power (USB-A ports): {label}")
    console.print(f"  Note: {model.note}")
    devs = espressif_devices(usb2, usb3)
    if devs:
        console.print("  Espressif devices:")
        for loc, p in devs:
            console.print(f"    hub {loc} port {p.number}: {p.device_id} {p.description}")
    else:
        console.print("  Espressif devices: none")


def _confirm(expected: bool) -> int:
    model, usb2, usb3 = read_status()
    state = is_powered(model, usb2, usb3)
    if state == expected:
        console.success(f"Port {model.control_port} is now {'ON' if expected else 'OFF'}")
        return 0
    console.error(f"Expected {'ON' if expected else 'OFF'} but state is "
                  f"{ {True: 'ON', False: 'OFF', None: 'MIXED'}[state] }")
    return 1


def do_cycle(off_time: float, sleep: Callable[[float], None] = time.sleep) -> int:
    """OFF, wait, ON. / OFF → 待機 → ON。"""
    model, usb2, usb3 = read_status()
    console.info(f"Power OFF (hub {usb3.location}, {usb2.location} port {model.control_port})")
    power_off(model, usb2, usb3)
    sleep(off_time)
    console.info("Power ON")
    power_on(model, usb2, usb3)
    return 0


def _report_device(wanted_id: Optional[str] = None) -> Optional[Tuple[str, PortInfo]]:
    console.info(f"Waiting up to {ENUMERATE_TIMEOUT:.0f} s for an Espressif device...")
    dev = wait_for_device(wanted_id=wanted_id)
    if dev:
        loc, p = dev
        console.success(f"Found {p.device_id} {p.description} (hub {loc} port {p.number})")
    return dev


def run(args: argparse.Namespace) -> int:
    """Execute hub command / hub コマンドを実行"""
    sub = getattr(args, "hub_command", None)
    if sub is None:
        console.error("Specify a subcommand: status, on, off, cycle")
        return 2
    if not uhubctl_available():
        _print_install_hint()
        return 1
    try:
        if sub == "status":
            _print_status(*read_status())
            return 0
        if sub == "off":
            power_off(*read_status())
            return _confirm(False)
        if sub == "on":
            power_on(*read_status())
            return _confirm(True)
        if sub == "cycle":
            if args.boot:
                return _cycle_boot(args.off_time, args.port)
            do_cycle(args.off_time)
            if _confirm(True) != 0:
                return 1
            if _report_device() is None:
                console.warning("No Espressif device appeared. / Espressif デバイスが現れませんでした。")
                return 1
            return 0
    except HubError as e:
        console.error(str(e))
        return 1
    return 2


ESPRESSIF_VID_INT = 0x303A
RELEASE_WAIT = 1.5     # seconds before probing / プローブ前の待ち（秒）
PROBE_TIMEOUT = 15.0   # seconds / 秒


def find_espressif_serial_port() -> Optional[str]:
    """Return the serial port of an Espressif (VID 303a) device, if any.
    Espressif（VID 303a）デバイスのシリアルポートを返す（無ければ None）。
    """
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for p in list_ports.comports():
        if p.vid == ESPRESSIF_VID_INT:
            return p.device
    return None


def esptool_available() -> bool:
    import importlib.util
    return importlib.util.find_spec("esptool") is not None


def probe_rom_bootloader(port: str, timeout: float = PROBE_TIMEOUT) -> bool:
    """Run esptool chip_id without any reset; only the ROM bootloader answers.
    リセット無しで esptool chip_id を実行する。応答するのはROMブートローダのみ。
    """
    cmd = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "-p", port,
           "--before", "no_reset", "--after", "no_reset", "chip_id"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, OSError):
        return False
    return r.returncode == 0


def _cycle_boot(off_time: float, port: Optional[str] = None,
                sleep: Callable[[float], None] = time.sleep) -> int:
    console.print("Hold the BOOT (G0) button on the board, then press Enter.")
    console.print("ボードの BOOT (G0) ボタンを押したまま Enter を押してください。")
    try:
        input()
    except EOFError:
        pass
    do_cycle(off_time)
    console.print("Release the BOOT button now. / BOOT ボタンを離してください。")
    dev = _report_device(wanted_id=DOWNLOAD_MODE_ID)
    sleep(RELEASE_WAIT)
    if not esptool_available():
        console.warning("esptool is not importable; falling back to the 303a:1001 check "
                        "(UNCONFIRMED: running firmware may also appear as 303a:1001). "
                        "/ esptool が無いため 303a:1001 の確認のみ（未確認）。")
        return 0 if dev else 1
    port = port or find_espressif_serial_port()
    if not port:
        console.error("No Espressif serial port found; use --port. / シリアルポートが見つかりません。--port で指定してください。")
        return 1
    console.info(f"Probing ROM bootloader on {port} ...")
    if probe_rom_bootloader(port):
        console.success(f"Download mode confirmed on {port}. / 書き込みモードを確認しました。")
        return 0
    console.error(f"ROM bootloader did not answer on {port}; download mode not reached. "
                  "/ 書き込みモードに入っていません。")
    return 1


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(COMMAND_NAME, help=COMMAND_HELP, description=__doc__)
    sub = parser.add_subparsers(dest="hub_command", metavar="<subcommand>")
    sub.add_parser("status", help="Show hub locations, port power and Espressif devices")
    sub.add_parser("off", help="Turn USB-A port power off")
    sub.add_parser("on", help="Turn USB-A port power on")
    c = sub.add_parser("cycle", help="Power off, wait, power on")
    c.add_argument("--off-time", type=float, default=DEFAULT_OFF_TIME,
                   help=f"Seconds to stay off (default: {DEFAULT_OFF_TIME:g})")
    c.add_argument("--boot", action="store_true",
                   help="Enter download mode: hold BOOT during the cycle (interactive)")
    c.add_argument("--port", default=None,
                   help="Serial port for the --boot ROM probe (auto-detect if omitted)")
    parser.set_defaults(func=run)
