"""
test_hub.py - tests for lib/sfcli/commands/hub.py (subprocess is mocked;
no real uhubctl or hardware is touched).
hub.py の試験（subprocess はモックし、実機・実uhubctlには触れない）。
"""

import argparse
import sys
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))  # lib/
from sfcli.commands import hub  # noqa: E402

SAMPLE = """Current status for hub 4-2 [14b0:0131 Generic 4-Port USB 3.0 Hub, USB 3.20, 4 ports, ppps]
  Port 1: 02a0 power 5gbps Rx.Detect
  Port 2: 02a0 power 5gbps Rx.Detect
Current status for hub 4-1 [14b0:013a Generic 4-Port USB 2.0 Hub, USB 2.10, 5 ports, ppps]
  Port 1: 0100 power
  Port 2: 0100 power
  Port 3: 0103 power enable connect [303a:1001 Espressif USB JTAG/serial debug unit CC:8D:A2:22:10:A8]
Current status for hub 32-1 [05ac:101d Apple USB2 Hub X, USB 2.10, 4 ports, ppps]
  Port 4: 0503 power highspeed enable connect [05ac:1114 Apple Inc. Studio Display 0000]
"""
OFF = SAMPLE.replace("Port 2: 02a0 power 5gbps Rx.Detect", "Port 2: 0080 off").replace(
    "Port 2: 0100 power", "Port 2: 0000 off").replace(
    "  Port 3: 0103 power enable connect [303a:1001 Espressif USB JTAG/serial debug unit CC:8D:A2:22:10:A8]\n", "")


def test_parse_and_find():
    hubs = hub.parse_uhubctl(SAMPLE)
    assert [h.location for h in hubs] == ["4-2", "4-1", "32-1"]
    model, usb2, usb3 = hub.find_hub(hubs)
    assert (usb2.location, usb3.location) == ("4-1", "4-2")
    assert hub.is_powered(model, usb2, usb3) is True
    devs = hub.espressif_devices(usb2, usb3)
    assert devs[0][0] == "4-1" and devs[0][1].number == 3
    assert devs[0][1].device_id == "303a:1001"


def test_off_state_and_no_hub():
    model, usb2, usb3 = hub.find_hub(hub.parse_uhubctl(OFF))
    assert hub.is_powered(model, usb2, usb3) is False
    assert hub.find_hub(hub.parse_uhubctl("Current status for hub 1-1 [05ac:1 x]")) is None


def _calls(m):
    return [c.args[0] for c in m.call_args_list]


def test_cycle_sequence():
    model, usb2, usb3 = hub.find_hub(hub.parse_uhubctl(SAMPLE))
    sleeps = []
    with mock.patch.object(hub, "read_status", return_value=(model, usb2, usb3)), \
            mock.patch.object(hub, "run_uhubctl", return_value="") as run:
        hub.do_cycle(2.0, sleep=sleeps.append)
    assert _calls(run) == [
        ["-e", "-l", "4-2", "-p", "2", "-a", "off"],
        ["-e", "-l", "4-1", "-p", "2", "-a", "off"],
        ["-e", "-l", "4-1", "-p", "2", "-a", "on"],
        ["-e", "-l", "4-2", "-p", "2", "-a", "on"],
    ]
    assert sleeps == [2.0]


def test_run_uhubctl_subprocess_mocked():
    ok = mock.Mock(returncode=0, stdout="x", stderr="")
    with mock.patch.object(hub.subprocess, "run", return_value=ok) as r:
        assert hub.run_uhubctl(["-l", "1"]) == "x"
    assert r.call_args.args[0] == ["uhubctl", "-l", "1"]
    bad = mock.Mock(returncode=1, stdout="", stderr="boom")
    with mock.patch.object(hub.subprocess, "run", return_value=bad):
        try:
            hub.run_uhubctl([])
            assert False
        except hub.HubError:
            pass


def test_run_without_uhubctl_exits_nonzero():
    with mock.patch.object(hub, "uhubctl_available", return_value=False):
        assert hub.run(argparse.Namespace(hub_command="status")) == 1


def test_off_reports_failure_when_still_powered():
    with mock.patch.object(hub, "uhubctl_available", return_value=True), \
            mock.patch.object(hub, "run_uhubctl", return_value=SAMPLE):
        # status never changes to OFF -> failure
        assert hub.run(argparse.Namespace(hub_command="off")) == 1


def test_wait_for_device_timeout_and_found():
    with mock.patch.object(hub, "run_uhubctl", return_value=OFF):
        assert hub.wait_for_device(timeout=1.0, sleep=lambda s: None) is None
    with mock.patch.object(hub, "run_uhubctl", return_value=SAMPLE):
        loc, p = hub.wait_for_device(timeout=1.0, wanted_id="303a:1001", sleep=lambda s: None)
        assert p.number == 3
        assert hub.wait_for_device(timeout=0.5, wanted_id="303a:0009", sleep=lambda s: None) is None


def _boot_patches(esptool=True, port="/dev/cu.usbmodem1", probe=True):
    return [
        mock.patch("builtins.input", return_value=""),
        mock.patch.object(hub, "do_cycle"),
        mock.patch.object(hub, "_report_device", return_value=("4-1", hub.PortInfo(3, True, "303a:1001", "x"))),
        mock.patch.object(hub, "esptool_available", return_value=esptool),
        mock.patch.object(hub, "find_espressif_serial_port", return_value=port),
        mock.patch.object(hub, "probe_rom_bootloader", return_value=probe),
    ]


def _run_boot(override=None, **k):
    ps = _boot_patches(**k)
    ms = [p.start() for p in ps]
    try:
        rc = hub._cycle_boot(2.0, override, sleep=lambda s: None)
    finally:
        for p in ps:
            p.stop()
    return rc, ms


def test_boot_confirmed_by_rom_probe():
    rc, ms = _run_boot()
    assert rc == 0
    ms[5].assert_called_once_with("/dev/cu.usbmodem1")


def test_boot_probe_failure_is_failure():
    assert _run_boot(probe=False)[0] == 1


def test_boot_port_override_and_no_port():
    rc, ms = _run_boot(override="/dev/ttyX")
    assert rc == 0
    ms[5].assert_called_once_with("/dev/ttyX")
    assert _run_boot(port=None)[0] == 1


def test_boot_without_esptool_falls_back():
    rc, ms = _run_boot(esptool=False)
    assert rc == 0
    ms[5].assert_not_called()


def test_probe_command_is_no_reset():
    ok = mock.Mock(returncode=0)
    with mock.patch.object(hub.subprocess, "run", return_value=ok) as r:
        assert hub.probe_rom_bootloader("/dev/p") is True
    cmd = r.call_args.args[0]
    assert cmd[1:3] == ["-m", "esptool"] and "no_reset" in cmd and cmd[-1] == "chip_id"
    assert r.call_args.kwargs["timeout"] == hub.PROBE_TIMEOUT
    with mock.patch.object(hub.subprocess, "run", return_value=mock.Mock(returncode=2)):
        assert hub.probe_rom_bootloader("/dev/p") is False
