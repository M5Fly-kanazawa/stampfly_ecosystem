# hid is optional: the simulator must start without a controller
# hid はオプション扱い: コントローラ無しでもシミュレータを起動できるようにする
try:
    import hid
except ImportError:
    hid = None

# StampFly Controller USB HID settings
# StampFlyコントローラのUSB HID設定
VENDOR_ID = 0x303a   # Espressif VID
PRODUCT_ID = 0x8001  # StampFly Controller PID (sdkconfig.defaults)

# Detect which HID package is installed
# hid パッケージ: hid.Device(vid, pid) — 大文字 Device, プロパティベース API
# hidapi パッケージ: hid.device() — 小文字 device, メソッドベース API
_USE_HID_PACKAGE = hid is not None and hasattr(hid, 'Device')
_USE_HIDAPI_PACKAGE = hid is not None and hasattr(hid, 'device') and not _USE_HID_PACKAGE


class Joystick:
    def __init__(self, vendor_id=VENDOR_ID, product_id=PRODUCT_ID):
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.device = None

    def open(self):
        # No hid package installed: run without a controller
        # hid パッケージ未インストール時はコントローラ無しで続行
        if hid is None:
            print("hid パッケージ未検出のためコントローラ無しで実行します / "
                  "Running without controller (no 'hid'/'hidapi' package)")
            self.device = None
            return
        try:
            if _USE_HID_PACKAGE:
                # 'hid' package (PyPI: hid>=1.0.0)
                # macOS: requires libhidapi (brew install hidapi)
                self.device = hid.Device(self.vendor_id, self.product_id)
                print("デバイスをオープンしました:",
                      self.device.manufacturer, self.device.product)
                # Set non-blocking mode
                # 非ブロッキングモードに設定
                self.device.nonblocking = True
            else:
                # 'hidapi' package (PyPI: hidapi>=0.14.0)
                # Bundles native library — works on Mac/Windows/Linux
                # ネイティブライブラリ同梱 — Mac/Windows/Linux で動作
                self.device = hid.device()
                self.device.open(self.vendor_id, self.product_id)
                manufacturer = self.device.get_manufacturer_string()
                product = self.device.get_product_string()
                print("デバイスをオープンしました:", manufacturer, product)
                # Set non-blocking mode
                # 非ブロッキングモードに設定
                self.device.set_nonblocking(1)
        except Exception as e:
            print(f"コントローラ接続エラー / Controller connection error: {e}")
            print(f"  VID=0x{self.vendor_id:04x} PID=0x{self.product_id:04x}")
            self._print_troubleshooting()
            self.device = None

    def close(self):
        try:
            if self.device is not None:
                self.device.close()
        except Exception:
            print("デバイスをクローズ失敗")

    def read(self):
        if self.device is None:
            return None
        data = self.device.read(8)  # Read up to 8 bytes / 最大8バイト取得
        if data:
            return data
        return None

    def write(self, data):
        if self.device is None:
            return
        self.device.write(data)

    def __del__(self):
        self.close()

    def list_hid_devices(self):
        """List connected HID devices
        接続されているHIDデバイスの情報を列挙する"""
        if hid is None:
            print("hid パッケージ未検出 / 'hid' package not installed")
            return
        print("=== 接続されているHIDデバイス一覧 / Connected HID Devices ===")
        for d in hid.enumerate():
            info = {
                'vendor_id': hex(d['vendor_id']),
                'product_id': hex(d['product_id']),
                'manufacturer': d.get('manufacturer_string'),
                'product': d.get('product_string')
            }
            print(info)
        print("====================================\n")

    def _print_troubleshooting(self):
        """Print troubleshooting info when connection fails
        接続失敗時のトラブルシューティング情報を表示"""
        import sys as _sys

        print("  --- トラブルシューティング / Troubleshooting ---")

        # Show which package is in use
        # 使用パッケージを表示
        if _USE_HID_PACKAGE:
            print("  HID package: hid (Device API)")
        else:
            print("  HID package: hidapi (device API)")

        # List matching devices
        # 一致するデバイスを検索
        found = False
        for d in hid.enumerate():
            if (d['vendor_id'] == self.vendor_id
                    and d['product_id'] == self.product_id):
                found = True
                print(f"  デバイスは検出されましたが接続に失敗しました")
                print(f"  Device detected but connection failed")
                print(f"    path: {d.get('path')}")
                print(f"    interface: {d.get('interface_number')}")
                break

        if not found:
            print("  デバイスが見つかりません / Device not found")
            print("  コントローラが接続されているか確認してください")
            print("  Check that the controller is plugged in")

        # Linux-specific: suggest udev rules
        # Linux固有: udevルールを案内
        if _sys.platform == "linux":
            print()
            print("  Linux: udevルールが必要です / udev rules required:")
            print("    sudo cp tools/udev/99-stampfly.rules /etc/udev/rules.d/")
            print("    sudo udevadm control --reload-rules")
            print("    sudo udevadm trigger")
            print("  その後、コントローラを再接続してください")
            print("  Then reconnect the controller")
