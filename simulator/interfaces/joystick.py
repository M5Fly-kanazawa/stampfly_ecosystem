import hid

# ※以下の VENDOR_ID と PRODUCT_ID はご利用のゲームパッド（ESP32‑S3 で HID ゲームパッドとして動作している場合）の値に合わせて変更してください
VENDOR_ID = 0x303a   # 例: 0x1209 (実際の値に合わせてください)
PRODUCT_ID = 0x1001  # 例: 0xABCD (実際の値に合わせてください)


class Joystick:
    def __init__(self, vendor_id=VENDOR_ID, product_id=PRODUCT_ID):
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.device = None

    def open(self):
        try:
            self.device = hid.Device(self.vendor_id, self.product_id)
            print("デバイスをオープンしました:", self.device.manufacturer, self.device.product)
            
            # 非ブロッキングモードに設定
            self.device.nonblocking = True
        except Exception as e:
            print("エラー:", e)

    def close(self):
        try:
            self.device.close()
        except Exception:
            print("デバイスをクローズ失敗")

    def read(self):
        if self.device is None:
            return None
        data = self.device.read(8)  # 1回の読み込みで最大 64 バイト取得
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
        """接続されているHIDデバイスの情報を列挙する"""
        print("=== 接続されているHIDデバイス一覧 ===")
        for d in hid.enumerate():
            info = {
                'vendor_id': hex(d['vendor_id']),
                'product_id': hex(d['product_id']),
                'manufacturer': d.get('manufacturer_string'),
                'product': d.get('product_string')
            }
            print(info)
        print("====================================\n")

