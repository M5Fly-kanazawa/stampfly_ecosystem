# 照会コマンド（Query Commands）

`sf battery`・`sf height` 等、WiFi CLI 経由で Tello 互換クエリコマンドを送信し、テレメトリ情報を
表示するトップレベルコマンド群（それぞれ独立したコマンドで、`sf query <command>` という
サブコマンドではない）。

## 構文

```bash
sf <query-command>
```

### コマンド一覧

| コマンド | 説明 |
|---------|------|
| `battery` | バッテリー残量 (%) |
| `height` | ESKF 推定高度 (cm) |
| `tof` | ToF 底面距離 (cm) |
| `baro` | 気圧高度 (cm) |
| `attitude` | 姿勢角 (pitch, roll, yaw in deg) |
| `acceleration` | 加速度 (x, y, z in cm/s^2) |
| `speed` | 設定速度 (cm/s) |

## 使用例

```bash
sf battery
sf height
sf attitude
```

> 詳細なドキュメントは今後追加予定です。
