# 飛行コマンド（Flight Commands）

`sf takeoff`・`sf land` 等、WiFi CLI 経由でフライトコマンドを StampFly に送信し、リアルタイムで
監視するトップレベルコマンド群（それぞれ独立したコマンドで、`sf flight <command>` という
サブコマンドではない）。

## 構文

```bash
sf <command> [args]
```

### コマンド一覧

| コマンド | 説明 |
|---------|------|
| `takeoff [alt]` | 指定高度まで離陸（デフォルト: 0.5m） |
| `land` | 着陸 |
| `hover [alt] [dur]` | 指定高度・時間でホバリング |
| `jump [alt]` | クイックジャンプ（上昇後下降） |
| `up/down <cm>` | 上下移動 |
| `cw/ccw <deg>` | 回転（時計回り/反時計回り） |
| `forward/back <cm>` | 前後移動 |
| `left/right <cm>` | 左右移動 |
| `emergency` | 緊急モーター停止 |
| `stop` | 停止してホバリング |

## 使用例

```bash
sf takeoff 0.5
sf hover 0.5 10
sf land
sf emergency
```

> 詳細なドキュメントは今後追加予定です。
