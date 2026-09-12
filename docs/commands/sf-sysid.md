# sf sysid

フライトログからのシステム同定コマンド。

バンドル（一式）を指定する引数は拡張子を省略でき、裸の名前は `logs/` 内も検索対象になります
（例: `sf sysid rate-fit flight_20260912T093015 --axis roll`）。

## 構文

```bash
sf sysid <subcommand> [options]
```

### サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `noise` | センサノイズ特性解析（アラン分散） |
| `inertia` | 慣性モーメント推定（ステップ応答） |
| `motor` | モータダイナミクス同定（Ct, Cq, tau_m） |
| `drag` | 空力抗力係数推定 |
| `params` | パラメータ管理（表示・差分・エクスポート） |
| `validate` | バリデーション・整合性チェック |
| `plan` | 飛行試験計画の生成 |

## 使用例

```bash
sf sysid noise flight_log.csv
sf sysid motor flight_log.csv
sf sysid params show
```

> 詳細なドキュメントは今後追加予定です。
