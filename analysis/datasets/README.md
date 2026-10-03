# datasets

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

解析・演習用のログとデータ。フライトログ一式（`.sflog.zip`）の形式は
`protocol/spec/flight_log.yaml`（基準ファイル）と `docs/reference/flight-log-format.md` を参照。

| ディレクトリ | 内容 |
|-------------|------|
| `education/` | 実機なしで解析を練習するための教育用サンプルログ5本（シミュレーションで生成）。詳細は `education/README.md` |
| `flightlog/` | 実機由来の基準フライトログ一式。`sf log`・`sf sysid` と CI の読み込み確認に使う。詳細は `flightlog/README.md` |
| `motor_sweep_20260714/` | モータの掃引計測（2026-07-14）。詳細は `motor_sweep_20260714/README.md` |
| `sysid/` | システム同定用の CSV（手元で生成。git 管理外） |

CSV へ書き出したファイル（`education/*.csv`、`sysid/*.csv`）は作り直せるので git 管理外とする。

---

<a id="english"></a>

# datasets

Logs and data for analysis and exercises. For the flight-log bundle format (`.sflog.zip`) see
`protocol/spec/flight_log.yaml` (source of truth) and `docs/reference/flight-log-format.md`.

| Directory | Contents |
|-----------|----------|
| `education/` | Five educational sample logs for practicing analysis without a vehicle (simulation-generated). See `education/README.md` |
| `flightlog/` | Real-vehicle reference flight-log bundles, used by `sf log`, `sf sysid` and CI as read-compatibility fixtures. See `flightlog/README.md` |
| `motor_sweep_20260714/` | Motor sweep measurements (2026-07-14). See `motor_sweep_20260714/README.md` |
| `sysid/` | CSV files for system identification (generated locally, not tracked) |

CSV exports (`education/*.csv`, `sysid/*.csv`) are regenerable and are not tracked.
