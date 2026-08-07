# control

制御設計資産。

## ディレクトリ構成

- `models/` - 数学モデル、同定結果。`models/stampfly_physical.yaml` は機体物理パラメータの唯一の正（SSOT）で、`sf params generate` がここからコードを生成する（詳細: `tools/params_audit/README.md`）
- `design/` - PID・ループ整形・MPC 等の設計（設計根拠を残す）
- `simulation/` - SILS 等の検証環境
- `validation/` - 実機ログとの照合、設計の妥当性評価

---

# control

Control systems design assets.

## Directory Structure

- `models/` - Mathematical models, system identification results. `models/stampfly_physical.yaml` is the single source of truth (SSOT) for vehicle physical parameters; `sf params generate` generates code from it (see `tools/params_audit/README.md`)
- `design/` - PID, loop shaping, MPC design (with design rationale)
- `simulation/` - SILS verification environments
- `validation/` - Comparison with real flight logs, design validation
