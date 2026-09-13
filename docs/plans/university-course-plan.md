# 大学講義教材の再建計画

状態: **計画中**（教材は 2026-09-13 に一度削除。本文書は作る予定を残すためのもの）。作成 2026-09-13。

本文書は、削除される大学講義教材（`docs/university/`・`examples/education/`・
`lib/stampfly_edu/`・`analysis/notebooks/education/`・`docs/setup/education.md`）から
**意図だけ**を抜き出して残す。実体はコミット時点のタグ（§6）から取り出せる。

## 1. 目的と対象

`docs/university/syllabus.md` の冒頭記載。

| 項目 | 内容 |
|------|------|
| 科目名 | 制御工学実習 — ドローン制御入門 |
| 開講期間 | 半期 15 回（各 90 分） |
| 対象 | 工学系学部 3-4 年生 |
| 前提知識 | 線形代数、微分方程式 |
| 機材 | 1 人 1 台 StampFly + ノート PC |

到達目標:

1. フィードバック制御の基本概念を説明し、PID コントローラを設計できる
2. ドローンの運動方程式を導出し、ホバー条件を計算できる
3. カスケード制御の構造を理解し、帯域設計の原則を説明できる
4. センサデータから Allan 分散（センサ出力のノイズの大きさを、平均する時間の長さごとに
   評価する統計量）を計算し、ノイズ特性を評価できる
5. Python SDK（StampFly を Python から操作するための命令セット。`connect_or_simulate()`
   等）を使ってドローンの自律飛行プログラムを実装できる

## 2. 15 回の題目一覧

`syllabus.md` の 15 回と、実装済みだった `analysis/notebooks/education/` の連番ノートブック
15 本・`examples/education/` の 8 テーマの対応。

| 回 | テーマ | ノートブック | 対応する examples/education/ |
|----|--------|--------------|-------------------------------|
| 1 | StampFly とドローン制御の世界 | `01_hello_stampfly.ipynb` | `hello_flight/`（3行で初飛行、離陸→前進→着陸） |
| 2 | プログラムで自律飛行 | `02_autonomous_flight.ipynb` | `square_path/`（1m四方の矩形飛行、軌跡記録） |
| 3 | フィードバック制御入門 | `03_feedback_basics.ipynb` | `pid_1d/`（P制御、1次系、ドローン不要） |
| 4 | PID 制御の理論と実装 | `04_pid_theory.ipynb` | `pid_1d/`（P/I/D 各項、Kp/Ti/Td の工学形式） |
| 5 | 実機で PID を感じる | `05_rate_control_tuning.ipynb` | `rate_step_test/`（レートPIDのステップ応答比較） |
| 6 | ドローンの数学モデル | `06_drone_dynamics.ipynb` | — |
| 7 | システム同定 | `07_system_identification.ipynb` | `noise_analysis/`（ジャイロの Allan 分散解析） |
| 8 | センサフュージョンとカルマンフィルタ | `08_sensor_fusion.ipynb` | — |
| 9 | 姿勢制御 — カスケードの概念 | `09_cascade_attitude.ipynb` | `cascade_sim/`（カスケード vs 単ループ比較） |
| 10 | 高度制御 — FF とアンチワインドアップ | `10_altitude_control.ipynb` | — |
| 11 | 位置制御 — 座標変換と外乱抑制 | `11_position_control.ipynb` | — |
| 12 | ウェイポイント飛行 | `12_waypoint_mission.ipynb` | `waypoint_mission/`（複数点経由の自律飛行） |
| 13 | カスタムコントローラ | `13_custom_controller.ipynb` | `custom_pid/`（`send_rc_control()`で外部PIDループ） |
| 14 | 最終プロジェクト実装 | `14_project_template.ipynb` | — |
| 15 | プレゼンテーション + デモフライト | （`15_analysis_toolkit.ipynb` はログ解析ユーティリティ集で全回共通） | — |

## 3. 評価の考え方

`assessment_rubric.md` の骨子（配点比率は `syllabus.md` §4 と一致）:

- ノートブック提出 40%（コード動作 30% / 考察 40% / 図表 20% / コメント 10%）
- 中間レポート 20%（Session 8 終了時点。P/I/D の役割・ゲイン調整・運動方程式・センサ融合の 4 本柱）
- 最終プロジェクト 30%（技術的達成度・理論的根拠・実験検証・プレゼン。デモ飛行成功で加点）
- 授業参加 10%（出席・質疑応答）

## 4. 技術上の前提として残す判断

- **`connect_or_simulate()` という考え方**: 実機 StampFly への WiFi 接続を試み、失敗（機体
  が手元にない、電源が入っていない等）した場合は自動的に軽量シミュレータ
  （`SimulatedStampFly`）へ切り替える。学生が実機を持っていない状況でも全回の大半が
  自己完結して進められることが、15 回カリキュラムの前提になっていた。次に作るときも
  この「実機無しでも進める」性質は維持する。
- **Tello 互換 API を土台にしていた事実**: `stampfly_edu` の SDK は Tello ドローン用
  Python ライブラリ（`djitellopy`）と同じ命令名（`takeoff()`・`move_forward()`・
  `send_rc_control()` 等）を土台にしていた。既存の Tello 学習資料・書籍がそのまま
  流用できる利点があった。

## 5. 次に作るときの条件

原典 `PROJECT_PLAN.md` §1「目指す姿」の P4（各階層用の講習資料が一通り揃う）・P6
（制御教育と組み込み教育に特に力を入れる。2026-09 時点でほぼ未着手）に照らすと、この
大学講義教材は「外側: Python」の階層に位置づけられていた（同 §16 の地図）。

**次に作るときは、まずどの階層（外側 Python か、L1 Topic API か）に位置づけるかを
決めてから作ること。** 本文書はその判断を行わない——旧教材をそのまま複製するのではなく、
`PROJECT_PLAN.md` §16 の整備状況の地図に沿って階層を選び直すのが前提になる。

## 6. 削除した実体の一覧とタグ名

タグは別担当が付ける。仮称 `archive/2026-09-13`。削除直前のコミットからの取り出しは
`git show archive/2026-09-13:<path>`。

| 実体 | 内容 |
|------|------|
| `docs/university/syllabus.md` | シラバス |
| `docs/university/assessment_rubric.md` | 評価ルーブリック |
| `examples/education/`（8 サブフォルダ） | 実働 Python サンプル |
| `examples/README.md` | 上記への参照を含む |
| `lib/stampfly_edu/` | 教育用 SDK・シミュレータ・ヘルパ一式 |
| `analysis/notebooks/education/`（15 本） | 連番 Jupyter ノートブック |
| `analysis/notebooks/README.md` | 上記の一覧・使い方 |
| `docs/setup/education.md` | 大学講義専用のセットアップ手順（判定は棚卸し参照） |

削除に伴い直す必要がある依存箇所は本文書の対象外（別途の棚卸し結果を参照）。
