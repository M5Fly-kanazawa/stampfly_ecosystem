---
name: sil-milestone
description: SIL マイルストーンの成果物バンドル（results.json＋レビュー動画＋図＋ゲート承認）を1コマンドで生成・検証する。閉ループ run → 動画 → 機械判定ゲートを順に実行し、アウトプット主導でマイルストーンの達成を確定する。RESET_PLAN.md §8〜§10 の実現形。
disable-model-invocation: true
---

# `/sil-milestone` — SIL マイルストーン・バンドル生成スキル

SIL の各マイルストーン（P1〜P4）で**必ず**作る成果物バンドルを1コマンドで揃える。
`/lecture-video` と同じ「決定論ツール＋オーケストレーション」型。**スキル＝作る手順、
フック＝抜けを防ぐ仕組み**（`sf sil gate` / `simulator/sil/tools/sil_gate.py`）。

> 設計の正本: `simulator/sil/RESET_PLAN.md`（§8 CLI/フック、§9 動画、§10 アウトプット主導）

## 単一入口 = `sf sil milestone`

このスキルの手順は**すべて `sf sil` サブコマンドに集約済み**。生のビルド／実行コマンドを
手で並べない（CLI が正本＝ Single Source of Truth）。マイルストーン1本は次の1コマンド:

```bash
source setup_env.sh          # sf CLI を有効化（SIL は host 専用・ESP-IDF 不要）
sf sil milestone -m P1 -e eskf
```

`sf sil milestone` が **build → run → video → gate** を順に実行する（`lib/sfcli/commands/sil.py`
の `run_milestone`）。各段の意味:

| 段 | 中身 | 成果物 |
|----|------|--------|
| build | ホスト SIL を cmake ビルド（`hover_smoke` のみ。初回は MuJoCo を取得） | `simulator/sil/build/hover_smoke` |
| run | 閉ループを実行（実タスクを疑似 RTOS で 400Hz 稼働＋ MuJoCo Plant） | `trajectory.csv` ＋ `results.json` |
| video | レビュー動画を描画（MuJoCo 3D ＋ 状態グラフ、§9・必須） | `<ms>_flight.mp4` |
| gate | バンドル完全性＋機械判定 pass を検査（揃わねば exit 1） | GATE APPROVED / REJECTED |

バンドル出力先は `simulator/sil/viz/out_<milestone小文字>/`。

## アウトプット（成果物バンドル）

各マイルストーンは「やることリスト」でなく成果物で定義する（RESET_PLAN §10）。
バンドルが揃って機械判定が pass して初めて「達成」:

1. `results.json` — 機械によるゲート合否（唯一の正）
2. レビュー動画 `*.mp4` — 飛行3Dアニメ＋状態グラフ（人間の確認＋アピール素材、§9）
3. `trajectory.csv` — 再現可能な時系列（同じ実行→同じ動画）
4. ゲート承認（`sf sil gate` が exit 0）

## 実行手順

### 1. マイルストーン1本を生成・ゲート

引数: マイルストーン名（既定 `P1`）と推定器（`eskf` / `complementary`、既定 `eskf`）。

```bash
source setup_env.sh
sf sil milestone -m P2 -e complementary
```

- 末尾に `Milestone P2 bundle complete and gated.` が出れば成功。
- 途中で停止したら `Milestone … stopped at <step>` が原因段を示す。バンドルを
  「達成」と宣言せず、原因を報告して止める（アウトプット主導の強制）。

> `sf sil milestone` は冪等。再実行すれば同じ `trajectory.csv`／動画を再生成する
> （SIL は決定論的＝同じ実行→同じ成果物）。

### 2. 段を個別に回したいとき（任意）

`milestone` は4段を束ねた糖衣。デバッグや再生成で段を分けたいときは個別に叩く:

```bash
sf sil build                       # ホスト SIL をビルド
sf sil run     -m P2 -e complementary   # 閉ループ実行 → trajectory.csv + results.json
sf sil video   -m P2               # レビュー動画を描画
sf sil status  -m P2               # 機械判定（results.json）を表示
sf sil gate    -m P2               # バンドル完全性＋ pass を検査
```

`sf sil status` は G3 の各チェック（took_off / attitude_bounded / yaw_spin /
yaw_stopped / alt_held / landed）と metrics を表示する＝ダッシュボード（RESET_PLAN §8）。

### 3. 動画の破損確認（任意）

```bash
ffprobe -v error -show_entries format=duration:stream=width,height,nb_frames \
  -of default=noprint_wrappers=1 simulator/sil/viz/out_<ms>/*.mp4
```

### 4. 動画の目視確認（サブエージェント限定・必須）

**動画フレームの画像 Read はメインコンテキストで行わない**（CLAUDE.md スライド画像
ルールに準ずる。画像 Read はコンテキストを大きく消費し rate limit の原因になる）。
サブエージェントで数フレーム（ホバー時・マニューバ時）を抽出・確認し、テキストの
所見だけ返させる:

```
ffmpeg -y -ss <t> -i simulator/sil/viz/out_<ms>/*.mp4 -frames:v 1 /tmp/sil_frame.png
```

確認項目: 3D に機体が映る／グラフが時刻カーソルと同期／指令追従が見える。

### 5. 報告

人間が確認できる形でまとめる:
- ゲート結果（APPROVED/REJECTED）と `results.json` の主要 metrics（`sf sil status`）
- 動画パス（`simulator/sil/viz/out_<ms>/<ms>_flight.mp4`）と長さ・解像度
- 目視所見（サブエージェント）
- 不合格なら不足項目と次アクション

## フック連携（抜けを防ぐ仕組み）

このスキルは**作る手順**。**抜けを防ぐ**のはゲート（`sf sil gate` →
`simulator/sil/tools/sil_gate.py`）と git フック:

- **ゲート**: マイルストーンを「達成」と宣言する前に必ず `sf sil gate` の exit 0 を
  要求する。バンドル（`results.json` ＋ レビュー動画 ＋ `trajectory.csv`）が揃い、
  機械判定が pass でなければ承認を拒否する。
- **git タグ・フック**（任意・推奨）: マイルストーンタグ（例 `sil-p1`）を打つ前に
  `simulator/sil/tools/sil_gate.py <bundle>` を走らせ、exit 1 ならタグを拒否する
  （`.githooks/pre-push`、opt-in: `git config core.hooksPath .githooks`）。
- **注意**: Claude Code の settings.json フックはツールイベント（PreToolUse 等）に
  反応するもので「マイルストーン」イベントは無い。マイルストーンの強制は上記の
  ゲートスクリプト＋git フックで行う（settings.json フックは用途が別）。
