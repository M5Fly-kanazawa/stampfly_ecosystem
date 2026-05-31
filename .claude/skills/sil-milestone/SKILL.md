---
name: sil-milestone
description: SIL マイルストーンの成果物バンドル（results.json＋レビュー動画＋図＋ゲート承認）を1コマンドで生成・検証する。閉ループ run → 動画 → 機械判定ゲートを順に実行し、アウトプット主導でマイルストーンの達成を確定する。RESET_PLAN.md §8〜§10 の実現形。
disable-model-invocation: true
---

# `/sil-milestone` — SIL マイルストーン・バンドル生成スキル

SIL の各マイルストーン（P1〜P4）で**必ず**作る成果物バンドルを1コマンドで揃える。
`/lecture-video` と同じ「決定論ツール＋オーケストレーション」型。**スキル＝作る手順、
フック＝抜けを防ぐ仕組み**（`simulator/sil/tools/sil_gate.py`）。

> 設計の正本: `simulator/sil/RESET_PLAN.md`（§8 CLI/フック、§9 動画、§10 アウトプット主導）

## アウトプット（成果物バンドル）

各マイルストーンは「やることリスト」でなく成果物で定義する（RESET_PLAN §10）。
バンドルが揃って機械判定が pass して初めて「達成」:

1. `results.json` — 機械によるゲート合否（唯一の正）
2. レビュー動画 `*.mp4` — 飛行3Dアニメ＋状態グラフ（人間の確認＋アピール素材、§9）
3. `trajectory.csv` — 再現可能な時系列（同じ実行→同じ動画）
4. ゲート承認（`sil_gate.py` が exit 0）

## 実行手順

引数: マイルストーン名（既定 `P1`）。バンドル出力先は `simulator/sil/viz/out_<milestone小文字>/`。

### 1. 環境とパスの確定

```bash
cd "$(git rev-parse --show-toplevel)"
MS="${1:-P1}"; MSL=$(echo "$MS" | tr 'A-Z' 'a-z')
BUNDLE="simulator/sil/viz/out_${MSL}"
MODEL="$PWD/simulator/sil/models/stampfly.xml"
PY="simulator/sil/viz/venv/bin/python"
mkdir -p "$BUNDLE"
```

venv（mujoco==3.9.0 + matplotlib + imageio）が無ければ作る:

```bash
test -x "$PY" || { python3 -m venv simulator/sil/viz/venv && \
  simulator/sil/viz/venv/bin/pip install -q "mujoco==3.9.0" numpy matplotlib imageio imageio-ffmpeg; }
```

### 2. SIL をビルド（ホスト、MuJoCo 込み）

```bash
cmake -S simulator/sil -B simulator/sil/build >/dev/null
cmake --build simulator/sil/build -j8 --target hover_smoke
```

`error` が出たら停止して原因を報告する（バンドルを作らない）。

### 3. 閉ループ run でバンドル（trajectory.csv ＋ results.json）を生成

現状は P1=`hover_smoke`（G3 閉ループホバー＋ヨー追従）。将来のマイルストーンは対応
する run バイナリに差し替える（例: P2 は `--estimator complementary`）。

```bash
./simulator/sil/build/hover_smoke "$MODEL" "$BUNDLE"
```

stdout の `OK — G3 closed-loop hover` と `bundle written to ...` を確認する。

### 4. レビュー動画を生成（§9・必須）

```bash
"$PY" simulator/sil/viz/render_video.py \
  --model "$MODEL" --bundle "$BUNDLE" --out "$BUNDLE/${MSL}_hover.mp4" --fps 50
```

3D 飛行アニメ＋状態グラフ（高度・傾き・ヨーレート・モータ duty を真値/推定/指令で
重ね描き）を合成した MP4 ができる。`ffprobe` で破損が無いか確認:

```bash
ffprobe -v error -show_entries format=duration:stream=width,height,nb_frames \
  -of default=noprint_wrappers=1 "$BUNDLE"/*.mp4
```

### 5. ゲート検査（機械判定＋バンドル完全性）

```bash
"$PY" simulator/sil/tools/sil_gate.py "$BUNDLE"
```

- **exit 0（GATE APPROVED）**: バンドル完全・機械判定 pass。マイルストーン達成。
- **exit 1（GATE REJECTED）**: 不足項目を報告して停止。揃うまでマイルストーンを
  「達成」と宣言しない（アウトプット主導の強制）。

### 6. 動画の目視確認（サブエージェント限定）

**動画フレームの画像 Read はメインコンテキストで行わない**（CLAUDE.md スライド画像
ルールに準ずる）。サブエージェントで数フレーム（ホバー時・マニューバ時）を抽出・
確認し、テキストの所見だけ返す:

```
ffmpeg -y -ss <t> -i "$BUNDLE"/*.mp4 -frames:v 1 /tmp/sil_frame.png
```
（3D に機体が映る／グラフが時刻カーソルと同期／指令追従が見える、を確認）

### 7. 報告

人間が確認できる形でまとめる:
- ゲート結果（APPROVED/REJECTED）と `results.json` の主要 metrics
- 動画パス（`$BUNDLE/<ms>_hover.mp4`）と長さ・解像度
- 目視所見（サブエージェント）
- 不合格なら不足項目と次アクション

## フック連携（抜けを防ぐ仕組み）

このスキルは**作る手順**。**抜けを防ぐ**のはゲート（`sil_gate.py`）と git フック:

- **ゲート**: マイルストーンを「達成」と宣言する前に必ず exit 0 を要求する。
  `sf sil gate <Gn> --approve` を将来作る場合も、内部でこのスクリプトを呼ぶ。
- **git タグ・フック**（任意・推奨）: マイルストーンタグ（例 `sil-p1`）を打つ前に
  `simulator/sil/tools/sil_gate.py <bundle>` を走らせ、exit 1 ならタグを拒否する。
  例（`.git/hooks/pre-push` 等にバンドル検査を仕込む、または CI で実行）。
- **注意**: Claude Code の settings.json フックはツールイベント（PreToolUse 等）に
  反応するもので「マイルストーン」イベントは無い。マイルストーンの強制は上記の
  ゲートスクリプト＋git フックで行う（settings.json フックは用途が別）。
