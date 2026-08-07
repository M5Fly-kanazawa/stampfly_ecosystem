# SILS GUI — ブラウザで触る SILS 実験環境 / Browser-based SILS workbench

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このツールについて

コマンドラインを叩かずに、ブラウザだけで SILS（ソフトウェアインザループ＝実機ファームをPC上で物理シミュレーションと閉ループ実行）の実験ができる GUI です。`sf sils scenario` で内部的にやっていること——シナリオ作成・パラメータ設定・実行・結果のグラフ化・飛行アニメ——を全部画面で行えます。

### 起動

```bash
source setup_env.sh
sf sils build          # 初回・SILS を変更したとき
sf sils gui            # ブラウザが自動で開く（http://127.0.0.1:8765）
```

オプション: `sf sils gui --port 9000`（ポート変更）、`sf sils gui --no-browser`（自動で開かない）。
止めるときは実行中のターミナルで `Ctrl-C`。

### 画面の構成

```
┌─ ヘッダー: [シナリオ▼] [時間] [ノイズ] [☑電池] [▶実行]   合否 ✅/❌ ─┐
├─ 左: タブ ──────────────────┬─ 右 ───────────────────────────────────┤
│  ［シナリオ作成］            │  飛行アニメーション（three.js ライブ3D）│
│   イベントを表で組む          │   ドラッグで視点回転・▶/スクラブで再生  │
│   （rc/wind/fault/handle…）  │  グラフ（Plotly: 高度/姿勢/モータ）      │
│  ［パラメータ］              │   時刻カーソルが3Dと同期                │
│   54個を検索・編集（実行に反映）│  合否ゲート（PASS/FAIL の各チェック）   │
└──────────────────────────────┴─────────────────────────────────────────┘
```

## 2. 使い方

### 既存シナリオを動かす

1. ヘッダーの「シナリオ」で `crash_refly` などを選ぶ → 左にイベントが読み込まれる。
2. 「▶ 実行」を押す → 物理シミュレーションが走る（数秒）。
3. 右上の3Dで飛行を再生（▶/⏸、スライダーで頭出し、ドラッグで視点回転）。
4. グラフで高度・姿勢（真値 vs 推定）・モータ duty を確認。時刻カーソルが3Dと同期。
5. 下に合否ゲート（その `.expect` の各チェックが PASS/FAIL）。

### シナリオを作る・編集する

- 左の「シナリオ作成」タブで「＋ イベント追加」→ 種類（rc/wind/fault/handle…）を選ぶ。
- 各イベントの時刻 `at`：`0`＝絶対ms、`+`＝直前イベントの直後、`+500`＝直後500ms。
- 「⠿」をドラッグで並べ替え、「✕」で削除。
- 「.scn を表示」で生成されるファイル内容を確認。「💾 .scn 保存」で `scenarios/` に保存。
- 編集すると自動的に「カスタム実行」になり、編集後のイベント列で走ります。

### パラメータを変えて試す

- 「パラメータ」タブで PID ゲインや ESKF 設定などを編集（54項目、検索で絞り込み）。
- 変更した項目だけが走行に反映されます（**再ビルド不要**）。変更行は色が付きます。
- 「変更をクリア」で全て既定に戻します。
- 仕組み: 変更は `SILS_EMU_PARAMS_FILE`（`name value` 行）で emu 起動時に `params::set_*` へ適用。

## 3. 仕組み（開発者向け）

| 部品 | ファイル | 役割 |
|------|---------|------|
| CLI | `lib/sfcli/commands/sils.py` `run_gui` | `sf sils gui` でサーバ起動 |
| サーバ | `gui/server.py` | 標準ライブラリのみ。静的配信＋JSON API。`sf sils scenario` を裏で実行しバンドルを読み返す |
| .scn 変換 | `gui/scn.py` | `.scn` ⇔ イベント配列（ビルダー往復）＋ 既定 duration 推定 |
| params 解析 | `gui/params_meta.py` | `params.cpp` の `table[]`（SSOT）を正規表現で読む |
| 画面 | `gui/static/{index.html,app.js,style.css}` | Plotly（グラフ）＋ three.js（3D、ENU→Y上写像＋FLU機体にquat適用） |

API: `GET /api/scenarios` `GET /api/scenario?name=` `GET /api/params` `POST /api/run` `POST /api/save`。
依存ゼロ（Python 標準ライブラリ）。three.js / Plotly は CDN（プロジェクトの landing page と同様）。

---

<a id="english"></a>

## 1. Overview

A browser GUI for the SILS (software-in-the-loop) workbench — author scenarios, set
parameters, run, and see interactive graphs + a live 3D flight animation, all without the
command line. It wraps the same pipeline `sf sils scenario` uses on the CLI.

### Launch

```bash
source setup_env.sh
sf sils build      # first time / after changing the SILS
sf sils gui        # opens the browser at http://127.0.0.1:8765
```

`--port N` to change the port, `--no-browser` to skip auto-open, `Ctrl-C` to stop.

## 2. Use

- **Run a scenario**: pick one in the header → ▶ 実行 (Run) → play the 3D, read the graphs
  (truth vs estimate), check the PASS/FAIL gates.
- **Build/edit**: the "シナリオ作成" tab — add events (rc/wind/fault/handle…), set each
  event's time (`0`=abs ms, `+`=right after the previous, `+500`=500 ms after). Save to
  `scenarios/` or just run the edited list.
- **Parameters**: the "パラメータ" tab — edit any of the 54 firmware params; only the
  changed ones are applied to the run (no rebuild), via `SILS_EMU_PARAMS_FILE`.

## 3. How it works

stdlib-only HTTP server (`gui/server.py`) serving a single-page app; runs shell out to
`sf sils scenario` and the run bundle (trajectory.csv / results.json / events.jsonl) is read
back for the browser. Graphs use Plotly; the live 3D uses three.js with a worldGroup that
maps ENU (z-up) to the three.js Y-up frame and applies the body framequat (FLU→ENU). The
`.scn` round-trip and parameter metadata are parsed from the firmware SSOT so the GUI never
drifts from the firmware. Zero Python dependencies; three.js/Plotly via CDN.
