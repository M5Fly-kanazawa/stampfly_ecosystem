# v2026.07.2 リリースノート（ドラフト）

`docs/contributing/versioning.md` §5「リリース手順」の手順3（前回リリースからの変更点整理）
に対応するドラフト。**`v2026.07.2` タグはまだ作成されていない**（本ドキュメント作成時点で
最新タグは `v2026.07.1`、作業ブランチは `feature/flasher-native-install`）。タグを打つ際は、
このドラフトの要点を GitHub の自動生成 release notes（`generate_release_notes: true`）に
追記する。

対象範囲: `v2026.07.1..main` の80コミット + `feature/flasher-native-install` ブランチの
4コミット（マージ後に `main` へ合流する想定）。

## 1. 目玉: GUIフラッシャのネイティブインストール（`sf flasher`）

これまで GUI フラッシャ（StampFly Flasher）は「実行ファイルをダウンロードして手動で置く」
運用しかできず、教員が `.exe` を手作業でコピーする必要があった（DXH講座準備で顕在化）。
本リリースでは、これを OS 標準の「アプリをインストールする」体験に揃えた。

| 変更 | コミット | 内容 |
|------|---------|------|
| `sf flasher` コマンド新設 | `e834558` | `install` / `uninstall` / `status` / `update`。GitHub Releases から最新版を取得し SHA256 検証の上、Windows は `%LOCALAPPDATA%\Programs\StampFly`＋スタートメニュー/デスクトップショートカット＋「アプリと機能」登録（管理者権限不要）、macOS は `~/Applications/StampFlyFlasher.app`（quarantine属性自動除去）、Linux は `~/.local/opt/stampfly`＋`.desktop`ランチャーとしてインストール |
| インストーラ Step 4/4 | `16a79df` | `install.bat` / `install.sh`（`scripts/installer.py`）が最後に任意の Step 4/4 として GUI フラッシャのインストールを提案（既定 Yes、`--no-flasher` でスキップ、失敗してもインストーラ全体は失敗にしないベストエフォート） |
| Linux版フラッシャビルド + 4OSスモークテスト | `fcaaf73` | リリースCIに `ubuntu-latest` を追加し `StampFlyFlasher_<tag>_linux-x64` を新規公開。4レッグ（windows-x64 / macos-arm64 / macos-x64 / linux-x64）全てで `sf flasher install --from-file` → 各OSの設置物存在確認 → `sf flasher uninstall` → 削除確認、のスモークテストを追加。Windowsは「設定 > アプリ」のアンインストール経路（`uninstall.cmd`）も別途検証 |
| controller CI修復 | `9304b7d` | `m5unified` を `^0.2.11` → `==0.2.11` に固定（0.2.18が公開されCI解決が壊れていたため）。挙動変化なし、CI安定化のみ |
| `sf flash --gui` の優先順位変更 | `e834558`（flash.py） | インストール済みのネイティブアプリを優先して起動し、未インストール時のみ従来のスクリプト起動にフォールバック |

**ドキュメント（本コミット、P4）:** `docs/commands/sf-flasher.md` 新設、
`tools/flasher_gui/README.md` / `docs/setup/windows.md` / `docs/setup/macos.md` /
`docs/setup/linux.md` / `docs/getting-started.md` / `docs/commands/README.md` /
`.mkdocs/mkdocs.yml` を更新（詳細は本コミットの変更ファイル一覧を参照）。

## 2. ファームウェアの変更点（`firmware/vehicle`, `v2026.07.1` 以降）

> **本ドラフト作成時の依頼メモには「ファームウェアはワークショップ再ビルド＋起動チャイム
> 以外は無変更」という前提があったが、`git log v2026.07.1..main -- firmware/vehicle` を
> 実際に確認したところ誤りだった。** 制御則のデフォルト値変更が複数含まれる
> （§2.1〜§2.2）。以下は実際のコミットとソース (`params.cpp`) の値を照合した内容。

### 2.1 既定挙動が変わるもの

| 変更 | コミット | 内容 |
|------|---------|------|
| ロールレートループ再チューニング | `1785143`（`544c6ac`を上書き） | `rate.roll.kp`: `9.759795e-4` → `1.0e-3`、`rate.roll.td`: `0.01` → `0.001`（パイロット手動調整、実質D項オフに近いPI）。7/17スタディ値(`td=0.02`,`kp`×1.3)はセッション間で汎化せず不採用、7/18に手動再調整した値が最終 |
| ヨーミキサー kappa の実測値補正 | `0ae4dea` | ミキサー定数 `KAPPA`: `9.71e-3` → `6.12e-3`（2026-07-15計測値）。`rate.yaw.kp`: `1.901691e-3` → `1.198594e-3`（物理ゲイン一定を保つよう再スケール）。新規パラメータ `rate.yaw.max_torque`（既定 `1.83e-3` Nm）を追加。NT金沢での突発ヨー回転（2026-06-27）の対策 |

### 2.2 新規実装だが既定は無変更（オプトインのみ）

| 変更 | コミット | 内容 |
|------|---------|------|
| 高度加速度DOB（外乱オブザーバ） | `42c984c` 実装 → `ffbdae5`/`ebe45c5` で既定 `fc=1.5`/`ti_hover=1.5` に一時昇格 → `0a3d8a3` で同日中に既定を撤回 | 現在の既定値は `altitude.dob.fc=0.0`（無効）、`altitude.vel.ti_hover=2.5`（no-op）— **`v2026.07.1` と同じ既定挙動**。実飛行で−67%の高度ばらつき改善を確認済みだが、機体ごとのオプトイン（`param set altitude.dob.fc 1.5` 等）として提供する運用に確定 |
| 高度vel-loop位相スケジューリングTi | `16a7e9b` | 上記と同じ `altitude.vel.ti_hover` パラメータの土台実装。既定no-op |

### 2.3 その他の追加機能

| 変更 | コミット | 内容 |
|------|---------|------|
| `param save_one`/`has_saved` + ユーザーLEDオーバーライド | `91a0894` | 単一パラメータのNVS保存API、およびCLIから任意色でLEDを上書きするコマンド系（ワークショップL0移行の地ならし） |
| モータスイープ電流試験・バッテリー電流テレメトリ | `f4592c8` | `sf` 側にモータ電流スイープ試験コマンドを追加、バッテリー電流をテレメトリに追加 |

### 2.4 ワークショップファーム（`firmware/workshop`）

| 変更 | コミット | 内容 |
|------|---------|------|
| vehicleコンポーネントベースへ全面リビルド | `996ff5e` | レイヤ命名の旧実装から、現行 `firmware/vehicle` のコンポーネントを流用する構成へ刷新 |
| 起動チャイム | `3b2a913` | workshop ファーム専用の起動音を追加: 授業開始チャイム風（ウェストミンスターの鐘・第1フレーズ、E5→C5→D5→G4、約2.7秒）。vehicle の起動音（C5→E5→G5）は不変で、耳でどちらのファームか判別できる |

依頼メモが挙げていた「ワークショップ再ビルド＋起動チャイム」自体は正しく含まれるが、
それ「以外は無変更」という部分は誤りだった。§2.1のロール/ヨーは既定挙動そのものが変わる、
§2.3はvehicle本体への機能追加。

## 3. 変更なし・確認事項

- `firmware/controller`（本体ロジック）: 変更コミットなし。`9304b7d` は依存バージョン
  ピン留めのみでビルド成果物の挙動は変わらない見込み（要: `sf build controller` の
  バイナリサイズ・動作確認）
- `firmware/common`: 変更コミットなし

## 4. リリースアセット

| アセット | 状態 |
|---------|------|
| `stampfly_vehicle_<tag>_full.bin` / `_app.bin` | 既存（§2の変更を含む） |
| `stampfly_controller_<tag>_full.bin` / `_app.bin` | 既存（挙動変化なし） |
| `StampFlyFlasher_<tag>_windows-x64.exe` | 既存 |
| `StampFlyFlasher_<tag>_macos-arm64.zip` / `_macos-x64.zip` | 既存 |
| `StampFlyFlasher_<tag>_linux-x64` | **新規**（拡張子なし、`chmod +x` 必要） |
| `SHA256SUMS.txt` | 既存（全アセット対象） |

リリース本文のテンプレートは `.github/workflows/release.yml` の
`Create GitHub Release` ステップ（`body:` ブロック）に既に更新済み
（Linux アセットと `sf flasher install` への言及を含む）。タグ作成時はこのテンプレートが
そのまま使われる。

## 5. リリース前チェックリスト（`versioning.md` §5 対応）

| # | 項目 | 状態 |
|---|------|------|
| 1 | SIL退行テストPASS | 各コミットのコミットメッセージに記載の結果を参照（例: `0ae4dea`は31/39 PASS、8件は既存インフラ起因で無関係）。本ドラフト作成時点で改めての一括実行は**未実施** |
| 2 | `sf build vehicle` / `sf build controller` ローカルビルド成功 | `9304b7d` で controller 1094.9KB、`91a0894` で vehicle 1118.8KB を確認済み（コミット時点） |
| 3 | 前回リリースからの変更点整理 | 本ドキュメント |
| 4-5 | タグ作成・push | 未実施（P4完了後の想定） |
| 6 | Release workflow成功確認 | `feature/flasher-native-install` での `workflow_dispatch` 実行（run 29662024927、2026-07-19）で**全6ジョブ緑を確認済み**: build(vehicle)/build(controller)/フラッシャ4レッグ（インストールスモーク・Windows `uninstall.cmd` 経路含む）。tag push 時の Release 発行ジョブのみ未実行（タグでのみ走る設計） |
| 7 | Release notes加筆 | 本ドキュメントがその元原稿 |

## 6. Next steps

- `feature/flasher-native-install` を `main` にマージ
- `git tag v2026.07.2` → push → Release workflow のグリーンを確認
- 本ドキュメント§1・§2の要点を GitHub Release の自動生成ノートに追記
- `docs/contributing/versioning.md` §6「カリキュラム互換表」に `v2026.07.2` の行を追加
  （本コミットでは未実施 — タグ作成後に確定情報で追記する）
