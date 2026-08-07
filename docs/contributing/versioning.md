# Versioning Policy
# バージョニング規約

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

---

## 1. 概要

StampFly Ecosystem 全体（vehicle・controllerファームウェア、教材、sf CLI 等）のリリースと
バージョン番号の付け方を定義する。ここで言う「リリース」は、Gitタグ `vYYYY.MM.PATCH` を打った
時点のリポジトリのスナップショットに対して、`.github/workflows/release.yml` がビルド・公開する
ファームウェアバイナリ一式を指す。

対象読者は、リリースタグを打つ担当者、および教育現場でファームウェアを配布・導入する教員・
学生である。

## 2. バージョン体系

**CalVer（Calendar Versioning）: `vYYYY.MM.PATCH`** を採用する（例: `v2026.07.0`）。

| 要素 | 意味 | 例 |
|------|------|-----|
| `YYYY` | リリースを打った年 | `2026` |
| `MM` | リリースを打った月（2桁、01〜12） | `07` |
| `PATCH` | 同一年月内での通し番号（0始まり） | `0`, `1`, `2`, ... |

同じ年月内に追加の修正リリースを出す場合は `PATCH` をインクリメントする（例: `v2026.07.0` →
`v2026.07.1`）。

SemVer（`vMAJOR.MINOR.PATCH`、互換性を意味付けするバージョン体系。例: `v2.3.1` の
MAJORが上がると後方互換性が壊れる、というのがSemVerの約束事）ではなくCalVerを採用する理由は
以下の2点:

| 理由 | 説明 |
|------|------|
| (a) 学期固定運用との相性 | 教育現場では「学期中はバージョンを固定する」運用を推奨しており、年月がそのままバージョンに出ることで、教員・学生が「いつ時点のものか」を直感的に判断できる |
| (b) 単一コンポーネントの互換性ではなくスナップショットが本質 | 1つのタグで vehicle・controller 両ファームウェアと教材を束ねてリリースする。SemVerが表現する「このAPIとの互換性」よりも「エコシステム全体がいつ時点のスナップショットか」の方が重要な情報になる |

## 3. リリースの範囲

1つのタグ（同一コミット）から vehicle・controller 両方のファームウェアをビルドし、Releaseに
添付する。教材（ワークショップ資料・大学カリキュラム）とファームウェアの整合を、1つのバージョン
番号で固定するためである。

成果物の命名規則は以下の通り（`<tag>` はGitタグ名、例 `v2026.07.0`）:

| ファイル名 | 内容 |
|-----------|------|
| `stampfly_vehicle_<tag>_full.bin` | vehicle: bootloader + パーティションテーブル + app を結合した工場出荷相当イメージ。`esptool.py write_flash 0x0` で一括書き込み可能 |
| `stampfly_vehicle_<tag>_app.bin` | vehicle: アプリケーション本体のみ |
| `stampfly_controller_<tag>_full.bin` | controller: bootloader + パーティションテーブル + OTAデータ + app を結合した工場出荷相当イメージ |
| `stampfly_controller_<tag>_app.bin` | controller: アプリケーション本体のみ |
| `StampFlyFlasher_<tag>_windows-x64.exe` | GUI フラッシャ: Windows 版実行ファイル |
| `StampFlyFlasher_<tag>_macos-arm64.zip` | GUI フラッシャ: macOS (Apple Silicon) 版 .app |
| `StampFlyFlasher_<tag>_macos-x64.zip` | GUI フラッシャ: macOS (Intel) 版 .app |
| `StampFlyFlasher_<tag>_linux-x64` | GUI フラッシャ: Linux 版実行ファイル（拡張子なし、`chmod +x` が必要） |
| `StampFlySetup_windows-x64.exe` | GUI インストーラ: Windows 版実行ファイル |
| `StampFlySetup_macos-arm64.zip` | GUI インストーラ: macOS (Apple Silicon) 版 .app |
| `StampFlySetup_macos-x64.zip` | GUI インストーラ: macOS (Intel) 版 .app |
| `StampFlySetup_linux-x64` | GUI インストーラ: Linux 版実行ファイル（拡張子なし、`chmod +x` が必要） |
| `SHA256SUMS.txt` | 上記全ファイルのSHA256チェックサム一覧 |

## 4. コンポーネント個別バージョンとの関係

以下のコンポーネントバージョンは、エコシステム全体のリリースタグとは独立して管理する。

| コンポーネント | バージョン管理方法 | 現状 |
|---------------|-------------------|------|
| sf CLI | `lib/sfcli/__init__.py` の `__version__` を手動管理 | `0.1.0` |
| vehicle / controller ファームウェア | ESP-IDF が `git describe` からビルド時に `PROJECT_VER` を自動埋め込み | タグを打てば、ファームウェア自身が `sf log info` 等で自分のビルド元バージョンを報告できる |

sf CLI のバージョンを個別に上げるかどうかは、エコシステムのリリースタグとは切り離して判断して
よい。

## 5. リリース手順

タグを打つ前に、以下のチェックリストを上から順に実施する。変更作業そのもの
（パラメータ既定値の変え方・既存機体への反映を含む前後の全体フロー）は
[release-workflow.md](release-workflow.md) を参照。

| # | 項目 | コマンド／確認内容 |
|---|------|-------------------|
| 1 | SILS退行テスト（変更で既存機能が壊れていないことをシミュレーション上で確認するテスト。英語では regression test）がPASSしていること | `sf sils regression`（`sf sils scenario` を対象シナリオ一式に一括適用）でPASSを確認。main へのpush・PRで GitHub Actions（`sils-regression.yml`）が自動実行するため、ローカル実行は事前確認・デバッグ用の補助 |
| 2 | vehicle・controller 両ターゲットのローカルビルドが通ること | `sf build vehicle` / `sf build controller` |
| 3 | 前回リリースからの変更点を整理すること | CHANGELOG的に、主要な `feat`/`fix` コミットを箇条書きで洗い出す |
| 4 | バージョンタグを作成すること | `git tag vYYYY.MM.P`（例: `git tag v2026.07.0`） |
| 5 | タグをリモートへpushすること | `git push origin vYYYY.MM.P` |
| 6 | Actions の Release workflow が成功し、アセット一式が揃っていることを確認すること | GitHub の Actions タブで `Release firmware binaries` の完走を確認し、Release ページに §3 の全アセット（ファーム4 + フラッシャ4 + セットアップ4 + `SHA256SUMS.txt` の13点）が添付されていることを確認 |
| 7 | Release notes を加筆すること | 自動生成された release notes（`generate_release_notes: true`）に、手順3で整理した要点や既知の制約を追記 |

## 6. カリキュラム互換表

リリースと教材（ワークショップ／大学カリキュラム）の互換性を以下の表で管理する。新しい
リリースを出す際は行を追加する。

| リリース | ワークショップ教材 | 大学カリキュラム | 状態 |
|---------|-------------------|-----------------|------|
| `v2026.07.0` | （未定） | （未定） | リリース済み (2026-07-08) |
| `v2026.07.2` | DXH 2026-07 版（vehicle 基盤 workshop ファーム） | （未定） | リリース済み (2026-07-20)。StampFly Setup 初出・`rate.roll.td=0.002` |
| `v2026.07.3` | 同上（ファーム無変更） | （未定） | リリース済み (2026-07-20)。StampFly Terminal ランチャー・Setup 修復モード修正 |
| `v2026.07.4` | 同上（ファーム無変更） | （未定） | Windows GUIインストーラ大幅強化（CRLF/cp932/exit9009/jinja2衝突修正）・Python対応3.10〜3.12確定・仮想環境マネージャ対応・シミュレータ symlink 廃止 |
| `v2026.07.5` | 同上（ファーム無変更） | （未定） | SILS シミュレータの Windows ネイティブ対応（MinGW-w64、WSL不要）・`sf sils regression` と CI 自動回帰ゲート・GUIインストーラに SILS 開発ツールチェーン導入オプション追加・hover_smoke ハーネス離陸フェーズ修正 |
| `v2026.07.6` | 同上（ファーム無変更） | （未定） | インストーラ・開発環境の互換性修正（pyenv-win shim による exit 9009 再発の根本修正、setup_env の対応 Python 誘導、sfcli の venv 経由 idf.py/esptool 起動、IDF venv と Python の整合検証、シミュレータ依存導入の検証強化） |

## 7. 教育機関向け推奨

| 推奨事項 | 理由 |
|---------|------|
| 学期中はバージョンを固定する | 授業の途中で機体の挙動が変わると、受講者の混乱や再現性の喪失につながる。学期開始時に決めたタグから動かさないことを推奨する |
| 配布には `full` bin を使う | `full` bin は工場出荷相当（bootloader・パーティションテーブル・アプリ一式を結合済み）で、`esptool.py write_flash 0x0` または M5Burner・Webフラッシャで書き込むだけで動作する。開発環境の構築（ESP-IDFのインストール等）を必要としないため、多数の実機を扱う教育現場での再現性・導入コストの観点で `app` bin より推奨する |

---

<a id="english"></a>

## 1. Overview

This document defines how releases and version numbers are assigned across the entire
StampFly Ecosystem (vehicle/controller firmware, teaching materials, sf CLI, etc.). A
"release" here refers to the set of firmware binaries built and published by
`.github/workflows/release.yml` for a repository snapshot tagged `vYYYY.MM.PATCH`.

The target audience is whoever cuts a release tag, as well as instructors and students who
distribute or deploy firmware in educational settings.

## 2. Versioning Scheme

We use **CalVer (Calendar Versioning): `vYYYY.MM.PATCH`** (e.g. `v2026.07.0`).

| Component | Meaning | Example |
|-----------|---------|---------|
| `YYYY` | Year the release was cut | `2026` |
| `MM` | Month the release was cut (2 digits, 01-12) | `07` |
| `PATCH` | Sequence number within the same year-month (starts at 0) | `0`, `1`, `2`, ... |

For an additional fix release within the same year-month, increment `PATCH` (e.g.
`v2026.07.0` -> `v2026.07.1`).

We chose CalVer over SemVer (`vMAJOR.MINOR.PATCH`, a versioning scheme where the numbers
encode compatibility guarantees, e.g. a MAJOR bump signals a backward-incompatible change) for
two reasons:

| Reason | Explanation |
|--------|-------------|
| (a) Fits "pin the version for a semester" operation | We recommend instructors pin a fixed version for the duration of a semester; encoding the year and month directly in the version lets instructors and students intuitively tell how recent a release is |
| (b) A release is a bundled snapshot, not a single component's compatibility contract | A single tag bundles both vehicle and controller firmware plus teaching materials from the same commit. What matters most is "which point-in-time snapshot of the ecosystem is this," rather than the API-compatibility semantics SemVer expresses |

## 3. Release Scope

A single tag (a single commit) builds both vehicle and controller firmware and attaches them
to the same Release. This pins the correspondence between teaching materials (workshop
handouts, university curriculum) and firmware to one version number.

Artifact naming convention (`<tag>` is the Git tag name, e.g. `v2026.07.0`):

| Filename | Contents |
|----------|----------|
| `stampfly_vehicle_<tag>_full.bin` | vehicle: factory-equivalent image combining bootloader + partition table + app. Flashable in one shot with `esptool.py write_flash 0x0` |
| `stampfly_vehicle_<tag>_app.bin` | vehicle: application binary only |
| `stampfly_controller_<tag>_full.bin` | controller: factory-equivalent image combining bootloader + partition table + OTA data + app |
| `stampfly_controller_<tag>_app.bin` | controller: application binary only |
| `StampFlyFlasher_<tag>_windows-x64.exe` | GUI flasher: Windows executable |
| `StampFlyFlasher_<tag>_macos-arm64.zip` | GUI flasher: macOS (Apple Silicon) .app |
| `StampFlyFlasher_<tag>_macos-x64.zip` | GUI flasher: macOS (Intel) .app |
| `StampFlyFlasher_<tag>_linux-x64` | GUI flasher: Linux executable (no extension; needs `chmod +x`) |
| `StampFlySetup_windows-x64.exe` | GUI installer: Windows executable |
| `StampFlySetup_macos-arm64.zip` | GUI installer: macOS (Apple Silicon) .app |
| `StampFlySetup_macos-x64.zip` | GUI installer: macOS (Intel) .app |
| `StampFlySetup_linux-x64` | GUI installer: Linux executable (no extension; needs `chmod +x`) |
| `SHA256SUMS.txt` | SHA256 checksums for all files above |

## 4. Relationship to Individual Component Versions

The following component versions are managed independently of the ecosystem-wide release tag.

| Component | Versioning method | Current |
|-----------|-------------------|---------|
| sf CLI | `__version__` in `lib/sfcli/__init__.py`, managed manually | `0.1.0` |
| vehicle / controller firmware | ESP-IDF automatically embeds `PROJECT_VER` from `git describe` at build time | Once a tag exists, the firmware itself can report which version it was built from (e.g. via `sf log info`) |

Whether to bump the sf CLI version is a decision independent of cutting an ecosystem release
tag.

## 5. Release Procedure

Before cutting a tag, work through the following checklist in order.

| # | Item | Command / What to check |
|---|------|--------------------------|
| 1 | SILS regression (a simulation-based check that changes have not broken existing behavior — "regression" here means software regression, not statistical regression) passes | Confirm PASS via `sf sils regression` (applies `sf sils scenario` to the full scenario suite in one shot). GitHub Actions (`sils-regression.yml`) runs this automatically on every push to main and every PR, so a local run is mainly for pre-check / debugging |
| 2 | Both vehicle and controller build locally | `sf build vehicle` / `sf build controller` |
| 3 | Summarize changes since the last release | Draft a CHANGELOG-style bullet list of the key `feat`/`fix` commits |
| 4 | Create the version tag | `git tag vYYYY.MM.P` (e.g. `git tag v2026.07.0`) |
| 5 | Push the tag to the remote | `git push origin vYYYY.MM.P` |
| 6 | Confirm the Release workflow succeeds and the full asset set is attached | Check the Actions tab for a green `Release firmware binaries` run, then confirm the Release page carries all §3 assets (4 firmware + 4 flasher + 4 setup + `SHA256SUMS.txt` = 13 files) |
| 7 | Fill in the Release notes | Add the highlights from step 3 and any known limitations on top of the auto-generated notes (`generate_release_notes: true`) |

## 6. Curriculum Compatibility Table

The table below tracks compatibility between releases and teaching materials (workshop /
university curriculum). Add a row whenever a new release is cut.

| Release | Workshop materials | University curriculum | Status |
|---------|--------------------|-----------------------|--------|
| `v2026.07.0` | TBD | TBD | released (2026-07-08) |
| `v2026.07.2` | DXH 2026-07 edition (vehicle-based workshop firmware) | TBD | released (2026-07-20); first StampFly Setup, `rate.roll.td=0.002` |
| `v2026.07.3` | same (firmware unchanged) | TBD | released (2026-07-20); StampFly Terminal launcher, Setup repair-mode fix |
| `v2026.07.4` | same (firmware unchanged) | TBD | major Windows GUI installer hardening (CRLF/cp932/exit 9009/jinja2 conflict fixes), Python support pinned to 3.10-3.12, virtualenv-manager support, simulator symlink removal |
| `v2026.07.5` | same (firmware unchanged) | TBD | native Windows SILS simulator support (MinGW-w64, no WSL needed), `sf sils regression` + automated CI regression gate, optional SILS dev toolchain install in the GUI installer, hover_smoke harness takeoff-phase fix |
| `v2026.07.6` | same (firmware unchanged) | TBD | installer / dev-environment compatibility fixes (root-cause fix for the pyenv-win shim exit 9009 regression, setup_env steering to supported Python, sfcli invoking idf.py/esptool via the venv python, IDF venv / Python match verification, hardened simulator dependency install) |

## 7. Recommendations for Educational Institutions

| Recommendation | Rationale |
|-----------------|-----------|
| Pin a fixed version for the duration of a semester | If the aircraft's behavior changes mid-course, it confuses students and breaks reproducibility. We recommend staying on the tag chosen at the start of the semester |
| Distribute using the `full` bin | The `full` bin is factory-equivalent (bootloader, partition table, and application already merged) and can be flashed with just `esptool.py write_flash 0x0`, M5Burner, or a web flasher. It requires no development environment setup (e.g. installing ESP-IDF), making it preferable to the `app` bin for reproducibility and low setup cost when handling many physical units in a classroom |
