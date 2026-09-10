# sf upgrade

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

追跡している（tracked）リモートブランチから最新の変更を取得し、ローカルの変更は自動stash（一時退避）で保護しつつ取り込みます。取り込んだ後は、Python依存関係の再同期、陳腐化したESP-IDFの `sdkconfig` の検出、ネイティブGUIフラッシャの更新まで一括で行う、`sf` を最新化するための唯一のコマンドです。

Git初心者向けの丁寧な解説（衝突の解決手順、インストール/アンインストールの内訳表を含む）は **[アップグレードガイド](../guides/upgrading.md)** を参照してください。このページはコマンドリファレンス（構文・オプション・終了コード）です。

## 2. 構文

```bash
sf upgrade [--yes] [--discard-local] [--no-flasher] [--skip-deps] [--migrate | --no-migrate]
```

## 3. オプション

| オプション | 説明 |
|-----------|------|
| `--yes`, `-y` | 更新内容のプレビュー確認を省略する（`--discard-local` の確認は省略されない — 破壊的操作のため常に確認）。フラッシャが未導入の場合の一回限りのインストール提案も尋ねない（機会は消費しない） |
| `--discard-local` | ローカルの変更をstashせず破棄してから更新する（破壊的操作。ファイル一覧を表示した上で必ず確認） |
| `--no-flasher` | ネイティブGUIフラッシャの更新提案、および未導入時の一回限りのインストール提案をスキップする（機会は消費しない） |
| `--skip-deps` | Python依存関係の再同期ステップをスキップする |
| `--migrate` | 専用環境（private Python 3.12 + ESP-IDF v5.5.2、SF_HOME配下に自己完結）への移行を、確認なしで実行する。旧来環境を選んだ後（`kind = "legacy"`記録済み）でも、このフラグを付ければ改めて移行できる |
| `--no-migrate` | 今回の実行では専用環境への移行を一切提案しない（`--migrate` と同時指定不可） |

## 4. やること（ステップ概要）

1. リポジトリ確認（gitクローンか、`origin` リモートが設定済みか）・現在ブランチ表示
2. `git fetch` して遅れているコミット数を確認（0件なら専用環境への移行提案・依存同期・書き込みアプリ未導入時の一回限り提案＝下記7だけ実行して終了）
3. 取り込まれるコミットの一覧をプレビュー表示し、`Y/n` で確認
4. ローカル変更を安全に取り込む（既定=stash→マージ→復元、`--discard-local`=確認の上で破棄）
5. `sdkconfig.defaults` / `partitions.csv` が変わっていれば、既存 `sdkconfig` を `*.pre-upgrade-backup` へ退避
6. **専用環境への移行提案**（まだ専用環境でなければ）: 対話実行では既定 `Y` で確認、`--migrate` なら確認なしで実行、非対話（CI等）では一度案内するだけでスキップする。承諾/`--migrate`時は `scripts/installer.py --dedicated --non-interactive --no-flasher` を実行し、成功すればそこで終了する（残りの依存同期・フラッシャ提案は専用環境が別途すべて備えているため省略）。辞退時は `.sf/config.toml` に `kind = "legacy"` を記録し、以後は尋ねない（`sf upgrade --migrate` でいつでも再提案可能）
7. Python依存関係を再同期（`pip install -e .`）※専用環境へ移行した場合はここへ到達しない
8. ネイティブGUIフラッシャ: 導入済みなら更新を提案。**未導入なら、チェックアウトにつき一回だけ**インストールを提案（`--yes` または `--no-flasher` 指定時はこの一回限りの機会を消費せずスキップ。一度尋ねたら `.sf/flasher_install_offered` に記録し、以後は二度と尋ねない）
9. サマリ表示（更新前後のコミットハッシュ・実施した処置・推奨次アクション）

> **専用環境への移行で作られるもの:** `SF_HOME`（既定は Windows `C:\StampFly`、macOS/Linux
> `~/.stampfly`）配下に、専用の Python 3.12（数十〜150MB程度）・専用の ESP-IDF v5.5.2
> （約0.65GB）・`IDF_TOOLS_PATH`（ツール+ビルド用仮想環境、合計で数GB）を新規に用意します。
> 合計の目安は**約4〜6GB**（ダウンロード＋ディスク使用量）です。**現在の環境（システム
> Python・既存のESP-IDF）は削除・変更されません** — 専用環境はその横に新規作成されるだけです。
> 移行が終わったら、**このターミナルを閉じて新しいターミナル（または StampFly Terminal）を
> 開き直してから**使い始めてください（`setup_env` を開き直すだけでも構いません）。
> 現在の環境（旧来環境）を手動で片付けたい場合は、[アップグレードガイド](../guides/upgrading.md)
> の「専用環境への移行」にある手動削除の対象・場所の一覧を参照してください。

詳細な各ステップの解説と、Gitコマンドとの対応表は [アップグレードガイド §3](../guides/upgrading.md) を参照してください。

> **自己ブートストラップ:** ステップ2で取得した更新に `sf` 自身（`lib/sfcli`）への変更が
> 含まれる場合、ステップ3のプレビューに進む前に、取得したばかりの最新版の `upgrade` ロジックへ
> 自動的に処理を引き継ぎます（`sf itself was updated upstream -- handing over...` と表示）。
> つまり `sf upgrade` 自体のバグ修正は、**1回の `sf upgrade` 実行だけで**その場で反映されます
> （古い実装で更新を取り込んでから改めて実行し直す、という2手を踏む必要がありません）。

## 5. 終了コード

| コード | 意味 |
|--------|------|
| `0` | 成功（既に最新だった場合、確認をキャンセルした場合を含む） |
| `1` | 一般的なエラー（git未インストール、fetch失敗、依存同期失敗など） |
| `2` | 安全には処理したがユーザー対応が必要（stash復元時の衝突、ローカルコミットで分岐しfast-forwardできない） |

終了コード `2` になった場合の対処は [アップグレードガイド §4 衝突（コンフリクト）の解決](../guides/upgrading.md#conflicts) を参照してください。

## 6. 使用例

```bash
# 通常の更新（プレビューを見てから Y で進める）
sf upgrade

# 確認なしで自動更新（CI・スクリプト向け）
sf upgrade --yes

# 依存関係の再同期・GUIフラッシャの更新提案をスキップ
sf upgrade --skip-deps --no-flasher

# ローカル変更を諦めて公式の最新版だけを取り込む
sf upgrade --discard-local

# 専用環境への移行を確認なしで実行する
sf upgrade --migrate

# 今回は専用環境への移行提案を出さない
sf upgrade --no-migrate
```

---

<a id="english"></a>

## 1. Overview

Fetches the latest changes from the tracked remote branch and folds them in while protecting local edits with an automatic stash. Afterward it resyncs Python dependencies, detects stale ESP-IDF `sdkconfig` files, and offers to update the native GUI Flasher app -- the one command meant to bring `sf` fully up to date.

For a beginner-friendly walkthrough (conflict resolution, the install/uninstall breakdown table), see the **[Upgrading Guide](../guides/upgrading.md)**. This page is the command reference (syntax, options, exit codes).

## 2. Syntax

```bash
sf upgrade [--yes] [--discard-local] [--no-flasher] [--skip-deps] [--migrate | --no-migrate]
```

## 3. Options

| Option | Description |
|--------|-------------|
| `--yes`, `-y` | Skip the update-preview confirmation (the `--discard-local` confirmation is never skipped -- it is destructive). Also skips the one-time "install the Flasher?" offer when it is not installed, without consuming that one-time chance |
| `--discard-local` | Discard local changes instead of stashing them before updating (destructive; the changed-file list is shown and always confirmed) |
| `--no-flasher` | Skip the offer to update the native GUI Flasher app, and the one-time install offer if it is not installed (without consuming that one-time chance) |
| `--skip-deps` | Skip the Python dependency resync step |
| `--migrate` | Migrate to the dedicated environment (private Python 3.12 + ESP-IDF v5.5.2, self-contained under SF_HOME) without asking. Works even after you previously chose the legacy environment (recorded as `kind = "legacy"`) |
| `--no-migrate` | Never offer the dedicated-environment migration this run (mutually exclusive with `--migrate`) |

## 4. What It Does (Step Overview)

1. Repository check (git clone with an `origin` remote?) and current-branch display
2. `git fetch`, then check how many commits behind (0 -> the dedicated-environment migration offer, dependency resync, and the one-time flasher-install offer (step 8) run, then it exits)
3. Show a preview of incoming commits and ask `Y/n`
4. Safely fold in local changes (default = stash -> merge -> restore, `--discard-local` = discard after confirmation)
5. If `sdkconfig.defaults` / `partitions.csv` changed, back up any existing `sdkconfig` to `*.pre-upgrade-backup`
6. **Dedicated-environment migration offer** (unless already dedicated): interactive runs default to `Y`; `--migrate` skips the prompt and always proceeds; a non-interactive run (CI, etc.) just prints a pointer and skips it. If accepted (or forced via `--migrate`), runs `scripts/installer.py --dedicated --non-interactive --no-flasher` and, on success, stops right there -- the remaining steps are skipped because the new dedicated environment already has everything they would have provided. Declining records `kind = "legacy"` in `.sf/config.toml` so you are not asked again (re-offer any time with `sf upgrade --migrate`)
7. Resync Python dependencies (`pip install -e .`) -- skipped if migration happened above
8. Native GUI Flasher: offer to update it if installed. If **not** installed, offer to install it **once per checkout** (`--yes`/`--no-flasher` skip this without consuming the one-time chance; once asked, the answer is recorded in `.sf/flasher_install_offered` and never asked again)
9. Print a summary (before/after commit hash, actions taken, recommended next step)

> **What the dedicated-environment migration creates:** under `SF_HOME` (default `C:\StampFly`
> on Windows, `~/.stampfly` on macOS/Linux), a private Python 3.12 (tens of MB to ~150MB), a
> private ESP-IDF v5.5.2 checkout (~0.65GB), and `IDF_TOOLS_PATH` (build tools + virtualenvs,
> several GB total) -- **about 4-6GB** of download/disk altogether. **Your current environment
> (system Python, any existing ESP-IDF) is left untouched** -- the dedicated one is created
> alongside it. Once migration finishes, **close this terminal and open a new one (or StampFly
> Terminal)** before using it (re-running `setup_env` also works). To manually clean up the old
> (legacy) environment afterward, see the list of what to remove and where under "Migrating to
> the dedicated environment" in the [Upgrading Guide](../guides/upgrading.md).

See [Upgrading Guide §3](../guides/upgrading.md#3-what-sf-upgrade-does-internally) for a detailed walkthrough of each step and its manual Git-command equivalent.

> **Self-bootstrap:** if the update fetched in step 2 includes changes to `sf` itself
> (`lib/sfcli`), execution automatically hands off to the just-fetched, updated `upgrade`
> logic before step 3's preview runs (printed as `sf itself was updated upstream --
> handing over...`). In other words, bug fixes to `sf upgrade` itself take effect in
> **a single `sf upgrade` run** -- there is no need to pull with the old implementation
> and then run it a second time.

## 5. Exit Codes

| Code | Meaning |
|------|---------|
| `0` | Success (including "already up to date" and a cancelled confirmation) |
| `1` | General error (git missing, fetch failed, dependency resync failed, etc.) |
| `2` | Handled safely but needs your attention (stash-restore conflict, or diverged local commits prevent a fast-forward) |

For exit code `2`, see [Upgrading Guide §4 Resolving Conflicts](../guides/upgrading.md#4-resolving-conflicts).

## 6. Examples

```bash
# Normal update (review the preview, then confirm with Y)
sf upgrade

# Fully unattended update (CI / scripts)
sf upgrade --yes

# Skip dependency resync and the GUI Flasher update offer
sf upgrade --skip-deps --no-flasher

# Give up local changes and take only the official latest version
sf upgrade --discard-local

# Migrate to the dedicated environment without asking
sf upgrade --migrate

# Do not offer the dedicated-environment migration this run
sf upgrade --no-migrate
```
