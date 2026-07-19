# sf upgrade

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

追跡している（tracked）リモートブランチから最新の変更を取得し、ローカルの変更は自動stash（一時退避）で保護しつつ取り込みます。取り込んだ後は、Python依存関係の再同期、陳腐化したESP-IDFの `sdkconfig` の検出、ネイティブGUIフラッシャの更新まで一括で行う、`sf` を最新化するための唯一のコマンドです。

Git初心者向けの丁寧な解説（衝突の解決手順、インストール/アンインストールの内訳表を含む）は **[アップグレードガイド](../guides/upgrading.md)** を参照してください。このページはコマンドリファレンス（構文・オプション・終了コード）です。

## 2. 構文

```bash
sf upgrade [--yes] [--discard-local] [--no-flasher] [--skip-deps]
```

## 3. オプション

| オプション | 説明 |
|-----------|------|
| `--yes`, `-y` | 更新内容のプレビュー確認を省略する（`--discard-local` の確認は省略されない — 破壊的操作のため常に確認）。フラッシャが未導入の場合の一回限りのインストール提案も尋ねない（機会は消費しない） |
| `--discard-local` | ローカルの変更をstashせず破棄してから更新する（破壊的操作。ファイル一覧を表示した上で必ず確認） |
| `--no-flasher` | ネイティブGUIフラッシャの更新提案、および未導入時の一回限りのインストール提案をスキップする（機会は消費しない） |
| `--skip-deps` | Python依存関係の再同期ステップをスキップする |

## 4. やること（ステップ概要）

1. リポジトリ確認（gitクローンか、`origin` リモートが設定済みか）・現在ブランチ表示
2. `git fetch` して遅れているコミット数を確認（0件なら依存同期のみ実行して終了）
3. 取り込まれるコミットの一覧をプレビュー表示し、`Y/n` で確認
4. ローカル変更を安全に取り込む（既定=stash→マージ→復元、`--discard-local`=確認の上で破棄）
5. Python依存関係を再同期（`pip install -e .`）
6. `sdkconfig.defaults` / `partitions.csv` が変わっていれば、既存 `sdkconfig` を `*.pre-upgrade-backup` へ退避
7. ネイティブGUIフラッシャ: 導入済みなら更新を提案。**未導入なら、チェックアウトにつき一回だけ**インストールを提案（`--yes` または `--no-flasher` 指定時はこの一回限りの機会を消費せずスキップ。一度尋ねたら `.sf/flasher_install_offered` に記録し、以後は二度と尋ねない）
8. サマリ表示（更新前後のコミットハッシュ・実施した処置・推奨次アクション）

詳細な各ステップの解説と、Gitコマンドとの対応表は [アップグレードガイド §3](../guides/upgrading.md) を参照してください。

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
```

---

<a id="english"></a>

## 1. Overview

Fetches the latest changes from the tracked remote branch and folds them in while protecting local edits with an automatic stash. Afterward it resyncs Python dependencies, detects stale ESP-IDF `sdkconfig` files, and offers to update the native GUI Flasher app -- the one command meant to bring `sf` fully up to date.

For a beginner-friendly walkthrough (conflict resolution, the install/uninstall breakdown table), see the **[Upgrading Guide](../guides/upgrading.md)**. This page is the command reference (syntax, options, exit codes).

## 2. Syntax

```bash
sf upgrade [--yes] [--discard-local] [--no-flasher] [--skip-deps]
```

## 3. Options

| Option | Description |
|--------|-------------|
| `--yes`, `-y` | Skip the update-preview confirmation (the `--discard-local` confirmation is never skipped -- it is destructive). Also skips the one-time "install the Flasher?" offer when it is not installed, without consuming that one-time chance |
| `--discard-local` | Discard local changes instead of stashing them before updating (destructive; the changed-file list is shown and always confirmed) |
| `--no-flasher` | Skip the offer to update the native GUI Flasher app, and the one-time install offer if it is not installed (without consuming that one-time chance) |
| `--skip-deps` | Skip the Python dependency resync step |

## 4. What It Does (Step Overview)

1. Repository check (git clone with an `origin` remote?) and current-branch display
2. `git fetch`, then check how many commits behind (0 -> only dependency resync runs, then exits)
3. Show a preview of incoming commits and ask `Y/n`
4. Safely fold in local changes (default = stash -> merge -> restore, `--discard-local` = discard after confirmation)
5. Resync Python dependencies (`pip install -e .`)
6. If `sdkconfig.defaults` / `partitions.csv` changed, back up any existing `sdkconfig` to `*.pre-upgrade-backup`
7. Native GUI Flasher: offer to update it if installed. If **not** installed, offer to install it **once per checkout** (`--yes`/`--no-flasher` skip this without consuming the one-time chance; once asked, the answer is recorded in `.sf/flasher_install_offered` and never asked again)
8. Print a summary (before/after commit hash, actions taken, recommended next step)

See [Upgrading Guide §3](../guides/upgrading.md#3-what-sf-upgrade-does-internally) for a detailed walkthrough of each step and its manual Git-command equivalent.

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
```
