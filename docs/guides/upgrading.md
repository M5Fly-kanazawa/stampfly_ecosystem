# アップグレードガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 3行でわかる結論

- エコシステムを最新版にするには **`sf upgrade` を実行するだけ**です。
- 自分が作業中に編集していたファイルは、自動的に安全な場所（stash：後述）に退避してから元に戻されます。消えません。
- 途中で詰まったら → [4. 衝突（コンフリクト）の解決](#conflicts)。

```bash
source setup_env.sh   # sf コマンドを使えるようにする
sf upgrade
```

> **初回だけの注意（ブートストラップ）:** `sf upgrade` コマンド自体が 2026-07-19 の更新
> （v2026.07.2 系）で追加されたものです。それ**より古いチェックアウト**には
> まだ `sf upgrade` が存在しないため、最初の1回だけは素の Git で更新してください:
> `git pull` （リポジトリのフォルダで実行）。それ以降はこのガイドのとおり
> `sf upgrade` が使えます。
> **One-time bootstrap note:** `sf upgrade` itself was added in the 2026-07-19
> update (the v2026.07.2 line). A checkout older than that does not have the
> command yet — run a plain `git pull` once (inside the repository folder);
> from then on, use `sf upgrade` as described in this guide.

## 2. 前提知識（Git初心者向け）

このガイドは **ソフトウェア開発が専門ではない教員・学生** を対象にしています。専門用語は都度かみ砕いて説明します。

| 用語 | 意味 |
|------|------|
| clone（クローン） | GitHub上の公式リポジトリを、自分のPCへ丸ごとコピーすること。`./install.sh` の前に行った `git clone` がこれです |
| pull（プル） | 公式リポジトリで加わった最新の変更を、自分のコピーへ取り込むこと |
| ローカル変更 | 自分のPC上でファイルを編集した状態（まだ公式には送っていない、自分だけの変更） |
| ブランチ | 開発の枝分かれ。通常ユーザーは `main` という「本流」のブランチにいます |
| コミット | 変更のまとまりに付ける記録（スナップショット） |

**安心情報:** ワークショップのレッスンで編集する `user_code.cpp` などの演習用ファイルは、`.gitignore`（Gitの追跡対象から除外する設定）で保護されているものが多く、`sf upgrade` の対象（＝上書きされうる「追跡ファイル」）にはなりません。もし追跡対象のファイルを自分で編集していても、後述のとおり自動的に退避されるため、いきなり消えることはありません。

## 3. `sf upgrade` が内部でやること

`sf upgrade` は、以下の8ステップを順番に実行します。各ステップに対応する「手動でやる場合」の Git コマンドも併記します（Gitの学習にもなります）。

| # | ステップ | 内容 | 手動でやる場合の等価コマンド |
|---|---------|------|------------------------------|
| 1 | リポジトリ確認 | Gitのクローンか、`origin`（公式リモート）が設定されているかを確認。現在のブランチを表示（`main` 以外なら警告のみ、続行） | `git remote get-url origin` |
| 2 | 取得＋差分確認 | 最新情報を取得し、何コミット遅れているか判定。0件（最新）なら以降は依存関係の同期のみ実行して終了 | `git fetch origin main`<br>`git rev-list --count HEAD..origin/main` |
| 3 | 更新内容のプレビュー | 取り込まれるコミットの一覧（最大15件）を表示し、`Y/n` で確認（`--yes` で省略可） | `git log --oneline HEAD..origin/main` |
| 4 | ローカル変更の取り込み | 下記4.1参照 | 下記参照 |
| 5 | 依存関係の再同期 | Python依存パッケージを常に再インストール（`--skip-deps` で省略可）。差分がなければ数秒で終わる | `pip install -e .` |
| 6 | sdkconfig陳腐化検出 | ファームウェアの既定設定（`sdkconfig.defaults` や `partitions.csv`）が変わっていたら、既存の `sdkconfig` を退避（詳細は下記） | （手動での再現は複雑なため `sf upgrade` 推奨） |
| 7 | GUIフラッシャの更新／インストール提案 | ネイティブGUIフラッシャ（デスクトップアプリ）が導入済みなら更新するか確認（`--no-flasher` でスキップ、`--yes` で自動承諾）。**未導入なら、チェックアウトにつき一回だけ**インストールするか確認（既定は入れない＝`n`）。`--yes` / `--no-flasher` 指定時はこの一回限りの機会を消費せずスキップし、一度尋ねたら結果を `.sf/flasher_install_offered` に記録して二度と尋ねない | `sf flasher install --yes` |
| 8 | サマリ表示 | 更新前後のコミットハッシュ、実施した処置、次にやるべきこと（例: `sf build vehicle`）を表示 | — |

### 3.1 ステップ4: ローカル変更の取り込み

`git status --porcelain` で確認した「追跡ファイルの変更」（`??` で始まる **未追跡ファイルは対象外** = 無視されます）を、次のいずれかで処理します。

- **変更なし（クリーン）** → そのまま早送りマージ（fast-forward merge）: `git merge --ff-only origin/main`
- **変更あり（既定の動作）** → 一時退避してからマージし、戻す:
  1. `git stash push -m "sf-upgrade-autostash <日時>"` （変更を安全な一時置き場＝stashへ退避）
  2. `git merge --ff-only origin/main`
  3. `git stash pop`（退避した変更を戻す）
  - 3.で衝突（コンフリクト）が起きた場合は [4. 衝突の解決](#conflicts) へ
- **変更あり（`--discard-local` 指定時）** → 変更ファイル一覧を表示した上で明示確認（**`--yes` を付けていても、この確認だけは省略されません** — 元に戻せない破壊的操作のため）。承諾すると `git checkout -- <files>` で変更を破棄してからマージ

**早送り（fast-forward）できない場合:** 自分のブランチにだけ存在するコミットがあり、公式の履歴と枝分かれしてしまっている状態です。`sf upgrade` は **rebase や reset を勝手に行いません**（作業を失わないため）。この場合はエラーメッセージと共に終了するので、Gitに詳しい人に相談するか、このガイドの内容を参考に手動で解決してください。

<a id="conflicts"></a>

## 4. 衝突（コンフリクト）の解決

`git stash pop`（自分の変更を戻す操作）の際に、**同じ場所を公式の更新と自分の変更の両方が触っていた**場合、Gitは「どちらを採用すればいいか自動で判断できない」として衝突を報告します。`sf upgrade` はこの状態でも **何も失われません**（変更はstashに残ったままです）。

### 4.1 何が起きているか

`sf upgrade` は以下を画面に表示して終了します（終了コード2 = 「安全には処理したが対応が必要」）。

```
[WARN] Your local changes are safely saved in the stash -- nothing is lost.
[WARN]   Stash entry: "sf-upgrade-autostash 2026-07-19T10:15:00"

Conflicting file(s):
  firmware/vehicle/main/some_file.cpp

Conflict markers look like: <<<<<<< / ======= / >>>>>>>
Edit each file to keep the lines you want and remove the markers.

Once every file is resolved:
  git add <file>
  git stash drop
```

### 4.2 マーカーの読み方

衝突したファイルを開くと、次のような印（マーカー）が挿入されています。

```
<<<<<<< Updated upstream
（公式の最新版の内容）
=======
（自分が編集していた内容）
>>>>>>> Stashed changes
```

- `<<<<<<<` から `=======` までが **公式の最新版**
- `=======` から `>>>>>>>` までが **自分の変更**

### 4.3 解決方法（2つの選択肢）

**VSCodeを使う場合（推奨・視覚的にわかりやすい）:**

1. 衝突したファイルを開くと、エディタ上部に「Accept Current Change」「Accept Incoming Change」「Accept Both Changes」というボタンが表示されます
2. **自分の変更を残したい** → 「Accept Current Change」
3. **公式の内容を採用したい** → 「Accept Incoming Change」
4. **両方とも残したい** → 「Accept Both Changes」（その後、内容が正しいか目視確認）
5. 全ての衝突を解決したら保存

**手動で編集する場合:**

1. `<<<<<<<` `=======` `>>>>>>>` の行そのものと、不要な方の内容をテキストエディタで削除
2. 残したい内容だけが残るように編集
3. 保存

### 4.4 解決後の仕上げ

```bash
git add <解決したファイル>
git stash drop   # 退避していた変更はもう不要なので削除
```

`git status` で衝突マーカーが残っていないか最終確認してから進めてください。

### 4.5 最終手段: 自分の変更を諦める

「自分の変更よりも公式の最新版を優先したい」「衝突の解決が難しい」という場合は、次回は `--discard-local` を付けて実行すると、確認の上でローカル変更を破棄してから更新できます。

```bash
sf upgrade --discard-local
```

なお、今まさに衝突で止まっている状態から抜け出したいだけなら、以下でも同じ結果になります（stashに退避した変更を破棄）。

```bash
git checkout -- .
git stash drop
```

## 5. インストール／アンインストールのライフサイクル

### 場面別 早見表 — どういう時にどうすればいいか

対象は2つあります: **①エコシステム本体**（リポジトリ + ESP-IDF + `sf`）と
**②書き込みアプリ**（StampFly Flasher）。迷ったら次の1行判定:
「最新にしたい → `sf upgrade`」「アプリを入れたい/消したい → `sf flasher install / uninstall`」
「環境ごと直したい → `install.bat --force`（Mac/Linux は `./install.sh --force`）」。

| 場面 | やること |
|------|---------|
| 新しい PC に何も無い（初インストール） | `git clone` → `install.bat` / `./install.sh`（Step 1〜4 一括。書き込みアプリは Y/n） |
| 機体に書き込みたいだけ（開発環境は不要） | リリースページから書き込みアプリをダウンロード（または Web フラッシャ）。本体のインストール不要 |
| 開発環境はあるが書き込みアプリが未導入 | `sf flasher install`（または `sf upgrade` 実行時にチェックアウトにつき一回だけ尋ねられる） |
| 通常の更新 | `sf upgrade`（書き込みアプリが未導入なら初回のみ入れるか尋ねられる。既定は入れない＝`n`。`--yes`/`--no-flasher` は尋ねずスキップ） |
| 古いチェックアウト（2026-07-19 の更新前で `sf upgrade` が無い） | 初回のみ `git pull` → 以後 `sf upgrade` |
| 自分の編集を残したまま更新 | そのまま `sf upgrade`（自動退避・復元。衝突時は §4 へ） |
| ローカル変更を捨てて公式最新へ（貸出 PC 整備等） | `sf upgrade --discard-local`（対象一覧を確認してから破棄） |
| `sf` が「command(s) unavailable」警告 / `ModuleNotFoundError` | `sf upgrade`（依存を入れ直す。upgrade 自体は依存欠落でも動く設計） |
| 環境が怪しいので確実に入れ直す | `install.bat --force` / `./install.sh --force` |
| 書き込みアプリだけ最新化 | `sf flasher update` |
| 書き込みアプリだけ削除 | `sf flasher uninstall`（Windows は「設定 > アプリ」からも可） |
| `sf` を環境から外す | `install.bat --uninstall`（書き込みアプリも一緒に削除。残るものは下表参照） |
| 完全撤去 | `--uninstall` 後、下表の「手動で消す場合」を実行 |
| 壊れた環境のリセット | `install.bat --clean`（設定+`sf` を消して入れ直し） |

### 何が置かれ、何が消えるか

`sf` エコシステムのインストーラ（`./install.sh` / `install.bat`）が何を・どこに置くか、`--uninstall` / `--clean` が何を削除し何を削除しないかの一覧です。

| 対象 | 場所 | `--uninstall` / `--clean` で削除されるか | 手動で消す場合 |
|------|------|------------------------------------------|----------------|
| sfcli パッケージ本体 | ESP-IDF用venv内 | される | `pip uninstall stampfly-ecosystem` |
| GUIフラッシャ（デスクトップアプリ） | OSごとのアプリ格納先 | される（sfcli経由で先に実行） | `sf flasher uninstall` |
| 設定ファイル | インストーラの設定ファイル | される | — |
| ESP-IDF 本体 | `~/esp/esp-idf`（または `--idf-path` 指定先） | **されない** | `rm -rf ~/esp/esp-idf` |
| IDF_TOOLS_PATH（ツールチェーン等） | `~/.espressif`（Windowsは `C:\Espressif`） | **されない** | `rm -rf ~/.espressif` |
| venvの他の依存パッケージ | `<IDF_TOOLS_PATH>/python_env/idf<バージョン>_py*_env` | **されない** | 該当venvディレクトリを `rm -rf` |
| udevルール（Linuxのみ） | `/etc/udev/rules.d/99-stampfly.rules` | **されない** | `sudo rm /etc/udev/rules.d/99-stampfly.rules` |
| リポジトリ本体（clone したフォルダ） | 任意（cloneした場所） | **されない** | `rm -rf <リポジトリのパス>` |

**あえて削除しない理由:** ESP-IDF本体・ツールチェーン・venv・udevルールは、他のプロジェクトや別のツールと共有されている可能性があります。`sf` が持ち主だと確信できないものを勝手に消さない設計です。消したい場合は表の「手動で消す場合」のコマンドを使ってください。

この表は `./install.sh --uninstall` / `--clean` 実行時にもコンソールへ同じ内容が表示されます。

## 6. よくある質問（FAQ）

### `ModuleNotFoundError` が出て `sf` が動かない

Python の依存パッケージが不足しています。次のいずれかで直せます。

```bash
sf upgrade
# または
pip install -e <リポジトリのルートパス>
```

`sf` 自体が動かないほど壊れている場合でも、`sf upgrade` は標準ライブラリのみで動くよう作られているため、依存関係が全滅していても実行できます（動かない場合は `pip install -e .` を直接使ってください）。

### `sf --help` に「N個のコマンドが利用不可」という警告が出る

一部のコマンドが必要とする依存パッケージだけが欠けている状態です。`sf` 自体や他のコマンドは通常どおり使えます。警告メッセージにある通り、`sf upgrade` または `pip install -e <リポジトリのルート>` で直ります。

### ビルドの様子がおかしい（古いキャッシュが悪さしていそう）

ファームウェアのビルドディレクトリを消してから再ビルドしてください。

```bash
rm -rf firmware/<対象>/build
sf build <対象>
```

### 「sdkconfigを退避しました」と表示された。何が起きた？

ファームウェアの既定設定ファイル（`sdkconfig.defaults` や `partitions.csv`）が更新された場合、ESP-IDFは既存の `sdkconfig`（あなたのビルド設定の実体）へ自動で反映してくれません。そこで `sf upgrade` が、古い `sdkconfig` を `sdkconfig.pre-upgrade-backup` という名前に変えて退避します。次回ビルド時に新しい既定値から `sdkconfig` が再生成されます。もし独自にカスタマイズしていた設定があれば、退避された `*.pre-upgrade-backup` ファイルを見比べて手動で反映してください。

### 「Your local changes would be overwritten」と表示された

これは Git が「あなたが編集したファイルと、これから取り込む公式の変更が衝突しそうです」と警告しているものです。`sf upgrade` は通常この状況を自動でstash（一時退避）してから処理するため目にすることは少ないはずですが、`git pull` を手動で行った場合などに表示されることがあります。対処は [4. 衝突の解決](#conflicts) と同じ考え方です。まず `git stash` で自分の変更を退避し、`git pull` してから `git stash pop` してください。

---

<a id="english"></a>

## 1. Three-Line Summary

- To update the ecosystem to the latest version, **just run `sf upgrade`**.
- Any files you were editing are automatically stashed (safely tucked away — see below) and restored afterward. Nothing is lost.
- If you get stuck, see [4. Resolving Conflicts](#4-resolving-conflicts).

```bash
source setup_env.sh   # make the sf command available
sf upgrade
```

## 2. Background (for Git Beginners)

This guide targets **teachers and students who are not software-development specialists**. Jargon is explained as it comes up.

| Term | Meaning |
|------|---------|
| clone | Copying the official GitHub repository to your own computer. The `git clone` you ran before `./install.sh` did this |
| pull | Bringing the latest changes from the official repository into your own copy |
| local changes | Files you have edited on your own machine that have not yet been sent to the official repository |
| branch | A line of development. Normal users stay on `main`, the "trunk" branch |
| commit | A named snapshot of a set of changes |

**Reassurance:** exercise files you edit in workshop lessons (e.g. `user_code.cpp`) are typically protected by `.gitignore` (Git's "do not track" list) and are therefore not "tracked files" that `sf upgrade` could touch. Even if you have edited a tracked file yourself, it is automatically stashed rather than silently discarded, as described below.

## 3. What `sf upgrade` Does Internally

`sf upgrade` runs the following 8 steps in order. The manual Git-command equivalent is listed alongside each one (useful if you want to learn Git along the way).

| # | Step | What happens | Manual equivalent |
|---|------|---------------|--------------------|
| 1 | Repository check | Confirms this is a git clone with an `origin` remote configured, and shows the current branch (a warning-only, non-blocking notice if it isn't `main`) | `git remote get-url origin` |
| 2 | Fetch + diff check | Fetches the latest state and counts how many commits behind you are. If 0 (already up to date), only the dependency resync step still runs | `git fetch origin main`<br>`git rev-list --count HEAD..origin/main` |
| 3 | Update preview | Shows up to 15 incoming commits and asks `Y/n` to proceed (skipped with `--yes`) | `git log --oneline HEAD..origin/main` |
| 4 | Local change handling | See 3.1 below | See below |
| 5 | Dependency resync | Always reinstalls Python dependencies (skip with `--skip-deps`); a no-op pull still finishes in a couple of seconds | `pip install -e .` |
| 6 | sdkconfig staleness check | If firmware defaults (`sdkconfig.defaults` / `partitions.csv`) changed, backs up any existing `sdkconfig` (details below) | (complex to reproduce manually; use `sf upgrade`) |
| 7 | Native GUI Flasher update / install offer | If the desktop Flasher app is installed, offers to update it (skip with `--no-flasher`, auto-accept with `--yes`). If **not** installed, offers to install it **once per checkout** (default No). `--yes`/`--no-flasher` skip this without consuming the one-time chance; once asked, the answer is recorded in `.sf/flasher_install_offered` and never asked again | `sf flasher install --yes` |
| 8 | Summary | Shows the before/after commit hash, actions taken, and the recommended next step (e.g. `sf build vehicle`) | — |

### 3.1 Step 4: Handling Local Changes

Changes to **tracked files** as reported by `git status --porcelain` (untracked files, marked `??`, are **ignored — out of scope**) are handled as follows.

- **No changes (clean)** → fast-forward merge directly: `git merge --ff-only origin/main`
- **Changes present (default behavior)** → stash, merge, then restore:
  1. `git stash push -m "sf-upgrade-autostash <timestamp>"` (tucks your changes into a safe holding area, the "stash")
  2. `git merge --ff-only origin/main`
  3. `git stash pop` (restores your stashed changes)
  - If step 3 conflicts, see [4. Resolving Conflicts](#4-resolving-conflicts)
- **Changes present, with `--discard-local`** → lists the changed files and asks for explicit confirmation (**this confirmation is never skipped, even with `--yes`** — it is a destructive, irreversible operation). If confirmed, discards with `git checkout -- <files>` before merging

**When a fast-forward is impossible:** your branch has commits of its own that have diverged from the official history. `sf upgrade` **never rebases or resets automatically** (to avoid losing work). It exits with an error message instead; ask someone familiar with Git for help, or use this guide as a reference to resolve it manually.

## 4. Resolving Conflicts

If the same spot in a file was touched by both the official update and your own edit during `git stash pop` (restoring your changes), Git cannot automatically decide which to keep and reports a conflict. Even in this state, `sf upgrade` **loses nothing** — your changes remain safely in the stash.

### 4.1 What You'll See

`sf upgrade` prints the following and exits (exit code 2 = "handled safely, but needs your attention"):

```
[WARN] Your local changes are safely saved in the stash -- nothing is lost.
[WARN]   Stash entry: "sf-upgrade-autostash 2026-07-19T10:15:00"

Conflicting file(s):
  firmware/vehicle/main/some_file.cpp

Conflict markers look like: <<<<<<< / ======= / >>>>>>>
Edit each file to keep the lines you want and remove the markers.

Once every file is resolved:
  git add <file>
  git stash drop
```

### 4.2 Reading the Markers

An affected file will contain markers like this:

```
<<<<<<< Updated upstream
(the official, latest content)
=======
(your own edited content)
>>>>>>> Stashed changes
```

- Between `<<<<<<<` and `=======` is the **official latest version**
- Between `=======` and `>>>>>>>` is **your own edit**

### 4.3 Two Ways to Resolve It

**Using VSCode (recommended — visual and easier):**

1. Opening a conflicted file shows "Accept Current Change" / "Accept Incoming Change" / "Accept Both Changes" buttons above the conflict
2. **Keep your own edit** → "Accept Current Change"
3. **Take the official version** → "Accept Incoming Change"
4. **Keep both** → "Accept Both Changes" (then visually double-check the result)
5. Save once every conflict is resolved

**Editing by hand:**

1. Delete the `<<<<<<<` / `=======` / `>>>>>>>` marker lines and the side you don't want
2. Edit so only the content you want to keep remains
3. Save

### 4.4 Finishing Up

```bash
git add <resolved-file>
git stash drop   # the stashed copy is no longer needed
```

Run `git status` once more to confirm no conflict markers remain before moving on.

### 4.5 Last Resort: Give Up Your Local Changes

If you'd rather take the official latest version over your own edit, or the conflict is too hard to resolve, run `sf upgrade` again next time with `--discard-local`, which discards local changes after an explicit confirmation.

```bash
sf upgrade --discard-local
```

If you just want to get out of the conflict you're currently stuck in, this has the same effect (discards the stashed changes):

```bash
git checkout -- .
git stash drop
```

## 5. Install / Uninstall Lifecycle

### Quick reference — which situation calls for which command

Two things are managed: the **ecosystem itself** (repository + ESP-IDF + `sf`) and the
**flashing app** (StampFly Flasher). One-line rule of thumb: "want the latest → `sf upgrade`",
"want the app in/out → `sf flasher install / uninstall`", "want the environment repaired →
`install.bat --force` (`./install.sh --force` on Mac/Linux)".

| Situation | What to do |
|-----------|------------|
| Brand-new PC (first install) | `git clone` → `install.bat` / `./install.sh` (Steps 1–4; flashing app offered as Y/n) |
| Just want to flash a craft (no dev environment) | Download the flashing app from the Releases page (or use the Web flasher); no ecosystem install needed |
| Dev environment present, flashing app missing | `sf flasher install` (or wait — `sf upgrade` also asks once per checkout) |
| Regular update | `sf upgrade` (asks once, the first time, whether to install the flashing app if it's missing; default No. `--yes`/`--no-flasher` skip the ask) |
| Checkout older than the 2026-07-19 update (no `sf upgrade` yet) | One-time plain `git pull`, then `sf upgrade` from there on |
| Update while keeping your local edits | Just run `sf upgrade` (auto-stash and restore; conflicts → §4) |
| Discard local changes and take the official latest (loaner-PC maintenance) | `sf upgrade --discard-local` (lists the files, then confirms) |
| `sf` warns "command(s) unavailable" / `ModuleNotFoundError` | `sf upgrade` (reinstalls dependencies; upgrade itself runs even with them missing) |
| Environment feels broken — reinstall for sure | `install.bat --force` / `./install.sh --force` |
| Update only the flashing app | `sf flasher update` |
| Remove only the flashing app | `sf flasher uninstall` (Windows: also possible from Settings > Apps) |
| Remove `sf` from the environment | `install.bat --uninstall` (also removes the flashing app; leftovers listed below) |
| Complete removal | After `--uninstall`, run the "Manual removal" commands in the table below |
| Reset a broken setup | `install.bat --clean` (removes config + `sf`, then reinstalls) |

### What is placed where, and what gets removed

What the ecosystem installer (`./install.sh` / `install.bat`) places where, and what `--uninstall` / `--clean` do and do not remove.

| Item | Location | Removed by `--uninstall` / `--clean`? | Manual removal |
|------|----------|----------------------------------------|-----------------|
| sfcli package itself | Inside the ESP-IDF venv | Yes | `pip uninstall stampfly-ecosystem` |
| GUI Flasher (desktop app) | OS-specific app location | Yes (run first, via sfcli) | `sf flasher uninstall` |
| Config file | Installer's config file | Yes | — |
| ESP-IDF checkout | `~/esp/esp-idf` (or your `--idf-path`) | **No** | `rm -rf ~/esp/esp-idf` |
| IDF_TOOLS_PATH (toolchain, etc.) | `~/.espressif` (`C:\Espressif` on Windows) | **No** | `rm -rf ~/.espressif` |
| Other venv dependencies | `<IDF_TOOLS_PATH>/python_env/idf<ver>_py*_env` | **No** | `rm -rf` that venv directory |
| udev rules (Linux only) | `/etc/udev/rules.d/99-stampfly.rules` | **No** | `sudo rm /etc/udev/rules.d/99-stampfly.rules` |
| Repository checkout (your cloned folder) | Wherever you cloned it | **No** | `rm -rf <path-to-repo>` |

**Why these are left alone:** ESP-IDF itself, its toolchain, the venv, and the udev rules may be shared with other projects or tools. The design deliberately never deletes things `sf` cannot be certain it solely owns. Use the "Manual removal" column if you want them gone.

The same table is printed to the console when you run `./install.sh --uninstall` / `--clean`.

## 6. FAQ

### I get `ModuleNotFoundError` and `sf` won't run

A Python dependency is missing. Fix it with either:

```bash
sf upgrade
# or
pip install -e <path-to-repo-root>
```

`sf upgrade` is built to run on the standard library alone, so it works even when the environment is broken badly enough that `sf` itself barely runs (if even that fails, use `pip install -e .` directly).

### `sf --help` shows a warning about "N command(s) unavailable"

Only the dependency needed by those specific commands is missing; `sf` itself and every other command keep working normally. As the message says, fix it with `sf upgrade` or `pip install -e <repo-root>`.

### The build looks wrong / a stale cache seems to be interfering

Remove the firmware build directory and rebuild.

```bash
rm -rf firmware/<target>/build
sf build <target>
```

### It says "backed up sdkconfig" — what happened?

When firmware default-configuration files (`sdkconfig.defaults` / `partitions.csv`) change upstream, ESP-IDF does not automatically re-apply them onto your existing `sdkconfig` (your actual build configuration). So `sf upgrade` renames the old `sdkconfig` to `sdkconfig.pre-upgrade-backup` and lets it regenerate from the new defaults on the next build. If you had made your own customizations, compare against the `*.pre-upgrade-backup` file and reapply them by hand.

### I see "Your local changes would be overwritten"

This is Git warning that a file you edited and the incoming official changes touch the same content. `sf upgrade` normally stashes your changes automatically before this can happen, so you're unlikely to see it from `sf upgrade` itself — but you might if you ran `git pull` manually. The fix follows the same idea as [4. Resolving Conflicts](#4-resolving-conflicts): stash your changes with `git stash`, `git pull`, then `git stash pop`.
