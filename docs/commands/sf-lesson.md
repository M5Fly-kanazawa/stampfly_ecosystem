# sf lesson

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

実習（レッスン）を滞りなく進めるための**近道コマンド**。課題コードの切替・編集・ビルド・
書き込み・モニタ・SILS 実行を、レッスン用ファームウェアに対して短い形で呼べる。

**`sf lesson` は正の手順ではなく近道である。** 各サブコマンドは下の「通常コマンドとの対応」に
示す通常コマンドをそのまま呼んでいる。仕組みを理解したい、あるいは実習以外の用途で使う場合は
通常コマンドを使うこと。

## 1. 構文

```bash
sf lesson <subcommand> [args]
```

### サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `list` | 利用可能なレッスン一覧を表示 |
| `list --course <id>` | 指定コースの実習一覧を表示（実習 N: 題名 (セッション)） |
| `switch <N or id or course:N> [--solution]` | 課題コードに切替（`student.cpp` または `solution.cpp` を `user_code.cpp` にコピー） |
| `solution <N or id or course:N>` | 課題の解答差分を表示 |
| `info <N or id or course:N>` | 課題の詳細情報を表示 |
| `edit` | 課題コード（`user_code.cpp`）をエディタで開く |
| `build` | レッスン用ファームウェアをビルド |
| `flash [-p PORT] [-b BAUD] [--no-monitor]` | レッスン用ファームウェアを書き込み（既定で続けてモニタを開く） |
| `monitor` | レッスン用ファームウェアのシリアルモニタを開く |
| `sils [--solution <course:N or N>] [--scenario acro\|step\|<path>] [--noise LEVEL] [--seed N]` | 課題コードをレッスン用ファームウェアに組み込んで SILS で飛ばす |

## 2. 通常コマンドとの対応

レッスン用ファームウェアは `firmware/workshop/`（ターゲット名 `workshop`）にあり、課題コードは
`firmware/workshop/main/user_code.cpp` の 1 ファイルである。`sf lesson` の各サブコマンドは
次の通常コマンドと同じ動作をする。

| 近道（`sf lesson`） | 実際に行われること（通常コマンド） |
|---------------------|------------------------------------|
| `sf lesson switch sci2026:N` | `firmware/workshop/lessons/<lesson>/student.cpp` を `firmware/workshop/main/user_code.cpp` にコピー（`--solution` なら `solution.cpp`） |
| `sf lesson edit` | `firmware/workshop/main/user_code.cpp` をエディタで開く |
| `sf lesson build` | `sf build workshop` |
| `sf lesson flash` | `sf flash workshop -m`（`--no-monitor` を付けると `sf flash workshop`） |
| `sf lesson monitor` | `sf monitor workshop` |
| `sf lesson sils` | `sf sils build --target workshop` → `sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop` |
| `sf lesson sils --scenario step` | 同上。シナリオは `workshop_acro_step.scn`（ロール +0.3 のステップ試験） |
| `sf lesson sils --solution sci2026:8` | 先に `sf lesson switch sci2026:8 --solution` を行ってから上の SILS 手順 |

`sf lesson switch` はファイルの内容だけをコピーし、`user_code.cpp` の更新時刻を新しくする。そのため
切替直後の `sf build workshop` / `sf sils build --target workshop` はそのまま新しいコードを取り込む
（`touch` は不要）。

## 3. 使用例

```bash
sf lesson list
```

```bash
sf lesson switch 3
```

```bash
sf lesson solution 3
```

```bash
sf lesson build
```

## 4. コース構文（イベント固有の順序）

マニフェスト（`firmware/workshop/lessons/lesson_manifest.yaml`）の
トップレベル `courses:` に、既存レッスンを別順序・別番号で辿る
イベント固有のコースを宣言できる。新しいレッスンディレクトリを
コピーする必要はなく、コース内番号を既存レッスン id に対応付けるだけ。

`switch` / `solution` / `info` は識別子として `<course_id>:<N>`
（例: `sci2026:8`）を受け付ける。内部で対応する既存レッスンへ解決される
ので、`--solution` 等のオプションもそのまま使える。`list --course` と
`switch course:N` の表示は「実習 N: 題名」で、元のレッスン番号は出さない
（受講者はコース内の番号だけを知ればよい）。

コースの実習一覧を表示する:

```bash
sf lesson list --course sci2026
```

コースの実習 8 に切り替える（`--solution` で解答に切替）:

```bash
sf lesson switch sci2026:8
```

```bash
sf lesson switch sci2026:8 --solution
```

```bash
sf lesson info sci2026:3
```

存在しないコースやステップ番号を指定するとエラーになり、利用可能な
コース／ステップの一覧が表示される。

## 5. 初回の動作確認（Lesson 1: モータ制御）

環境構築済みの状態から、モータを 1 個回すまでの最短手順:

```bash
sf lesson switch 1
sf lesson build
sf lesson flash -m
```

**必ずプロペラを外した状態で行う**こと。ARM すると `user_code.cpp`
（`sf lesson edit` で編集）で設定した Duty でモータが回転する。環境構築
自体（ESP-IDF・sf CLI 導入）は `docs/setup/` 配下の OS 別ガイドを参照。

---

<a id="english"></a>

# sf lesson

**Shortcut commands** for moving through the exercises (lessons) smoothly: switch, edit, build,
flash, monitor and SILS-run the exercise code against the lesson firmware.

**`sf lesson` is a shortcut, not the canonical procedure.** Every subcommand runs the ordinary
command shown in "Mapping to Ordinary Commands" below. Use the ordinary commands when you want
to understand what happens, or for anything beyond the exercises.

## 1. Syntax

```bash
sf lesson <subcommand> [args]
```

### Subcommands

| Subcommand | Description |
|------------|-------------|
| `list` | List available lessons |
| `list --course <id>` | List the exercises of a course (実習 N: title (session)) |
| `switch <N or id or course:N> [--solution]` | Switch the exercise code (copy `student.cpp` or `solution.cpp` to `user_code.cpp`) |
| `solution <N or id or course:N>` | Show the solution diff |
| `info <N or id or course:N>` | Show lesson details |
| `edit` | Open the exercise code (`user_code.cpp`) in an editor |
| `build` | Build the lesson firmware |
| `flash [-p PORT] [-b BAUD] [--no-monitor]` | Flash the lesson firmware (opens the monitor afterwards by default) |
| `monitor` | Open the serial monitor of the lesson firmware |
| `sils [--solution <course:N or N>] [--scenario acro\|step\|<path>] [--noise LEVEL] [--seed N]` | Build the exercise code into the lesson firmware and fly it in SILS |

## 2. Mapping to Ordinary Commands

The lesson firmware lives in `firmware/workshop/` (target name `workshop`) and the exercise code
is the single file `firmware/workshop/main/user_code.cpp`. Each `sf lesson` subcommand does the
same as the ordinary command below.

| Shortcut (`sf lesson`) | What actually runs |
|------------------------|--------------------|
| `sf lesson switch sci2026:N` | Copy `firmware/workshop/lessons/<lesson>/student.cpp` to `firmware/workshop/main/user_code.cpp` (`solution.cpp` with `--solution`) |
| `sf lesson edit` | Open `firmware/workshop/main/user_code.cpp` in an editor |
| `sf lesson build` | `sf build workshop` |
| `sf lesson flash` | `sf flash workshop -m` (`--no-monitor` gives `sf flash workshop`) |
| `sf lesson monitor` | `sf monitor workshop` |
| `sf lesson sils` | `sf sils build --target workshop` → `sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop` |
| `sf lesson sils --scenario step` | Same, with `workshop_acro_step.scn` (roll +0.3 step test) |
| `sf lesson sils --solution sci2026:8` | `sf lesson switch sci2026:8 --solution` first, then the SILS steps above |

`sf lesson switch` copies file content only and gives `user_code.cpp` a fresh mtime, so a
`sf build workshop` / `sf sils build --target workshop` right after switching picks up the new code
(no `touch` needed).

## 3. Examples

```bash
sf lesson list
```

```bash
sf lesson switch 3
```

```bash
sf lesson solution 3
```

```bash
sf lesson build
```

## 4. Course Syntax (Event-Specific Ordering)

The manifest (`firmware/workshop/lessons/lesson_manifest.yaml`) can
declare event-specific courses under the top-level `courses:` field.
A course walks through existing lessons in a different order with its
own tutorial-local numbering; no new lesson directory is copied, a
course step just maps its number onto an existing lesson id.

`switch` / `solution` / `info` accept `<course_id>:<N>` (e.g.
`sci2026:8`) as an identifier. It resolves internally to the underlying
lesson, so flags like `--solution` keep working unchanged. `list --course`
and `switch course:N` print "実習 N: title" and never the underlying
lesson number (participants only need the course-local number).

Show the course's exercises:

```bash
sf lesson list --course sci2026
```

Switch to exercise 8 of the course (`--solution` for the solution):

```bash
sf lesson switch sci2026:8
```

```bash
sf lesson switch sci2026:8 --solution
```

```bash
sf lesson info sci2026:3
```

An unknown course id or step number fails with an error listing the
available courses/steps.

## 5. First Motor Check (Lesson 1: Motor Control)

From an already set-up environment, the shortest path to spinning one motor:

```bash
sf lesson switch 1
sf lesson build
sf lesson flash -m
```

**Always remove the propellers first.** Arming spins the motors at the duty
cycle you set in `user_code.cpp` (edit it with `sf lesson edit`). For setting
up the environment itself (ESP-IDF, sf CLI), see the OS-specific guide under
`docs/setup/`.
