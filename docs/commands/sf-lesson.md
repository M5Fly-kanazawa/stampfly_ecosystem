# sf lesson

ワークショップレッスンの管理: 一覧、切替、解答表示、ビルド、フラッシュ。

## 構文

```bash
sf lesson <subcommand> [args]
```

### サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `list` | 利用可能なレッスン一覧を表示 |
| `list --course <id>` | 指定コースのステップ一覧を表示 |
| `switch <N or id or course:N>` | レッスンに切替（student.cpp を user_code.cpp にコピー） |
| `solution <N or id or course:N>` | レッスンの解答差分を表示 |
| `info <N or id or course:N>` | レッスンの詳細情報を表示 |
| `build` | ワークショップファームウェアをビルド |
| `flash` | ワークショップファームウェアを書き込み |

## 使用例

```bash
sf lesson list
sf lesson switch 3
sf lesson solution 3
sf lesson build
```

## コース構文（イベント固有の順序）

マニフェスト（`firmware/workshop/lessons/lesson_manifest.yaml`）の
トップレベル `courses:` に、既存レッスンを別順序・別番号で辿る
イベント固有のコースを宣言できる。新しいレッスンディレクトリを
コピーする必要はなく、コース内番号を既存レッスンidに対応付けるだけ。

`switch` / `solution` / `info` は識別子として `<course_id>:<N>`
（例: `sci2026:8`）を受け付ける。内部で対応する既存レッスンへ解決される
ので、`--solution` 等のオプションもそのまま使える。

```bash
sf lesson list --course sci2026     # コースのステップ一覧を表示
sf lesson switch sci2026:8          # コースのステップ8 → 実体のレッスンへ切替
sf lesson switch sci2026:8 --solution
sf lesson info sci2026:3
```

存在しないコースやステップ番号を指定するとエラーになり、利用可能な
コース／ステップの一覧が表示される。

## Course Syntax (Event-Specific Ordering)

The manifest (`firmware/workshop/lessons/lesson_manifest.yaml`) can
declare event-specific courses under the top-level `courses:` field.
A course walks through existing lessons in a different order with its
own tutorial-local numbering — no new lesson directory is copied; a
course step just maps its number onto an existing lesson id.

`switch` / `solution` / `info` accept `<course_id>:<N>` (e.g.
`sci2026:8`) as an identifier. It resolves internally to the underlying
lesson, so flags like `--solution` keep working unchanged.

```bash
sf lesson list --course sci2026     # show the course's steps
sf lesson switch sci2026:8          # step 8 -> resolves to the underlying lesson
sf lesson switch sci2026:8 --solution
sf lesson info sci2026:3
```

An unknown course id or step number fails with an error listing the
available courses/steps.

> 詳細なドキュメントは今後追加予定です。
