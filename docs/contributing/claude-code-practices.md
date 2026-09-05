# Claude Code 活用ノウハウ

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

本ドキュメントは、StampFly Ecosystem 開発で蓄積した Claude Code の実践的な使い方をまとめたものである。2026年4月の初版執筆時点で50件超、2026年9月時点で1,800件を超えるコミットを通じて得られた知見に基づく。

## 1. セッション管理

### CLAUDE.md による行動制御

CLAUDE.md はセッションごとに自動で読み込まれ、Claude Code の振る舞いを規定する最重要ファイルである。

| 設定項目 | 効果 | 本プロジェクトでの実例 |
|---------|------|---------------------|
| セッション開始時の読み込み指定 | コンテキスト復帰の自動化 | `PROJECT_PLAN.md` + 前回コミットログの読み込み |
| 応答言語の指定 | 一貫した言語で対話 | 「応答は日本語で行うこと」 |
| コミットルールの明記 | 変更の取りこぼし防止 | 「コードを変更したら必ずコミット」 |
| ツール優先順位 | 一貫したワークフロー | 「sf CLI を積極的に使用すること」 |

**教訓:** CLAUDE.md に曖昧な指示を書くと無視される。「〜すること」と断定的に書く。

### Next steps によるセッション間の継続性

全コミットに `Next steps:` セクションを含めることで、セッション再開時にどこから作業を始めるか明確になる。

```
Next steps:
- sf flash vehicle -m で書き込み
- テレメトリログで BA が飛行中に変化しないことを確認
- バッテリー限界まで飛行してトリム安定性を検証
```

**良い Next steps の特徴:**
- 具体的なコマンドを含む（`sf flash vehicle -m`）
- 検証すべき指標が明確（「BA が飛行中に変化しない」）
- 優先順位順に記載

**悪い例:** 「テスト」「動作確認」「その他の改善」。これでは次のセッションで何をすべきか分からない。

## 2. コミット戦略

### Conventional Commits の一貫した使用

本プロジェクトのコミットは、マージコミットなど一部の例外を除き `type(scope): subject` 形式に統一されている（2026年9月5日時点で総数1,800件超）。

| タイプ | 使用回数（2026-09-05時点） | 典型的な使い方 |
|--------|---------|--------------|
| `feat` | 589 | 新機能・新ツール追加 |
| `fix` | 646 | バグ修正・パラメータ修正 |
| `docs` | 338 | ドキュメント更新 |
| `refactor` | 98 | ESKF V1→V2 統合など |
| `chore` | 54 | 設定変更、デッドコード削除 |
| `test` | 26 | PC ユニットテスト追加 |
| `revert` | 11 | PID ゲインの巻き戻し |

件数は `git log --pretty=%s | grep -c '^<type>('` の形で type ごとに再集計できる。コミット数は日々増え続けるため、上表は目安として参照すること。

**教訓:** `/commit` スキルを使えばガイドラインに沿ったメッセージが自動生成される。手動でコミットメッセージを書かない。

### 小さく頻繁にコミットする

ESKF（Error State Kalman Filter、姿勢・位置推定に使う拡張カルマンフィルタの一種）のリファクタリングの例が良い手本:

```
1. feat(eskf): add ESKF_V2 with active_mask based P-matrix isolation
2. feat(fusion): connect SensorFusion to ESKF_V2
3. test(eskf): add ESKF_V2 PC unit tests (69 pass, 0 fail)
4. refactor(eskf): unify config control, remove dead code and fallbacks
5. refactor(eskf): remove ESKF V1 completely
6. refactor(eskf): rename ESKF_V2 to ESKF — single implementation
7. chore(cleanup): remove V1/V2 references, dead code, unused methods
```

1つの大きなリファクタリングを7段階に分割。各段階でビルドが通り、テストが通る状態を維持。ロールバック可能なポイントが7箇所ある。

## 3. デバッグのワークフロー

### ログ駆動デバッグ

本プロジェクトで最も効果的だったデバッグパターン:

```
問題発見 → テレメトリログ取得 → データ可視化 → 根本原因特定 → 修正 → 再フライト検証
```

具体例（ESKF 姿勢リセット問題）:

1. **症状:** 飛行中に ~13秒周期で姿勢がリセットされる
2. **ログ取得:** `sf log wifi` で 400Hz テレメトリを取得
3. **可視化:** `sf log viz` でクォータニオン・バイアスの時系列を確認
4. **原因特定:** active_mask で状態が凍結されているのに predict() が位置を更新 → 発散 → リセット発動
5. **修正:** 3段階のコミットで修正（着陸検出統合 → predict のスキップ → リセット削除）
6. **検証:** 再フライトでリセットが消えたことをログで確認

**教訓:** Claude Code にログデータの解析を依頼すると、時系列の異常パターンを素早く発見できる。ただし「ログを見て」だけでは不十分。「13秒周期の姿勢リセットの原因を特定して」のように具体的に指示する。

### 段階的な修正

1つのバグに対して一度に全てを修正しようとせず、段階を分ける:

```
fix(landing): unify landing detection, prevent in-flight false triggers
fix(eskf): skip position prediction when states frozen by active_mask
fix(imu): remove in-flight ESKF reset, keep NaN detection as log only
```

各修正が独立してテスト可能で、問題が起きた場合にどの修正が原因かを切り分けられる。

## 4. コンテキスト管理

### 大きなファイルの扱い

Claude Code のコンテキストウィンドウは有限。以下のプラクティスで効率的に使う:

| 状況 | やるべきこと | やってはいけないこと |
|------|------------|-------------------|
| 画像確認（スライド PDF） | サブエージェントで完結 | メインコンテキストで Read |
| 大規模ファイルの調査 | 必要な行範囲だけ Read | ファイル全体を読む |
| 複数ファイルの並行調査 | Explore サブエージェント | 1ファイルずつ順番に Read |

**Slide Rules の教訓:** スライド PDF をメインコンテキストで画像として読み込むと、コンテキストを大量消費して rate limit に抵触する。サブエージェントに画像確認を委譲し、テキストの指摘事項だけを返させる方式に切り替えた。

### PROJECT_PLAN.md の活用

アーキテクチャの判断を行う前に `PROJECT_PLAN.md` を参照する。このファイルが設計判断のSSOT（Single Source of Truth）として機能する。

ドキュメントとコードの乖離を発見した場合は、ドキュメント側を更新する:

```
docs: update PROJECT_PLAN.md and init.hpp to reflect current architecture

PROJECT_PLAN.md had a conceptual component structure (sensors/,
estimation/, control/) that diverged significantly from the actual
implementation.
```

## 5. ツール統合

### sf CLI への統合方針

散在するスクリプトを sf CLI サブコマンドとして統合することで、Claude Code からの呼び出しが統一される。

```
# 非推奨: 個別スクリプトを直接実行
python tools/log_analyzer/plot_timeseries.py data.csv

# 推奨: sf CLI 経由
sf log viz data.csv
```

**効果:**
- Claude Code に「sf コマンドを使え」とだけ指示すれば、全ツールにアクセスできる
- コマンド名が統一されているため、CLAUDE.md に網羅的なコマンド表を記載できる
- 新ツール追加時も同じパターンに従うため、学習コストが低い

### サブエージェントの使い分け

| 用途 | サブエージェントタイプ | 理由 |
|------|---------------------|------|
| コードベース調査 | `Explore` | 複数ファイルを横断的に検索 |
| スライド PDF 確認 | `general-purpose` | 画像 Read をメインから隔離 |
| 実装計画 | `Plan` | 変更の影響範囲を事前評価 |

## 6. ドキュメントの自動更新

### コードとドキュメントの同期

ファームウェアを変更したら、関連ドキュメントも同じセッションで更新する。本プロジェクトでは、ESKF 実装変更後に以下のドキュメントを一括更新した:

```
docs: update documentation to match current ESKF implementation

Changes:
- CLAUDE.md: update Firmware Structure section
- docs/getting-started.md: add required sensors column to flight mode table
- docs/plans/vehicle-firmware.md: clarify external ESKF repo is reference only
- sf_algo_fusion/README.md: rewrite to document sensor control
```

**教訓:** ドキュメント更新を「後でやる」と忘れる。コード変更と同じセッションで完結させる。

## 7. よくある失敗と対策

### Claude Code に指示する際の注意点

| 失敗パターン | 対策 |
|-------------|------|
| 「テストして」とだけ言う | 「69個のPCユニットテストを実行して全パスを確認」と具体的に |
| 大きな変更を一度に依頼 | 段階に分けて1つずつ依頼（ESKF リファクタリングの例） |
| コンテキスト切れで作業ロスト | Next steps をコミットに含めて復帰可能にする |
| ドキュメント更新を後回し | 「コード変更と同時にドキュメントも更新して」と明記 |
| 不要なファイル生成 | Claude Code のツール既定方針（明示的な要求がない限り README やドキュメントファイルを作成しない）に従う。プロジェクト固有の運用にしたい場合は CLAUDE.md に明記する |

### CLAUDE.md の反復改善

CLAUDE.md は一度書いて終わりではない。作業中に発見した問題をルールとして追記していく:

- **Slide Rules**: スライド変更時の自動レビューが漏れた失敗を教訓に、ルール化した
- **画像確認はサブエージェント限定**: rate limit（一定期間内に使えるトークン量・利用量の上限）に抵触する問題が発生し、ルール化した
- **BL8（コードブロック下端）**: 同じレイアウト崩れが6回発生し、チェックリストに追加した

CLAUDE.md 自体が「プロジェクト固有の学習済みルールブック」として成長する。

---

<a id="english"></a>

# Claude Code Best Practices

This document summarizes practical know-how for using Claude Code, accumulated through the development of the StampFly Ecosystem. It was first written in April 2026, based on insights from 50+ commits; by September 2026 the project had grown past 1,800 commits.

## 1. Session Management

### Behavioral Control via CLAUDE.md

CLAUDE.md is automatically loaded at session start and governs Claude Code's behavior.

| Setting | Effect | Example from This Project |
|---------|--------|--------------------------|
| Startup file reads | Automatic context restoration | Load `PROJECT_PLAN.md` + previous commit log |
| Response language | Consistent communication | "Respond in Japanese" |
| Commit rules | Prevent forgotten changes | "Always commit after code changes" |
| Tool priorities | Consistent workflow | "Use sf CLI preferentially" |

**Lesson:** Vague instructions in CLAUDE.md get ignored. Write assertively: "Do X" not "Consider doing X".

### Session Continuity via Next Steps

Every commit includes a `Next steps:` section, making it clear where to resume.

**Good Next steps characteristics:**
- Include specific commands (`sf flash vehicle -m`)
- Clear metrics to verify ("BA should not change during flight")
- Listed in priority order

## 2. Commit Strategy

### Consistent Conventional Commits

Commits follow `type(scope): subject` format, with merge commits as the main exception (roughly 1,800 total as of September 2026). Use the `/commit` skill for automatic guideline-compliant messages.

### Small, Frequent Commits

The ESKF (Error State Kalman Filter, the attitude/position estimator) refactoring split one large change into 7 steps, each maintaining a buildable/testable state with 7 rollback points.

## 3. Debugging Workflow

### Log-Driven Debugging

Most effective pattern: Symptom → Telemetry capture → Visualization → Root cause → Fix → Re-flight verification.

**Lesson:** When asking Claude Code to analyze logs, be specific: "Identify the cause of the 13-second periodic attitude reset" rather than just "look at the log."

### Incremental Fixes

Split one bug into multiple independent, testable fixes to enable bisection when issues arise.

## 4. Context Management

### Handling Large Files

| Situation | Do | Don't |
|-----------|----|-------|
| Image review (slide PDFs) | Delegate to sub-agent | Read in main context |
| Large file investigation | Read specific line ranges | Read entire file |
| Multi-file exploration | Use Explore sub-agent | Read files one by one |

## 5. Tool Integration

### sf CLI Unification

Consolidating scripts under sf CLI subcommands unifies tool access. One instruction in CLAUDE.md ("use sf commands") covers all tools.

## 6. Document Synchronization

Update related documentation in the same session as code changes. "Do it later" means "never."

## 7. Common Pitfalls and Solutions

| Pitfall | Solution |
|---------|----------|
| Vague test instructions | Specify: "Run 69 PC unit tests, verify all pass" |
| Large changes at once | Break into stages (ESKF refactoring example) |
| Context loss between sessions | Include Next steps in every commit |
| Deferred documentation | "Update docs alongside code changes" in CLAUDE.md |
| Iterating CLAUDE.md | Add rules as problems are discovered; it grows into a project-specific rulebook over time |
