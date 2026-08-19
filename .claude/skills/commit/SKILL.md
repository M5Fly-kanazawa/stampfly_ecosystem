---
name: commit
description: プロジェクトのコミットガイドラインに基づいてGitコミットを作成
---

# Git Commit with Guidelines
# ガイドラインに基づいたGitコミット

このスキルは、プロジェクトのコミットガイドライン（`docs/contributing/commit-guidelines.md`）に基づいて適切なコミットメッセージを作成し、コミットを実行します。

## 実行手順

### 1. 変更の確認

まず、git status と git diff で変更内容を確認してください：

```bash
git status
git diff --cached  # Staged changes
git diff           # Unstaged changes
```

### 2. コミットメッセージの作成

以下の構造でコミットメッセージを作成してください：

```
<type>(<scope>): <subject>

<body>

Next steps:
- <次の作業1>
- <次の作業2>
```

#### Type（種類）
- `feat`: 新機能追加
- `fix`: バグ修正
- `refactor`: リファクタリング
- `test`: テスト追加・修正
- `docs`: ドキュメント更新
- `chore`: ビルド・設定変更

#### Scope（範囲）
- コンポーネント名: `flight_command`, `imu`, `led`, `control`
- ドキュメント: `docs`, `guidelines`
- フェーズ名: `phase3`, `phase4`

#### Subject（要約）
- 50文字以内
- 現在形・命令形（英語）
- ピリオド不要

#### Body（本文）

**必須要素:**
- **Changes:** 変更内容の箇条書き（3-5項目）

**推奨要素（状況に応じて）:**
- **問題の背景**: `This fixes <問題>. The problem was:` で始め、原因を1-5ステップで説明
- **修正後の動作**: `With this fix:` で始め、改善内容を1-5ステップで説明
- **技術的詳細**: パラメータ、制御則、コード位置など
- **設計判断の理由**: なぜこのアプローチを選んだか
- **影響範囲**: `Related files:` で変更ファイルとコード位置
- **フェーズ進捗**: `Phase X-Y Status: Complete ✅`

#### Next steps（次の作業）- **必須**

セッション中断時の再開ポイントを明記してください：

**書くべき内容:**
1. **すぐ次にやること** - ビルド、書き込み、テスト
2. **確認すべきこと** - 期待される動作、パフォーマンス指標
3. **次の実装タスク** - 関連機能、ドキュメント更新

**良い例:**
```
Next steps:
- Flash firmware to vehicle (idf.py flash)
- Test jump 0.15m command via WiFi CLI:
    python3 tools/stampfly_cli.py --ip 192.168.2.19 jump 0.15
- Verify altitude control with ±2cm tolerance
- Monitor ToF raw values in serial log during climb/descent
- Test consecutive jumps to verify queue handling
- Document simplified Jump command control law in docs/
```

**避けるべき例:**
```
Next steps:
- テスト  ← 具体性がない
- 動作確認  ← 何を確認するのか不明
```

### 3. コミットの実行

変更をステージングしてコミットしてください：

```bash
# Unstaged changes がある場合は追加
git add <files>

# コミットメッセージを heredoc で作成（フォーマット保持）
git commit -m "$(cat <<'EOF'
<type>(<scope>): <subject>

Changes:
- <変更1>
- <変更2>

<body>

Next steps:
- <次の作業1>
- <次の作業2>
EOF
)"
```

### 4. 確認

コミット後、ログを確認してください：

```bash
git log -1 --format="%H%n%s%n%b"
git status
```

## テンプレート

### バグ修正（Fix）

```bash
git commit -m "$(cat <<'EOF'
fix(<scope>): <50文字以内の要約>

Changes:
- <変更内容1>
- <変更内容2>

This fixes <問題の説明>. The problem was:
1. <根本原因のステップ1>
2. <根本原因のステップ2>

With this fix:
1. <修正後の動作1>
2. <修正後の動作2>

Next steps:
- <次にやるべきこと1>
- <次にやるべきこと2>
EOF
)"
```

### 新機能（Feature）

```bash
git commit -m "$(cat <<'EOF'
feat(<scope>): <50文字以内の要約>

<機能の概要説明>

Changes:
- <変更内容1>
- <変更内容2>

Behavior:
- <新しい動作1>
- <新しい動作2>

Phase <X-Y> Status: Complete ✅
Next: Phase <X-Z> (<次のフェーズの説明>)

Next steps:
- <次の実装タスク>
- <テスト計画>
EOF
)"
```

### リファクタリング（Refactor）

```bash
git commit -m "$(cat <<'EOF'
refactor(<scope>): <50文字以内の要約>

<リファクタリングの目的>

Changes:
- <変更内容1>
- <変更内容2>

Technical details:
- <技術的詳細1>
- <技術的詳細2>

<設計判断の理由>

Next steps:
- <次のタスク>
EOF
)"
```

### ドキュメント更新（Docs）

```bash
git commit -m "$(cat <<'EOF'
docs(<scope>): <50文字以内の要約>

<ドキュメント更新の目的>

Changes:
- <変更内容1>
- <変更内容2>

Contents:
- <追加/更新した内容1>
- <追加/更新した内容2>

Next steps:
- <関連ドキュメントの更新>
- <実装への反映>
EOF
)"
```

## チェックリスト

コミット前に以下を確認してください：

### 必須
- [ ] ヘッダーがConventional Commits形式
- [ ] Subjectが50文字以内
- [ ] Changesセクションで変更内容を箇条書き
- [ ] **Next stepsセクションを含める**

### 推奨
- [ ] 問題の背景と根本原因を説明（fixの場合）
- [ ] 修正後の動作を説明（fixの場合）
- [ ] 技術的詳細を記載（パラメータ、コード位置）
- [ ] 設計判断の理由を記載
- [ ] 次の作業が具体的（コマンド、検証項目）

### オプション
- [ ] フェーズ進捗を記載
- [ ] 影響範囲（Related files）を記載
- [ ] テスト結果サマリー
- [ ] Breaking changes警告

## 重要事項

1. **Next stepsは必須**: セッション中断時の再開ポイントとして重要
2. **具体的なコマンドを記載**: 再現性を確保
3. **検証項目を明記**: テスト計画を明確に
4. **heredoc使用**: コミットメッセージのフォーマットを保持

## 参考

詳細は以下を参照：
- プロジェクト内: `docs/contributing/commit-guidelines.md`
- Conventional Commits: https://www.conventionalcommits.org/
