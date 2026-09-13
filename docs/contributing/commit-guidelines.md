# Commit Message Guidelines
# コミットメッセージガイドライン

> **Note:** [English version follows after the Japanese section.](#english)

---

## 現在のコミットメッセージ構造

### 基本フォーマット（Conventional Commits準拠）

```
<type>(<scope>): <subject>

<body>

<footer>
```

### 現在含めている要素

#### 1. ヘッダー（必須）
```
<type>(<scope>): <subject>
```

**Type（種類）:**
- `feat`: 新機能追加
- `fix`: バグ修正
- `refactor`: リファクタリング
- `test`: テスト追加・修正
- `docs`: ドキュメント更新
- `chore`: ビルド・設定変更

**Scope（範囲）:**
- コンポーネント名: `flight_command`, `imu`, `led`, `control`
- フェーズ名: `phase3`, `init`

**Subject（要約）:**
- 50文字以内
- 現在形・命令形
- ピリオド不要

**例:**
```
fix(flight_command): remove premature current_command_id_ assignment
refactor(flight_command): simplify Jump command to use ToF raw values
feat(led): complete Phase 3-3 event-driven LED control
```

#### 2. 本文（Body）- 推奨

現在含めている情報：

**a) 変更内容の箇条書き**
```
Changes:
- Add FlightCommandService include to imu_task.cpp
- Check if WiFi command is active (isRunning())
- During WiFi command: require only 1 sample for is_grounded transition
```

**b) 問題の背景説明**
```
This fixes the 40cm overshoot issue in jump command. The problem was:
1. OPEN_LOOP_CLIMB reached 0.10m and transitioned to CLIMBING
2. But is_grounded transition took 50ms (20 consecutive samples)
3. During this delay, ESKF position estimation not yet active
```

**c) 技術的詳細**
- 制御パラメータ: `duty = 0.60 + 0.8 * (target - tof_altitude)`
- コード位置: `flight_command.cpp:318-350`
- シーケンス説明: ステップバイステップの動作

**d) 設計判断の理由**
```
Jump command does not assume reaching stable flight altitude, so optical
flow sensor and ESKF position estimation are not required.
```

**e) 影響範囲**
```
Related files:
- flight_command.hpp: Add OPEN_LOOP_CLIMB to ExecutionPhase enum
- flight_command.cpp:318-350: Implement open-loop climb logic
- imu_task.cpp:208-219: Remove forced is_grounded=false on ARM
```

**f) フェーズ進捗**
```
Phase 3-3 Status: Complete ✅
Next: Phase 3-4 (ControlArbiter integration with SystemStateManager)
```

#### 3. フッター（Footer）- オプション

現在は使用していないが、追加すべき要素：

**a) Next steps（次回作業予定）** ⭐ **NEW**
```
Next steps:
- Flash firmware and test jump command on hardware
- Verify 15cm target altitude with ±2cm tolerance
- Monitor ToF raw values during climb and descent
- Test consecutive command execution (rapid commits)
```

**b) Breaking changes（互換性破壊）**
```
BREAKING CHANGE: Removed OPEN_LOOP_CLIMB phase from ExecutionPhase enum.
External code using this phase will need to be updated.
```

**c) 関連issue参照**
```
Fixes #123
Related to #456
```

**d) テスト結果サマリー**
```
Tested:
- ✅ Boot sequence: INIT → CALIBRATING → IDLE
- ✅ Jump command: climb to 15cm, free fall, landing
- ⚠️  Pending: consecutive command execution test
```

**e) パフォーマンス影響**
```
Performance impact:
- Binary size: +44 lines, -81 lines (net reduction)
- Memory: No heap allocation changes
- CPU: Reduced computational overhead (no ESKF in jump)
```

---

## 推奨コミットメッセージテンプレート

### テンプレート1: バグ修正（Fix）

```
fix(<scope>): <50文字以内の要約>

Changes:
- <変更内容1>
- <変更内容2>
- <変更内容3>

This fixes <問題の説明>. The problem was:
1. <根本原因のステップ1>
2. <根本原因のステップ2>
3. <影響>

With this fix:
1. <修正後の動作1>
2. <修正後の動作2>
3. <期待される結果>

Next steps:
- <次にやるべきこと1>
- <次にやるべきこと2>
```

**実例:**
```
fix(imu): skip takeoff counter delay during WiFi command execution

Changes:
- Add FlightCommandService include to imu_task.cpp
- Check if WiFi command is active (isRunning())
- During WiFi command: require only 1 sample for is_grounded transition
- During normal flight: require 20 samples (50ms) for chattering prevention

This fixes the 40cm overshoot issue in jump command. The problem was:
1. OPEN_LOOP_CLIMB reached 0.10m and transitioned to CLIMBING
2. But is_grounded transition took 50ms (20 consecutive samples)
3. During this delay, ESKF position estimation not yet active
4. CLIMBING phase used stale altitude (0.03m) for control
5. Large error caused excessive throttle -> climbed to 40cm

With this fix:
1. ToF reaches 0.10m during OPEN_LOOP_CLIMB
2. is_grounded = false immediately (1 sample)
3. resetPositionVelocity() executes without delay
4. CLIMBING phase uses fresh ESKF altitude estimate
5. Correct feedback control to 15cm target

Next steps:
- Test on hardware with jump 0.15m command
- Verify altitude tolerance (±2cm)
- Test consecutive jump commands
```

### テンプレート2: 新機能（Feature）

```
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
```

**実例:**
```
feat(led): complete Phase 3-3 event-driven LED control with SystemStateManager

Migrated LED updates from polling-based manual calls to event-driven callbacks,
reducing coupling and improving responsiveness.

Changes:
- init.cpp: Registered LED state callback with SystemStateManager
- imu_task.cpp: Use SystemStateManager::setFlightState() for transitions
- Removed manual LEDManager::onFlightStateChanged() calls

Behavior:
- LED updates now triggered automatically via event callbacks
- Calibration state changes → FlightState transitions → LED callback
- Reduced polling overhead and code duplication

Phase 3-3 Status: Complete ✅
Next: Phase 3-4 (ControlArbiter integration with SystemStateManager)

Next steps:
- Implement Phase 3-4: ControlArbiter integration
- Test LED updates during state transitions on hardware
- Verify event callback performance in 400Hz control loop
```

### テンプレート3: リファクタリング（Refactor）

```
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
```

**実例:**
```
refactor(flight_command): simplify Jump command to use ToF raw values

Redesigned Jump command with simple proportional control:
- Removed OPEN_LOOP_CLIMB phase (now INIT → CLIMBING → DESCENDING → DONE)
- Use ToF raw values directly instead of ESKF position estimation
- Control law: duty = 0.60 + 0.8 * (target - tof_altitude)
- Target reached: tof >= target - 0.02m
- Descending: duty = 0 (free fall with attitude control ON)
- Landing detection: tof < 0.05m

Jump command does not assume reaching stable flight altitude, so optical
flow sensor and ESKF position estimation are not required.

Next steps:
- Flash firmware and test jump command on hardware
- Verify 15cm target altitude with ±2cm tolerance
- Monitor ToF raw values during climb and descent
- Test consecutive command execution (rapid commits)
```

---

## チェックリスト

コミット前に以下を確認：

### 必須項目
- [ ] ヘッダーがConventional Commits形式
- [ ] Subjectが50文字以内
- [ ] 変更内容が箇条書きで明確

### 推奨項目
- [ ] 問題の背景と根本原因を説明
- [ ] 修正後の動作を説明
- [ ] 技術的詳細（パラメータ、コード位置）を記載
- [ ] 設計判断の理由を記載
- [ ] **Next stepsセクションを追加** ⭐ **NEW**

### オプション項目
- [ ] フェーズ進捗を記載（該当する場合）
- [ ] 影響範囲（Related files）を記載
- [ ] テスト結果サマリー
- [ ] パフォーマンス影響
- [ ] Breaking changes警告（該当する場合）

---

## Next stepsセクションのガイドライン

### 書くべき内容

1. **すぐ次にやること**（最優先）
   - ファームウェアの書き込み
   - ハードウェアテスト
   - 実機検証

2. **確認すべきこと**
   - 期待される動作の検証項目
   - パフォーマンス確認
   - 極端な条件のテスト

3. **次の実装タスク**
   - 関連機能の実装
   - ドキュメント更新
   - リファクタリング計画

### 良い例

```
Next steps:
- Flash firmware to vehicle (idf.py flash)
- Test jump 0.15m command via WiFi CLI
- Verify altitude control: target ±2cm tolerance
- Monitor serial log for ToF raw values during flight
- Test consecutive jumps (rapid git commits)
- Document simplified Jump command in FLIGHT_COMMANDS.md
```

### 避けるべき例

```
Next steps:
- テスト  ← 具体性がない
- 動作確認  ← 何を確認するのか不明
- その他の改善  ← 曖昧
```

---

## コミットメッセージ作成のベストプラクティス

### 1. コミットは論理的な単位で
- 1つのバグ修正 = 1コミット
- 1つの機能追加 = 1コミット
- 複数の無関係な変更を混ぜない

### 2. コミット前にdiffを確認
```bash
git diff --cached
```

### 3. コミットメッセージは未来の自分へのメモ
- 3ヶ月後に読んで理解できるか？
- なぜその変更が必要だったか明確か？
- どうやってテストしたか記録されているか？

### 4. Next stepsは作業の連続性を保つ
- セッション中断時の再開ポイント
- チーム開発時の引き継ぎ情報
- CI/CDパイプラインのトリガー情報

---

<a id="english"></a>

## Current Commit Message Structure

### Basic Format (Conventional Commits)

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Currently Included Elements

#### 1. Header (Required)
- Type: `feat`, `fix`, `refactor`, `test`, `docs`, `chore`
- Scope: Component or phase name
- Subject: 50 chars max, imperative mood

#### 2. Body (Recommended)
- **Changes:** Bullet points of modifications
- **Problem explanation:** Root cause analysis
- **Technical details:** Parameters, code locations, sequences
- **Design rationale:** Why this approach was chosen
- **Impact scope:** Related files and line numbers
- **Phase progress:** Status updates for multi-phase work

#### 3. Footer (Optional)

**NEW: Next steps section** ⭐
```
Next steps:
- Flash firmware and test on hardware
- Verify 15cm target altitude
- Test consecutive command execution
```

Other footer elements:
- Breaking changes
- Issue references
- Test results
- Performance impact

---

## Recommended Commit Message Templates

See Japanese section for detailed templates and examples.

---

## Checklist

### Required
- [ ] Header follows Conventional Commits format
- [ ] Subject ≤ 50 characters
- [ ] Changes clearly listed

### Recommended
- [ ] Problem background explained
- [ ] Solution behavior described
- [ ] Technical details included
- [ ] **Next steps section added** ⭐ **NEW**

### Optional
- [ ] Phase progress noted
- [ ] Related files listed
- [ ] Test results included
- [ ] Performance impact noted

---

## Next Steps Section Guidelines

### What to Include

1. **Immediate next tasks** (highest priority)
   - Firmware flashing
   - Hardware testing
   - Real device verification

2. **Verification items**
   - Expected behavior checks
   - Performance validation
   - Edge case testing

3. **Follow-up implementation**
   - Related features
   - Documentation updates
   - Refactoring plans

### Good Example

```
Next steps:
- Flash firmware to vehicle (idf.py flash)
- Test jump 0.15m command via WiFi CLI
- Verify altitude control: target ±2cm tolerance
- Monitor serial log for ToF raw values during flight
- Test consecutive jumps (rapid git commits)
- Document simplified Jump command in FLIGHT_COMMANDS.md
```

### Bad Example

```
Next steps:
- Test  ← Too vague
- Check operation  ← What to check?
- Other improvements  ← Ambiguous
```

---

## Best Practices

1. **One logical change per commit**
2. **Review diff before committing** (`git diff --cached`)
3. **Write for future readers** (Will you understand this in 3 months?)
4. **Use Next steps for continuity** (Session resumption, handoff, CI/CD triggers)
