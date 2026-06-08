# 次セッション指示書 — ペアリング実装完了。本道＝実機ブリングアップへ

最終更新: 2026-06-09（**ペアリング P1〜P3 実装＋SIL 検証完了。次は実機ブリングアップ（飛ばすにはペアリングが前提）**）

---

## ★最初にやること（優先順）

リファクタ計画（Phase0〜8）に続き、**ペアリング機能（P1〜P3）を実装・SIL 検証完了**。これで実機を
飛ばす前提（コントローラが機体の実 MAC を学習して ControlPacket を届ける）が整った。本道は
**SIL→実機**（development_roadmap）。

1. **本道：実機ブリングアップ（development_roadmap Phase 2 → Phase 3）** ← メイン
   - **Phase 2「HAL 接続（実機が動く）」**: 実機センサ値が推定器へ、制御出力がモータへ届く経路を確認。
     合格基準＝機体を手で持って ARM→スロットル中立で全モータ等速、スティックで1軸ずつ duty 変化、
     テレメトリで全センサ表示。**まだ飛ばさない。** （詳細 `development_roadmap.md §4 Phase 2`）
   - **ペアリング実機検証（Phase 2 の一部）**: 電源ON→LED 青速点滅＋ブザー（自動 Pairing）→
     コントローラ側でペアリング（`peering_process` が CH スキャンで PairingPacket を発見）→ 成立で
     点滅停止 → ARM 可能に。コントローラ・protocol・旧 vehicle は不変、vehicle_new のみ実装済み。
   - **Phase 3「実機初飛行 — ACRO で同定」（最重要マイルストーン）**: SIL で確定したレート PID＋
     プラントモデルが実機で成立するか。ACRO 手動ホバー → 実機ログを SIL に注入して差分診断
     （gyro RMS が SIL 予測 ±50%、ステップ立上り時定数 ±20% 以内）。
   - まず `sf doctor` → `sf build vehicle_new` → `sf flash vehicle_new -m` で実機が起動するか。

2. **データ駆動ノイズ** （auto-memory `project_sil_noise_data_driven`）: 実機ログ解析→SIL ノイズ
   プロファイル注入。実機で飛ばした後（Model Fidelity, development_roadmap Phase 5）の軸。

3. **ペアリング P4（per-drone channel + 同時マスペアリング堅牢化, 30機スケール）** ← 30機ワーク
   ショップ運用の直前。**既知の弱点**: ペア成立は両側「先着＝採用」で、複数の未ペア機を**同時に**
   ペアリングすると取り違え（クロスペアリング）が起こり得る（ペア成立後の混信は src MAC フィルタで
   対策済み）。**現状の運用＝1ペアずつ**で回避（ユーザー判断 B, 2026-06-09）。堅牢化は P4 で
   per-drone channel + RSSI/ボタン確認（RSSI 等はコントローラ改修要）。詳細 `docs/pairing_plan.md` P4。

4. **任意・低優先（リファクタ Phase 7 残り、安定性影響小）**: 相補フィルタゲイン params 化、
   pid 飛行リミット config 化。

## ★ペアリング実装の要点（2026-06-09 完了, `docs/pairing_plan.md` 参照）

- **状態モデル**: `PairingState{NotPaired, Pairing, Paired}` を FlightState と並行の独立状態機械として
  StateManager が単一所有（旧 vehicle 踏襲・ユーザー承認）。未ペア起動で自動 Pairing、Pairing 中は
  ARM 拒否、ボタン長押し3s で再ペア。
- **ハンドシェイク**: 機体が PairingPacket（11B: ch+自MAC+署名 AA5516 88）を 500ms 周期 broadcast →
  コントローラが学習し機体 MAC へ ControlPacket をユニキャスト → 機体が src MAC を相手として確定・
  NVS 保存（namespace sf_pair）。以降、相手以外の src MAC を破棄（混信対策）。
- **配線**: comm=事実 publish（pairing_complete）/ state=判断（pairing_state）/ notify=LED青速点滅+
  pairingTone（R5 Pub-Sub）。CLI `pair [start|status]` 追加。
- **SIL 検証**: `sf sil scenario simulator/sil/scenarios/pairing.scn --target vehicle_new --unpaired`。
  飛行系シナリオは起動時に NVS へペア済み MAC を seed（`--unpaired` でスキップ）。

---

## 完了したこと（参考・git log に詳細）

### リファクタ計画 `valiant-frolicking-sun.md`（Phase 0〜8 全完了）
場当たりコード全廃→あるべき姿へ。reset 集約・責務分離・HW 所有一元化・起動骨格・未実装機能配線・
品質仕上げ・ロバスト再飛行 SIL 検証。検証スイート全 PASS、ESP-IDF 実機ビルド可。

- **χ²過剰棄却の根治（commit 90093c1）**: `eskf.obs.accel_att_noise 0.06→0.8`。マニューバ中の運動
  加速度で 66% 棄却→姿勢ドリフトしていた「崖」を解消。詳細 `docs/chi2_latchup_finding.md §8`、
  auto-memory `project_estimator_attitude_comparison`。
- **Phase 8 ロバスト再飛行（commit 7b3692c, 04d698f）**: SIL Plant に物理ハンドリング機構（墜落機を
  持上→正立SLERP→運搬→設置の連続キネマティック軌道、IMU比力 解析合成、teleportなし）。
  `crash_refly.scn`(21/21) / `modeswitch.scn`(17/17)。crash_refly が **2つの実ファーム欠陥**を炙り出し
  修正＝①ESKF姿勢latch（墜落級大誤差は χ² で回復不能→設置時 ESKF Reset）②モード未伝播（接地
  STABILIZE リセットが制御器に伝わらず再離陸不能→onModeChange 発火）。

### SIL GUI（寄り道だが完成 — commit 7d7140b〜1729e29）
`sf sil gui` でブラウザからシナリオ作成・実行・グラフ・**実寸 StampFly のライブ3D飛行アニメ**。
詳細 `simulator/sil/gui/README.md`、auto-memory `project_sil_gui`。本道とは独立。

---

## 0. 着手前に読む（順番に）

1. **`firmware/vehicle_new/docs/development_roadmap.md` §3〜§4** — 本道の正典。プラント同定戦略
   （ACRO 起点・Layer 1〜4）と Phase 2〜6、3原則（Code/Param/Model Identity）。
2. CLAUDE.md vehicle_new 6文書（特に requirements §2 状態モデル / architecture / detailed_design §3）。
3. **`docs/pairing_plan.md`** — ペアリング調査結果と実装計画（本セッションで作成）。
4. 直近コミットログ（Phase 8＝`7b3692c`/`04d698f`、χ²根治＝`90093c1`）。
5. auto-memory: `reference_params_ssot`、`project_estimator_attitude_comparison`、
   `project_stampfly_emulator`、`feedback_plain_japanese_terms`。

---

## SIL 検証スイート（ファーム変更時は必ず全 PASS → /commit）

```bash
source setup_env.sh
sf sil build
for s in pos_roll pos_pitch pos_flight pos_yaw alt_flight stab_flight acro_flight \
         disturb commloss calib prearm modeswitch; do
  sf sil scenario simulator/sil/scenarios/$s.scn --target vehicle_new
done
sf sil scenario simulator/sil/scenarios/pairing.scn --target vehicle_new --unpaired   # ペアリング＋混信拒否
sf sil scenario simulator/sil/scenarios/crash_refly.scn --target vehicle_new --duration 33000000
sf sil scenario simulator/sil/scenarios/hover_espnow.scn --target vehicle   # legacy 無回帰
simulator/sil/build/hover_smoke simulator/sil/models/stampfly.xml           # G2+G3 物理真値
sf build vehicle_new   # ESP-IDF 実機ビルド（ファーム変更時は必須）
```

実機作業に入ったら（development_roadmap §4 Phase 2/3）、SIL ゲートに加えて実機の手持ち確認・
ACRO 手動飛行・差分診断（実機 vs SIL）を行う。

---

## メモ
- 制御/ESKF パラメータの変更提案は**必ず SIL の数値検証で裏付けてから**（CLAUDE.md 原則）。
- 安全機能は実機で命に関わる。emu で実経路を発火させてから実機へ。
- 用語は平易な日本語（auto-memory `feedback_plain_japanese_terms`）。「回帰」→「検証スイート」。
- ペアリングは設計文書に状態が無い。実装前に状態モデルへの位置づけを確認・文書更新（設計矛盾の即時報告）。
