# セッション再開ノート（最終更新 2026-06-17）

> このファイルは作業再開用の引き継ぎ。**いま読むべきはここ。**

## 0. 1分で再開する

1. **現在地:** ブランチ `experiment/wobble-min`（main + 2コミット）。`main`（commit c63de44）は無傷。
   ```bash
   cd /Users/kouhei/tmp/github/stampfly_ecosystem
   git checkout experiment/wobble-min     # ふらつき研究の続き
   # 復帰したいとき: git checkout main  （+ 必要なら sf flash vehicle_new）
   ```
2. **いま何をしている:** 「**ホバーのふらつき最小化**」の制御則比較研究。中心ドキュメント = `firmware/vehicle_new/docs/wobble_minimization_study.md`（**最初に通読**）。
3. **次の一手（ユーザー判断待ち）:** 下記 §3 の3択。ユーザー推奨は「①M1a を params.cpp に適用＋全SIL回帰＋実機フラッシュ → 実機ログで再同定して微調整」。

## 1. このセッションでやったこと（コミット済み・全て検証済み）

### main にマージ済み（着陸＋ESKF, 実機未検証）
| commit | 内容 |
|--------|------|
| da6c49b | 地面効果フロート用の降下停滞接地検出器（host単体テスト4本）|
| 6a31d1a | 近地面の着地アシスト（推力上限ランプダウン＝粘り解消, 実機OK報告済）|
| 83b5a33 | 初回離陸オーバーシュート調査（ESKFバイアス収束差・修正なし）|
| 4a194b7〜c63de44 | **実機ALT_HOLDログ解析＋ESKFチューニング**: accel_att_noise 0.8→1.2 / tof_noise 0.03→0.01 / 姿勢更新に accel LPF 30Hz 追加（オフライン再生ハーネス eskf_replay.cpp で実データ最適化, Code Identity）|

直前のセッション群（main）: ALT/POS自動着陸・着陸操縦化(INV-1/2)・場当たりパッチ再発防止(architecture.md INV節+CLAUDE.mdルール) 等。詳細は auto-memory 参照。

### experiment/wobble-min ブランチ（ふらつき研究, 未マージ）
| commit | 内容 |
|--------|------|
| 7dfd9f7 | SILふらつきベンチ枠組み + 文献/コードベース結論(M0) |
| 5b588c0 | **M1 カスケードPID再整形の結果 + ★SIL対実機の発振教訓** |

## 2. ふらつき研究の現状（中心 = wobble_minimization_study.md）

### 結論（文献＋コードベース＋SIL＋実機余裕が一致）
**ふらつき(1-6Hz)はカスケードPIDのループ整形問題。最良手法 = M1（レートループ再整形＝交差を5→7-8Hzへ上げ＋小Tdで位相リード）。INDI/ADRC不要**（INDIはRPM必須＋152Hz増幅）。`線形ADRC≡PID+設定点重み+2次フィルタ`(arXiv:2501.11374)。

### ★最重要の教訓（絶対に忘れない）
**SILの乱流ベンチを直接最適化すると「実機で発振する」ゲインに収束する。** 例: Td素朴増で SIL ふらつき−75%(Td0.08 Kp×2)だが実機プラントPM=−375°発振。理由: D経路(η=0.125)が高域利得を上げ交差を32-89Hzへ押上げ→実機遅延14.7msが位相を食う。SILは遅延小で罠を隠す。
**∴ レートゲインは実機同定プラントでループ整形(tune_pid, 交差制御+PM確保)→SILで効果量検証→実機で最終確認、の順。**

### M1 推奨ゲイン（実機プラントで安定・SILで−33%実証）
`rate_sysid.tune_pid` を実機ログ同定プラント(roll b=1/1.57e-5 L=14.7ms / pitch b=1/2.96e-5 L=8.4ms)に適用:
- **M1a（推奨, wc7 PM55）:** roll `kp=6.88e-4 ti=0.23 td=0.0032` / pitch `kp=1.39e-3 ti=0.23 td=0.0063` → 実機PM55°/GM9-12dB, SILふらつき roll4.79→3.28°(−33%)/pitch2.80→1.82°(−35%), duty減。yaw は flown(3.43e-4/0.20/0)維持（同定不確かゆえ別途chirp要）。
- M1b（攻め, wc8 PM50）: roll `7.86e-4/0.20/0.0029` pitch `1.62e-3/0.20/0.0059` → −42%。

### 補完手法（未実装）
- **M2 = 152Hz固定ノッチ＋ジャイロフィルタ**: 実機振動下でDの増幅を抑える。**SIL乱流ベンチは振動が無く検証不可**（n2要だがn2は離陸を壊す別バグ §4）。
- M3 = 2-DOF FF: 外乱駆動には的外れ（文献, 低期待）。

## 3. 次の一手（ユーザー判断待ちの3択）
1. **M1a を params.cpp に適用＋全SIL回帰(30シナリオ)で非破壊確認＋実機フラッシュ** → 実機でM1aを飛ばしログ取得 → そのログで再同定して微調整（★ユーザー推奨）。
   - 注意: params.cpp の rate ゲイン既定(Kp1.365e-3/Ti0.7/Td0.01)は**実機NVS飛行値(Kp4.96e-4/Ti0.40/Td0)と乖離**。M1適用時に実機飛行値ベースで設定し、実機は flash→`param reset`→`param save` 必須。
2. **M2(152Hzノッチ)実装** して M1 と併用、実機振動耐性を上げる。
3. INDI/ADRC も実装してSILで反証データを残す。

## 4. 既知のバグ／未解決（別途対応）
- **N2鉛直推定発散**: `--noise n2` で離陸が壊れる（"Takeoff complete"出るが真高度0.01m）。ふらつきベンチでn2を使えない原因。要調査。
- **地面効果env配線ミス(main側)**: `SIL_EMU_GROUND_EFFECT`/`--ground-effect` は emu_main_generic.cpp に配線されていたが emu_vehicle_new は emu_main.cpp を使う→**main側で no-op**。ブランチでは emu_main.cpp に修正済み(commit 7dfd9f7)。**main にも反映が必要**（地面効果のSIL検証が実は効いていなかった）。
- **右前ドリフト（トリム誤差・別問題）**: 手放しで右前へ流れる（ESKF姿勢 mean roll+1.4°/pitch+1.65°, accel_bias by+0.41大）。**`sf cal accel`(6方向)＋水平校正**が第一手。ダメならCG/モータ/IMU実装の機械的点検。ふらつきとは別。
- **コード vs NVS ゲイン乖離**: 上記§3注。params.cpp と実機NVSのrateゲインが違う。

## 5. 再現コマンド
```bash
source setup_env.sh
# ふらつきベンチ(M0'=実機飛行ゲイン): rate gainは SIL_EMU_PARAMS_FILE で再ビルド無し上書き
cat > /tmp/g.params <<'EOF'
rate.roll.kp 6.88e-4
rate.roll.ti 0.23
rate.roll.td 0.0032
rate.pitch.kp 1.39e-3
rate.pitch.ti 0.23
rate.pitch.td 0.0063
rate.yaw.kp 3.43e-4
rate.yaw.ti 0.20
rate.yaw.td 0.0
EOF
SIL_EMU_PARAMS_FILE=/tmp/g.params sf sil scenario simulator/sil/scenarios/wobble_bench.scn \
  --target vehicle_new --turbulence 0.03 --duration 30000000
python3 analysis/scripts/wobble_bench.py simulator/sil/viz/out_scn_wobble_bench/trajectory.csv

# 実機プラントでのゲイン設計・余裕計算: tools/log_analyzer/rate_sysid.py の tune_pid / loop_margins
# 実機ログ解析(再): python3 analysis/scripts/altlog_sysid_eskf.py <log.jsonl> <out_dir>
# ESKFパラメータ実データ掃引: analysis/scripts/eskf_replay_preprocess.py + eskf_sweep.py
```

## 6. 主要ファイル
- `firmware/vehicle_new/docs/wobble_minimization_study.md` — ★ふらつき研究本体（逐次更新中）
- `analysis/reports/altlog_20260614T214537/REPORT.md` — 最新実機ログ解析（ふらつき診断・図7枚, .gitignoreでローカル）
- `analysis/scripts/wobble_bench.py` / `eskf_replay.cpp` / `eskf_sweep.py` / `altlog_sysid_eskf.py`
- `simulator/sil/scenarios/wobble_bench.scn`, `simulator/sil/plant/plant.cpp`(乱流), `simulator/sil/emu/emu_main.cpp`(env)
- ワークフロー出力(手法分析6種/文献5領域): `/private/tmp/claude-501/.../tasks/wwrvhkxbh.output` と `w3o9iftca.output`

## 7. ルール(忘れず)
日本語応答 / `/commit`スキル / 制御変更は必ずSIL裏付け→**だがレートゲインはSIL直接最適化禁止(§2教訓)・実機プラントでループ整形** / architecture.md の INV節照合 / `sf` CLI優先 / バイリンガルコメント。
