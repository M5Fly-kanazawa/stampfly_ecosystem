# SILS シナリオ・テストマトリクス / SILS Scenario Test Matrix

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

vehicle の飛行を SILS（物理真値）で検証するシナリオスイートを、**層（Layer 1〜4）× 軸（roll/pitch/yaw/複合）× 判定（G1〜G4）** の観点で一覧化する。発展的プラント同定（`development_roadmap.md` §3）の各層を、まず物理真値の SILS 判定を通してから実機に進む。

### 合格判定の2系統

各 `*.scn` には `*.expect` が付き、2系統で合否を機械判定する：

| 系統 | 何を見るか | 判定区分 |
|------|-----------|--------|
| **ログ文字列** | 状態遷移の並び・順序（`ARM accepted` 等） | G1 |
| **数値メトリクス** | 実行結果のフライトログ一式（`.sflog.zip`。`truth.csv` の物理真値＋`attitude.csv`/`posvel.csv`/`motor.csv` の推定から算出。仕様は `protocol/spec/flight_log.yaml`、角度は SI 単位のラジアン）から算出（`metric <name> <op> <value> in <t0> <t1>`） | G2/G3/G4 |

判定区分の定義（`RESET_PLAN.md` §4）：

| 判定区分 | 意味 | 代表メトリクス |
|--------|------|----------------|
| **G1** 起動・状態遷移 | ARM→離陸→飛行→着陸の遷移が正しく進むか | ログ（`log_contains`/`order`） |
| **G2** 推定の追従 | 推定が物理真値にどれだけ一致するか | `att_rmse` / `alt_rmse` |
| **G3** 閉ループの安定 | 姿勢・位置が発散せず有界か | `horizontal_drift_max` / `tilt_max` / `alt_band` |
| **G4** アクチュエータ健全性 | モータ指令が飽和しないか | `duty_max` |

## 2. マトリクス

### 軸別→複合の構成

各層は **個別軸を1軸ずつ → 最後に複合** で検証する（`<layer>_<axis>.scn` が個別、`<layer>_flight.scn` が複合キャップストーン）。

| 層 | モード | シナリオ | 励振軸 | G1 | G2 | G3 | G4 |
|----|--------|----------|--------|----|----|----|----|
| **L1** | ACRO（レート） | `acro_flight` | roll±/pitch±/yaw+（1本に全軸ダブレット） | ✅ | att_rmse<3 | tilt_max<25 | （注1） |
| **L2** | STABILIZE（姿勢） | `stab_flight` | roll±/pitch+（複合） | ✅ | att_rmse<3 | tilt_max<18 | duty<0.92 |
| **L3** | ALT_HOLD（高度） | `alt_flight` | 鉛直のみ | ✅ | alt_rmse<0.08 | alt_band<0.35 | duty<0.92 |
| **L4** | POS_HOLD（位置） | `pos_roll` | **ロール単独** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_pitch` | **ピッチ単独** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_flight` | **斜め複合（capstone）** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_yaw` | **ヨー回転後に保持** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |

**注1（ACRO の G4）:** ACRO のレートダブレットは設計上モータを瞬間飽和させる（広帯域励振、roadmap §3.1.3）ため、duty は意図的に判定対象外とする。

### その他のシナリオ（非層別）

| シナリオ | 目的 | ターゲット |
|----------|------|-----------|
| `disturb` | P7 外乱回復（横風＋モータ故障） | vehicle |
| `modeswitch` | P8 飛行中モード切替（ALT↔POS）で姿勢/高度有界 | vehicle |
| `crash_refly` | P8 ★ロバスト再飛行（墜落→自動DISARM→物理ハンドリング→再校正→再飛行）。`--duration 33000000` 必須 | vehicle |
| `stab_ground_windup` | **STABILIZE 地上 ARM 中の積分巻き上がり回帰**（実機ログ ground_spin_stab 由来）: 校正後に加速度バイアスを注入し床の上で低スロットル 25 s → 地上で姿勢/レート積分器を保持するので `duty_spread_max`<0.05・離床でも傾き<5°。保持が無いと duty 差 0.59、離床で転倒（傾き 180°）。`--duration 40000000` 必須 | vehicle |
| `boot_motion` | 静止判定付き起動校正（運搬中は校正完了せず ARM 拒否、設置後に完了→飛行） | vehicle |
| `alt_auto_takeoff` | **ARM トリガ** ALT_HOLD 自動離陸（再設計 2026-06-14）: スプール中 duty=0、固定 0.3 m/s 上昇、**目標 0.5m を捕捉**（行き過ぎでなく目標値） | vehicle |
| `pos_auto_takeoff` | ARM トリガ POS_HOLD 自動離陸（上昇中の発進点保持を含む、目標 0.5m 捕捉） | vehicle |
| `alt_recenter_gate` | **スロットル再センターロック Case A**（離陸後）: 上げスティックを保持しても捕捉 0.5m を保持（ロック）、中央 3072 を通すとロック解除→上昇 | vehicle |
| `alt_inflight_switch` | **スロットル再センターロック Case B**（飛行中切替）: STABILIZE→ALT_HOLD 切替で高度ジャンプなし（捕捉＋ロック）、中央通過でロック解除 | vehicle |
| `alt_arm_rollpitch` | **ARM トリガ ALT_HOLD 離陸「後」にロール/ピッチが効く、既存動作の破壊を防ぐ試験**（実機バグ 2026-06-14）: TakeoffClimb で止まると roll_sp=0 でスティック死。離陸完了の片側到達＋タイムアウトで Airborne へ抜け、ロール指令で機体が傾く（tilt_max>6°）ことを assert | vehicle |
| `alt_takeoff_steer` | **自動離陸の「上昇中」にロール/ピッチが効く**（鉛直のみ自動・姿勢は常にパイロット, ユーザー判断 2026-06-14）: ARM時からロール右を保持 → TakeoffClimb 窓 [8.4,9.2] で機体が傾く（tilt_max>6°）かつ鉛直は自動で 0.5m 到達。旧水平保持なら tilt≈0 で FAIL | vehicle |
| `alt_disarm_land` | **ALT_HOLD でのパイロット DISARM が自動着陸を起動**（ユーザー要望 2026-06-14, 注5）: ARM→自動離陸→ホバー中に DISARM → 即カットせず緩降下（0.3m/s, モータ稼働）→接地→本当の DISARM。降下中 duty>0.5・DISARM 0.4s 後も alt>0.2（自由落下でない）・終端 alt<0.05 を assert | vehicle |
| `alt_disarm_land_steer` | **パイロット着陸は降下中も操縦可**（着陸則統一 INV-1/INV-2, リファクタA 注6）: DISARM 後の降下中にロール保持 → `tilt_max=11°`＝機体が傾く。旧 `computeLanding` 水平強制なら≈0 で FAIL。中立版 `alt_disarm_land` は tilt≈0 | vehicle |
| `commloss_land_level` | **フェイルセーフ着陸は水平**（INV-2 敵対ガード）: リンク途絶直前にロール右保持 → 猶予中(FLYING)は古いロールで `tilt 11.6°` だが、LANDING 突入後は水平判定で `tilt 3.6°` に水平化。リンク生存判定=設定点の新鮮さ(500ms) | vehicle |
| `api_flight` | Tello 風 API 飛行の全鎖（command→takeoff→forward/cw/up→land、移動は到達後 ok、中立 RC が解除則を誤作動させない）。離陸高度は **0.5m に統一**（手動 RC と同一ルーチン、2026-06-14）。`--duration 40000000` 必須 | vehicle |
| `api_flip_roll` | Tello 風 API 宙返り（ロール軸: command→takeoff→up 60→`flip r`→`flip l`→land）。各宙返りは FLYING→FLIP→FLYING で `ok`、真値の回転角 ±330〜400°、宙返り前の高度を下回らない（`alt_drop_max`）、上昇 < 0.6 m、窓基準 2.0 s 以内に整定（F3 の完了後 1 s）、FLIP 中の GYRO_ANOMALY は無視、ToF 再取り込み。1.1 m から実施するのは SILS の ToF 模擬上限 1.4 m（2026-10-31 から 1.63 m）のため（実センサは約 3 m）。`--duration 39000000` 必須（2026-09-17） | vehicle |
| `api_flip_pitch` | 同、ピッチ軸（`flip f`→`flip b`）。宙返りの間隔は 5 s（前方宙返りで付く最大約 1.4 m/s の水平速度が実行条件 C4 の 0.3 m/s 未満に減衰するのを待つ）。`--duration 41000000` 必須（2026-09-17） | vehicle |
| `rc_flip` | コントローラ FLIP ボタン（Phase 3, flip-maneuver-plan.md §4.1/§9-6）: RC のみ（API 不使用）で ARM→ALT_HOLD 自動離陸→スロットルで約1.5mへ上昇（2026-10-31 から全押下を1.2m以上で実施）→FLIP ボタン3回（ロール右・ロール左・中立スティック1.5s保持→既定方向Right、押しっぱなしでも1回だけ要求）→DISARM自動着陸→新規サイクルでもう1回（ピッチ前）→DISARM。ALT_HOLD の PITCH 宙返り後の高度損失（~1m、rc_flip.expectで判定を拡幅）と3回目離陸の発散は、ボタン経路ではなくバックログ#12（ヨーレート微分項の200Hzリミットサイクル）と#16（着陸衝撃後の姿勢推定の固着）に追跡済みの既存問題のため、1本の連続飛行でなく2サイクル・PITCH方向はForwardのみで構成。`--duration 64000000` 必須（2026-09-18） | vehicle |
| `flip_disabled` | **FLIP は既定で無効**（`flip.enable` = 0、実機初回の宙返り失敗 2026-10-02）: 既定 param で宙返り高度に達し、API `flip r` が `error flip: disabled`、FLIP ボタンが `Flip button: rejected (Disabled)` で拒否され、FLIP 状態に入らない。宙返り系シナリオ（`api_flip_*`・`rc_flip`）は `--param flip.enable=1` 付きで実行。`--duration 30000000` 必須 | vehicle |
| `flip_abort_recover` | 途中で打ち切られた宙返り（`flip.gyro_abort_dps=1000`）: 実機初回の失敗（Brake が終わらない）の回帰。Brake が終了し Recover が機体を水平に戻して FLYING で終わる（`error flip: aborted gyro_limit`、傾き最大 > 51°、正味の回転 < 150°、最低高度 > 0.6 m、26 s 以降 15° 未満）→ 着陸。`--param flip.enable=1 --param flip.gyro_abort_dps=1000`（2026-10-02） | vehicle |
| `flip_brake_timeout` | Brake のタイムアウト経路（INV-2）: 通常の出口を無効化（`handoff_rate_dps=50`、`handoff_force_deg=360`）し `brake_timeout_ms=50`、`brake_ramp_rps2=250` にして、約 1500 °/s のまま Recover に入る。`error flip: aborted brake_timeout`、約 1 回転、その後水平で FLYING → 着陸（2026-10-02） | vehicle |
| `flip_test2_repro` | **実機 2026-10-02 の flip_test2 の失敗の再現**（SILS が実機の失敗で成功してしまわないための歯止め）: ロール宙返りを実機と同じ param（`flip.rate_ramp_rps2=300`・`brake_ramp_rps2=500`・`gyro_abort_dps=1800`、ホバー時の基板電圧は SILS 約 3.88 V ＝ ログ 3.90 V）と追加遅れ `--motor-delay 4`（飛行データが示す約 4 ms。プラント既定では打ち切りが紙一重のため頑健さのために付ける）で飛ばし、ジャイロ上限で**打ち切られる**ことを必須とする（フライトログの `flip_result` = 3、打ち切り時のファーム積算回転角 80〜130°＝ログ 90.0°（SILS 91°）、Spin 開始から 0.08〜0.20 s ＝ログ 0.1225 s（SILS 0.120 s）。console の応答は衝撃検知の DISARM が先に起きるため `aborted disarmed`）。旧 D×V プラント（V0）は打ち切りなしで完走した（ピーク 1577 deg/s）。現在の既定（145/145/1950）の宙返りは別（`api_flip_roll`）。`--duration 36000000` 必須（2026-10-31） | vehicle |
| `rc_flip_latch` | FLIP ボタン保持（2026-10-02）: スロットルパルス上昇の終わり（|vz| > 0.2 m/s、NotSteady）でボタンを押す → `Flip button: waiting up to 1000 ms` → 落ち着いたら自動で `Flip requested`、1 回だけ宙返り（右ロール約 360°）、拒否ログなし。`--param flip.enable=1` | vehicle |
| `sysid_rate` | 飛行中レートループ同定励振（API `sysid roll chirp 25 4`: POS_HOLD ホバーで ±25dps 対数チャープ、有界・定点維持・正常終了）。`--duration 32000000` 必須 | vehicle |
| `acro_crash_relevel` | 墜落復帰リセット後、保持されたモードスイッチが IDLE_GROUND で再適用される（実機 LED バグの固定） | vehicle |
| `autotune` | オンボード自動チューン全鎖（API `autotune roll 60 50`: 9点ステップドサイン掃引→同定→PID設計→ライブ適用→新ゲインでホバー・着陸）。`--duration 55000000` 必須 | vehicle |
| `pairing` | ペアリングハンドシェイク（未ペア起動→自動Pairing→bind）＋混信拒否（誤MAC送信機のARM/離陸をフィルタが破棄）。`--unpaired` 必須 | vehicle |
| `yaw_hold` | 定在ヨー外乱（M1 80% 故障）下のヘディングホールド: 8 秒手放しで方位有界（yaw_band<1.5°、無効化対照は 3.1° で単調流出）。実機の「勝手に回る」現象（2026-06-11）の固定 | vehicle |

**P8 ロバスト再飛行（`crash_refly`）が炙り出した2つのファーム欠陥（修正済）:**
1. **ESKF 姿勢の latch**: 墜落で姿勢推定が真値から大きく外れると accel-attitude χ² 判定（カイ二乗判定）が補正自体を棄却し続け自己復帰しない。設置時（IDLE_HELD→IDLE_GROUND、機体が level・静止と既知）に ESKF を Reset して姿勢を level へ再初期化することで解決。
2. **モード未伝播**: 接地時の飛行モード STABILIZE リセットが `StateManager::mode_` を変えるだけで制御器に伝わらず（制御器は `ControllerCmd::ModeChange` 経由でのみモードを知る）、ALT/POS 飛行後の再離陸が古いホバー推力モードのまま上昇しない。リセット時に onModeChange を作動させて解決。

## 3. 実行方法

```bash
source setup_env.sh
sf sils build vehicle
sf sils scenario simulator/sils/scenarios/pos_flight.scn --target vehicle          # 合否判定
sf sils scenario simulator/sils/scenarios/pos_flight.scn --target vehicle --video  # ＋レビュー動画
```

`metric` メトリクス名一覧（`lib/sfcli/commands/sils.py` `_traj_metric`）:
`horizontal_drift_max`, `roll_rmse`, `pitch_rmse`, `att_rmse`, `alt_rmse`,
`tilt_max`, `alt_band`, `alt_mean`, `alt_min`, `alt_max`, `duty_max`。

## 4. カバレッジ状況（2026-06-06）

- **L1〜L4 全シナリオが G1（ログ）＋ G2/G3/G4（数値）で PASS**（`SILS_EMU_NOISE=off`、決定論）。
- **L4 POS_HOLD は roll/pitch/斜め/yaw の全4軸がタイトに保持**（採用した運動加速度補償 `eskf.accel_comp.enable=1` ＋ 位置速度ループ `position.vel.kp=0.8`）。drift ≤ 1.1m・終端オフセット ≤ 1.0m、N0 センサノイズ下でも保持（drift ≤ 1.3m）。推定器の汚染を直したことで速度ゲインを 0.3→0.8 に上げられた（旧版は推定器が壊れ 0.3 が限界・斜めで発散）。
- **残課題:** n1/n2（過酷な振動）下の POS_HOLD は baseline 同様に飛び去る（振動処理は別課題）。実機 ESP-IDF ビルドは未検証（host SILS は全ソース通過）。

## 5. 2026-10-31 R_out プラントに伴うシナリオ入力・閾値の変更

方針: 物理測定を SILS より上に置く。シナリオの入力や閾値を変えてよいのは、(a) 旧プラントに合わせた値で意図は不変、または (b) 実機の飛行データに根拠がある場合に限る。理由はファイル内のコメントにも 1 行ずつ記した。

| シナリオ・対象 | 旧 → 新 | 理由 | 実機の根拠 |
|---|---|---|---|
| STABILIZE/ACRO 開ループのスロットルを使う 18 シナリオ（`acro_flight`・`alt_flight`・`alt_inflight_switch` 行E・`boot_motion`・`calib`・`commloss`・`commloss_land_level`・`crash_refly` 行C/N・`modeswitch`・`pairing_held`・`pos_flight`・`pos_pitch`・`pos_roll`・`pos_yaw`・`stab_flight`・`stab_ground_windup`・`sysid_roll_step`・`yaw_hold`） | 生値 3243 → 3171（離陸）、3176 → 3108（ホバー付近）。2048 からの偏差を一律 ×0.94 | (a) 旧値は 2026-08-22 の再較正（ホバー生値 3154 を基準に ×0.8386）で旧プラントに合わせたもの。R_out プラントの開ループ・ホバー生値は 3088（SILS で生値を掃引し、上昇加速度が 0 になる点を求めた。電池 3.87 V、局所傾き 0.0045 m/s² 毎 生値、ホバー近傍の誤差は約 ±5 生値）。係数 = (3088−2048)/(3154−2048) = 0.94。離陸トリガ（生値 3072）を下回るため、ホバー以下の「ゆるい離陸」行（3123・3106・3090: `acro_crash_relevel`・`pairing`・`attitude_trim_test`・`autotrim_learn`・`disturb`）は変更しない（実測の上昇加速度は 3090 で +0.02、3106 で +0.09、3123 で +0.17 m/s²。意図の「小さな正の上昇」を満たす） | 実機の対応値なし（実機 14 ログに STABILIZE のホバー区間がない）。係数は SILS の掃引による |
| `rc_flip`・`rc_flip_latch`（スロットルパルスによる事前上昇） | `rc_flip`: 行D 5000→4300 ms、行E 2000→2700 ms、行R 5000→4300 ms、行S 2000→2700 ms、再上昇パルス（生値 3600）を行H2（500 ms）・行J2（600 ms）に追加。`rc_flip_latch`: 行D 5000→4300 ms、行E 1800→2500 ms。時刻表は不変（D と E、R と S の合計は同じ） | (a) 上昇入力が旧プラントに合わせたものだった。ボタン押下の高度を 1.2 m 以上にする（R_out では 0.93〜0.98 m で拒否された）。意図は不変。R_out では宙返りと解放のたびに約 0.1 m/s で沈むため、2・3 回目の押下の前に再上昇パルスを入れた。押下の高度は `rc_flip` で 1.204 / 1.274 / 1.232 / 1.223 m、`rc_flip_latch` で 1.244 m | 実機の宙返り開始高度 1.41 m・1.70 m（`logs/flip_test1/2.sflog.zip`）と矛盾しない |
| `rc_flip.expect` の `alt_mean` 上限（窓 13.5〜16.5 s と 45.3〜48.3 s） | 1.35 → 1.75 | (a)+(b) 上昇入力の延長で窓の平均高度が約 1.39 m になるため。上限は実機の宙返り開始高度の最大 1.70 m を切り上げた値 | 実機 2 回（1.41 m・1.70 m） |
| `alt_auto_takeoff`・`pos_auto_takeoff` の `alt_band`（窓 12〜16 s） | < 0.20 → < 0.55（SILS の値 0.2668） | (b) 旧閾値は旧プラントの値 0.008 に合わせたもので、実機の 14 回の離陸（ALT_HOLD 5・POS_HOLD 9、離陸から +4.2〜+8.2 s の ESKF 高度）はいずれも 0.062 以上。実機の最大 0.521 を切り上げた。p90 の 0.443 では実機の 2/14 回（0.500・0.521）を不合格にしてしまうため採らない | 実機 14 回: p50 0.164、p90 0.443、最大 0.521 |
| `api_flight` の `alt_mean` 下限（窓 14〜17.5 s） | > 0.42 → > 0.30（SILS の値 0.4130） | (b) 旧閾値は旧プラントの値 0.518 に合わせたもので、実機の 4/14 回が下回る。実機の最小 0.326 を切り下げた | 実機 14 回（離陸から +7.7〜+11.2 s）: 最小 0.326、p10 0.348、p50 0.473 |
| `calib.expect` の xfail 印 | 削除 | xfail の理由（0.6〜0.7 Hz の自励振動）が出なくなり、回帰試験 2 回で XPASS、直接実行 2 回で PASS（att_rmse 0.55 deg） | 実機の対応値なし |
| SILS の下向き ToF 模擬の最大距離（`simulator/sils/devices/vl53_device.cpp` の `MAX_MM`） | 1400 → 1630 mm | (b) 実機ログ（121 本＋flip_test1/2、約 15 万サンプル）では 1.4〜1.7 m で 95〜99 % が有効（status 0）、1.7〜1.8 m で 91 %、1.8〜2.0 m で 66〜70 %、2.0〜2.1 m で 22 %（無効時の status は 255 が大半。SILS も 255）。模擬の上限は 24 bin バッファが決める 1636 mm で、1630 は 2 bin 分割がバッファに収まる値（プローブで 1.63 m まで有効・誤差 17 mm 以内）。1.63〜2.0 m は未模擬のまま | 実機の有効率（高度は ESKF 高度） |
| `alt_inflight_switch.expect` の `alt_band` 上限（窓 14.7〜18.7 s） | < 0.32 → < 0.38（SILS の値 0.351） | STABILIZE→ALT_HOLD 捕捉後の減衰振動。閾値側のコメントが「開始外乱の大きさによらない固定振幅」と記録しているもの。R_out の値 0.351 に約 8 % の余裕 | 実機の対応値なし・未検証（飛行中に ALT_HOLD へ切り替えた実機ログが必要） |
| `crash_refly`（`.scn` 行J・行L、`.expect` の 2 つの窓） | 行J 2500→500 ms、行L 8000→10000 ms（再校正以降の時刻は不変）。`.expect`: 墜落静止の窓 14〜16 s → 12〜14 s、持上げの窓 16.8〜18.4 s → 14.8〜16.4 s（閾値は不変、手が届く時刻と同じ −2.0 s の移動） | (a) R_out の墜落は機体が側臥（ロール 90 deg）で止まり、衝突の約 0.2 s 後に始まる 3.5 s の起動校正が側臥のまま完了して level_offset 1.5708 rad が推定器に残る。手が水平に直した後も ARM の判定（姿勢推定が重力と食い違う、正しい挙動で不変）が再 ARM を拒否した。ハンドリング軌道は任意の墜落姿勢を水平へ戻すので、手を校正の最中（14.0 s）に届かせて水平な設置後に校正をやり直させる。再飛行は実際に飛ぶ（上昇 1.33 m、終端ホバー 0.87 m） | 実機の対応値なし |
| `pos_flight.expect` の xfail 印 | 削除 | ヨートルク権限飽和（デューティ天井固定）が R_out プラントでは出なくなった（ホバー duty が低くヨーに余裕がある）。回帰試験 2 回で XPASS。`pos_yaw`・`yaw_hold` の印は残す | 実機の対応値なし |

**変更しなかったもの（未確認を含む）:** `commloss_land_level`・`pos_yaw`・`yaw_hold` の xfail 印（KNOWN-FAIL のまま。`commloss_land_level` は着陸後の `alt_max` 0.156 m > 0.1 m で不合格）。

---

<a id="english"></a>

## 1. Overview

This document enumerates the vehicle SILS flight-validation scenarios along
**Layer (1–4) × axis (roll/pitch/yaw/combined) × gate (G1–G4)**. Each layer of the
layer-by-layer plant identification (`development_roadmap.md` §3) must pass the
physical-truth SILS gates before moving to hardware.

### Two verdict tracks

Each `*.scn` has a `*.expect` judged two ways: **log strings** (state-transition
order ⇒ G1) and **numerical metrics** computed from the run's flight-log bundle
(`.sflog.zip` — `truth.csv`'s physical ground truth plus `attitude.csv`/`posvel.csv`/
`motor.csv` estimates; spec `protocol/spec/flight_log.yaml`; angles in SI radians)
(`metric <name> <op> <value> in <t0> <t1>` ⇒ G2/G3/G4). Gate definitions are in
`RESET_PLAN.md` §4 (G1 boot/transitions, G2 estimate tracking, G3 closed-loop
boundedness, G4 actuator health).

## 2. Matrix

Per layer, isolate each axis first (`<layer>_<axis>.scn`), then a combined capstone
(`<layer>_flight.scn`).

| Layer | Mode | Scenario | Excited axis | G1 | G2 | G3 | G4 |
|-------|------|----------|--------------|----|----|----|----|
| **L1** | ACRO (rate) | `acro_flight` | roll±/pitch±/yaw+ (all-axis doublets) | ✅ | att_rmse<3 | tilt_max<25 | (note 1) |
| **L2** | STABILIZE (attitude) | `stab_flight` | roll±/pitch+ (combined) | ✅ | att_rmse<3 | tilt_max<18 | duty<0.92 |
| **L3** | ALT_HOLD (altitude) | `alt_flight` | vertical only | ✅ | alt_rmse<0.08 | alt_band<0.35 | duty<0.92 |
| **L4** | POS_HOLD (position) | `pos_roll` | **roll only** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_pitch` | **pitch only** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_flight` | **diagonal (capstone)** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_yaw` | **hold at rotated heading** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |

**Note 1 (ACRO G4):** ACRO rate doublets saturate the motors briefly BY DESIGN
(broadband excitation, roadmap §3.1.3), so duty is intentionally not gated.

## 3. How to run

```bash
source setup_env.sh
sf sils build vehicle
sf sils scenario simulator/sils/scenarios/pos_flight.scn --target vehicle
sf sils scenario simulator/sils/scenarios/pos_flight.scn --target vehicle --video
```

## 4. Coverage status (2026-06-06)

- All L1–L4 scenarios PASS on G1 (logs) + G2/G3/G4 (numerical), `SILS_EMU_NOISE=off`,
  deterministic.
- L4 POS_HOLD holds TIGHTLY on all four axes (roll/pitch/diagonal/yaw) with the adopted
  acceleration compensation (`eskf.accel_comp.enable=1`) + the position velocity loop at
  `position.vel.kp=0.8` (drift ≤ 1.1 m, final offset ≤ 1.0 m; ≤ 1.3 m under N0). Fixing the
  estimator contamination is what let the velocity gain rise from 0.3 to 0.8 (with the old
  contaminated estimate 0.3 was the limit and the diagonal diverged).
- Open: POS_HOLD under n1/n2 (severe vibration) flies away as the baseline does
  (vibration handling is separate); the on-target ESP-IDF build is unverified.

## 5. Scenario input/threshold changes on 2026-10-31 for the R_out plant

Policy: physical measurement ranks above the SILS. A scenario input or threshold may change only when (a) it was tuned to the old plant and the scenario's intent is unchanged, or (b) real-flight data give the basis. Each change also carries a one-line reason in the file's comment.

| Scenario / item | Old -> new | Reason | Real basis |
|---|---|---|---|
| The 18 scenarios that use open-loop STABILIZE/ACRO throttle (`acro_flight`, `alt_flight`, `alt_inflight_switch` row E, `boot_motion`, `calib`, `commloss`, `commloss_land_level`, `crash_refly` rows C/N, `modeswitch`, `pairing_held`, `pos_flight`, `pos_pitch`, `pos_roll`, `pos_yaw`, `stab_flight`, `stab_ground_windup`, `sysid_roll_step`, `yaw_hold`) | Raw 3243 -> 3171 (take-off), 3176 -> 3108 (near hover). Deviation from 2048 scaled by 0.94 uniformly | (a) The old values were tuned to the old plant by the 2026-08-22 recalibration (x0.8386, anchored on hover raw 3154). The R_out plant's open-loop hover raw is 3088 (found by sweeping the raw stick in SILS and locating the zero of the climb acceleration; battery 3.87 V, local slope 0.0045 m/s^2 per raw, about +-5 raw near hover). Factor = (3088-2048)/(3154-2048) = 0.94. The gentle take-off rows (3123, 3106, 3090: `acro_crash_relevel`, `pairing`, `attitude_trim_test`, `autotrim_learn`, `disturb`) are left unchanged because the scaled value would fall below the take-off trigger (raw 3072); measured climb acceleration is +0.02 (3090), +0.09 (3106), +0.17 (3123) m/s^2, which keeps the intended small positive climb | No real counterpart (the 14 real logs hold no STABILIZE hover segment). The factor comes from the SILS sweep |
| `rc_flip`, `rc_flip_latch` (pre-flip climb by throttle pulse) | `rc_flip`: row D 5000->4300 ms, row E 2000->2700 ms, row R 5000->4300 ms, row S 2000->2700 ms, plus re-climb pulses (raw 3600) in rows H2 (500 ms) and J2 (600 ms). `rc_flip_latch`: row D 5000->4300 ms, row E 1800->2500 ms. The timeline is unchanged (D+E and R+S sums are the same) | (a) The climb input was tuned to the old plant. The button press must happen at >= 1.2 m (on the R_out plant it was refused at 0.93-0.98 m); intent unchanged. Because the craft sinks about 0.1 m/s after each flip and release on the R_out plant, a re-climb pulse precedes the 2nd and 3rd press. Press heights: `rc_flip` 1.204 / 1.274 / 1.232 / 1.223 m, `rc_flip_latch` 1.244 m | Consistent with the real flip start heights of 1.41 m and 1.70 m (`logs/flip_test1/2.sflog.zip`) |
| `rc_flip.expect` `alt_mean` upper bound (windows 13.5-16.5 s and 45.3-48.3 s) | 1.35 -> 1.75 | (a)+(b) The longer climb input raises the window-mean height to about 1.39 m. The bound is the real maximum flip start height, 1.70 m, rounded up | 2 real flips (1.41 m, 1.70 m) |
| `alt_band` of `alt_auto_takeoff` and `pos_auto_takeoff` (window 12-16 s) | < 0.20 -> < 0.55 (SILS value 0.2668) | (b) The old bound was tuned to the old plant's value 0.008; all 14 real take-offs (5 ALT_HOLD, 9 POS_HOLD; ESKF altitude, take-off +4.2 to +8.2 s) are at or above 0.062. The real maximum 0.521 is rounded up. The p90 (0.443) is not used because it would reject 2 of the 14 real take-offs (0.500, 0.521) | 14 real take-offs: p50 0.164, p90 0.443, max 0.521 |
| `alt_mean` lower bound of `api_flight` (window 14-17.5 s) | > 0.42 -> > 0.30 (SILS value 0.4130) | (b) The old bound was tuned to the old plant's value 0.518; 4 of 14 real take-offs are below it. The real minimum 0.326 is rounded down | 14 real take-offs (take-off +7.7 to +11.2 s): min 0.326, p10 0.348, p50 0.473 |
| xfail marker of `calib.expect` | Removed | The xfail reason (0.6-0.7 Hz self-oscillation) no longer appears: XPASS in two regression runs, PASS in two direct runs (att_rmse 0.55 deg) | No real counterpart |
| Maximum distance of the SILS bottom ToF model (`MAX_MM` in `simulator/sils/devices/vl53_device.cpp`) | 1400 -> 1630 mm | (b) In real logs (121 logs + flip_test1/2, about 150k samples) the ToF is valid (status 0) 95-99 % of the time at 1.4-1.7 m, 91 % at 1.7-1.8 m, 66-70 % at 1.8-2.0 m and 22 % at 2.0-2.1 m (the invalid status is mostly 255, as in SILS). The model's ceiling is set by the 24-bin buffer at 1636 mm; 1630 keeps the two-bin split inside the buffer (probed: valid with error <= 17 mm up to 1.63 m). The 1.63-2.0 m range stays unmodelled | Real validity rates (height = ESKF altitude) |
| `alt_band` upper bound of `alt_inflight_switch.expect` (window 14.7-18.7 s) | < 0.32 -> < 0.38 (SILS value 0.351) | The damped oscillation after the STABILIZE->ALT_HOLD capture, which the bound's own comment records as a fixed amplitude independent of the initiating disturbance. R_out value 0.351 plus about 8 % | No real counterpart, unverified (needs a hardware log of an in-flight switch to ALT_HOLD) |
| `crash_refly` (`.scn` rows J and L, two windows in `.expect`) | Row J 2500->500 ms, row L 8000->10000 ms (times from the re-calibration dwell on unchanged). `.expect`: crash-rest window 14-16 s -> 12-14 s, lift window 16.8-18.4 s -> 14.8-16.4 s (thresholds unchanged, shifted by the same -2.0 s as the hand's arrival) | (a) The R_out crash leaves the craft lying on its side (roll 90 deg); the 3.5 s boot calibration that starts about 0.2 s after impact completes on its side and leaves level_offset 1.5708 rad in the estimator, so even after the hand levels the craft the ARM guard (attitude estimate disagrees with gravity; correct behaviour, unchanged) rejected every re-ARM. The handling trajectory already levels any crash attitude, so the hand now arrives during the calibration (14.0 s) and the calibration restarts on the levelled craft. The re-fly really flies (climb to 1.33 m, final hover 0.87 m) | No real counterpart |
| xfail marker of `pos_flight.expect` | Removed | The yaw torque-authority saturation (duty ceiling lock) no longer appears on the R_out plant (lower hover duty leaves yaw headroom). XPASS in two regression runs. The markers of `pos_yaw` and `yaw_hold` stay | No real counterpart |

**Not changed (including unverified items):** the xfail markers of `commloss_land_level`, `pos_yaw` and `yaw_hold` (still KNOWN-FAIL; `commloss_land_level` fails on `alt_max` 0.156 m > 0.1 m after touchdown).
