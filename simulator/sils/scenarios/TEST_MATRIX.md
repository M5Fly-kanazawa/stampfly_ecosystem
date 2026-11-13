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
| `stab_ground_windup` | **STABILIZE 地上 ARM 中の積分巻き上がりの再確認試験**（実機ログ ground_spin_stab 由来）: 校正後に加速度バイアスを注入し床の上で低スロットル 25 s → 地上で姿勢/レート積分器を保持するので `duty_spread_max`<0.05・離床でも傾き<5°。保持が無いと duty 差 0.59、離床で転倒（傾き 180°）。`--duration 40000000` 必須 | vehicle |
| `calib_side_lying` | **校正の水平チェック**（detailed_design §3 注10）: 墜落で横倒しになった機体を校正窓 1 回分（3.5 s）より長く置いても `Calibration window rejected: not level` で棄却し続け、横倒しのまま校正が完了して加速度バイアス約 ±9.8 m/s² が入らない。手が水平に置いた後は校正が完了して ARM が受理される。水平チェックの無いビルドでは失敗（ARM 全拒否）。`--duration 39000000` 必須 | vehicle |
| `calib_supersede` | **校正の置き換え規則**（detailed_design §3 注10）: 起動校正が注入した加速度のずれ 0.2 m/s² を適用した後、注入を外して手が置き直すと、「ずれなし」の再校正が先の値を置き換える（`previous seed superseded`）。置き換え規則の無いビルドでは `estimator left untouched` となり 0.2 の値が残るので失敗。`--duration 22000000` 必須 | vehicle |
| `boot_motion` | 静止判定付き起動校正（運搬中は校正完了せず ARM 拒否、設置後に完了→飛行） | vehicle |
| `alt_auto_takeoff` | **ARM トリガ** ALT_HOLD 自動離陸: スプール中 duty=0、固定 0.3 m/s 上昇、**目標 0.5m を捕捉**（行き過ぎでなく目標値） | vehicle |
| `pos_auto_takeoff` | ARM トリガ POS_HOLD 自動離陸（上昇中の発進点保持を含む、目標 0.5m 捕捉） | vehicle |
| `alt_recenter_gate` | **スロットル再センターロック Case A**（離陸後）: 上げスティックを保持しても捕捉 0.5m を保持（ロック）、中央 3072 を通すとロック解除→上昇 | vehicle |
| `alt_inflight_switch` | **スロットル再センターロック Case B**（飛行中切替）: STABILIZE→ALT_HOLD 切替で高度ジャンプなし（捕捉＋ロック）、中央通過でロック解除 | vehicle |
| `alt_arm_rollpitch` | **ARM トリガ ALT_HOLD 離陸「後」にロール/ピッチが効く、既存動作の破壊を防ぐ試験**: TakeoffClimb で止まると roll_sp=0 になりスティックが効かなくなるのを防ぐ。離陸完了の片側到達＋タイムアウトで Airborne へ抜け、ロール指令で機体が傾く（tilt_max>6°）ことを assert | vehicle |
| `alt_takeoff_steer` | **自動離陸の「上昇中」にロール/ピッチが効く**（鉛直のみ自動・姿勢は常にパイロット）: ARM時からロール右を保持 → TakeoffClimb 窓 [8.4,9.2] で機体が傾く（tilt_max>6°）かつ鉛直は自動で 0.5m 到達。上昇中に水平を保持する実装なら tilt≈0 で FAIL | vehicle |
| `alt_disarm_land` | **ALT_HOLD でのパイロット DISARM が自動着陸を起動**（注5）: ARM→自動離陸→ホバー中に DISARM → 即カットせず緩降下（0.3m/s, モータ稼働）→接地→本当の DISARM。降下中 duty>0.5・DISARM 0.4s 後も alt>0.2（自由落下でない）・終端 alt<0.05 を assert | vehicle |
| `alt_disarm_land_steer` | **パイロット着陸は降下中も操縦可**（着陸則統一 INV-1/INV-2, リファクタA 注6）: DISARM 後の降下中にロール保持 → `tilt_max=11°`＝機体が傾く。`computeLanding` が水平を強制する実装なら≈0 で FAIL。中立版 `alt_disarm_land` は tilt≈0 | vehicle |
| `commloss_land_level` | **フェイルセーフ着陸は水平**（INV-2 敵対ガード）: リンク途絶直前にロール右保持 → 猶予中(FLYING)は古いロールで `tilt 11.6°` だが、LANDING 突入後は水平判定で `tilt 3.6°` に水平化。リンク生存判定=設定点の新鮮さ(500ms) | vehicle |
| `api_flight` | Tello 風 API 飛行の全鎖（command→takeoff→forward/cw/up→land、移動は到達後 ok、中立 RC が解除則を誤作動させない）。離陸高度は **0.5m**（手動 RC と同一ルーチン）。`--duration 40000000` 必須 | vehicle |
| `api_flip_roll` | Tello 風 API 宙返り（ロール軸: command→takeoff→up 60→`flip r`→`flip l`→land）。各宙返りは FLYING→FLIP→FLYING で `ok`、真値の回転角 ±330〜400°、宙返り前の高度を下回らない（`alt_drop_max`）、上昇 < 0.6 m、窓基準 2.0 s 以内に整定（F3 の完了後 1 s）、FLIP 中の GYRO_ANOMALY は無視、ToF 再取り込み。1.1 m から実施するのは SILS の ToF 模擬上限（1.63 m）のため（実センサは約 3 m）。`--duration 39000000` 必須 | vehicle |
| `api_flip_pitch` | 同、ピッチ軸（`flip f`→`flip b`）。宙返りの間隔は 5 s（前方宙返りで付く最大約 1.4 m/s の水平速度が実行条件 C4 の 0.3 m/s 未満に減衰するのを待つ）。`--duration 41000000` 必須 | vehicle |
| `rc_flip` | コントローラ FLIP ボタン（Phase 3, flip-maneuver-plan.md §4.1/§9-6）: RC のみ（API 不使用）で ARM→ALT_HOLD 自動離陸→スロットルで約1.5mへ上昇（全押下を 1.2 m 以上で実施）→FLIP ボタン3回（ロール右・ロール左・中立スティック1.5s保持→既定方向Right、押しっぱなしでも1回だけ要求）→DISARM自動着陸→新規サイクルでもう1回（ピッチ前）→DISARM。ALT_HOLD の PITCH 宙返り後の高度損失（~1m、rc_flip.expectで判定を拡幅）と3回目離陸の発散は、ボタン経路ではなくバックログ#12（ヨーレート微分項の200Hzリミットサイクル）と#16（着陸衝撃後の姿勢推定の固着）に追跡済みの既存問題のため、1本の連続飛行でなく2サイクル・PITCH方向はForwardのみで構成。`--duration 64000000` 必須 | vehicle |
| `flip_disabled` | **FLIP は既定で無効**（`flip.enable` = 0。実機初回の宙返りの失敗が理由）: 既定 param で宙返り高度に達し、API `flip r` が `error flip: disabled`、FLIP ボタンが `Flip button: rejected (Disabled)` で拒否され、FLIP 状態に入らない。宙返り系シナリオ（`api_flip_*`・`rc_flip`）は `--param flip.enable=1` 付きで実行。`--duration 30000000` 必須 | vehicle |
| `flip_abort_recover` | 途中で打ち切られた宙返り（`flip.gyro_abort_dps=1000`）: 実機初回の失敗（Brake が終わらない）の再確認試験。Brake が終了し Recover が機体を水平に戻して FLYING で終わる（`error flip: aborted gyro_limit`、傾き最大 > 51°、正味の回転 < 150°、最低高度 > 0.6 m、26 s 以降 15° 未満）→ 着陸。`--param flip.enable=1 --param flip.gyro_abort_dps=1000` | vehicle |
| `flip_brake_timeout` | Brake のタイムアウト経路（INV-2）: 通常の出口を無効化（`handoff_rate_dps=50`、`handoff_force_deg=360`）し `brake_timeout_ms=50`、`brake_ramp_rps2=250` にして、約 1500 °/s のまま Recover に入る。`error flip: aborted brake_timeout`、約 1 回転、その後水平で FLYING → 着陸 | vehicle |
| `flip_test2_repro` | **実機の flip_test2 の失敗の再現**（SILS が実機の失敗で成功してしまわないための歯止め）: ロール宙返りを実機と同じ param（`flip.rate_ramp_rps2=300`・`brake_ramp_rps2=500`・`gyro_abort_dps=1800`、ホバー時の基板電圧は SILS 約 3.88 V ＝ ログ 3.90 V）と追加遅れ `--motor-delay 4`（飛行データが示す約 4 ms。プラント既定では打ち切りが紙一重のため頑健さのために付ける）で飛ばし、ジャイロ上限で**打ち切られる**ことを必須とする（フライトログの `flip_result` = 3、打ち切り時のファーム積算回転角 80〜130°＝ログ 90.0°（SILS 91°）、Spin 開始から 0.08〜0.20 s ＝ログ 0.1225 s（SILS 0.120 s）。console の応答は衝撃検知の DISARM が先に起きるため `aborted disarmed`）。D×V プラント（V0）では打ち切りなしで完走する（ピーク 1577 deg/s）。既定（145/145/1950）の宙返りは別（`api_flip_roll`）。`--duration 36000000` 必須 | vehicle |
| `rc_flip_latch` | FLIP ボタン保持: スロットルパルス上昇の終わり（|vz| > 0.2 m/s、NotSteady）でボタンを押す → `Flip button: waiting up to 1000 ms` → 落ち着いたら自動で `Flip requested`、1 回だけ宙返り（右ロール約 360°）、拒否ログなし。`--param flip.enable=1` | vehicle |
| `sysid_rate` | 飛行中レートループ同定励振（API `sysid roll chirp 25 4`: POS_HOLD ホバーで ±25dps 対数チャープ、有界・定点維持・正常終了）。`--duration 32000000` 必須 | vehicle |
| `acro_crash_relevel` | 墜落復帰リセット後、保持されたモードスイッチが IDLE_GROUND で再適用される（実機 LED バグの固定） | vehicle |
| `autotune` | オンボード自動チューン全鎖（API `autotune roll 60 50`: 9点ステップドサイン掃引→同定→PID設計→ライブ適用→新ゲインでホバー・着陸）。`--duration 55000000` 必須 | vehicle |
| `pairing` | ペアリングハンドシェイク（未ペア起動→自動Pairing→bind）＋混信拒否（誤MAC送信機のARM/離陸をフィルタが破棄）。`--unpaired` 必須 | vehicle |
| `yaw_hold` | 定在ヨー外乱（M1 80% 故障）下のヘディングホールド: 8 秒手放しで方位有界（yaw_band<1.5°、無効化対照は 3.1° で単調流出）。実機の「勝手に回る」現象の固定 | vehicle |
| `tof_silent_takeoff` | **下向き ToF が自動離陸の上昇中に出力停止**（ToF 出力停止による上昇事故の再現、故障注入 `tof_silent 1`、POS_HOLD、`--duration 21000000`）: 最後のサンプルの後は何も来ない → Failsafe が TOF_SILENT を発報 → 0.3 s 後に TAKEOFF → LANDING（上昇中止、FLYING に至らない）→ 接地フラグを信用しないので位置・速度の固定が外れ、ToF なしの降下は**推力が常に 支持推力 × 0.95 以下**（上昇できない形。窓が無いので支持推力は低くした見積もり）→ 接地（ToF 出力停止中の LANDING での 3 G の衝撃は非常でなく接地）→ 出力停止中は ARM が理由つきで拒否される。実測: ToF 停止 8.39 s、`alt_max` 0.229 m、接地速度 0.91 m/s（上限の一定推力で沈むため。対策の無い main は ToF なしで上昇し続け、最高 42 m。`detailed_design.md` §3 注12） | vehicle |
| `tof_silent_api_takeoff` | **事故の連鎖そのもの**（Tello 風 API `takeoff`、POS_HOLD）: 自動離陸の上昇中に ToF が出力停止 → TAKEOFF → LANDING で上昇中止（`alt_max` 0.175 m、接地速度 0.81 m/s。対策の無い main は上昇し続け、最高 46 m）→ API は 12 s のタイムアウトを待たず `error takeoff aborted` を返す → ToF が出力停止したままの 二つ目の `takeoff` は最初に `error tof silent - check the sensor`。`--duration 16000000` | vehicle |
| `tof_silent_hover` | **0.5 m でホバー中（ALT_HOLD・FLYING）に下向き ToF が出力停止**（`--duration 25000000`）: 0.3 s 後に FLYING → LANDING → 降下 → 床に達して IDLE_GROUND。実測: ToF 停止 14.0 s、接地速度 0.60 m/s（通常着陸は 0.3 m/s。対策の無い main は実行の終わりまで LANDING のままで、接地を検出しない）。ARM は理由つきで拒否される | vehicle |
| `tof_silent_hover_1p2` | **高度上限の近く（約 1.1 m）でホバー中に下向き ToF が出力停止**（`--duration 31100000`）: 同じ連鎖。スティック上昇が上限 1.2 m へ向けて絞られるので、上昇に 5 s かかり約 1.1 m で終わる。実測: 停止 20.5 s（高度 1.1 m）、接地速度 0.84 m/s（対策の無い main は 2.9 m まで上昇した） | vehicle |
| `tof_silent_alt_climb` | **ALT_HOLD のスティックで上昇中（約 0.8 m、約 0.3 m/s）に出力停止**（`--duration 26500000`）: 速度ループが上昇を止め（最大 0.92 m、出力停止時から 0.12 m）、接地速度 0.77 m/s（対策の無い main は 1.5 m まで上昇した） | vehicle |
| `tof_silent_est_high` | **ホバー推力の見積もりが真のホバーより 19 % 高いまま（`param` 事象で起動時に設定、学習なし）、約 1.2 m で出力停止**（`--duration 28000000`）: 支持推力は加速度計で較正した値（見積もりではない）、`touchdown by time` にならず接地。実測: 最大 1.24 m、接地速度 0.96 m/s（対策の無い main は 2.2 m まで上昇し、接地を検出しない） | vehicle |
| `tof_silent_takeoff_est_high` | **ホバー推力の見積もりが 28 % 高い（`param` 事象 `hover.thrust_corr 1.2`）まま、自動離陸の上昇初期に出力停止**（`--duration 21000000`）: 致命的とされた場合（対策の無い main で真値の最高 6.8 m）。上昇は速く（LANDING で 1.3 m/s）、降下は上限（支持推力の 0.95 倍。窓が無いので、空中の全サンプルでの商 × 0.85）の下。実測: 真値の最高 0.80 m、接地速度 1.04 m/s、モータは床の上で止まる。`.expect` は最高 < 1.0 m・接地速度 < 1.3 m/s | vehicle |
| `tof_silent_api_takeoff_est_high` | 同、Tello 風 API `takeoff`（`--duration 16000000`、`hover.thrust_corr 1.2`）: `error takeoff aborted`、二つ目の `takeoff` は最初に拒否。実測: 真値の最高 0.72 m、接地速度 0.99 m/s（対策の無い main は上昇し続け 120 m）。最高 < 1.0 m・接地速度 < 1.25 m/s | vehicle |
| `tof_silent_takeoff_noise` | `tof_silent_takeoff_est_high` を `hover.thrust_corr 1.1` とセンサノイズ n2（seed 1、`--noise n2 --seed 1`）で（`--duration 21000000`）: ノイズで上昇が目標に達してから出力停止が判定され、FLYING/POS_HOLD から 1.7 m/s で上昇中に LANDING。実測: 真値の最高 1.20 m、接地速度 1.99 m/s（n2 のプラントは真のホバー推力が低く、窓が無いときの低くした見積もりがその分だけ強く沈める。対策の無い main は 4.6 m まで上昇した）。最高 < 1.5 m・接地速度 < 2.4 m/s | vehicle |
| `tof_silent_touchdown_detect` | `tof_silent_hover` を 3 G の衝撃カットを無効にして（`--param safety.impact.accel_g=10`、`--duration 25000000`）: 接地検出器（衝撃のあとの静止）だけが着陸を終わらせる。`Touchdown detected (impact, then still` → LANDING → IDLE_GROUND、`Impact detected` と `touchdown by time` が無い。実測: 接地速度 0.60 m/s、床の上で静止（真値 0.021 m） | vehicle |
| `tof_silent_bias_plus` / `tof_silent_bias_minus` | **加速度計のオフセット誤差 ±0.2 m/s² のまま約 1.1 m のホバーで ToF が出力停止**（`tof_silent_hover_1p2` に `bias 0 0 ±0.2 0 0 0` を ToF 停止の直前に足す、`--duration 31100000`。再上昇の再現）: 速度推定が何であれ推力は 支持推力 × 0.95 を超えないので**再上昇しない**（対策の無い main: +0.2 で最高 8.0 m、−0.2 で 2.9 m）。ToF 出力停止中の LANDING での 3 G の衝撃は非常でなく接地（`Touchdown while the ToF is silent (impact)`）。実測: 最高 1.099 m（停止時の高度）、接地速度 0.80 / 0.95 m/s | vehicle |
| `tof_silent_takeoff_bias_est_high` | **見積もり 28 % 高い + 加速度計のオフセット誤差 +0.2 m/s²（離陸の開始から）+ 自動離陸の上昇初期に出力停止**（`--duration 21000000`）: 未知が 2 つ同時（支持推力は較正できず、速度推定も誤る）。対策の無い main は 18.5 m まで上昇した。実測: 最高 0.80 m、接地速度 0.97 m/s、`emergency DISARM` なし | vehicle |
| `tof_silent_est_high_noise` | **見積もりが 35 % 高く（ノイズ n2）、自動離陸が天井 1.2 m を行き過ぎる**（`tof_silent_est_high` を `--noise n2`、`--duration 28000000`。1.68 m まで上がった場合の再現）: 高度の天井の実効化（1.35 m を超えたら上昇の指令を禁止し、上昇中は推力を見積もりの 0.5 倍以下）。実測: 最高 1.49 m（3 seed: 1.49 / 1.50 / 1.51 m。対策の無い main 2.7 m）。`.expect` は最高 < 1.55 m | vehicle |
| `tof_silent_stabilize_commloss` | **STABILIZE を ToF 出力停止のまま 21 s 飛んだ後に通信断**（`--duration 50000000 --noise n0`、スロットルは開ループで手調整）: 速度推定の最後のリセットは 24 s 前なので、降下の最初の周期から PI を使わず（`trusted -22 s more`）、支持推力（21 s のホバー近傍のサンプルで較正）× 0.95 の一定推力で降り、衝撃で接地する。実測: 飛行高度 0.65〜0.75 m、LANDING 33.8 s、接地速度 0.71 m/s。対策の無い main は `emergency DISARM` で終わる | vehicle |
| `tof_silent_stabilize` | **STABILIZE の手動飛行中に出力停止: 強制着陸しない**（`--duration 17000000`）: LANDING に入らず警告のみ（`NO forced landing, fly on`）、パイロットのスロットルで飛び続ける。出力停止中の POS_HOLD への切り替えは拒否（`Mode change to POS_HOLD refused`）。パイロットが着陸した後の ARM は拒否 | vehicle |
| `tof_silent_mode_refused` | **出力停止中の ALT_HOLD・POS_HOLD 拒否**（地上、`--duration 12000000`）: POS_HOLD スイッチは拒否（理由ログと `SystemMode.mode_block`）、ACRO は受理、拒否したエッジは持続し ToF 復帰で自動適用される | vehicle |
| `tof_silent_hover_timed` | 同、衝撃カットと接地検出器（衝撃の閾値と、2 つ目の経路の「床が機体を支えている」）を無効化（`--param safety.impact.accel_g=10 --param safety.blind_touchdown.accel_g=10 --param safety.blind_touchdown.floor_s=100`）して**ToF なしの着陸の時間**（10 s、LANDING に入った時刻から数える）を検証: LANDING 約 14.3 s → `touchdown by time` → IDLE_GROUND 約 24.3 s。`--duration 29000000` | vehicle |
| `tof_silent_prearm` | **ToF 出力停止中は ARM 拒否、復帰後は受理**（`--duration 15000000`）: 3.0 s に停止 → 7 s の ARM は `ARM rejected: downward ToF is silent`（`arm_block` = 7）→ 9.0 s に再開 → 10.5 s の ARM は受理（出力停止はレベル条件でラッチでない）。モータは回らない | vehicle |
| `tof_silent_armed_ground` | **出力停止の 0.2 s 後に押した ARM**（まだ出力停止と判定されない窓）が受理され、0.3 s に達した時点で ARMED_GROUND から DISARM される（TAKEOFF に入らない）。`--duration 12000000` | vehicle |
| `tof_recover_frozen_ground` | **離陸の前に ToF が出力停止し、レンジのはるか上で復帰**（`--duration 24000000`）: 5.0 s に停止（地上）、約 1.8 m まで ToF なしで上昇、8.6 s に復帰するが最初のサンプルは範囲外。対策の無い main には出力停止のエピソードの概念が無く、`Downward ToF silence episode` が出ない（固定されたままの接地フラグが再び信用される）。この対策では、生のフラグを「空中」へ倒し、推定器が ToF を取り込み直す（イノベーション 1.5 m）まで出力停止のエピソードを続け、真値 alt_max 2.5 m、二つ目の飛行（ALT_HOLD で ARM）は推定が真値の 0.1 m 以内 | vehicle |
| `tof_recover_edge_consumed` | **飛行中に ToF 出力停止中に拒否したモードスイッチが復帰の瞬間にパイロットの操作なしに適用されない**（`--duration 24000000`）: STABILIZE 飛行中の 9.0 s に POS_HOLD スイッチ → 拒否・エッジ消費（`IN FLIGHT the switch request is CONSUMED`）、11.0 s に復帰。対策の無い main は復帰の瞬間に POS_HOLD へパイロットの操作なしに入った。地上（墜落後）では同じ拒否は先送りで、エピソード終了で自動適用。墜落はパイロットのスロットル操作 | vehicle |
| `tof_recover_climb_alt` | **ToF なしの 4.5 s の上昇で推定が 0.77 m ずれて復帰**（純粋な形、`--duration 25000000`）: 復帰の最初のサンプルは有効（0.75 m）だがイノベーション 0.77 m が判定 0.5 m を超える。対策の無い main は全サンプルが棄却され alt_rmse 1.39 m。対策ありでは 0.02 m。ALT スイッチ（11.0 s）は推定器の取り込み直しの後に受理され高度を保持 | vehicle |
| `tof_recover_reflip` | **飛行中に拒否したスイッチは消費され、復帰後の入れ直しで適用される**（`--duration 24000000`）: ALT スイッチを ToF 出力停止中に入れ、復帰後も入れたままでは何も起きず（対策の無い main はパイロットの操作なしに ALT_HOLD になり、パイロットの OFF で STABILIZE に戻った）、9.6 s の入れ直しで ALT_HOLD が入り高度保持（推定 RMS 0.017 m） | vehicle |
| `tof_recover_invalid_first` | **サンプルは戻るが 1.5 s の間全て無効、その後に有効**（故障注入 `tof_invalid`、`--duration 27000000`）: 無効のまま届くサンプルは復帰でなく、エピソードは続く（`only invalid samples while flying`）。有効サンプルが戻ると推定器が取り込み直し（イノベーション 0.93 m）、推定は真値と一致（RMS 0.025 m） | vehicle |
| `tof_recover_prearm` | **復帰の 0.3 s 後の ARM は拒否、約 1 s 後の ARM は受理**（`--duration 14000000`）: 拒否の理由は `estimator has not re-acquired the ToF yet`（対策の無い main はサンプルが届いただけで受理） | vehicle |
| `tof_invalid_alt_hover` | **ホバー中（ALT_HOLD）に ToF が無効サンプルだけを出し続ける**（故障注入 `tof_invalid`、`--duration 27000000`）: 0.5 s 後に出力停止のエピソード（`only invalid samples while flying`）→ FLYING → LANDING → ToF なしの降下（接地速度 0.69 m/s）→ IDLE_GROUND。地上の無効は正常なので次の ARM は受理され、自動離陸は、上昇量の推定が 5 cm に達してから 0.15 s で打ち切られる（真値の最大高度 0.20 m。ノイズ n2 とホバー推力の誤差で 0.15〜0.41 m。打ち切りを「TAKEOFF 突入から 0.5 s」にする案は 0.16 m で済むが、離陸の遅い機体まで打ち切るので採らない。継続を 0.5 s のままにすると n2 で 0.99〜1.24 m） | vehicle |
| `tof_recover_landing_invalid` | **ToF なしの着陸の最中にサンプルが戻るが、全部が無効**（`--duration 22000000`）: LANDING の無効サンプルは一致でなく証拠として数える（最後の有効な高さ 0.5 m）ので、0.5 s 後に `only invalid samples while flying` と判定され、エピソードは接地まで続く。対策の無い main では約 1 s で取り込み直し済みと判定されて ToF なし降下が空中で放棄された | vehicle |
| `alt_ceiling_stick`・`alt_ceiling_stick_noise` | **ALT_HOLD でスロットルスティックを上げ続けても高度上限 1.2 m で止まる**（`--duration 29000000`、後者は `--noise n2 --seed 1`）: 上昇は上限へ向けて絞られ（0.45 1/s × 距離）、真値 1.17〜1.18 m（n2 の 3 seed で 1.185〜1.187 m）で止まり、スティックを下げれば降下できる。上限が無いと同じスティックで 1.97 m まで上昇した | vehicle |
| `landing_dead_zone` | **近距離の不感帯を模擬した、柔らかい接地（3 G 未満）の ToF 着陸**（`alt_disarm_land` に不感帯を入れたもの）。降下の最後の数 cm でセンサが無効になる（読みは真値に遅れ、真の高さ 0.038 m で 0.085 m）ので、LANDING の降下中に 0.10 m 以下の有効な読み（0.085 m）の後で無効サンプルだけが 0.2 s 続いたら接地とみなす（`TakeoffLandingMgr::deadZoneTouchdown()`、`detailed_design.md` §3 注7）。ログの `Landing detected (ToF dead zone ...)` を確認する。実測: 検出は最後の有効な読みの 0.2 s 後、真の高さ 0.021 m・鉛直速度 0.001 m/s で床の上。`alt_disarm_land` は `tof_deadzone 0` で DISARM の振り分けだけを検証する | vehicle |
| `tof_invalid_stabilize` | **STABILIZE 飛行中に無効サンプルだけが続く: 強制着陸しない**（`--duration 17000000`）: 警告のみ（`NO forced landing, fly on`）、飛行中の POS_HOLD スイッチは拒否・消費 | vehicle |
| `weak_spool_stabilize` | **STABILIZE で、モータが弱く床がまだ支える TAKEOFF は、ToF が床の上で無効を返すと、床の上で FLYING になる（許容）**（モータ健全度 0.9、`--duration 13000000`）: スロットルが離陸の閾値より上でホバーより下（生値 3100）の 3.5 s、機体は床の上にいて、ToF は最小レンジ未満（無効サンプルだけ）。TAKEOFF の 2 s + 0.15 s の後に ToF 出力停止のエピソード（`Ground contact unknown`）、`Takeoff complete → FLYING/STABILIZE`、`NO forced landing, fly on`（強制着陸しない）。その後パイロットのスロットルで浮く。5.0〜7.0 s の最大高度 < 0.05 m、浮いた後の最大高度 > 0.10 m を確認する（床の上で FLYING にしない仕組みは、加速度計に基づくため、ゆっくり浮く機体と区別できず採らない） | vehicle |
| `weak_spool_alt` | **ALT_HOLD の自動離陸で、浮けない機体（健全度 0.85）は、ToF が床の上で無効を返すと、ToF 出力停止として床の上で LANDING に入り、床が機体を支えると検出されてモータが止まる（許容）**（`--duration 16000000`）: 2 s + 0.15 s の後に `Downward ToF silence episode`・`TAKEOFF → LANDING`、ToF なしの降下の推力は支持推力の 0.95 倍以下なので浮かない。`Touchdown detected (the floor holds the craft up ...)`・`LANDING → IDLE_GROUND`、床の上の最大高度 < 0.05 m、離陸の完了も中止も無いことを確認する | vehicle |
| `weak_spool_alt_offset` | **`weak_spool_alt` に加速度計のオフセット（上向きの読み +0.3 m/s²）を足した変種**（`--duration 16000000`）: ここでの判断は加速度計に基づく上昇量に依らないので、期待値は `weak_spool_alt` と同じ | vehicle |
| `weak_spool_stabilize_offset` | **`weak_spool_stabilize` に同じオフセットを足した変種**（`--duration 13000000`）: 期待値は `weak_spool_stabilize` と同じ | vehicle |
| `weak_spool_api` | **浮けない機体（健全度 0.85）の API `takeoff`: ToF 無効の規則が離陸を LANDING で終わらせ、`takeoff` は `error takeoff aborted` を返す**（`--duration 18000000`）: 約 8.6 s に `TAKEOFF → LANDING`、`Touchdown detected (the floor holds the craft up ...)` でモータが止まり、床の上の最大高度 < 0.05 m、15.5 s 以降の duty_max < 0.01 を確認する（`error takeoff timeout` は出ない） | vehicle |
| `weak_spool_alt_floor` | **健全な ToF が床を有効に読み（`tof_deadzone 0`）、読みが増えない機体（健全度 0.85）の自動離陸は、5 s で中止して DISARM**（`--duration 16000000`）: 制御器の備え（3 s）が離陸完了を報告しても TAKEOFF に保ち（床の上で FLYING にならない）、約 10.8 s に `Auto-takeoff aborted: the healthy ToF has shown the craft on the floor for 5000 ms`、`TAKEOFF → IDLE_GROUND`、モータ停止（11.5 s 以降の duty_max < 0.01）、床の上の最大高度 < 0.05 m を確認する（床の上で FLYING になると不合格） | vehicle |
| `weak_spool_alt_slow_lift` | **ゆっくり浮く機体（健全度 0.88、`tof_deadzone 0`）は中止されず、離陸を完了して飛ぶ**（`--duration 25000000`）: ToF の読みが 0.02 → 0.17 m と増えるので床の証拠は無く、約 10 s に `Auto-takeoff complete`、`TAKEOFF → FLYING`、中止なし。浮いた後の最大高度 > 0.15 m、降下速度の最大 < 0.6 m/s を確認する（空中で DISARM して落下すると不合格。加速度計に基づく規則は 0.11〜0.17 m で DISARM するので採らない） | vehicle |
| `commloss_takeoff` | **完了しない TAKEOFF の間にリンクが途絶えると、ホバーの猶予（3 s）の後に LANDING になる**（STABILIZE、健全度 0.9、`tof_deadzone 0`、`--duration 22000000`）: ToF が床を有効に読むので ToF による空中の判定は偽のまま、手動の TAKEOFF は完了しない。5.5 s からの 8 s の無音で `Comm lost → hover 3000 ms then LANDING`、`Comm lost → LANDING (hover grace elapsed)`、`TAKEOFF → LANDING`（TAKEOFF のまま着陸しないと不合格） | vehicle |
| `stab_lowthrottle_lift_commloss` | **STABILIZE で、離陸の閾値（0.5）以下のスロットルで浮いた機体は、ARMED_GROUND にとどまらず TAKEOFF → FLYING になり、リンクが途絶えると着陸する**（`--duration 20000000`）: スロットルは生値 3060（正規化 0.494）、プラントには健全な機体を閾値以下で浮かせる手段が無いので、上向きの突風 `wind 0 0 -0.05 400`（0.4 s、0.05 N）が揚力を代用する。5.0〜6.0 s は床の上（高度 < 0.05 m）、突風の約 1.2 s 後（床を離れて約 1 s、5 cm を超える有効な ToF の読みが途切れず 1 s）に `ARMED_GROUND → TAKEOFF`、続いて `TAKEOFF → FLYING`、7.5 s からの 8 s の無音で `Comm lost → LANDING (hover grace elapsed)`、`FLYING → LANDING`、`LANDING → IDLE_GROUND`。7.0〜9.0 s の最大高度 > 0.3 m（実測 0.78 m）。浮上の証拠が無いビルドでは遷移が 1 つも起きず不合格 | vehicle |
| `stab_lowthrottle_lift_tofsilent` | **同じ浮上の最中に ToF が出力停止しても、モータは止まらない**（`--duration 16000000`）: 7.6 s に `tof_silent 1`（機体は約 0.5 m 上）。FLYING・STABILIZE の出力停止は警告だけ（`NO forced landing, fly on`）で、`ARMED_GROUND → IDLE_GROUND` が出ず、8.5〜10.0 s のモータ duty の最大 > 0.3（実測 0.63）。浮上の証拠が無いビルドでは ARMED_GROUND の出力停止が DISARM になり、duty が 0 で不合格 | vehicle |
| `stab_lowthrottle_floor` | **床の上でスロットルが閾値より下なら、ToF が床の上で何を読んでも ARMED_GROUND にとどまる**（`--duration 28000000`）: 生値 2900〜3060（0.416〜0.494）で、小さなロール・ピッチ・ヨーのスティックを含めて 22 s、前半は不感帯あり（無効サンプルだけ。実際の床）、後半は `tof_deadzone 0`（床で 0.021 m の有効な読み）。`ARMED_GROUND → TAKEOFF` も `Takeoff detected` も出ず、全区間の最大高度 < 0.05 m（実測 0.0239 m）、最後のパイロットの DISARM が `ARMED_GROUND → IDLE_GROUND`。浮上の証拠の誤判定の検出用 | vehicle |
| `landing_rise_est_high` | **サポート推力の較正の前に始まる LANDING で、ホバー推力の見積もりが 28 % 高い（`hover.thrust_corr 1.2`、ノイズ n2）: 機体は再上昇しない**（`--duration 28000000 --noise n2`）: API の `takeoff` の 6 s 後の `land` で LANDING が 12.0 s に始まる。基準が見積もり（機体を支える推力の約 1.28 倍）のままだと、1.19 m から 1.98 m まで上昇し、ToF の範囲外の ToF なしの降下に入って 2.20 m/s（真値の鉛直速度の、床への最初の接触の瞬間 = 高さ 3 cm 未満の値。`descent_speed_max` も同じ量）で接地した。基準を支持推力の基準（較正前は早い推定）の 1.08 倍までに抑え（`landingBaseThrust()`）、上昇中は推力を支持推力の基準の割合以下に制限する（`brakeRiseInLanding()`）。`.expect` は LANDING 後の `alt_max` < 1.35 m（実測 1.19 m）・`descent_speed_max` < 1.0 m/s（実測 0.38 m/s）・`Blind landing` が無いこと（基準を抑えないと不合格） | vehicle |
| `landing_low_start` | **低い高さ（0.117 m）で始まる LANDING は、始まった周期にモータが止まらず、床で終わる（沈む機体を静止と見なさない）**（`--duration 26000000`）: モータの健全度 0.8725 の機体が TAKEOFF で低くホバーし、リンクが 12 s 途絶える。LANDING は 11.02 s に 0.117 m（接地フラグのヒステリシス帯の中）で始まり、機体は 0.133 m で向きを変え、最大 0.58 m/s で沈み、LANDING の 0.5 s 後の 11.52 s に衝撃のカット（`Impact/anomaly → emergency DISARM`）で真の高さ 0.017 m でモータが止まる（接地フラグが立ったまま推定の鉛直速度が 0 に固定されているので、静止は ToF の読みで判定する: 読みの区間が動くので降下停滞の判定は出ない。注7）。`.expect` は LANDING の 0.1〜0.4 s 後の `duty_max` > 0.1（実測 0.63）、`Landing detected (stalled descent` が無いこと、最後は床の上（`alt_max` 0.021 m）。**このシナリオは、判定のやり直しそのものは検査しない**（やり直しと刻印の確認を外しても合格する）: 以前から残った判定（接地フラグが立ち、機体が静止し、0.05〜0.15 m）を持つ FLYING・TAKEOFF を作る入力は、SILS では見つからなかった（健全度 0.80〜0.8825 の 17 通りで試した。0.865 以下は浮かずに TAKEOFF が中止されるか、0.07 m 未満で LANDING になり、0.8725 以上は LANDING の後に判定が出るのが降下の途中〔停滞または衝撃〕で、持ち越しの判定は現れなかった）。やり直しと刻印の確認はホスト試験 `land_notice_*`（`test_main`）と `a_touchdown_verdict_counts_only_with_the_stamp_of_the_present_landing`（`test_takeoff_policy`）が検査する。静止し続ける機体の降下停滞の判定はホスト試験 `land_low_hover_with_a_held_estimate_ends_by_the_stalled_descent`（`test_main`）が検査する | vehicle |
| `landing_low_hover` | **定常の低いホバー（0.106 m）で始まる LANDING は、有限の時間で床の上で終わる**（`--duration 26000000`）: ALT_HOLD の自動離陸の後、スロットルスティックを中央より下に 3.7 s 保って 0.106 m（鉛直速度 -0.003 m/s）のホバーにし、パイロットの DISARM で自動着陸。LANDING は 20.72 s、LANDING → IDLE_GROUND は 21.44 s（0.72 s）、真の高さ 0.020 m。接地フラグは解けている（0.15 m を超えたことがある）ので、推定の鉛直速度は有効で、判定は従来どおり（変更の前後で時刻・高さが同じ）。床を離れたことのないホバーで接地フラグが立ったままの場合は SILS では作れなかった（健全度 0.85〜1.0 の通信途絶 216 通りのうち、0.07〜0.13 m で始まった LANDING は 1 通りで、衝撃のカットで終わる）ので、ホスト試験 `land_low_hover_with_a_held_estimate_ends_by_the_stalled_descent` が検査する | vehicle |
| `landing_ground_effect_float` | **地面効果が床の上に留める LANDING は、降下停滞の判定で終わる**（`--duration 25000000 --ground-effect`）: `alt_disarm_land` の自動着陸をプラントの地面効果モデルを入れて行う。機体は床の 0.045 m 上（真の高さ。床は 0.021 m）で止まり、`Landing detected (stalled descent / ground effect)`、15.62 s に LANDING → IDLE_GROUND。変更の前後で時刻・高さが同じ | vehicle |
| `takeoff_disarm_stops` | **高度モードの TAKEOFF 中のパイロットの DISARM は、即モータ停止（着陸要求ではない）**（`--duration 16000000`）: ALT_HOLD の自動離陸、モータの健全度 0.85（浮けない）、`tof_deadzone 0`（ToF が床を有効に読む）。TAKEOFF の中の約 8.6 s に ARM ボタンをもう一度押す（トグル）。`DISARM accepted (TAKEOFF → IDLE_GROUND)` と `TAKEOFF → IDLE_GROUND`。`TAKEOFF → LANDING`・システム自身の中止（`Auto-takeoff aborted`、約 10.8 s）は起きない。`alt_max` 0.0212 m、9〜16 s の `duty_max` 0.0000 | vehicle |
| `battery_emergency_takeoff_floor` | **床の上の自動離陸の最初に電池の EMERGENCY が出ると、すぐモータを止める（LANDING にしない）**（`--duration 16000000`）: ALT_HOLD、`hover.thrust_corr` 1.3（学習なし）。TAKEOFF の開始（約 7.82 s）の 0.1 s 後に事象 `battery 2.99`（危険閾値のすぐ下）。機体はまだ床の上で、`Battery emergency in TAKEOFF without evidence of the air ... → DISARM`、`DISARM accepted (TAKEOFF → IDLE_GROUND)`、`TAKEOFF → IDLE_GROUND`。`Battery emergency → LANDING`・`TAKEOFF → LANDING`・`TAKEOFF → FLYING` は出ない。`alt_max` 0.0215 m、8.5〜16 s の `duty_max` 0.0000。規則を外すと `Battery emergency → LANDING` が出て不合格 | vehicle |
| `battery_emergency_takeoff_airborne` | **自動離陸の空中で電池の EMERGENCY が出ると LANDING になり、静かに接地する**（`--duration 22000000`）: 上と同じ離陸、事象は TAKEOFF の開始の 0.4 s 後（約 0.2 m、ToF は帯 5 cm より上を読む）。`Battery emergency → LANDING`、`TAKEOFF → LANDING`、`LANDING → IDLE_GROUND`。`without evidence of the air`・`TAKEOFF → IDLE_GROUND`・`TAKEOFF → FLYING` は出ない。`alt_max` 0.386 m（0.6 m 未満）、20〜22 s は床の上でモータ停止（`alt_max` 0.0211 m、`duty_max` 0.0000）、接地速度 `descent_speed_max` 0.422 m/s（0.7 m/s 未満） | vehicle |
| `battery_emergency_flying` | **FLYING の電池の EMERGENCY は LANDING**（これまでシナリオで確かめられていなかった規則）（`--duration 24000000`）: ALT_HOLD の自動離陸、約 9.2 s に FLYING、13.0 s に事象 `battery 2.99`。`TAKEOFF → FLYING` の後に `Battery emergency → LANDING`、`FLYING → LANDING`、`LANDING → IDLE_GROUND`。11〜13 s の `alt_mean` 0.691 m、22〜24 s は床の上でモータ停止、`descent_speed_max` 0.551 m/s（0.7 m/s 未満） | vehicle |
| `battery_emergency_takeoff_floor_manual` | **手動（STABILIZE）の TAKEOFF の床の上の電池の EMERGENCY も、すぐモータを止める**（`--duration 12000000`）: ARM（4.0 s）、5.0 s にスロットル 3171（離陸の閾値超）で TAKEOFF、その 0.06 s 後に `battery 2.99`。`Takeoff detected` の後に `Battery emergency in TAKEOFF without evidence of the air ... → DISARM`、`TAKEOFF → IDLE_GROUND`。`Battery emergency → LANDING`・`TAKEOFF → LANDING` は出ない。`alt_max` 0.0212 m、6〜12 s の `duty_max` 0.0000。規則を外すと不合格 | vehicle |
| `battery_emergency_takeoff_airborne_manual` | **手動（STABILIZE）の TAKEOFF で、機体が床を離れた後の電池の EMERGENCY は LANDING**（`--duration 14000000`）: スロットル 3300、TAKEOFF の 0.5 s 後（真の高さ約 0.08 m、ToF は帯 5 cm より上、TAKEOFF は未完了）に `battery 2.99`。`Battery emergency → LANDING`、`TAKEOFF → LANDING`、`LANDING → IDLE_GROUND`。`without evidence of the air`・`TAKEOFF → IDLE_GROUND` は出ない。`alt_max` 0.1225 m、12〜13.5 s は床の上でモータ停止 | vehicle |
| `battery_emergency_takeoff_climbing` | **低い高さを上昇中の自動離陸で電池の EMERGENCY が出ても、LANDING は空中で終わらない**（`--duration 22000000`）: `battery_emergency_takeoff_airborne` の離陸で `hover.thrust_corr` 1.1（学習なし）。LANDING は 8.32 s に 0.083 m を 0.2 m/s で上昇中に始まる。接地フラグは立ったまま、推定の鉛直速度は 0 に固定される。静止を ToF の読みで判定するので降下停滞の判定は出ず、モータは床まで回る（8.85〜9.2 s の `duty_max` > 0.1、実測 0.84。接地速度 0.65 m/s）。判定を外すと 8.76 s に 0.187 m の空中でモータが止まり、接地速度 1.72 m/s で不合格 | vehicle |
| `battery_emergency_takeoff_apex` | **上昇の頂点で始まる LANDING も空中で終わらない**（`--duration 22000000`）: `hover.thrust_corr` 1.0、事象は TAKEOFF の開始の 0.7 s 後。LANDING は 8.62 s に 0.135 m で、上昇（+0.2 m/s）から降下（-0.38 m/s）へ向きを変える機体で始まる。SILS の ToF は 0.4 s の間 0.107〜0.123 m（広がり 0.016 m、静止の幅 0.02 m の内側）と読むので、幅だけでは足りず、保持時間 1 s（`kLandingRestHoldMs`）が効く。モータは床まで回る（9.10〜9.18 s の `duty_max` > 0.1、実測 0.79。接地速度 0.58 m/s）。判定を外すと 9.06 s に 0.086 m の空中でモータが止まり、接地速度 0.93 m/s で不合格 | vehicle |
| `battery_emergency_takeoff_climbing_manual` | **手動（STABILIZE）の TAKEOFF でも、低い高さを上昇中に始まる LANDING は空中で終わらない**（`--duration 14000000`）: `battery_emergency_takeoff_airborne_manual` の離陸で `hover.thrust_corr` 1.2。LANDING は 5.52 s に 0.081 m を 0.3 m/s で上昇中に始まり、モータは床まで回る（6.05〜6.3 s の `duty_max` > 0.1、実測 0.81。接地速度 0.54 m/s）。判定を外すと 6.00 s に 0.160 m の空中でモータが止まり、接地速度 1.35 m/s で不合格 | vehicle |
| `api_land_in_takeoff` | **`takeoff` の待機中の `land`（浮けない機体）は後回しになり、LANDING は始まらない。中止でモータが止まり、後回しの `land` は `error not flying`**（`--duration 22000000`）: モータの健全度 0.85、`tof_deadzone 0`。API `takeoff` の約 2 s 後（TAKEOFF の中）の `land` は `cmd: land (during a blocking command` を出さず、`Landing requested` も `TAKEOFF → LANDING` も出ない。約 11.3 s にシステム自身の中止（`Auto-takeoff aborted`、`TAKEOFF → IDLE_GROUND`）、`reply: error takeoff aborted`、続けて後回しの `land` が `reply: error not flying`。`alt_max` 0.024 m、13〜22 s の `duty_max` 0.0000。`land` が TAKEOFF の間に LANDING を始めると不合格 | vehicle |
| `api_land_deferred_in_takeoff` | **`takeoff` の上昇中の `land` は後回しになり、離陸が完了してから実行される**（`--duration 22000000`）: API `takeoff` の約 0.9 s 後の `land`。`TAKEOFF → FLYING`（`takeoff` は `ok`）の後に `cmd: land`、`FLYING → LANDING`、`LANDING → IDLE_GROUND`。`TAKEOFF → LANDING` は出ない。離陸の最高高度は `land` が無い離陸と同じで、`alt_max` は 0.45 m 超（6〜8 s、実測 0.54 m）・0.70 m 未満、最後は床の上でモータ停止 | vehicle |
| `api_land_in_armed_ground` | **高度モードの ARMED_GROUND（スプール中）での `land`: 即時 DISARM、後回しの行を捨てる**（`--duration 12000000`）: `takeoff` の 30 ms 後に 2 つ目の `takeoff` と `height?` を後回しにし、50 ms 後に `land`。モータは TAKEOFF に入らず、`land` は `ok`、待っていた `takeoff` と後回しの 2 行は `error cancelled by land`。`alt_max` < 0.05 m、6.5 s 以降の `duty_max` < 0.01 | vehicle |
| `api_land_manual_armed_ground` | **手動モード（STABILIZE）の ARMED_GROUND での `land` は拒否**（`--duration 14000000`）: `error not flying`、DISARM しない（`Landing requested on the floor` が出ない）。手動モードはスロットルだけで浮けるので、床の上とは限らない | vehicle |
| `api_land_tof_never_valid` | **下向き ToF が一度も有効な高さを読まない機体の TAKEOFF 中の `land` は後回しになり、ToF の規則が離陸を ToF なしの着陸で終わらせる**（`--duration 16000000 --noise n2`、`hover.thrust_corr 1.2`、加速度計のオフセット +0.3 m/s²、`tof_invalid 1`）: `land` は `takeoff` の 0.7 s 後で、機体は上昇中（高さ約 0.2 m）。`cmd: land (during a blocking command` も `Landing requested` も出ず、ToF 無効の規則（`Downward ToF unusable`）が `TAKEOFF → LANDING`、LANDING は ToF なしの降下（`Blind landing: ToF silent`）、床での降下速度は 1.5 m/s 未満（実測 0.9 m/s）、最後は床の上でモータが止まる |
| `ceiling_brake_stale_velocity` | **長い ToF 出力停止（オフセット −0.4 m/s²）の直後に ALT_HOLD へ入っても、天井ブレーキが古い加速度積分を信用しない**（`--duration 36000000`）: 積分は 3.7 m/s（上昇していない機体で）。古い積分を信用すると、推力が重さの半分になり、機体は床へ 2.57 m/s で降りた（`alt_min` 0.01 m）。信用しないと `alt_min` 1.06 m、降下速度の最大 0.19 m/s | vehicle |
| `ceiling_brake_stale_velocity_alt` | **同上、1.24 m（天井）で ALT_HOLD へ入る**（`--duration 30000000`）: 古い積分を信用すると 0.59 m まで落下（`alt_min` 0.59 m）。信用しないと 1.18〜1.26 m を保つ | vehicle |
| `tof_silent_flip_end` | **宙返りの最中に ToF が出力停止: 宙返りが終わった瞬間に判断**（`--duration 33000000 --param flip.enable=1`）: FLIP → FLYING の「その場で」LANDING（`ToF silence episode during the maneuver: decided now that FLIP is over`）、宙返りは中断されない（真値 -358 deg） | vehicle |
| `api_takeoff_emergency` | **`takeoff` の待機中に届いた `emergency` が即座に効く**: 6.0 s に takeoff、7.0 s（上昇中）に `emergency` → 7.04 s にモータ停止、`takeoff` は `error interrupted by emergency`。対策の無い main は待機中に `emergency` を読まず、7.5〜13 s の間もモータが回る（`duty_max` 0.65）。さらに待機中に届いた 二つ目の `takeoff`・`height?`・`rc` を **`emergency` が捨てる**（`error cancelled by emergency`、`rc` はその場で捨てる）ので再離陸しない（モータ停止が 7.5〜13 s 続く）。`--duration 14000000` | vehicle |
| `api_land_emergency` | **`land` の待機中の `emergency`**: 降下開始 0.6 s 後（16.6 s）に注入 → 16.64 s にモータ停止。対策の無い main は接地まで待たされ、17.1〜20.6 s の間もモータが回る（`duty_max` 0.66）。待機中に届いた `takeoff`・`left 100` は捨てられ再離陸しない。`--duration 22000000` | vehicle |
| `api_land_stop` | **`land` の待機中の `stop`**: `stop` は着陸中は意味を持たないので通常の拒否 `error not flying or guidance released` を接地を待たず直ちに返し、着陸は続行して接地する。`--duration 22000000` | vehicle |
| `disarm_long_idle` | **38 分の無音 DISARM のあとも次の飛行が壊れない**（`--duration 2300000000`、仮想時間なので実時間は約 3 分）: RC リンクを DISARM のまま 38 分（> 35.8 分）無音にし、再開後に ALT_HOLD の自動離陸→ホバー（0.5 m）→DISARM 自動着陸。出力停止の監視器が「停止」に固着しない（ARM 不可・誤った `Comm lost → LANDING` が起きない）ことを守る。途絶したリンクを生存と読む危険側はホスト試験 `test_sensor_freshness`（36/40/60 分の観測の空き）が担う | vehicle |
| `api_move_stop` | **`forward 150` の待機中の `stop`**: 開始 1.0 s 後に `stop` → 移動は `error interrupted by stop` で終わり、最大 0.14 m 進んだところで制動（対策の無い main は待機中の `stop` を読まず、1.48 m 全行程を進んでから止まる）。待機中に届いた `left 50` は `stop` が捨てる（`error cancelled by stop`、実行されない）。`--duration 26000000` | vehicle |
| `api_rotate_large_angle` | **`cw 270`・`ccw 360`・`cw 360` が指令の向きに指令の角度だけ回る**: 窓ごとの真値方位の符号付き総変化 `yaw_turn_deg` が +270 / −360 / +360 度（±9 度）。修正前は `cw 270` が逆向きに 90 度、`cw 360` は即 `ok` で 0 度。`--duration 74000000` | vehicle |
| `api_rotate_direction` | **`ccw 270`・`cw 180`・`ccw 180` が指令の向きを保つ**: `yaw_turn_deg` が −270 / +180 / −180 度（±9 度）。180 度の回頭に最短の向きは無く、向きは指令で決まる。`--duration 74000000` | vehicle |
| `api_rotate_accumulate` | **`cw 90` の 4 回が合計でちょうど 1 周**: 各 90 度（±9 度）、4 回の合計 360 度（±2 度）。回頭は目標に累積し、推定値からのずれを持ち越さない。`--duration 56000000` | vehicle |
| `api_rotate_ok_when_complete` | **`cw 360` は機体が 1 周してはじめて `ok` を返す**: 同時に送った `down 30` は回頭の後ろに置かれ、回頭中（t=23〜29 s）の最低高度が 1.10 m を上回り、回頭の後に降下が実行される。修正前は `ok` が直ちに返り、回頭中に 0.9 m まで降りた。`--duration 48000000` | vehicle |
| `api_rotate_stop_midturn` | **`cw 360` の途中（172 度）の `stop`**: 待っていた回頭は `error interrupted by stop`、向きは保持（その後 3 s の変化 1.5 度以内）、続く `cw 90` は保持した向きから +90 度、`ccw 90` が戻す。`--duration 52000000` | vehicle |
| `api_rotate_land_midturn` | **`ccw 270` の途中の `land` は回頭の後ろに置かれる**（`cw 90` でも同じ経路）: 回頭は -270 度（許容差 9.7 度以内）まで続いてから着陸する。`--duration 44000000` | vehicle |
| `api_rotate_emergency_midturn` | **`ccw 270` の途中の `emergency`**: モータ即停止（FLYING → IDLE_GROUND）、`error interrupted by emergency`、それ以上回らない（約 -148 度）。`--duration 34000000` | vehicle |
| `api_wait_deferred` | **待機中に届いた `emergency`/`stop` 以外のコマンドは捨てず順序も変えない**: `takeoff` の待機中に `height?` と 二つ目の `takeoff` を注入 → FLYING の後にこの順で実行され、二つ目は `error not on ground`。対策の有無で結果は変わらない。`--duration 20000000` | vehicle |
| `api_invalid_numbers` | **有限でない・範囲外の数は誘導目標に届かない**: ホバー中に `forward nan`・`forward 1e39`・`go inf 0 0 50`・`rc nan 0 0 0`・`speed nan`・`cw 1e39`・`cw 9999999999`・`ccw 0`・`ccw 361`・`sysid roll chirp 25 600`・`sysid roll chirp nan 4`・`autotune roll nan` を送る → 全て `error out of range`（`rc` は応答なし）、機体は動かない（水平 0 m・高度の幅 0.004 m・ヨー 0）、続く `forward 50` は通常の移動になる。`--duration 32000000` | vehicle |
| `api_sysid_emergency` | **`sysid` の待機中の `emergency`**: `sysid roll chirp 25 8` の励振の 1.5 s 後に `emergency` → モータが直ちに止まり、`sysid` は `error interrupted by emergency`、後回しの `height?` は取り消される。`--duration 22000000` | vehicle |
| `api_move_flight_end` | **移動の待機中に飛行が終わる**: `forward 150` の待機中にパイロットの DISARM 押下（POS_HOLD の自動着陸）→ 待機は `error flight ended` で終わり、後回しの `takeoff`・`height?` は `error cancelled: flight ended` で捨てられ、再離陸しない。`--duration 40000000` | vehicle |
| `api_emergency_without_command` | **`command` なしの `emergency`**: 送信機の ARM（ARMED_GROUND）の後、`land` は `error not in command mode`、`emergency` はモータを止める（`EMERGENCY STOP`）。`--duration 12000000` | vehicle |
| `api_takeoff_refused_mode` | **ARM が拒否された API `takeoff` はモードを残さない**: `--unpaired` でペアリングが開いたままの機体に `takeoff` → POS_HOLD への切り替えと ARM の拒否の後、送信機の電文より前に STABILIZE へ戻る（"Mode re-leveled" なし）。後の送信機の ARM は受理され、自動離陸は始まらない。`--unpaired --duration 16000000` | vehicle |
| `api_takeoff_slow_target` | **上昇が遅く、時間切れで FLYING になった機体の API `takeoff` は、到達した高度でなく自動離陸の目標高度（0.5 m）を保持する**（モータの健全度 0.88・`tof_deadzone 0`、`--duration 50000000`）: 約 11.5 s に機体が約 0.23 m で `TAKEOFF → FLYING`、`takeoff` は `ok` を返し、誘導目標は `-0.50`（NED 下向き）。機体は約 0.79 m まで行き過ぎ（上限 1.2 m に触れない）、目標へ数十秒かけて戻る。FLYING 付近の最大高度 < 0.40 m（時間切れの場合であること）、12〜20 s の最大高度 < 0.95 m（実測 0.79 m）、整定後の 40〜49 s の平均高度 0.45〜0.55 m（実測 0.483 m）・最大高度 < 0.55 m（0.496 m）・最小高度 > 0.40 m（0.462 m）を確認する（到達した高度を目標にすると同じ区間で 0.17 m に留まり不合格） | vehicle |

**P8 ロバスト再飛行（`crash_refly`）が炙り出した2つのファーム欠陥（修正済）:**
1. **ESKF 姿勢の latch**: 墜落で姿勢推定が真値から大きく外れると accel-attitude χ² 判定（カイ二乗判定）が補正自体を棄却し続け自己復帰しない。設置時（IDLE_HELD→IDLE_GROUND、機体が level・静止と既知）に ESKF を Reset して姿勢を level へ再初期化することで解決。
2. **モード未伝播**: 接地時の飛行モード STABILIZE リセットが `StateManager::mode_` を変えるだけで制御器に伝わらず（制御器は `ControllerCmd::ModeChange` 経由でのみモードを知る）、ALT/POS 飛行後の再離陸が古いホバー推力モードのまま上昇しない。リセット時に onModeChange を作動させて解決。

### 飛行安全の改修の評価（ToF の出力停止・無効、床の積極的な証拠、LANDING、TAKEOFF）

#### 1. 何を確かめる試験か

| 確かめること | 条件 | 合格の見方 | 試験 |
|-------------|------|-----------|------|
| 床の積極的な証拠による TAKEOFF の中止・完了 | ALT_HOLD / POS_HOLD の自動離陸。(a) 浮けない機体（モータの健全度 0.85）で ToF が床を有効に読む → 5 s で DISARM。(b) ゆっくり浮く機体（健全度 0.88）→ 中止せず離陸を完了。(c) 読みが 5 cm の帯をまたぐ → ラッチが立った後だけ、5 s 後に LANDING | 床の上で FLYING にならない。ゆっくり浮く機体を空中で DISARM しない | `weak_spool_alt_floor`、`weak_spool_alt_slow_lift`、ホスト試験 `test_takeoff_policy`（24 本。全セルと、ラッチの出口・無効サンプルの間に保持時間が過ぎる場合） |
| ToF 無効の規則（TAKEOFF・FLYING・LANDING） | ToF が無効サンプルだけを返す: 床の上から、ホバー中、LANDING の最中、TAKEOFF の最中に `land` を出したとき | 空中では 0.15〜0.5 s で出力停止と判定して ToF なしの降下に移る。地上・床の近くの無効サンプルは正常として数えない | `weak_spool_alt`・`weak_spool_alt_offset`・`weak_spool_api`・`weak_spool_stabilize`・`weak_spool_stabilize_offset`、`tof_invalid_alt_hover`、`tof_invalid_stabilize`、`tof_recover_landing_invalid`、`tof_recover_invalid_first`、`api_land_tof_never_valid`、ホスト試験 `test_tof_health`（31 本） |
| ToF なしの降下は上昇できない | ToF が出力停止: ホバー中、自動離陸の上昇中、API 離陸の最中、ホバー推力の見積もりが 19〜35 % 高い、加速度計のオフセット ±0.2 m/s² | 出力停止から接地までの上昇が小さく、接地速度が 1〜2 m/s 以内、空中でモータが止まらない | `tof_silent_*`、ホスト試験 `test_blind_landing` |
| 接地の検出 | 柔らかい接地（3 G 未満）、不感帯、衝撃のカットを無効にした接地、低い高さで始まる LANDING | LANDING は床で終わり、空中では終わらない | `landing_dead_zone`、`landing_low_start`、`tof_silent_touchdown_detect`、`tof_silent_hover_timed`、ホスト試験 `land_*`（`test_main`） |
| 高度の上限と LANDING の再上昇 | ALT_HOLD でスティックを上げ続ける、ホバー推力の見積もりが 28 % 高い LANDING、古い加速度積分のまま ALT_HOLD へ入る | 上限 1.2 m で止まる。LANDING の後に再上昇しない | `alt_ceiling_stick`・`alt_ceiling_stick_noise`、`landing_rise_est_high`、`ceiling_brake_stale_velocity`・`ceiling_brake_stale_velocity_alt` |
| 手動モードの ARMED_GROUND から空中へ（スロットル以外の証拠） | STABILIZE で、スロットルが離陸の閾値以下のまま機体が浮く（上向きの突風）。浮かずに床の上でアイドル・スプールアップ・小さなスティック | 健全な ToF が 5 cm を超える有効な高さを 1 s 途切れずに読むと TAKEOFF → FLYING（通信断の着陸と ToF 出力停止の扱いが空中の規則になる）。床の上では ARMED_GROUND に留まる | `stab_lowthrottle_lift_commloss`、`stab_lowthrottle_lift_tofsilent`、`stab_lowthrottle_floor`、ホスト試験 `test_takeoff_policy` |
| TAKEOFF 中の操作 | パイロットの DISARM、API `land`、通信途絶 | DISARM は即モータ停止。API `land` は後回しで、離陸が終わってから実行される（TAKEOFF では LANDING にならない）。通信途絶は LANDING | `takeoff_disarm_stops`、`api_land_in_takeoff`、`api_land_deferred_in_takeoff`、`commloss_takeoff` |
| TAKEOFF 中の電池の EMERGENCY | ALT_HOLD / POS_HOLD の自動離陸と手動（STABILIZE）の離陸。電池電圧を 2.99 V に保つ事象 `battery` を、機体が床の上のとき・床を離れた後・FLYING の間に置く。格子: TAKEOFF の開始の 50〜2000 ms 後 × `hover.thrust_corr` 0.9〜1.3 × ToF の不感帯あり・なし × ノイズなし・n2 × 加速度計のオフセット 0・±0.4 m/s²（モードと電圧ごとに 336 通り、`detailed_design.md` §3.1 の「TAKEOFF の電池の EMERGENCY のセル」） | 床の上（真の高さ 5 cm 以下）で出た EMERGENCY は、LANDING が浮き上がる・空中でモータが止まる 9 通りが 0 通りになる（DISARM）。床を離れた後に出たものは LANDING で終わる（低い高さから上昇中に始まる LANDING も、接地検出器が静止を ToF の読みで判定する（注7）ので、空中でモータは止まらない（格子 1344 通りのうち LANDING に入った 984 通りで、真の高さ 0.05 m 超でのモータ停止は 56 通りから 0 通り））。FLYING は LANDING | `battery_emergency_takeoff_floor`、`battery_emergency_takeoff_airborne`、`battery_emergency_flying`、`battery_emergency_takeoff_floor_manual`、`battery_emergency_takeoff_airborne_manual`、`battery_emergency_takeoff_climbing`・`battery_emergency_takeoff_apex`・`battery_emergency_takeoff_climbing_manual`、ホスト試験 `test_takeoff_policy`・`test_state_manager` |
| API の待機中の割り込み | `takeoff`・`land`・`forward` の待機中の `emergency`・`stop`・`land`・その他の行 | `emergency` と ARMED_GROUND の `land` は後回しの行を捨てる。手動モードの ARMED_GROUND の `land` は拒否 | `api_takeoff_emergency`、`api_land_emergency`、`api_land_stop`、`api_move_stop`、`api_wait_deferred`、`api_land_in_armed_ground`、`api_land_manual_armed_ground`、`api_move_flight_end`、`api_sysid_emergency` |

各シナリオの条件の書き下しは、上の「その他のシナリオ」の表と、各 `.scn` のヘッダにある。

#### 2. main との比較

比較の相手「main」は、これらの対策を入れる前の main のファームウェア。ToF 無効の注入（`tof_invalid`・`tof_deadzone`）などのシナリオ入力は SILS 側の機能なので、main のファームウェアにも同じシナリオを与えた。数値はすべて SILS（物理真値）で、実機は未確認。

##### TAKEOFF の間や直後に `land` を出す場合

`land` は TAKEOFF の間は後回しになり、離陸が終わってから実行される（`api_land_deferred_in_takeoff`）。次の組は、ToF の状態・加速度計のオフセット・ホバー推力の見積もりの誤差・`land` の時刻を振った（すべて `--noise n2` を含む。P は `--noise` なしも含む）。

| 組 | 条件 | 本数 |
|----|------|------|
| J | ToF 正常（不感帯あり）。API `takeoff` の 3 s 後（FLYING）に `land`、LANDING の 0.45〜0.825 s 後に上向きの突風（0.5・0.6 N、15・30 ms）。`hover.thrust_corr` 1.0 / 1.2、加速度計のオフセット +0.6 m/s²、モータの効率 0.9 / 1.0、seed 3 | 384（main は 379） |
| P | ToF 正常。`land` は API `takeoff` の 50〜2000 ms 後（50〜300 ms は ARMED_GROUND のスプール中で即時 DISARM、500〜2000 ms は TAKEOFF の中で後回し）。`hover.thrust_corr` 0.9 / 1.0 / 1.2 / 1.3、ノイズなし / n2 | 108 |
| L | API `takeoff` の 6 s 後に `land`。加速度計のオフセット −0.6〜+0.6 m/s²、`hover.thrust_corr` 0.9 / 1.0 / 1.2、モータの効率 0.9 / 1.0、ノイズなし / n2（seed 3） | 120 |
| B | ToF が起動時から無効（`tof_invalid 1`）。`land` は `takeoff` の 20 / 450 ms 後。オフセット ±0.6〜±1.0 m/s²、モータの効率 0.9 | 18 |
| X | B と同じで、オフセット −1.5〜+1.5 m/s² を 0.1 刻み、モータの効率 0.9 / 1.0、seed 2 | 248 |
| V | ToF が起動時から無効。`land` は `takeoff` の 650〜1300 ms 後、および `takeoff` を 2 回出してから `land`。ノイズなし / n2 | 10 |
| T1 | ToF が起動時から無効。`land` は `takeoff` の 0.3〜2.0 s 後。オフセット −0.6〜+0.3 m/s²、`hover.thrust_corr` 1.0 / 1.2、モータの効率 0.9 / 1.0 | 128 |
| T2 | TAKEOFF の 0.35〜0.65 s 後に ToF が無効になり、0.1〜0.4 s 後に `land`。オフセット −0.4 / 0 / +0.4、不感帯 あり / なし | 108 |
| T3 | ToF が起動時から無効。`land` は 0.35〜0.9 s 後。`hover.thrust_corr` 0.9 / 1.0 / 1.3、オフセット −0.6 / 0 / +0.6、モータの効率 0.85〜1.0 | 324 |
| T3b | 不感帯あり。TAKEOFF の 0.33〜0.51 s 後に ToF が無効になり、0.05 / 0.15 s 後に `land` | 80 |
| T4 | 手動モード（STABILIZE）の ARMED_GROUND で、スロットルを 2900〜3090 に保って `land` | 12 |
| T5 | ToF は正常（不感帯 あり / なし）。`land` は `takeoff` の 0.02〜2 s 後。`hover.thrust_corr` 0.9 / 1.2 / 1.3、オフセット ±0.6、モータの効率 0.85 / 1.0 | 408 |
| T6 | T3 と同じ ToF 無効、オフセット ±1.0 / ±1.5 m/s² | 72 |
| T7 | ToF 正常（不感帯あり）、`hover.thrust_corr` 1.3。`land` が ARMED_GROUND と TAKEOFF の境（`takeoff` の 0.301〜0.319 s 後）に届く | 10 |
| T8 | ALT_HOLD、ToF 無効。TAKEOFF の中でパイロットが ARM ボタンを押し直す（DISARM） | 10 |

指標: **空中でのモータ停止** = LANDING が終わった（モータが DISARM で止まった）ときの真値の高さが 0.06 m を超える本数。**終わらない LANDING** = 実行の終わりに LANDING のままの本数。**床からの再離陸** = 床に着いた後、モータが回ったまま 0.06 m を超えて上がる本数。**接地速度** = 高さが 3 cm 未満になる直前の真値の降下速度。

| 組 | 空中でのモータ停止（main → 今） | 終わらない LANDING | 床からの再離陸 | 接地 > 1.5 m/s の本数（最大 [m/s]） |
|----|-------------------------------|-------------------|---------------|------------------------------------|
| J | 0 → 0 | 226 → 0 | 0 → 0 | 0（0.37）→ 0（0.43） |
| P | 0 → 1（下記） | 18 → 0 | 0 → 0 | 9（1.61）→ 0（1.41） |
| L | 0 → 0 | 21 → 0 | 0 → 0 | 9（2.15）→ 0（0.70） |
| B | 18 → 0 | 0 → 0 | 0 → 0 | 18（13.78）→ 0（1.19） |
| X | 248 → 0 | 0 → 0 | 0 → 0 | 248（19.02）→ 2（2.50） |
| V | 8 → 0 | 0 → 0 | 0 → 0 | 8（17.82）→ 0（1.05） |
| T1 | 128 → 0 | 0 → 0 | 0 → 0 | 128（23.90）→ 0（1.29） |
| T2 | 108 → 0 | 0 → 0 | 0 → 0 | 108（17.83）→ 0（1.19） |
| T3 | 324 → 0 | 0 → 0 | 0 → 0 | 324（25.67）→ 9（5.52） |
| T3b | 80 → 0 | 0 → 0 | 0 → 0 | 80（25.66）→ 0（1.13） |
| T4 | 2 → 2 | 9 → 8（ARMED_GROUND のまま。`land` を拒否） | 0 → 0 | 2（6.46）→ 2（6.46） |
| T5 | 0 → 0 | 138 → 0 | 0 → 0 | 100（5.31）→ 0（0.68） |
| T6 | 72 → 0 | 0 → 0 | 0 → 0 | 72（13.77）→ 18（1.77） |
| T7 | 0 → 0 | 10 → 0 | 0 → 0 | 0（0.00）→ 0（0.42） |
| T8 | 10 → 0 | 0 → 0 | 0 → 0 | 10（35.84）→ 0（1.06） |

読み取り:

- 空中でのモータ停止は、P の 1 本（`P_2000_1.3_off_s1`: `hover.thrust_corr` 1.3、ノイズなし、`land` が 2000 ms 後）を除いて 0 本。T4 の 2 本は、手動モードの ARMED_GROUND でスロットルだけで浮いた機体を、シナリオの最後にパイロットが DISARM した高さ（0.24 m と 2.11 m）で、main も同じ。P の 1 本は、LANDING が 1.30 m から降りて 0.11 m で止まった地面効果の浮きを降下停滞の判定（`near_ground_tof_m` 0.15 m）が終わらせ、1.08 m/s で接地した（main は 1.61 m/s の別の経過）。終わらない LANDING と床からの再離陸は 0 本。
- 加速度計のオフセットが ToF 無効の LANDING に与える影響は、X の 248 本（オフセット −1.5〜+1.5 m/s²）で、LANDING が終わらない本数 0。接地が 1.5 m/s を超えたのは +1.4 / +1.5 m/s² の 2 本（2.50 / 1.62 m/s）。
- 接地が 1.5 m/s を超えるのは、T3 の 9 本（`hover.thrust_corr` 1.3・モータの効率 0.85・オフセット +0.6 m/s²: 次節）、T6 の 18 本（ToF 無効でオフセット ±1.5 m/s²。1.50〜1.77 m/s）、T4 の 2 本（手動モード）。
- 離陸の最高高度: TAKEOFF から FLYING までの真値の最高は、`land` を後回しにしたので `land` の有無・時刻に依らない。P の 60 本（`land` の時刻 0.5 / 0.8 / 1.0 / 1.5 / 2.0 s × `hover.thrust_corr` 0.9 / 1.0 / 1.2 / 1.3 × ノイズなし / n2）で、`land` を出さない離陸（0.45〜0.59 m）との差は 0.0000 m。`land` が 0.05〜0.3 s の 36 本は、ARMED_GROUND の即時 DISARM で離陸しない。

##### 床の上で始まる LANDING（TAKEOFF から高さ 0.15 m 未満で LANDING に入る場合）

TAKEOFF から LANDING に入る道は、通信途絶・ToF の規則・離陸の中止の判断だけなので、弱い機体（モータの健全度 0.80〜0.95）で、ARM 起動の ALT_HOLD の自動離陸と、手動 STABILIZE（スロットル 3100）に通信途絶（ARM の 0.6 / 1.5 / 2.5 s 後）を足した場面を測った。`hover.thrust_corr` 0.9 / 1.0 / 1.2 / 1.3 × ToF の不感帯あり・なし × オフセット −0.4 / 0 / +0.4 m/s² × ノイズ n2（W: 960 本）、健全度 0.86〜0.93 を細かく振った Y（240 本）。

| 組 | 本数 | 高さ 0.15 m 未満で LANDING に入った本数 | LANDING の後の上昇の最大 | 空中でのモータ停止・終わらない LANDING・床からの再離陸 |
|----|------|----------------------------------------|----------------------|--------------------------------------------------|
| W | 960 | 90（健全度 0.80・0.85 だけ） | 0.011 m | 0 |
| Y | 240 | 0（浮く機体は通信途絶の猶予 3 s より前に離陸が完了する） | — | 0 |

床の上で始まる LANDING の推力に上限（ホバー推力の見積もりの 0.5 倍）を置いた場合と置かない場合で、W の 952 本の結果は一致した。離陸の推力で浮かなかった機体は、着陸の推力でも浮かないので、上限は置かない（`detailed_design.md` §3 注8 (3)）。

##### 全件の再確認試験とビルド

`.expect` 付き 103 本（SILS 全件）: **PASS 99 / FAIL 0 / KNOWN-FAIL 3（`commloss_land_level`・`pos_yaw`・`yaw_hold`）/ SKIP 1（`workshop_acro`）**。`api_land_in_rc_takeoff` は、この対策が無いビルドでは `error land timeout` で不合格になる。ホスト試験 9 ファイル 236 本（`test_takeoff_policy` 24 本を含む）は全て合格。`sf build vehicle` と、複製した workshop ファームのビルドは成功。

#### 3. 既知の取引と未確認

- **実機はすべて未確認。** 実機の ToF は床の上で無効（status 254）を返すので、床の積極的な証拠（有効な読みが 5 cm 以下）は実機の床ではほとんど立たない（マット・傾きの上でだけ）。実機の床の上の挙動は、ToF 無効の規則と ToF なしの降下が担う。
- **較正前の ToF なし降下の上限 `kSupportClimbDerate` の 0.91:** 0.92 では、加速度計のオフセット +0.6 m/s²・`hover.thrust_corr` 1.2〜1.3・ノイズなしで、機体が 5 s にホバーして 2.02 m/s で接地する組がある。340 本（API 離陸の後に出力停止 140 本、離陸直後に出力停止 200 本）の掃引で、悪い組は 0.92 で 4、0.915 で 2、0.91 以下で 0。代償: 強い接地（1.5 m/s 超）は離陸直後の 200 本で、0.92 では 18 本、0.91 では 33 本（増え幅の最大 +0.5 m/s）。
- **T3 の 9 本（`hover.thrust_corr` 1.3・モータの効率 0.85・オフセット +0.6 m/s²）:** LANDING は TAKEOFF の ToF 無効の規則で ToF なしの降下として始まり、較正前の支持推力の上限が重さを上回って、再上昇 2.9 m、接地 5.5 m/s になる。較正前の上限の余裕（`kSupportClimbDerate`）の限界で、この改修では変えていない。
- **P の 1 本（`P_2000_1.3_off_s1`）:** ホバー推力の見積もりが 30 % 高いと、LANDING の推力が地面効果の中でほぼ釣り合い、0.11 m で浮いたまま降下停滞の判定が終わらせる（空中 0.11 m からの接地 1.08 m/s）。直していない。
- **手動モードの ARMED_GROUND（T4）:** スロットル 0.499 付近で浮ける既存の問題は直していない。API `land` は手動モードでは拒否する（12 本のうち 8 本は ARMED_GROUND のまま、2 本はスロットルで浮いて落ち、接地 1.91 / 6.46 m/s。main も同じ）。
- **`land` が ARMED_GROUND から TAKEOFF へ移る瞬間に届く競合:** StateManager は TAKEOFF の着陸要求を無視する。ApiTask は発行の後に IDLE_GROUND を確かめてから `ok` を返し、TAKEOFF に移っていたら TAKEOFF を抜けるまで待って `land` を発行し直す（待機の中なら後回しの行にする）。窓は数 ms で、SILS の通常のシナリオでは再現できず未確認（要求の発行を省いて模擬した場合の後回しの経路だけ確かめた）。RC の ARM が始めた自動離陸の最中（待機の外）の `land` は `api_land_in_rc_takeoff`（修正前のビルドでは `error land timeout` で不合格）。
- **較正前の着陸基準推力の上限（`landingBaseThrust()`）の代償:** 加速度計のバイアスが負で大きいとき、接地速度が増える。バイアス −1.5 m/s² で最大 0.95 m/s（対策の無い main は 0.45 m/s）、−0.6 m/s² で 0.65 対 0.44 m/s。`hover.thrust_corr` 1.2 では 0.93 対 3.43 m/s で改善する。直していない。
- **ToF が 0.05〜0.15 m のヒステリシス帯（上下で別のしきい値を持つ帯）にある間の降下停滞の判定:** ESKF の鉛直状態が地上に固定されるため、床の近くで始まった LANDING が上昇中でも降下停滞と判定され、0.09〜0.16 m で止まることがある（main でも同じ種類で、件数は同じ）。固定されない速度を使う対策は、気圧計の高度を取り入れる作業（計画中）で扱う。直していない。
- 上昇ブレーキの上昇率の不感帯（0 / 0.03 / 0.05 m/s を 120 本で比較）は、上昇が増えず接地速度が下がる値が無いので採らない。判定のやり直しを「床を離れたことのある機体」に限る案は、64 本で LANDING の長さが変わらないので採らない。
- 着陸の判定を持ち越す SILS の入力（以前から残った判定）は見つからなかった。判定のやり直しと刻印の確認は、ホスト試験だけが検査する。

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
- **L4 POS_HOLD は roll/pitch/斜め/yaw の全4軸がタイトに保持**（採用した運動加速度補償 `eskf.accel_comp.enable=1` ＋ 位置速度ループ `position.vel.kp=0.8`）。drift ≤ 1.1m・終端オフセット ≤ 1.0m、N0 センサノイズ下でも保持（drift ≤ 1.3m）。推定器の汚染が無いので速度ゲインを 0.8 にできる（推定器が汚染されていると 0.3 が限界で、斜めで発散する）。
- **残課題:** n1/n2（過酷な振動）下の POS_HOLD は baseline 同様に飛び去る（振動処理は別課題）。実機 ESP-IDF ビルドは未検証（host SILS は全ソース通過）。

## 5. 2026-10-31 R_out プラントに伴うシナリオ入力・閾値の変更

方針: 物理測定を SILS より上に置く。シナリオの入力や閾値を変えてよいのは、(a) 旧プラントに合わせた値で意図は不変、または (b) 実機の飛行データに根拠がある場合に限る。理由はファイル内のコメントにも 1 行ずつ記した。

| シナリオ・対象 | 旧 → 新 | 理由 | 実機の根拠 |
|---|---|---|---|
| STABILIZE/ACRO 開ループのスロットルを使う 18 シナリオ（`acro_flight`・`alt_flight`・`alt_inflight_switch` 行E・`boot_motion`・`calib`・`commloss`・`commloss_land_level`・`crash_refly` 行C/N・`modeswitch`・`pairing_held`・`pos_flight`・`pos_pitch`・`pos_roll`・`pos_yaw`・`stab_flight`・`stab_ground_windup`・`sysid_roll_step`・`yaw_hold`） | 生値 3243 → 3171（離陸）、3176 → 3108（ホバー付近）。2048 からの偏差を一律 ×0.94 | (a) 旧値は 2026-08-22 の再較正（ホバー生値 3154 を基準に ×0.8386）で旧プラントに合わせたもの。R_out プラントの開ループ・ホバー生値は 3088（SILS で生値を掃引し、上昇加速度が 0 になる点を求めた。電池 3.87 V、局所傾き 0.0045 m/s² 毎 生値、ホバー近傍の誤差は約 ±5 生値）。係数 = (3088−2048)/(3154−2048) = 0.94。離陸トリガ（生値 3072）を下回るため、ホバー以下の「ゆるい離陸」行（3123・3106・3090: `acro_crash_relevel`・`pairing`・`attitude_trim_test`・`autotrim_learn`・`disturb`）は変更しない（実測の上昇加速度は 3090 で +0.02、3106 で +0.09、3123 で +0.17 m/s²。意図の「小さな正の上昇」を満たす） | 実機の対応値なし（実機 14 ログに STABILIZE のホバー区間がない）。係数は SILS の掃引による |
| `rc_flip`・`rc_flip_latch`（スロットルパルスによる事前上昇） | `rc_flip`: 行D 5000→4300 ms、行E 2000→2700 ms、行R 5000→4300 ms、行S 2000→2700 ms、再上昇パルス（生値 3600）を行H2（500 ms）・行J2（600 ms）に追加。`rc_flip_latch`: 行D 5000→4300 ms、行E 1800→2500 ms。時刻表は不変（D と E、R と S の合計は同じ） | (a) 上昇入力が旧プラントに合わせたものだった。ボタン押下の高度を 1.2 m 以上にする（R_out では 0.93〜0.98 m で拒否された）。意図は不変。R_out では宙返りと解放のたびに約 0.1 m/s で沈むため、2・3 回目の押下の前に再上昇パルスを入れた。押下の高度は `rc_flip` で 1.204 / 1.274 / 1.232 / 1.223 m、`rc_flip_latch` で 1.244 m | 実機の宙返り開始高度 1.41 m・1.70 m（`logs/flip_test1/2.sflog.zip`）と矛盾しない |
| `rc_flip.expect` の `alt_mean` 上限（窓 13.5〜16.5 s と 45.3〜48.3 s） | 1.35 → 1.75 | (a)+(b) 上昇入力の延長で窓の平均高度が約 1.39 m になるため。上限は実機の宙返り開始高度の最大 1.70 m を切り上げた値 | 実機 2 回（1.41 m・1.70 m） |
| `alt_auto_takeoff`・`pos_auto_takeoff` の `alt_band`（窓 12〜16 s） | < 0.20 → < 0.55（SILS の値 0.2668）。さらに下限 > 0.05 を追加 | (b) 旧閾値は旧プラントの値 0.008 に合わせたもので、実機の 14 回の離陸（ALT_HOLD 5・POS_HOLD 9、離陸から +4.2〜+8.2 s の ESKF 高度）はいずれも 0.062 以上。実機の最大 0.521 を切り上げた。p90 の 0.443 では実機の 2/14 回（0.500・0.521）を不合格にしてしまうため採らない | 実機 14 回: p50 0.164、p90 0.443、最大 0.521 |
| `api_flight` の `alt_mean` 下限（窓 14〜17.5 s） | > 0.42 → > 0.30（SILS の値 0.4130） | (b) 旧閾値は旧プラントの値 0.518 に合わせたもので、実機の 4/14 回が下回る。実機の最小 0.326 を切り下げた | 実機 14 回（離陸から +7.7〜+11.2 s）: 最小 0.326、p10 0.348、p50 0.473 |
| `calib.expect` の xfail 印 | 削除 | xfail の理由（0.6〜0.7 Hz の自励振動）が出なくなり、再確認試験 2 回で XPASS、直接実行 2 回で PASS（att_rmse 0.55 deg） | 実機の対応値なし |
| SILS の下向き ToF 模擬の最大距離（`simulator/sils/devices/vl53_device.cpp` の `MAX_MM`） | 1400 → 1630 mm | (b) 実機ログ（121 本＋flip_test1/2、約 15 万サンプル）では 1.4〜1.7 m で 95〜99 % が有効（status 0）、1.7〜1.8 m で 91 %、1.8〜2.0 m で 66〜70 %、2.0〜2.1 m で 22 %（無効時の status は 255 が大半。SILS も 255）。模擬の上限は 24 bin バッファが決める 1636 mm で、1630 は 2 bin 分割がバッファに収まる値（プローブで 1.63 m まで有効・誤差 17 mm 以内）。1.63〜2.0 m は未模擬のまま | 実機の有効率（高度は ESKF 高度） |
| `alt_inflight_switch.expect` の `alt_band` 上限（窓 14.7〜18.7 s） | < 0.32 → < 0.38（SILS の値 0.351） | STABILIZE→ALT_HOLD 捕捉後の減衰振動。閾値側のコメントが「開始外乱の大きさによらない固定振幅」と記録しているもの。R_out の値 0.351 に約 8 % の余裕 | 実機の対応値なし・未検証（飛行中に ALT_HOLD へ切り替えた実機ログが必要） |
| `crash_refly`（`.scn` 行J・行L、`.expect` の 2 つの窓） | 行J 2500→500 ms、行L 8000→10000 ms（再校正以降の時刻は不変）。`.expect`: 墜落静止の窓 14〜16 s → 12〜14 s、持上げの窓 16.8〜18.4 s → 14.8〜16.4 s（閾値は不変、手が届く時刻と同じ −2.0 s の移動） | (a) R_out の墜落は機体が横倒し（ロール 90 deg）で止まり、衝突の約 0.2 s 後に始まる 3.5 s の起動校正が横倒しのまま完了して level_offset 1.5708 rad が推定器に残る。手が水平に直した後も ARM の判定（姿勢推定が重力と食い違う、正しい挙動で不変）が再 ARM を拒否した。ハンドリング軌道は任意の墜落姿勢を水平へ戻すので、手を校正の最中（14.0 s）に届かせて水平な設置後に校正をやり直させる。再飛行は実際に飛ぶ（上昇 1.33 m、終端ホバー 0.87 m） | 実機の対応値なし |
| `pos_flight.expect` の xfail 印 | 削除 | ヨートルク権限飽和（デューティ天井固定）が R_out プラントでは出なくなった（ホバー duty が低くヨーに余裕がある）。再確認試験 2 回で XPASS。`pos_yaw`・`yaw_hold` の印は残す | 実機の対応値なし |

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

**`calib_side_lying` (vehicle, `--duration 39000000`):** test for the level check of the still-calibration (detailed_design §3 note 10). A craft left lying on its side after a crash for longer than one 3.5 s calibration window keeps being rejected (`Calibration window rejected: not level`), so no calibration completes on its side and no accel_bias of about +-9.8 m/s^2 is seeded; once the hand places it level, calibration completes and ARM is accepted. It fails on a build without the level check (every ARM rejected by the tilt guard).

**Added — the battery EMERGENCY in TAKEOFF and the scenario event `battery <volts|model>`** (holds the battery terminal voltage; a scenario that does not use it is not affected: its output is byte-identical): `battery_emergency_takeoff_floor` and `battery_emergency_takeoff_floor_manual` (the EMERGENCY on the floor in an auto / a manual TAKEOFF: the motors are cut, no LANDING), `battery_emergency_takeoff_airborne` and `battery_emergency_takeoff_airborne_manual` (after the craft has left the floor: LANDING, gentle touchdown), `battery_emergency_flying` (FLYING: LANDING). Cells and the grid in `detailed_design.md` §3.1 ("Cells of the TAKEOFF battery EMERGENCY"): on the floor the runs that float up (9 of 138 at 2.99 V) and the runs that stop the motors in the air (9) become 0.

**Flight-safety revision (ToF silence and invalid samples, the positive floor evidence, LANDING, TAKEOFF).** The evaluation is in the Japanese section "飛行安全の改修の評価" above: (1) what each test checks (conditions written out), (2) the comparison before the change and now (a 1152-run SILS matrix of a `land` during the TAKEOFF of a craft whose ToF returns invalid samples: motors stopped in the air 0 -> 0, touchdown above 1.5 m/s 261 -> 5 runs, re-rise of the LANDING above 0.10 m 0 -> 2 runs, LANDING that never ends 0 -> 6 runs; the auto-takeoff peak with a +20 % hover estimate and noise n2 1.96-2.03 m -> 1.25-1.27 m), (3) the full SILS regression, the host tests and the builds, (4) the known trade-offs and what is unconfirmed. Everything is SILS truth; nothing is confirmed on a real vehicle. **ARMED_GROUND in a manual mode (ACRO / STABILIZE):** a craft that rises with the throttle at or below the takeoff threshold is in the air for the state machine once the healthy ToF has read valid heights above 5 cm for 1 s without a break (`LiftoffWatch`, `takeoff_policy.hpp`): ARMED_GROUND -> TAKEOFF -> FLYING, so the comm-loss landing and the ToF-silence rule of the air apply. `stab_lowthrottle_lift_commloss` and `stab_lowthrottle_lift_tofsilent` lift the craft with an upward gust (`wind 0 0 -0.05 400`: the plant cannot make a healthy craft lift below the threshold); `stab_lowthrottle_floor` checks that a craft on the floor (dead zone on and off, small sticks) never starts a TAKEOFF. The battery EMERGENCY cannot be set in SILS (no scenario event for the voltage): it is covered by the host test only.

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
| `alt_band` of `alt_auto_takeoff` and `pos_auto_takeoff` (window 12-16 s) | < 0.20 -> < 0.55 (SILS value 0.2668); lower bound > 0.05 added | (b) The old bound was tuned to the old plant's value 0.008; all 14 real take-offs (5 ALT_HOLD, 9 POS_HOLD; ESKF altitude, take-off +4.2 to +8.2 s) are at or above 0.062. The real maximum 0.521 is rounded up. The p90 (0.443) is not used because it would reject 2 of the 14 real take-offs (0.500, 0.521) | 14 real take-offs: p50 0.164, p90 0.443, max 0.521 |
| `alt_mean` lower bound of `api_flight` (window 14-17.5 s) | > 0.42 -> > 0.30 (SILS value 0.4130) | (b) The old bound was tuned to the old plant's value 0.518; 4 of 14 real take-offs are below it. The real minimum 0.326 is rounded down | 14 real take-offs (take-off +7.7 to +11.2 s): min 0.326, p10 0.348, p50 0.473 |
| xfail marker of `calib.expect` | Removed | The xfail reason (0.6-0.7 Hz self-oscillation) no longer appears: XPASS in two re-check runs, PASS in two direct runs (att_rmse 0.55 deg) | No real counterpart |
| Maximum distance of the SILS bottom ToF model (`MAX_MM` in `simulator/sils/devices/vl53_device.cpp`) | 1400 -> 1630 mm | (b) In real logs (121 logs + flip_test1/2, about 150k samples) the ToF is valid (status 0) 95-99 % of the time at 1.4-1.7 m, 91 % at 1.7-1.8 m, 66-70 % at 1.8-2.0 m and 22 % at 2.0-2.1 m (the invalid status is mostly 255, as in SILS). The model's ceiling is set by the 24-bin buffer at 1636 mm; 1630 keeps the two-bin split inside the buffer (probed: valid with error <= 17 mm up to 1.63 m). The 1.63-2.0 m range stays unmodelled | Real validity rates (height = ESKF altitude) |
| `alt_band` upper bound of `alt_inflight_switch.expect` (window 14.7-18.7 s) | < 0.32 -> < 0.38 (SILS value 0.351) | The damped oscillation after the STABILIZE->ALT_HOLD capture, which the bound's own comment records as a fixed amplitude independent of the initiating disturbance. R_out value 0.351 plus about 8 % | No real counterpart, unverified (needs a hardware log of an in-flight switch to ALT_HOLD) |
| `crash_refly` (`.scn` rows J and L, two windows in `.expect`) | Row J 2500->500 ms, row L 8000->10000 ms (times from the re-calibration dwell on unchanged). `.expect`: crash-rest window 14-16 s -> 12-14 s, lift window 16.8-18.4 s -> 14.8-16.4 s (thresholds unchanged, shifted by the same -2.0 s as the hand's arrival) | (a) The R_out crash leaves the craft lying on its side (roll 90 deg); the 3.5 s boot calibration that starts about 0.2 s after impact completes on its side and leaves level_offset 1.5708 rad in the estimator, so even after the hand levels the craft the ARM guard (attitude estimate disagrees with gravity; correct behaviour, unchanged) rejected every re-ARM. The handling trajectory already levels any crash attitude, so the hand now arrives during the calibration (14.0 s) and the calibration restarts on the levelled craft. The re-fly really flies (climb to 1.33 m, final hover 0.87 m) | No real counterpart |
| xfail marker of `pos_flight.expect` | Removed | The yaw torque-authority saturation (duty ceiling lock) no longer appears on the R_out plant (lower hover duty leaves yaw headroom). XPASS in two re-check runs. The markers of `pos_yaw` and `yaw_hold` stay | No real counterpart |

**Not changed (including unverified items):** the xfail markers of `commloss_land_level`, `pos_yaw` and `yaw_hold` (still KNOWN-FAIL; `commloss_land_level` fails on `alt_max` 0.156 m > 0.1 m after touchdown).

**The ToF close-range dead zone in the SILS chip model and the altitude ceiling on the stick climb.**

| Scenario | What it checks (measured) |
|----------|---------------------------|
| every scenario | The VL53 model has a close-range dead zone (`vl53_device.cpp` `MIN_MM` = 31 mm, the smallest valid distance of the real logs; the SILS craft rests 21 mm above the floor): the ground reads "no target" as the real craft does, and the first valid sample comes ~0.4 s after the TAKEOFF entry. `tof_deadzone 0` turns it off. Compared with the build before it (72 scenarios present in both), the truth trajectory of 56 differs from the first takeoff on (the first valid ToF sample comes later, which shifts the ESKF ground-to-air handoff: metrics move by a few cm, e.g. a 0.03 m higher hover mean); 16 are identical (no takeoff, or an API takeoff that is cut at the ground). Every pass/fail is unchanged except the scenarios retuned below |
| `alt_ceiling_stick`, `alt_ceiling_stick_noise` | The ALT_HOLD stick held UP stops at the 1.2 m ceiling (tapered climb, truth 1.17-1.18 m; noise n2, 3 seeds: 1.185-1.187 m; 1.97 m without the ceiling) |
| `tof_recover_landing_invalid` | Samples return during the blind landing, all invalid: the episode lasts to the touchdown (without the fix it ends after ~1 s) |
| `tof_invalid_alt_hover` | Takeoff with a permanently invalid ToF is cut 0.15 s after the estimated rise reaches 5 cm: peak 0.20 m (0.15-0.41 m over n2 / thrust error) |
| `landing_dead_zone` | A soft ToF landing (contact below 3 G) with the close-range dead zone modelled : the dead-zone touchdown ends it, 0.2 s after the last valid reading (0.085 m), the craft at rest on the floor (detailed_design §3 note 7) |
| `alt_disarm_land` | Dead zone off (event `tof_deadzone 0`): tests only the routing of the DISARM |
| `tof_silent_hover_1p2`, `tof_silent_alt_climb`, `rc_flip`, `rc_flip_latch`, `alt_inflight_switch`, `alt_recenter_gate` | Retuned because the ceiling bounds the stick climb at 1.2 m (reasons in each `.scn` / `.expect`): the hover is ~1.1 m (silence 20.5 s), the climb silence is at ~0.8 m (0.92 m peak, 0.40 m/s touchdown), the flip scenarios run with `flip.min_height_m` 0.8 / 0.75 m, the two stick-climb scenarios have lower bounds |
