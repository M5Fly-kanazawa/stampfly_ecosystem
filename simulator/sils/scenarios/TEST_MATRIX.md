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
| `calib_side_lying` | **校正の水平チェック**（2026-11-01、detailed_design §3 注10）: 墜落で横倒しになった機体を校正窓 1 回分（3.5 s）より長く置いても `Calibration window rejected: not level` で棄却し続け、横倒しのまま校正が完了して加速度バイアス約 ±9.8 m/s² が入らない。手が水平に置いた後は校正が完了して ARM が受理される。水平チェックの無いビルドでは失敗（ARM 全拒否）。`--duration 39000000` 必須 | vehicle |
| `calib_supersede` | **校正の置き換え規則**（2026-11-01、detailed_design §3 注10）: 起動校正が注入した加速度のずれ 0.2 m/s² を適用した後、注入を外して手が置き直すと、「ずれなし」の再校正が先の値を置き換える（`previous seed superseded`）。置き換え規則の無いビルドでは `estimator left untouched` となり 0.2 の値が残るので失敗。`--duration 22000000` 必須 | vehicle |
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
| `tof_silent_takeoff` | **下向き ToF が自動離陸の上昇中に出力停止**（2026-11 の事故の再現、故障注入 `tof_silent 1`、POS_HOLD、`--duration 21000000`）: 最後のサンプルの後は何も来ない → Failsafe が TOF_SILENT を発報 → 0.3 s 後に TAKEOFF → LANDING（上昇中止、FLYING に至らない）→ 接地フラグを信用しないので位置・速度の固定が外れ、ToF なしの降下は**推力が常に 支持推力 × 0.95 以下**（上昇できない形。窓が無いので支持推力は低くした見積もり）→ 接地（ToF 出力停止中の LANDING での 3 G の衝撃は非常でなく接地）→ 出力停止中は ARM が理由つきで拒否される。実測（2026-11-09 の 5 回目の見直し）: ToF 停止 8.39 s、`alt_max` 0.229 m、接地速度 0.91 m/s（4 回目まで 0.43 m/s。上限の一定推力で沈むため。`detailed_design.md` §3 注12） | vehicle |
| `tof_silent_api_takeoff` | **事故の連鎖そのもの**（Tello 風 API `takeoff`、POS_HOLD）: 自動離陸の上昇中に ToF が出力停止 → TAKEOFF → LANDING で上昇中止（`alt_max` 0.175 m、接地速度 0.81 m/s。4 回目まで 0.43 m/s）→ API は 12 s のタイムアウトを待たず `error takeoff aborted` を返す → ToF が出力停止したままの 2 回目の `takeoff` は最初に `error tof silent - check the sensor`。`--duration 16000000` | vehicle |
| `tof_silent_hover` | **0.5 m でホバー中（ALT_HOLD・FLYING）に下向き ToF が出力停止**（`--duration 25000000`）: 0.3 s 後に FLYING → LANDING → 降下 → 床に達して IDLE_GROUND。実測: ToF 停止 14.0 s、接地速度 0.60 m/s（4 回目まで 0.43 m/s、通常着陸は 0.3 m/s）。ARM は理由つきで拒否される | vehicle |
| `tof_silent_hover_1p2` | **高度上限の近く（約 1.1 m）でホバー中に下向き ToF が出力停止**（`--duration 31100000`）: 同じ連鎖。スティック上昇が上限 1.2 m へ向けて絞られるので、上昇に 5 s かかり約 1.1 m で終わる。実測: 停止 20.5 s（高度 1.1 m）、接地速度 0.84 m/s（4 回目まで 0.43 m/s） | vehicle |
| `tof_silent_alt_climb` | **ALT_HOLD のスティックで上昇中（約 0.8 m、約 0.3 m/s）に出力停止**（`--duration 26500000`）: 速度ループが上昇を止め（最大 0.92 m、出力停止時から 0.12 m）、接地速度 0.77 m/s（4 回目まで 0.40 m/s） | vehicle |
| `tof_silent_est_high` | **ホバー推力の見積もりが真のホバーより 19 % 高いまま（`param` 事象で起動時に設定、学習なし）、約 1.2 m で出力停止**（`--duration 28000000`）: 支持推力は加速度計で較正した値（見積もりではない）、`touchdown by time` にならず接地。実測: 最大 1.24 m、接地速度 0.96 m/s（4 回目まで 0.43 m/s） | vehicle |
| `tof_silent_takeoff_est_high` | **ホバー推力の見積もりが 28 % 高い（`param` 事象 `hover.thrust_corr 1.2`）まま、自動離陸の上昇初期に出力停止**（`--duration 21000000`）: 独立レビューが「致命的」とした場合（旧ビルドで真値の最高 11 m、天井）。上昇は速く（LANDING で 1.3 m/s）、降下は上限（支持推力の 0.95 倍。窓が無いので、空中の全サンプルでの商 × 0.85）の下。実測: 真値の最高 0.80 m、接地速度 1.04 m/s、モータは床の上で止まる。`.expect` は最高 < 1.0 m・接地速度 < 1.3 m/s | vehicle |
| `tof_silent_api_takeoff_est_high` | 同、Tello 風 API `takeoff`（`--duration 16000000`、`hover.thrust_corr 1.2`）: `error takeoff aborted`、2 回目の `takeoff` は最初に拒否。実測: 真値の最高 0.72 m、接地速度 0.99 m/s（旧ビルドで 11 m）。最高 < 1.0 m・接地速度 < 1.25 m/s | vehicle |
| `tof_silent_takeoff_noise` | `tof_silent_takeoff_est_high` を `hover.thrust_corr 1.1` とセンサノイズ n2（seed 1、`--noise n2 --seed 1`）で（`--duration 21000000`）: ノイズで上昇が目標に達してから出力停止が判定され、FLYING/POS_HOLD から 1.7 m/s で上昇中に LANDING。実測: 真値の最高 1.20 m、接地速度 1.99 m/s（4 回目まで 0.65 m/s。n2 のプラントは真のホバー推力が低く、窓が無いときの低くした見積もりがその分だけ強く沈める）。最高 < 1.5 m・接地速度 < 2.4 m/s | vehicle |
| `tof_silent_touchdown_detect` | `tof_silent_hover` を 3 G の衝撃カットを無効にして（`--param safety.impact.accel_g=10`、`--duration 25000000`）: 接地検出器（衝撃のあとの静止）だけが着陸を終わらせる。`Touchdown detected (impact, then still` → LANDING → IDLE_GROUND、`Impact detected` と `touchdown by time` が無い。実測: 接地速度 0.60 m/s、床の上で静止（真値 0.021 m） | vehicle |
| `tof_silent_bias_plus` / `tof_silent_bias_minus` | **加速度計のオフセット誤差 ±0.2 m/s² のまま約 1.1 m のホバーで ToF が出力停止**（`tof_silent_hover_1p2` に `bias 0 0 ±0.2 0 0 0` を ToF 停止の直前に足す、`--duration 31100000`。独立レビューの再上昇の再現）: 速度推定が何であれ推力は 支持推力 × 0.95 を超えないので**再上昇しない**（旧ビルド: +0.2 で 2.7 m まで上昇し 5 m/s で接地）。ToF 出力停止中の LANDING での 3 G の衝撃は非常でなく接地（`Touchdown while the ToF is silent (impact)`）。実測: 最高 1.099 m（停止時の高度）、接地速度 0.80 / 0.95 m/s | vehicle |
| `tof_silent_takeoff_bias_est_high` | **見積もり 28 % 高い + 加速度計のオフセット誤差 +0.2 m/s²（離陸の開始から）+ 自動離陸の上昇初期に出力停止**（`--duration 21000000`）: 未知が 2 つ同時（支持推力は較正できず、速度推定も誤る）。旧ビルドは 3.26 m まで上昇した。実測: 最高 0.80 m、接地速度 0.97 m/s、`emergency DISARM` なし | vehicle |
| `tof_silent_est_high_noise` | **見積もりが 35 % 高く（ノイズ n2）、自動離陸が天井 1.2 m を行き過ぎる**（`tof_silent_est_high` を `--noise n2`、`--duration 28000000`。独立レビューが 1.68 m まで上がったとした再現）: 高度の天井の実効化（1.35 m を超えたら上昇の指令を禁止し、上昇中は推力を見積もりの 0.5 倍以下）。実測: 最高 1.49 m（3 seed: 1.49 / 1.50 / 1.51 m。旧ビルド 1.79 m）。`.expect` は最高 < 1.55 m | vehicle |
| `tof_silent_stabilize_commloss` | **STABILIZE を ToF 出力停止のまま 21 s 飛んだ後に通信断**（`--duration 50000000 --noise n0`、スロットルは開ループで手調整）: 速度推定の最後のリセットは 24 s 前なので、降下の最初の周期から PI を使わず（`trusted -22 s more`）、支持推力（21 s のホバー近傍のサンプルで較正）× 0.95 の一定推力で降り、衝撃で接地する。実測: 飛行高度 0.65〜0.75 m、LANDING 33.8 s、接地速度 0.71 m/s。旧ビルドは `emergency DISARM` で終わる | vehicle |
| `tof_silent_stabilize` | **STABILIZE の手動飛行中に出力停止: 強制着陸しない**（`--duration 17000000`）: LANDING に入らず警告のみ（`NO forced landing, fly on`）、パイロットのスロットルで飛び続ける。出力停止中の POS_HOLD への切り替えは拒否（`Mode change to POS_HOLD refused`）。パイロットが着陸した後の ARM は拒否 | vehicle |
| `tof_silent_mode_refused` | **出力停止中の ALT_HOLD・POS_HOLD 拒否**（地上、`--duration 12000000`）: POS_HOLD スイッチは拒否（理由ログと `SystemMode.mode_block`）、ACRO は受理、拒否したエッジは持続し ToF 復帰で自動適用される | vehicle |
| `tof_silent_hover_timed` | 同、衝撃カットと接地検出器（衝撃の閾値と、2 つ目の経路の「床が機体を支えている」）を無効化（`--param safety.impact.accel_g=10 --param safety.blind_touchdown.accel_g=10 --param safety.blind_touchdown.floor_s=100`）して**ToF なしの着陸の時間**（10 s、LANDING に入った時刻から数える）を検証: LANDING 約 14.3 s → `touchdown by time` → IDLE_GROUND 約 24.3 s。`--duration 29000000` | vehicle |
| `tof_silent_prearm` | **ToF 出力停止中は ARM 拒否、復帰後は受理**（`--duration 15000000`）: 3.0 s に停止 → 7 s の ARM は `ARM rejected: downward ToF is silent`（`arm_block` = 7）→ 9.0 s に再開 → 10.5 s の ARM は受理（出力停止はレベル条件でラッチでない）。モータは回らない | vehicle |
| `tof_silent_armed_ground` | **出力停止の 0.2 s 後に押した ARM**（まだ出力停止と判定されない窓）が受理され、0.3 s に達した時点で ARMED_GROUND から DISARM される（TAKEOFF に入らない）。`--duration 12000000` | vehicle |
| `tof_recover_frozen_ground` | **離陸の前に ToF が出力停止し、レンジのはるか上で復帰**（2 回目の独立レビュー D1+D2、`--duration 24000000`）: 5.0 s に停止（地上）、約 1.8 m まで ToF なしで上昇、8.6 s に復帰するが最初のサンプルは範囲外。修正前は固定されたままの接地フラグが再び信用され、推定が 0 に固定されて ALT_HOLD で 10.9 m まで上昇、高所で着陸完了（モータ停止）。修正後: 生のフラグを「空中」へ倒し、推定器が ToF を取り込み直す（イノベーション 1.5 m）まで出力停止のエピソードを続け、真値 alt_max 2.5 m、2 回目の飛行（ALT_HOLD で ARM）は推定が真値の 0.1 m 以内 | vehicle |
| `tof_recover_edge_consumed` | **飛行中に ToF 出力停止中に拒否したモードスイッチが復帰の瞬間に勝手に適用されない**（D3、`--duration 24000000`）: STABILIZE 飛行中の 9.0 s に POS_HOLD スイッチ → 拒否・エッジ消費（`IN FLIGHT the switch request is CONSUMED`）、11.0 s に復帰。修正前は復帰の瞬間に POS_HOLD へ勝手に入った。地上（墜落後）では同じ拒否は先送りで、エピソード終了で自動適用。墜落はパイロットのスロットル操作 | vehicle |
| `tof_recover_climb_alt` | **ToF なしの 4.5 s の上昇で推定が 0.77 m ずれて復帰**（D2 の純粋な形、`--duration 25000000`）: 復帰の最初のサンプルは有効（0.75 m）だがイノベーション 0.77 m が判定 0.5 m を超える。修正前は全サンプルが棄却され alt_rmse 1.19 m。修正後 0.02 m。ALT スイッチ（11.0 s）は推定器の取り込み直しの後に受理され高度を保持 | vehicle |
| `tof_recover_reflip` | **飛行中に拒否したスイッチは消費され、復帰後の入れ直しで適用される**（D3、`--duration 24000000`）: ALT スイッチを ToF 出力停止中に入れ、復帰後も入れたままでは何も起きず（修正前は勝手に ALT_HOLD→パイロットの OFF で STABILIZE に戻った）、9.6 s の入れ直しで ALT_HOLD が入り高度保持（推定 RMS 0.017 m） | vehicle |
| `tof_recover_invalid_first` | **サンプルは戻るが 1.5 s の間全て無効、その後に有効**（D1、故障注入 `tof_invalid`、`--duration 27000000`）: 無効のまま届くサンプルは復帰でなく、エピソードは続く（`only invalid samples while flying`）。有効サンプルが戻ると推定器が取り込み直し（イノベーション 0.93 m）、推定は真値と一致（RMS 0.025 m） | vehicle |
| `tof_recover_prearm` | **復帰の 0.3 s 後の ARM は拒否、約 1 s 後の ARM は受理**（`--duration 14000000`）: 拒否の理由は `estimator has not re-acquired the ToF yet`（修正前はサンプルが届いただけで受理） | vehicle |
| `tof_invalid_alt_hover` | **ホバー中（ALT_HOLD）に ToF が無効サンプルだけを出し続ける**（D4、故障注入 `tof_invalid`、`--duration 27000000`）: 0.5 s 後に出力停止のエピソード（`only invalid samples while flying`）→ FLYING → LANDING → ToF なしの降下（接地速度 0.69 m/s）→ IDLE_GROUND。地上の無効は正常なので次の ARM は受理され、自動離陸は、上昇量の推定が 5 cm に達してから 0.15 s で打ち切られる（真値の最大高度 0.20 m。ノイズ n2 とホバー推力の誤差で 0.15〜0.41 m。以前の「TAKEOFF 突入から 0.5 s」は 0.16 m だが、離陸の遅い機体を打ち切った。継続 0.5 s のままだと n2 で 0.99〜1.24 m） | vehicle |
| `tof_recover_landing_invalid` | **ToF なしの着陸の最中にサンプルが戻るが、全部が無効**（C5、`--duration 22000000`）: LANDING の無効サンプルは一致でなく証拠として数える（最後の有効な高さ 0.5 m）ので、0.5 s 後に `only invalid samples while flying` と判定され、エピソードは接地まで続く。旧ビルドでは約 1 s で取り込み直し済みと判定されてToF なし降下が空中で放棄された | vehicle |
| `alt_ceiling_stick`・`alt_ceiling_stick_noise` | **ALT_HOLD でスロットルスティックを上げ続けても高度上限 1.2 m で止まる**（C3、`--duration 29000000`、後者は `--noise n2 --seed 1`）: 上昇は上限へ向けて絞られ（0.45 1/s × 距離）、真値 1.17〜1.18 m（n2 の 3 seed で 1.185〜1.187 m）で止まり、スティックを下げれば降下できる。上限が無いと同じスティックで 1.97 m まで上昇した | vehicle |
| `landing_dead_zone` | **近距離の不感帯を模擬した、柔らかい接地（3 G 未満）の ToF 着陸**（`alt_disarm_land` に不感帯を入れたもの。以前は既知失敗、2026-11-09 に PASS）。降下の最後の数 cm でセンサが無効になる（読みは真値に遅れ、真の高さ 0.038 m で 0.085 m）ので、LANDING の降下中に 0.10 m 以下の有効な読み（0.085 m）の後で無効サンプルだけが 0.2 s 続いたら接地とみなす（`TakeoffLandingMgr::deadZoneTouchdown()`、`detailed_design.md` §3 注7）。ログの `Landing detected (ToF dead zone ...)` を確認する。実測: 検出は最後の有効な読みの 0.2 s 後、真の高さ 0.021 m・鉛直速度 0.001 m/s で床の上。`alt_disarm_land` は `tof_deadzone 0` で DISARM の振り分けだけを検証する | vehicle |
| `tof_invalid_stabilize` | **STABILIZE 飛行中に無効サンプルだけが続く: 強制着陸しない**（D4、`--duration 17000000`）: 警告のみ（`NO forced landing, fly on`）、飛行中の POS_HOLD スイッチは拒否・消費 | vehicle |
| `weak_spool_stabilize` | **STABILIZE で、モータが弱く床がまだ支える TAKEOFF は、ToF が床の上で無効を返すと、床の上で FLYING になる（許容。8 回目の判断 D2）**（モータ健全度 0.9、`--duration 13000000`）: スロットルが離陸の閾値より上でホバーより下（生値 3100）の 3.5 s、機体は床の上にいて、ToF は最小レンジ未満（無効サンプルだけ）。TAKEOFF の 2 s + 0.15 s の後に ToF 出力停止のエピソード（`Ground contact unknown`）、`Takeoff complete → FLYING/STABILIZE`、`NO forced landing, fly on`（強制着陸しない）。その後パイロットのスロットルで浮く。5.0〜7.0 s の最大高度 < 0.05 m、浮いた後の最大高度 > 0.10 m を確認する（6・7 回目の「床の上では FLYING にしない」は加速度計に基づく仕組みで、8 回目で取り下げた。7 回目のビルドでは不合格、HEAD では合格） | vehicle |
| `weak_spool_alt` | **ALT_HOLD の自動離陸で、浮けない機体（健全度 0.85）は、ToF が床の上で無効を返すと、ToF 出力停止として床の上で LANDING に入り、床が機体を支えると検出されてモータが止まる（許容。8 回目の判断 D1）**（`--duration 16000000`）: 2 s + 0.15 s の後に `Downward ToF silence episode`・`TAKEOFF → LANDING`、ToF なしの降下の推力は支持推力の 0.95 倍以下なので浮かない。`Touchdown detected (the floor holds the craft up ...)`・`LANDING → IDLE_GROUND`、床の上の最大高度 < 0.05 m、離陸の完了も中止も無いことを確認する（7 回目のビルドでは不合格、HEAD では合格） | vehicle |
| `weak_spool_alt_offset` | **`weak_spool_alt` に加速度計のオフセット（上向きの読み +0.3 m/s²）を足した変種**（`--duration 16000000`）: 8 回目の判断は加速度計に基づく上昇量に依らないので、期待値は `weak_spool_alt` と同じ | vehicle |
| `weak_spool_stabilize_offset` | **`weak_spool_stabilize` に同じオフセットを足した変種**（`--duration 13000000`）: 期待値は `weak_spool_stabilize` と同じ | vehicle |
| `weak_spool_api` | **浮けない機体（健全度 0.85）の API `takeoff`: ToF 無効の規則が離陸を LANDING で終わらせ、`takeoff` は `error takeoff aborted` を返す**（`--duration 18000000`）: 約 8.6 s に `TAKEOFF → LANDING`、`Touchdown detected (the floor holds the craft up ...)` でモータが止まり、床の上の最大高度 < 0.05 m、15.5 s 以降の duty_max < 0.01 を確認する（`error takeoff timeout` は出ない。7 回目のビルドでは不合格、HEAD では合格） | vehicle |
| `weak_spool_alt_floor` | **健全な ToF が床を有効に読み（`tof_deadzone 0`）、読みが増えない機体（健全度 0.85）の自動離陸は、5 s で中止して DISARM**（8 回目の判断 D4、`--duration 16000000`）: 制御器の備え（3 s）が離陸完了を報告しても TAKEOFF に保ち（床の上で FLYING にならない）、約 10.8 s に `Auto-takeoff aborted: the healthy ToF has shown the craft on the floor for 5000 ms`、`TAKEOFF → IDLE_GROUND`、モータ停止（11.5 s 以降の duty_max < 0.01）、床の上の最大高度 < 0.05 m を確認する（HEAD では床の上で FLYING になり不合格、7 回目のビルドでは中止の文言が違い不合格） | vehicle |
| `weak_spool_alt_slow_lift` | **ゆっくり浮く機体（健全度 0.88、`tof_deadzone 0`）は中止されず、離陸を完了して飛ぶ**（8 回目の判断 D4、`--duration 25000000`）: ToF の読みが 0.02 → 0.17 m と増えるので床の証拠は無く、約 10 s に `Auto-takeoff complete`、`TAKEOFF → FLYING`、中止なし。浮いた後の最大高度 > 0.15 m、降下速度の最大 < 0.6 m/s を確認する（7 回目のビルドは 0.11〜0.17 m で DISARM して落下し不合格） | vehicle |
| `commloss_takeoff` | **完了しない TAKEOFF の間にリンクが途絶えると、ホバーの猶予（3 s）の後に LANDING になる**（8 回目の判断 D3、STABILIZE、健全度 0.9、`tof_deadzone 0`、`--duration 22000000`）: ToF が床を有効に読むので ToF による空中の判定は偽のまま、手動の TAKEOFF は完了しない。5.5 s からの 8 s の無音で `Comm lost → hover 3000 ms then LANDING`、`Comm lost → LANDING (hover grace elapsed)`、`TAKEOFF → LANDING`（HEAD と 7 回目のビルドでは TAKEOFF のまま着陸せず不合格） | vehicle |
| `ceiling_brake_stale_velocity` | **長い ToF 出力停止（オフセット −0.4 m/s²）の直後に ALT_HOLD へ入っても、天井ブレーキが古い加速度積分を信用しない**（7 回目の C1、`--duration 36000000`）: 積分は 3.7 m/s（上昇していない機体で）。6 回目は推力が重さの半分になり、機体は床へ 2.57 m/s で降りた（`alt_min` 0.01 m）。7 回目は `alt_min` 1.06 m、降下速度の最大 0.19 m/s | vehicle |
| `ceiling_brake_stale_velocity_alt` | **同上、1.24 m（天井）で ALT_HOLD へ入る**（7 回目の C1、`--duration 30000000`）: 6 回目は 0.59 m まで落下（`alt_min` 0.59 m）。7 回目は 1.18〜1.26 m を保つ | vehicle |
| `tof_silent_flip_end` | **宙返りの最中に ToF が出力停止: 宙返りが終わった瞬間に判断**（D6、`--duration 33000000 --param flip.enable=1`）: FLIP → FLYING の「その場で」LANDING（`ToF silence episode during the maneuver: decided now that FLIP is over`）、宙返りは中断されない（真値 -358 deg） | vehicle |
| `api_takeoff_emergency` | **`takeoff` の待機中に届いた `emergency` が即座に効く**: 6.0 s に takeoff、7.0 s（上昇中）に `emergency` → 7.04 s にモータ停止、`takeoff` は `error interrupted by emergency`。修正前は 7.76 s（待機の残り 0.76 s 後）。さらに待機中に届いた 2 回目の `takeoff`・`height?`・`rc` を **`emergency` が捨てる**（`error cancelled by emergency`、`rc` はその場で捨てる）ので再離陸しない（モータ停止が 7.5〜13 s 続く）。`--duration 14000000` | vehicle |
| `api_land_emergency` | **`land` の待機中の `emergency`**: 降下開始 0.6 s 後（16.6 s）に注入 → 16.64 s にモータ停止。修正前は接地まで待たされ 18.12 s。待機中に届いた `takeoff`・`left 100` は捨てられ再離陸しない。`--duration 22000000` | vehicle |
| `api_land_stop` | **`land` の待機中の `stop`**: `stop` は着陸中は意味を持たないので通常の拒否 `error not flying or guidance released` を接地を待たず直ちに返し、着陸は続行して接地する。`--duration 22000000` | vehicle |
| `disarm_long_idle` | **38 分の無音 DISARM のあとも次の飛行が壊れない**（`--duration 2300000000`、仮想時間なので実時間は約 3 分）: RC リンクを DISARM のまま 38 分（> 35.8 分）無音にし、再開後に ALT_HOLD の自動離陸→ホバー（0.5 m）→DISARM 自動着陸。出力停止の監視器が「停止」に固着しない（ARM 不可・誤った `Comm lost → LANDING` が起きない）ことを守る。途絶したリンクを生存と読む危険側はホスト試験 `test_sensor_freshness`（36/40/60 分の観測の空き）が担う | vehicle |
| `api_move_stop` | **`forward 150` の待機中の `stop`**: 開始 1.0 s 後に `stop` → 移動は `error interrupted by stop` で終わり、最大 0.14 m 進んだところで制動（修正前は 1.48 m 全行程を進んでから `stop`）。待機中に届いた `left 50` は `stop` が捨てる（`error cancelled by stop`、実行されない）。`--duration 26000000` | vehicle |
| `api_wait_deferred` | **待機中に届いた `emergency`/`stop` 以外のコマンドは捨てず順序も変えない**: `takeoff` の待機中に `height?` と 2 回目の `takeoff` を注入 → FLYING の後にこの順で実行され、2 回目は `error not on ground`。修正前と同じ結果。`--duration 20000000` | vehicle |

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
| `alt_auto_takeoff`・`pos_auto_takeoff` の `alt_band`（窓 12〜16 s） | < 0.20 → < 0.55（SILS の値 0.2668）。さらに下限 > 0.05 を 2026-11-01 にユーザー決定で追加 | (b) 旧閾値は旧プラントの値 0.008 に合わせたもので、実機の 14 回の離陸（ALT_HOLD 5・POS_HOLD 9、離陸から +4.2〜+8.2 s の ESKF 高度）はいずれも 0.062 以上。実機の最大 0.521 を切り上げた。p90 の 0.443 では実機の 2/14 回（0.500・0.521）を不合格にしてしまうため採らない | 実機 14 回: p50 0.164、p90 0.443、最大 0.521 |
| `api_flight` の `alt_mean` 下限（窓 14〜17.5 s） | > 0.42 → > 0.30（SILS の値 0.4130） | (b) 旧閾値は旧プラントの値 0.518 に合わせたもので、実機の 4/14 回が下回る。実機の最小 0.326 を切り下げた | 実機 14 回（離陸から +7.7〜+11.2 s）: 最小 0.326、p10 0.348、p50 0.473 |
| `calib.expect` の xfail 印 | 削除 | xfail の理由（0.6〜0.7 Hz の自励振動）が出なくなり、回帰試験 2 回で XPASS、直接実行 2 回で PASS（att_rmse 0.55 deg） | 実機の対応値なし |
| SILS の下向き ToF 模擬の最大距離（`simulator/sils/devices/vl53_device.cpp` の `MAX_MM`） | 1400 → 1630 mm | (b) 実機ログ（121 本＋flip_test1/2、約 15 万サンプル）では 1.4〜1.7 m で 95〜99 % が有効（status 0）、1.7〜1.8 m で 91 %、1.8〜2.0 m で 66〜70 %、2.0〜2.1 m で 22 %（無効時の status は 255 が大半。SILS も 255）。模擬の上限は 24 bin バッファが決める 1636 mm で、1630 は 2 bin 分割がバッファに収まる値（プローブで 1.63 m まで有効・誤差 17 mm 以内）。1.63〜2.0 m は未模擬のまま | 実機の有効率（高度は ESKF 高度） |
| `alt_inflight_switch.expect` の `alt_band` 上限（窓 14.7〜18.7 s） | < 0.32 → < 0.38（SILS の値 0.351） | STABILIZE→ALT_HOLD 捕捉後の減衰振動。閾値側のコメントが「開始外乱の大きさによらない固定振幅」と記録しているもの。R_out の値 0.351 に約 8 % の余裕 | 実機の対応値なし・未検証（飛行中に ALT_HOLD へ切り替えた実機ログが必要） |
| `crash_refly`（`.scn` 行J・行L、`.expect` の 2 つの窓） | 行J 2500→500 ms、行L 8000→10000 ms（再校正以降の時刻は不変）。`.expect`: 墜落静止の窓 14〜16 s → 12〜14 s、持上げの窓 16.8〜18.4 s → 14.8〜16.4 s（閾値は不変、手が届く時刻と同じ −2.0 s の移動） | (a) R_out の墜落は機体が横倒し（ロール 90 deg）で止まり、衝突の約 0.2 s 後に始まる 3.5 s の起動校正が横倒しのまま完了して level_offset 1.5708 rad が推定器に残る。手が水平に直した後も ARM の判定（姿勢推定が重力と食い違う、正しい挙動で不変）が再 ARM を拒否した。ハンドリング軌道は任意の墜落姿勢を水平へ戻すので、手を校正の最中（14.0 s）に届かせて水平な設置後に校正をやり直させる。再飛行は実際に飛ぶ（上昇 1.33 m、終端ホバー 0.87 m） | 実機の対応値なし |
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

**Added 2026-11-01 — `calib_side_lying` (vehicle, `--duration 39000000`):** regression for the level check of the still-calibration (detailed_design §3 note 10). A craft left lying on its side after a crash for longer than one 3.5 s calibration window keeps being rejected (`Calibration window rejected: not level`), so no calibration completes on its side and no accel_bias of about +-9.8 m/s^2 is seeded; once the hand places it level, calibration completes and ARM is accepted. It fails on a build without the level check (every ARM rejected by the tilt guard).

**Updated 2026-11-10 (eighth pass) — the independent reviews of the 7th pass found one root cause: "the craft has left the floor" was built from the accelerometer alone and gated safety decisions. A missing proof is no longer taken as proof that the craft is on the floor.** Withdrawn: the accelerometer lift evidence (`kVerticalVelocityLiftEvidenceMps2`, `left_floor`, `leftFloor()`, `kManualTakeoffMinRiseM`, `craftIsOffTheFloor()`, the workshop `ControllerStatus`); the ToF-unusable auto-takeoff (D1), the manual TAKEOFF -> FLYING (D2) and `tof_recover_*` / `tof_invalid_alt_hover` are back at their HEAD behaviour and inputs. New: TAKEOFF -> LANDING after the comm-loss grace (D3, `commloss_takeoff`), the auto-takeoff abort only on the POSITIVE floor evidence of a healthy ToF (D4, `weak_spool_alt_floor`, `weak_spool_alt_slow_lift`), the pure functions of `takeoff_policy.hpp` checked cell by cell by `test_takeoff_policy` (D6), `landingBaseThrust()` against the climb after a `land` (E1: 0.00-0.09 m, 7th build 0.65 m), `kBlindLandingMinSilentUs` 5.5 s (E3), the proportional ceiling brake (E4: at most 3 switches per run instead of 136). Not adopted: the sweep of `kTakeoffVelocityDisagreeMps` (E2). SILS, 94 scenarios with `.expect`: PASS 90 / FAIL 0 / KNOWN-FAIL 3 / SKIP 1 (the 7th build: PASS 79 / FAIL 11 on the same set). Host tests: 215. Tables in the Japanese "8th pass" section at the end of the matrix. Not verified on a real vehicle.

**Updated 2026-11-09 (sixth pass) — the remains of the independent review: the auto-takeoff no longer overshoots with a wrong hover-thrust estimate (R1), the support thrust before the calibration is the early estimate as it is (R2), a TAKEOFF that the floor still holds is not a flight (R3), the LANDING time limit needs 4 s of silence, the 5th-pass tables re-measured on the final build (R4); new scenarios `weak_spool_stabilize`, `weak_spool_alt`.**

Changes: (R1) the velocity loop of the auto-takeoff adds the part of the climb that the accelerometer integral shows and the ESKF does not (beyond 0.6 m/s: `takeoffClimbRate()`); the altitude-ceiling brake starts when the PREDICTED stop height (altitude + v x 80 ms + v^2 / (2 a), a = 0.7 x g x 0.5, v = max of the ESKF and the accelerometer-based climb rate) exceeds ceiling + 0.05 m, and its thrust is 0.5 x the SUPPORT thrust (not 0.5 x the hover estimate). (R2) before the support thrust is calibrated, the early estimate (quotient over all airborne samples x 0.92) is used as it is; the 0.75 x hover-estimate cut is only the start value. (R3) the time limit of the invalid-sample count in TAKEOFF needs `leftFloor()`; the manual TAKEOFF -> FLYING needs a 5 cm rise while a ToF-silence episode is in progress; the auto-takeoff time backstop needs the craft off the floor. (R4) the time limit of a LANDING with a silent ToF needs the LANDING time (10 s) AND 4 s of continuous silence; the other items are listed in `detailed_design.md` §3.1 note 12 (6th revision).

### 自動離陸・API 離陸の真値の最高高度（R1、目標 0.5 m、`--param hover.thrust.learn=0`）

`hover.thrust_corr` 0.9 / 1.0 / 1.2 × ノイズ n0 / n2（seed 12345・1・2）。最高高度は最初の LANDING までの真値。旧 = この見直しの前のビルド（HEAD）、新 = 最終ビルド。原因（旧ビルドの `hover.thrust_corr` 1.2・n2、`rv8_h12a`）: ESKF の鉛直速度は ToF の引き継ぎ高さ 0.15 m まで 0 に固定され、指令推力が重さの 1.27 倍のまま 1.2 m/s で上昇し、引き継ぎの後も推力は速度ループの下限（見積もりの 0.78 倍）で、上限の制動は 1.35 m を超えてから、しかも見積もりの 0.5 倍の推力で、1.96 m まで上がって ToF の有効範囲を出て LANDING に入り、2.51 m/s で接地した。

| 場面 | hover.thrust_corr | ノイズ | 旧の最高 [m] | 新の最高 [m] | ≤ 1.35 m |
|---|---|---|---|---|---|
| ALT_HOLD 自動離陸 | 0.9 | n0 | 0.59 | 0.59 | ○ |
| ALT_HOLD 自動離陸 | 0.9 | n2（3 seed の最大） | 0.84 | 0.83 | ○ |
| ALT_HOLD 自動離陸 | 1.0 | n0 | 0.84 | 0.84 | ○ |
| ALT_HOLD 自動離陸 | 1.0 | n2（3 seed の最大） | 1.19 | 1.17 | ○ |
| ALT_HOLD 自動離陸 | 1.2 | n0 | 1.43 | 1.25 | ○ |
| ALT_HOLD 自動離陸 | 1.2 | n2（3 seed の最大） | 1.96 | 1.25 | ○ |
| API 離陸 | 0.9 | n0 | 0.60 | 0.60 | ○ |
| API 離陸 | 0.9 | n2（3 seed の最大） | 0.83 | 0.82 | ○ |
| API 離陸 | 1.0 | n0 | 0.84 | 0.84 | ○ |
| API 離陸 | 1.0 | n2（3 seed の最大） | 1.17 | 1.14 | ○ |
| API 離陸 | 1.2 | n0 | 1.45 | 1.26 | ○ |
| API 離陸 | 1.2 | n2（3 seed の最大） | 2.03 | 1.27 | ○ |
| POS_HOLD 自動離陸 | 0.9 | n0 | 0.59 | 0.59 | ○ |
| POS_HOLD 自動離陸 | 0.9 | n2（3 seed の最大） | 0.84 | 0.83 | ○ |
| POS_HOLD 自動離陸 | 1.0 | n0 | 0.84 | 0.84 | ○ |
| POS_HOLD 自動離陸 | 1.0 | n2（3 seed の最大） | 1.19 | 1.17 | ○ |
| POS_HOLD 自動離陸 | 1.2 | n0 | 1.43 | 1.25 | ○ |
| POS_HOLD 自動離陸 | 1.2 | n2（3 seed の最大） | 1.96 | 1.26 | ○ |

最悪: 旧 2.03 m → 新 1.27 m（合格: 全条件で 1.35 m 以下）。n0・corr 1.2 で 1.45 → 1.26 m。corr 0.9・1.0 の高さは主にホバー推力の見積もりの誤差（学習を切っているので補正が ±0.15 N に制限される）によるもので、この見直しでは変えていない（普通の離陸の挙動を変えないために、速度の食い違いの余裕 0.6 m/s を置いた: 余裕なしの版は `alt_recenter_gate` の `alt_band` 0.193 → 0.211 と `landing_dead_zone`（接触が 3 G 未満でなくなる）を変えた）。

### 較正前の支持推力（R2）

実機ログ 3 本の再生（`front_tof_poshold_yaw360`・`front_tof_poshold_yaw4x90`・`flip_test2`。較正前に FLYING で出力停止が来たと仮定した全時刻）。「上限 ÷ 機体を支えた推力」= 較正前の上限（支持推力の 0.95 倍）÷ その時刻の前後 1 s の FLYING の平均指令推力:

| ログ | 時刻の数 | 旧（見積もり × 0.75 で切る） | 新（早い推定 × 0.92、そのまま） |
|---|---|---|---|
| front_tof_poshold_yaw360 | 7（離陸の開始から 2.0〜5.2 s） | 0.69〜0.71 | 0.87〜0.90 |
| front_tof_poshold_yaw4x90 | 1（3.45 s 浮上しても較正されない） | 0.63 | 0.86 |
| flip_test2 | 5 | 0.79〜0.91 | 0.85〜0.98 |

目標の 0.85 倍以上で、重さを超えない（最大 0.98）。0.94 では flip_test2 で 1.00 に届く（不採用）。補正の値の比較（ログ 3 本の較正前の最小〜最大）: 0.85: 0.79〜0.91、0.90: 0.83〜0.96、0.92: 0.85〜0.98、0.94: 0.87〜1.00。

SILS（出力停止を ARM の 0.9〜4.5 s 後に置く: 離陸直後の出力停止と短いホバー。`hover.thrust_corr` 0.9 / 1.0 / 1.2、ノイズ n0 / n2（seed 3 通り）、出力停止時の加速度計のオフセット −0.2 / 0 / +0.2 m/s²、`tof_silent_hover` 基準。1 条件 96 本）。上限 ÷ 真の重さ（同条件の較正済みの値）: 較正前は 0.86〜0.97（重さを超えない）、較正後 約 0.95。旧と新の比較（両方で場面が成立した 78 本）:

| ARM から出力停止までの時間 [s] | 実行数 | 出力停止時からの上昇増分 最大 [m] 旧→新 | 接地速度 最大 [m/s]（ノイズなし・n0）旧→新 | 接地速度 最大 [m/s]（n2）旧→新 |
|---|---|---|---|---|
| 0.9 | 12 | 0.20 → 0.18 | 1.83 → 1.39 | 2.63 → 1.91 |
| 1.2 | 12 | 0.15 → 0.20 | 1.93 → 1.38 | 2.67 → 2.01 |
| 1.6 | 9 | 0.08 → 0.08 | 1.91 → 1.38 | 2.10 → 1.76 |
| 2.0 | 9 | 0.05 → 0.05 | 1.88 → 1.42 | 2.04 → 1.76 |
| 2.5 | 9 | 0.05 → 0.04 | 1.81 → 1.43 | 1.96 → 1.65 |
| 3.0 | 9 | 0.00 → 0.00 | 1.76 → 1.41 | 1.86 → 1.56 |
| 3.5 | 9 | 0.00 → 0.00 | 0.92 → 0.92 | 1.83 → 1.53 |
| 4.5 | 9 | 0.00 → 0.00 | 0.75 → 0.85 | 1.59 → 1.36 |

再上昇は最大 0.007 m（旧: 0.000 m）。接地速度は、較正前（出力停止が ARM の 3.0 s 以内）で、ノイズなしの最大 1.93 → 1.43 m/s、n2 の最大 2.67 → 2.01 m/s に下がる。

### ToF なしの降下の評価表（6 回目、最終ビルド）

5 回目の見直しと同じ条件（6 場面 × 加速度計バイアス 5 通り（ToF 停止と同時 / 離陸開始から）× `hover.thrust_corr` 0.9 / 1.0 / 1.2 × ノイズ off / n0 / n2（3 seed）、1 場面 162 本、計 972 本）で最終ビルドを測り直した。旧 = この見直しの前のビルド（HEAD）。

| 場面 | corr | 成立/全 旧→新 | 出力停止時からの上昇増分 [m] 旧→新 | LANDING 以後の余分な上昇 [m] 旧→新 | 再上昇 [m] 旧→新 | 接地速度 [m/s]（ノイズなし・n0 最大）旧→新 | 接地速度 [m/s]（n2 最大）旧→新 | 空中停止 旧→新 | 床から再離陸 旧→新 |
|---|---|---|---|---|---|---|---|---|---|
| 自動離陸の上昇初期 | 0.9 | 54/54 → 54/54 | 0.58 → 0.49 | 0.11 → 0.09 | 0.000 → 0.000 | 0.54 → 0.54 | 1.87 → 1.45 | 0 → 0 | 0 → 0 |
| 自動離陸の上昇初期 | 1.0 | 54/54 → 54/54 | 0.77 → 0.65 | 0.13 → 0.11 | 0.000 → 0.000 | 1.00 → 0.86 | 2.05 → 1.61 | 0 → 0 | 0 → 0 |
| 自動離陸の上昇初期 | 1.2 | 54/54 → 54/54 | 0.98 → 0.90 | 0.14 → 0.12 | 0.000 → 0.000 | 1.22 → 0.89 | 2.08 → 1.61 | 0 → 0 | 0 → 0 |
| API 離陸中 | 0.9 | 54/54 → 54/54 | 0.47 → 0.43 | 0.10 → 0.08 | 0.000 → 0.000 | 0.39 → 0.39 | 1.59 → 1.28 | 0 → 0 | 0 → 0 |
| API 離陸中 | 1.0 | 54/54 → 54/54 | 0.82 → 0.63 | 0.17 → 0.11 | 0.000 → 0.000 | 0.84 → 0.84 | 1.98 → 1.47 | 0 → 0 | 0 → 0 |
| API 離陸中 | 1.2 | 54/54 → 54/54 | 1.19 → 0.95 | 0.22 → 0.17 | 0.000 → 0.000 | 1.10 → 0.72 | 2.05 → 1.58 | 0 → 0 | 0 → 0 |
| 0.5 m ホバー | 0.9 | 54/54 → 54/54 | 0.03 → 0.02 | 0.03 → 0.03 | 0.000 → 0.000 | 0.88 → 0.88 | 1.53 → 1.31 | 0 → 0 | 0 → 0 |
| 0.5 m ホバー | 1.0 | 54/54 → 54/54 | 0.00 → 0.01 | 0.09 → 0.07 | 0.000 → 0.000 | 0.88 → 0.88 | 1.44 → 1.22 | 0 → 0 | 0 → 0 |
| 0.5 m ホバー | 1.2 | 27/54 → 54/54 | 0.00 → 0.01 | 0.07 → 0.08 | 0.000 → 0.000 | 0.91 → 0.93 | - → 1.85 | 0 → 0 | 0 → 0 |
| 1.2 m ホバー | 0.9 | 54/54 → 54/54 | 0.02 → 0.02 | 0.02 → 0.02 | 0.000 → 0.000 | 1.01 → 1.02 | 1.42 → 1.43 | 0 → 0 | 0 → 0 |
| 1.2 m ホバー | 1.0 | 54/54 → 54/54 | 0.01 → 0.01 | 0.03 → 0.03 | 0.002 → 0.002 | 1.00 → 1.00 | 1.37 → 1.38 | 0 → 0 | 0 → 0 |
| 1.2 m ホバー | 1.2 | 27/54 → 54/54 | 0.00 → 0.01 | 0.03 → 0.04 | 0.000 → 0.000 | 0.89 → 0.92 | - → 1.41 | 0 → 0 | 0 → 0 |
| ALT_HOLD 手動上昇中 | 0.9 | 54/54 → 54/54 | 0.13 → 0.13 | 0.03 → 0.03 | 0.000 → 0.000 | 1.02 → 1.03 | 2.02 → 1.68 | 0 → 0 | 0 → 0 |
| ALT_HOLD 手動上昇中 | 1.0 | 54/54 → 54/54 | 0.13 → 0.13 | 0.01 → 0.02 | 0.000 → 0.000 | 1.03 → 1.03 | 2.02 → 1.75 | 0 → 0 | 0 → 0 |
| ALT_HOLD 手動上昇中 | 1.2 | 27/54 → 54/54 | 0.14 → 0.12 | 0.02 → 0.02 | 0.000 → 0.000 | 1.09 → 1.19 | - → 2.04 | 0 → 0 | 0 → 0 |
| STABILIZE 21 s 飛行 → 通信断 | 0.9 | 20/54 → 20/54 | 0.00 → 0.00 | 0.00 → 0.00 | 0.000 → 0.000 | 0.76 → 0.76 | - → - | 0 → 0 | 0 → 0 |
| STABILIZE 21 s 飛行 → 通信断 | 1.0 | 20/54 → 20/54 | 0.00 → 0.00 | 0.00 → 0.00 | 0.000 → 0.000 | 0.76 → 0.76 | - → - | 0 → 0 | 0 → 0 |
| STABILIZE 21 s 飛行 → 通信断 | 1.2 | 20/54 → 20/54 | 0.00 → 0.00 | 0.00 → 0.00 | 0.000 → 0.000 | 0.76 → 0.76 | - → - | 0 → 0 | 0 → 0 |

成立した実行: 旧 789 本 / 新 870 本。空中停止: 旧 0 / 新 0; 再上昇>0.01 m: 旧 0 / 新 0; 床から再離陸: 旧 0 / 新 0; 未終了: 旧 0 / 新 0

### 加速度計バイアス別（hover.thrust_corr 0.9 / 1.0 / 1.2 を合わせた最悪、旧 → 新）

| 場面 | 加速度計バイアス | 成立/全 旧→新 | 出力停止時からの上昇増分 最大 [m] | 接地速度 最大 [m/s]（ノイズなし・n0） | 接地速度 最大 [m/s]（n2） | 再上昇 最大 [m] |
|---|---|---|---|---|---|---|
| 自動離陸の上昇初期 | -0.2 m/s²（ToF 停止と同時） | 18/18 → 18/18 | 0.95 → 0.79 | 1.14 → 0.77 | 2.02 → 1.57 | 0.000 → 0.000 |
| 自動離陸の上昇初期 | -0.2 m/s²（離陸開始から） | 18/18 → 18/18 | 0.94 → 0.75 | 1.22 → 0.89 | 2.08 → 1.61 | 0.000 → 0.000 |
| 自動離陸の上昇初期 | -0.1 m/s²（ToF 停止と同時） | 18/18 → 18/18 | 0.96 → 0.80 | 1.10 → 0.70 | 1.98 → 1.54 | 0.000 → 0.000 |
| 自動離陸の上昇初期 | -0.1 m/s²（離陸開始から） | 18/18 → 18/18 | 0.95 → 0.78 | 1.14 → 0.76 | 2.01 → 1.57 | 0.000 → 0.000 |
| 自動離陸の上昇初期 | +0.0 m/s²（なし） | 18/18 → 18/18 | 0.96 → 0.82 | 1.05 → 0.62 | 1.95 → 1.51 | 0.000 → 0.000 |
| 自動離陸の上昇初期 | +0.1 m/s²（ToF 停止と同時） | 18/18 → 18/18 | 0.97 → 0.83 | 1.01 → 0.55 | 1.93 → 1.47 | 0.000 → 0.000 |
| 自動離陸の上昇初期 | +0.1 m/s²（離陸開始から） | 18/18 → 18/18 | 0.97 → 0.86 | 0.99 → 0.86 | 1.90 → 1.46 | 0.000 → 0.000 |
| 自動離陸の上昇初期 | +0.2 m/s²（ToF 停止と同時） | 18/18 → 18/18 | 0.97 → 0.85 | 0.98 → 0.86 | 1.90 → 1.45 | 0.000 → 0.000 |
| 自動離陸の上昇初期 | +0.2 m/s²（離陸開始から） | 18/18 → 18/18 | 0.98 → 0.90 | 0.96 → 0.84 | 1.87 → 1.43 | 0.000 → 0.000 |
| API 離陸中 | -0.2 m/s²（ToF 停止と同時） | 18/18 → 18/18 | 1.16 → 0.92 | 1.10 → 0.84 | 2.05 → 1.58 | 0.000 → 0.000 |
| API 離陸中 | -0.2 m/s²（離陸開始から） | 18/18 → 18/18 | 1.16 → 0.94 | 1.06 → 0.79 | 2.03 → 1.54 | 0.000 → 0.000 |
| API 離陸中 | -0.1 m/s²（ToF 停止と同時） | 18/18 → 18/18 | 1.17 → 0.93 | 1.05 → 0.81 | 2.00 → 1.50 | 0.000 → 0.000 |
| API 離陸中 | -0.1 m/s²（離陸開始から） | 18/18 → 18/18 | 1.17 → 0.93 | 1.03 → 0.79 | 1.99 → 1.49 | 0.000 → 0.000 |
| API 離陸中 | +0.0 m/s²（なし） | 18/18 → 18/18 | 1.17 → 0.94 | 1.00 → 0.79 | 1.97 → 1.43 | 0.000 → 0.000 |
| API 離陸中 | +0.1 m/s²（ToF 停止と同時） | 18/18 → 18/18 | 1.18 → 0.94 | 0.96 → 0.77 | 1.93 → 1.39 | 0.000 → 0.000 |
| API 離陸中 | +0.1 m/s²（離陸開始から） | 18/18 → 18/18 | 1.18 → 0.94 | 0.98 → 0.79 | 1.94 → 1.39 | 0.000 → 0.000 |
| API 離陸中 | +0.2 m/s²（ToF 停止と同時） | 18/18 → 18/18 | 1.19 → 0.95 | 0.93 → 0.75 | 1.90 → 1.36 | 0.000 → 0.000 |
| API 離陸中 | +0.2 m/s²（離陸開始から） | 18/18 → 18/18 | 1.19 → 0.94 | 0.98 → 0.79 | 1.89 → 1.35 | 0.000 → 0.000 |
| 0.5 m ホバー | -0.2 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.02 → 0.01 | 0.75 → 0.78 | 1.38 → 1.77 | 0.000 → 0.000 |
| 0.5 m ホバー | -0.2 m/s²（離陸開始から） | 15/18 → 18/18 | 0.01 → 0.01 | 0.91 → 0.93 | 1.53 → 1.85 | 0.000 → 0.000 |
| 0.5 m ホバー | -0.1 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.02 → 0.02 | 0.70 → 0.74 | 1.37 → 1.75 | 0.000 → 0.000 |
| 0.5 m ホバー | -0.1 m/s²（離陸開始から） | 15/18 → 18/18 | 0.02 → 0.01 | 0.79 → 0.80 | 1.00 → 1.78 | 0.000 → 0.000 |
| 0.5 m ホバー | +0.0 m/s²（なし） | 15/18 → 18/18 | 0.02 → 0.02 | 0.66 → 0.70 | 1.36 → 1.73 | 0.000 → 0.000 |
| 0.5 m ホバー | +0.1 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.02 → 0.02 | 0.63 → 0.66 | 1.36 → 1.71 | 0.000 → 0.000 |
| 0.5 m ホバー | +0.1 m/s²（離陸開始から） | 15/18 → 18/18 | 0.01 → 0.01 | 0.56 → 0.63 | 1.37 → 1.57 | 0.000 → 0.000 |
| 0.5 m ホバー | +0.2 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.03 → 0.02 | 0.59 → 0.63 | 1.35 → 1.70 | 0.000 → 0.000 |
| 0.5 m ホバー | +0.2 m/s²（離陸開始から） | 15/18 → 18/18 | 0.02 → 0.02 | 0.48 → 0.58 | 1.44 → 1.71 | 0.000 → 0.000 |
| 1.2 m ホバー | -0.2 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.02 → 0.02 | 1.00 → 1.00 | 1.34 → 1.34 | 0.000 → 0.000 |
| 1.2 m ホバー | -0.2 m/s²（離陸開始から） | 15/18 → 18/18 | 0.01 → 0.01 | 1.01 → 1.02 | 1.42 → 1.43 | 0.000 → 0.000 |
| 1.2 m ホバー | -0.1 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.02 → 0.02 | 0.95 → 0.95 | 1.32 → 1.33 | 0.000 → 0.000 |
| 1.2 m ホバー | -0.1 m/s²（離陸開始から） | 15/18 → 18/18 | 0.02 → 0.02 | 0.96 → 0.96 | 1.33 → 1.35 | 0.000 → 0.000 |
| 1.2 m ホバー | +0.0 m/s²（なし） | 15/18 → 18/18 | 0.02 → 0.02 | 0.91 → 0.91 | 1.31 → 1.31 | 0.000 → 0.000 |
| 1.2 m ホバー | +0.1 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.02 → 0.02 | 0.87 → 0.87 | 1.29 → 1.29 | 0.000 → 0.000 |
| 1.2 m ホバー | +0.1 m/s²（離陸開始から） | 15/18 → 18/18 | 0.01 → 0.01 | 0.85 → 0.85 | 1.25 → 1.25 | 0.000 → 0.000 |
| 1.2 m ホバー | +0.2 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.02 → 0.02 | 0.83 → 0.83 | 1.26 → 1.26 | 0.000 → 0.000 |
| 1.2 m ホバー | +0.2 m/s²（離陸開始から） | 15/18 → 18/18 | 0.01 → 0.01 | 0.79 → 0.79 | 1.15 → 1.16 | 0.002 → 0.002 |
| ALT_HOLD 手動上昇中 | -0.2 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.14 → 0.11 | 0.91 → 0.99 | 1.84 → 1.95 | 0.000 → 0.000 |
| ALT_HOLD 手動上昇中 | -0.2 m/s²（離陸開始から） | 15/18 → 18/18 | 0.13 → 0.13 | 1.09 → 1.19 | 2.00 → 2.04 | 0.000 → 0.000 |
| ALT_HOLD 手動上昇中 | -0.1 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.14 → 0.12 | 0.87 → 0.95 | 1.83 → 1.93 | 0.000 → 0.000 |
| ALT_HOLD 手動上昇中 | -0.1 m/s²（離陸開始から） | 15/18 → 18/18 | 0.14 → 0.13 | 0.96 → 1.06 | 2.02 → 2.01 | 0.000 → 0.000 |
| ALT_HOLD 手動上昇中 | +0.0 m/s²（なし） | 15/18 → 18/18 | 0.14 → 0.12 | 0.82 → 0.91 | 1.81 → 1.95 | 0.000 → 0.000 |
| ALT_HOLD 手動上昇中 | +0.1 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.14 → 0.12 | 0.78 → 0.87 | 1.80 → 1.94 | 0.000 → 0.000 |
| ALT_HOLD 手動上昇中 | +0.1 m/s²（離陸開始から） | 15/18 → 18/18 | 0.14 → 0.12 | 0.65 → 0.74 | 1.97 → 1.69 | 0.000 → 0.000 |
| ALT_HOLD 手動上昇中 | +0.2 m/s²（ToF 停止と同時） | 15/18 → 18/18 | 0.14 → 0.12 | 0.74 → 0.83 | 1.79 → 1.92 | 0.000 → 0.000 |
| ALT_HOLD 手動上昇中 | +0.2 m/s²（離陸開始から） | 15/18 → 18/18 | 0.13 → 0.13 | 0.53 → 0.57 | 2.02 → 1.86 | 0.000 → 0.000 |
| STABILIZE 21 s 飛行 → 通信断 | -0.2 m/s²（ToF 停止と同時） | 0/18 → 0/18 | - → - | - → - | - → - | - → - |
| STABILIZE 21 s 飛行 → 通信断 | -0.2 m/s²（離陸開始から） | 6/18 → 6/18 | 0.00 → 0.00 | 0.61 → 0.61 | - → - | 0.000 → 0.000 |
| STABILIZE 21 s 飛行 → 通信断 | -0.1 m/s²（ToF 停止と同時） | 0/18 → 0/18 | - → - | - → - | - → - | - → - |
| STABILIZE 21 s 飛行 → 通信断 | -0.1 m/s²（離陸開始から） | 9/18 → 9/18 | 0.00 → 0.00 | 0.63 → 0.63 | - → - | 0.000 → 0.000 |
| STABILIZE 21 s 飛行 → 通信断 | +0.0 m/s²（なし） | 9/18 → 9/18 | 0.00 → 0.00 | 0.62 → 0.62 | - → - | 0.000 → 0.000 |
| STABILIZE 21 s 飛行 → 通信断 | +0.1 m/s²（ToF 停止と同時） | 9/18 → 9/18 | 0.00 → 0.00 | 0.74 → 0.74 | - → - | 0.000 → 0.000 |
| STABILIZE 21 s 飛行 → 通信断 | +0.1 m/s²（離陸開始から） | 9/18 → 9/18 | 0.00 → 0.00 | 0.76 → 0.76 | - → - | 0.000 → 0.000 |
| STABILIZE 21 s 飛行 → 通信断 | +0.2 m/s²（ToF 停止と同時） | 9/18 → 9/18 | 0.00 → 0.00 | 0.75 → 0.75 | - → - | 0.000 → 0.000 |
| STABILIZE 21 s 飛行 → 通信断 | +0.2 m/s²（離陸開始から） | 9/18 → 9/18 | 0.00 → 0.00 | 0.76 → 0.76 | - → - | 0.000 → 0.000 |

旧 → 新で成立数が増えた場面（0.5 m ホバー・1.2 m ホバー・ALT_HOLD 手動上昇中の 15/18 → 18/18）は、旧ビルドでは離陸が行き過ぎて場面が成立しなかった `hover.thrust_corr` 1.2・ノイズ n2 の 3 本が加わったためで、n2 の最大が増えて見える（同じ条件どうしの比較は上の `hover.thrust_corr` 別の表: 0.5 m ホバー n2 の最大は corr 0.9 で 1.53 → 1.31、corr 1.0 で 1.44 → 1.22 m/s）。加わった 3 本の n2 の接地速度は 0.5 m ホバーで最大 1.85、1.2 m ホバーで 1.41、手動上昇中で 2.04 m/s。

空中でのモータ停止は 0 件のまま、降下が始まった後の再上昇は 0.01 m 以下、床からの再離陸は 0 件。`hover.thrust_corr` 1.2 のノイズ n2 は、旧ビルドでは離陸が行き過ぎて場面が成立しなかった（27/54）が、新ビルドでは成立する（54/54）。STABILIZE の場面は n2 の open-loop の機体が上昇し続けるため、旧・新とも成立しない（20/54）。

### 再確認試験（SILS 全件）と既存シナリオの metric の変化（6 回目）

再確認試験（変更で既存の動作が壊れていないかを自動で確かめる試験）の結果。`.expect` 付き 86 本（既存 84 本 + 新規 2 本）: **PASS 82 / FAIL 0 / KNOWN-FAIL 3（`commloss_land_level`・`pos_yaw`・`yaw_hold`）/ SKIP 1（`workshop_acro`）**。基準（84 本で PASS 80 / FAIL 0 / KNOWN-FAIL 3 / SKIP 1）に新規 2 本の PASS を足した形で、合否の変化は無い。既存 84 本のうち `disarm_long_idle`・`workshop_*` を除く全シナリオの metric 321 件を旧ビルドと最終ビルドで比べた: 259 件は同じ値、62 件が変化したが、閾値と合否は変わらない（KNOWN-FAIL の 3 本も同じ理由で不合格のまま）。変化の理由:

| 変化したシナリオ | 値の変化 | 理由 |
|---|---|---|
| `tof_silent_takeoff`・`tof_silent_takeoff_est_high`・`tof_silent_takeoff_bias_est_high`・`tof_silent_api_takeoff_est_high`・`tof_silent_takeoff_noise`・`tof_silent_est_high`・`tof_silent_est_high_noise` | `descent_speed_max` 0.92 → 0.54、1.04 → 0.59、0.97 → 0.36、1.00 → 0.56、1.99 → 1.58、0.98 → 0.96。`alt_max` は 0.72 → 0.62（api est_high）、1.20 → 1.10（takeoff_noise）、1.49 → 1.25（est_high_noise） | R2（較正前の支持推力を早い推定のままにした: 上限が高くなり、降下が遅くなる）と R1（離陸の行き過ぎが減る） |
| `calib` | `att_rmse` 0.0015 → 0.0071 rad（0.40 deg）、`tilt_max` 0.011 → 0.020 rad（閾値 0.087・0.314） | この飛行は 1.2〜1.34 m（STABILIZE で上昇し、ALT_HOLD に入る）で、R1 の「止まる高さの予測」の制動が 8 s の高度を 1.34 → 1.21 m に変えた。姿勢の推定誤差の窓が変わっただけで、制御則の姿勢側は不変 |
| `crash_refly`・`alt_ceiling_stick_noise`・`api_flight`・`boot_motion`・`pos_flight`・`tof_recover_invalid_first`・`tof_recover_reflip`・`commloss_land_level` | `alt_max` 1.38 → 1.22・1.37 → 1.23（crash_refly）、`duty_max` 0.69 → 0.66（boot_motion）、`alt_rmse` 0.024 → 0.012（tof_recover_reflip）など | 上限（1.2 m）の近くを飛ぶ・離陸が行き過ぎる飛行で、R1 の制動が高度を少し変えた（どれも閾値の内側）。`commloss_land_level` は着陸後の `alt_max` 0.321 → 0.283 m で、閾値 0.1 m を超えたまま（KNOWN-FAIL のまま） |
| `api_flip_pitch`・`api_flip_roll`・`flip_abort_recover`・`flip_brake_timeout` | `alt_min` 0.99 → 1.06、1.09、`alt_drop_max` 0.027 → 0.079 など（宙返りの角度は 360〜364 deg で同じ） | 宙返りの前の上昇（`up 70` で 1.2 m）の高度が R1 の制動で変わり、宙返りの開始高度が少し違う。宙返りは今回の対象外（持ち主の決定）で、角度・整定時間の判定は変わらない |
| `weak_spool_alt` | `alt_mean` 0.021 → 0.330 | 新規シナリオ（旧ビルドでは床の上で LANDING に入り不合格） |

### STABILIZE・ToF 出力停止中の通信断で上昇中の機体を止めにくい件（R4、採否の評価）

案: 速度推定を信用できない（最後のリセットから `kVerticalVelocityTrustS` = 2 s を超えた）ときも、降下の開始時の推定速度が上向きなら、推定速度 ÷ 0.75 g（最大 1 s）だけ下限推力（支持推力の 0.25 倍）で減速する。

1. 推定の誤差: `tof_silent_stabilize_commloss`（21 s 出力停止）の LANDING 開始時の推定は、加速度計のオフセット 0 / +0.1 / +0.2 m/s² で、誤差 +0.23 / −2.22 / −4.68 m/s（ノイズなし）。STABILIZE の上昇は屋内で 1 m/s 前後なので、誤差は信号より大きい。
2. 1 次元の力学モデル（80 ms の遅れ、後半の低下込み。屋内の 1 m・2 m から）:

| 高さ [m] | 真の上昇 [m/s] | 推定の誤差 [m/s] | 案なし 最高 / 接地速度 | 案あり 最高 / 接地速度 |
|---|---|---|---|---|
| 1.0 | 0 | +2 | 1.00 m / 0.99 m/s | 1.00 m / **2.20 m/s** |
| 1.0 | +1 | 0 | 2.02 m / 1.41 m/s | 1.13 m / 1.06 m/s |
| 1.0 | +1 | −2 | 2.02 m / 1.41 m/s | 2.02 m / 1.41 m/s（効かない） |
| 1.0 | +1 | +2 | 2.02 m / 1.41 m/s | 1.12 m / **2.23 m/s** |
| 1.0 | +2 | 0 | 5.08 m / 6.30 m/s | 1.41 m / 1.18 m/s |
| 1.0 | +2 | −2 | 5.08 m / 6.30 m/s | 5.08 m / 6.30 m/s（効かない） |
| 2.0 | 0 | +2 | 2.00 m / 1.40 m/s | 2.00 m / **2.42 m/s** |
| 2.0 | +2 | 0 | 6.08 m / 7.18 m/s | 2.41 m / 1.54 m/s |

3. 判定: **不採用**。効くのは推定の誤差が上昇の速さより小さいときだけ（誤差 0 の行）で、21 s の出力停止の後の誤差（±2〜5 m/s）はそれより大きく、誤差が上向きのとき（約半分）は、ホバー中の機体の接地速度が 1.0 → 2.2 m/s（1 m から）、1.4 → 2.4 m/s（2 m から）に上がる。誤差が小さい（信用時間の 2 s 以内）ときは既存の PI が上昇を止める。上昇中の機体が 3 m の天井へ届く恐れ（真の上昇 2 m/s で 5〜6 m）は残るが、案はそれを防げない（誤差が逆向きの行）。残る対策は ToF の復旧・気圧計の追加（次の段階）。


**Updated 2026-11-09 (fifth pass) — the descent without a ToF can no longer climb: the thrust is always at most 0.95 x the support thrust; the support thrust is calibrated over near-hover samples after the craft left the floor; the velocity estimate is trusted for 2 s after its last reset; a floor-holds-the-craft touchdown path; the 3 G impact in a blind LANDING is the touchdown; the LANDING time limit counts from the entry into LANDING; the altitude ceiling is effective; new scenarios `tof_silent_bias_plus`, `tof_silent_bias_minus`, `tof_silent_takeoff_bias_est_high`, `tof_silent_est_high_noise`, `tof_silent_stabilize_commloss`.** An independent review (real logs + SILS) showed that the accelerometer-integral velocity drifts with the time since its last reset (real logs through the estimator after a ToF loss in the hover: -0.21 m/s after 1 s, -0.25 .. -0.39 after 3 s, -0.44 .. -0.71 after 5 s; flip_test2 -0.34 after 2 s) and that a PI which raises the thrust against a "too fast" descent climbs when the estimate is low (old build, accelerometer offset 0.1 / 0.15 / 0.2 m/s^2: rises again, touchdown at 1.43 / 3.38 / 5.06 m/s; a soft touchdown followed by a re-take-off to 1.21 m). Changes: (1) thrust <= `kNoTofThrustCapRatio` (0.95) x the support thrust always (the hover ESTIMATE is no reference of the ceiling); (2) the support thrust = mean thrust x g / mean upward reading over the samples within 5 % of g, from the moment the craft left the floor (healthy ToF says airborne / rise 0.10 m / reading 1 m/s^2 above g), 2 s of them; without a window min(quotient over all airborne samples x 0.85, 0.75 x the hover estimate); (3) the PI only while the velocity estimate is within 2 s of its last reset, else the constant ceiling (also when a manual flight with the ToF stopped for 20+ s ends in LANDING); (4) touchdown = impact then stillness OR the floor holding the craft up (thrust < 0.7 x support and |accel| within 0.1 g of 1 g for 0.5 s), and the 3 G impact in a blind LANDING completes the landing (`Touchdown while the ToF is silent (impact)`, `lastLandingEnd()` = `ToFSilentImpact`) instead of an emergency DISARM; (5) above the ceiling + 0.15 m no climb command, the target returns to the ceiling, thrust <= 0.5 x the estimate while still rising; the altitude velocity loop is reset when the ToF-alive landing resumes; (6) the 10 s limit counts from the entry into LANDING (`LandingClock`, host test). Evaluation: 6 scenes x accelerometer offset error -0.2 / -0.1 / 0 / +0.1 / +0.2 m/s^2 (injected with the silence, and 0.3 s after the ARM / the takeoff command) x `hover.thrust_corr` 0.9 / 1.0 / 1.2 x noise off / n0 / n2 (seeds 12345, 1, 2; 1134 runs, 951 valid: in the others the scene itself fails before the silence - the 1.2 m hover and the stick climb with `hover.thrust_corr` 1.2 and n2, the open-loop STABILIZE program with another plant). Result over the 951 valid runs: motor stop in the air 0, re-rise after the descent started 0 (largest 0.002 m), re-take-off from the floor 0, runs without a touchdown 0. NOT met: touchdown speed <= 0.8 m/s with noise off / n0 in the 1.2 m hover (0.79-1.04 m/s), the early climb (up to 1.22) and the stick climb (up to 1.10); with noise n2 up to 2.08 m/s (the n2 plant hovers at 0.30 N against 0.34 N, so the lowered estimate of an uncalibrated start sinks the craft harder); the rise after LANDING beyond the braking distance of the climb is 0.10-0.22 m in the two early-climb scenes (limit 0.1 m; the braking assumes 7.4 m/s^2). The tables below were re-measured after the PI of the descent got a one-sided anti-windup that acts in the same cycle (`blind_descent.hpp`, 2026-11-09, sixth pass): the same 1134 runs and 951 valid ones, the four zero counts above unchanged (largest re-rise 0.002 m), and the touchdown speed moved by at most +0.04 m/s (the integral now unwinds while the thrust is clamped, so the descent is slightly faster). Tables (worst over the cells; "rise" = max altitude after the silence minus the altitude at the silence; "extra rise after LANDING" = max altitude after LANDING minus the altitude at LANDING minus v^2 / (2 x 7.4 m/s^2) of the climb speed at LANDING; "time" = silence to contact):

### hover.thrust_corr 別（出力停止時の加速度計バイアス −0.2〜+0.2 m/s² の 9 通りの注入 × 3 通りの時刻 × ノイズ off / n0 / n2）

| 場面 | 条件 | 有効/全 | 出力停止時からの上昇増分 [m] | LANDING 以後の余分な上昇 [m] | 再上昇 [m] | 出力停止→接地 [s]（ノイズなし最大） | 接地速度 [m/s]（ノイズなし・n0 最大） | 接地速度 [m/s]（n2 最大） | 空中停止 | 床から再離陸 | 終わり方 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 自動離陸の上昇初期 (tof_silent_takeoff) | hover.thrust_corr 0.9 | 63/63 | 0.58 | 0.11 | 0.000 | 0.5 | 0.55 | 1.87 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| 自動離陸の上昇初期 (tof_silent_takeoff) | hover.thrust_corr 1.0 | 63/63 | 0.77 | 0.13 | 0.000 | 0.9 | 1.00 | 2.05 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| 自動離陸の上昇初期 (tof_silent_takeoff) | hover.thrust_corr 1.2 | 63/63 | 0.98 | 0.14 | 0.000 | 1.9 | 1.22 | 2.08 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| API 離陸中 (tof_silent_api_takeoff) | hover.thrust_corr 0.9 | 63/63 | 0.47 | 0.10 | 0.000 | 0.5 | 0.40 | 1.59 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| API 離陸中 (tof_silent_api_takeoff) | hover.thrust_corr 1.0 | 63/63 | 0.82 | 0.17 | 0.000 | 0.8 | 0.84 | 1.98 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| API 離陸中 (tof_silent_api_takeoff) | hover.thrust_corr 1.2 | 63/63 | 1.19 | 0.22 | 0.000 | 1.8 | 1.10 | 2.05 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| 0.5 m ホバー (tof_silent_hover) | hover.thrust_corr 0.9 | 63/63 | 0.03 | 0.03 | 0.000 | 2.4 | 0.88 | 1.53 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| 0.5 m ホバー (tof_silent_hover) | hover.thrust_corr 1.0 | 63/63 | 0.00 | 0.09 | 0.000 | 2.6 | 0.88 | 1.44 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| 0.5 m ホバー (tof_silent_hover) | hover.thrust_corr 1.2 | 36/63 | 0.00 | 0.07 | 0.000 | 2.0 | 0.91 | - | 0 | 0 | 衝撃(ToF なし接地) 36 |
| 1.2 m ホバー (tof_silent_hover_1p2) | hover.thrust_corr 0.9 | 63/63 | 0.02 | 0.02 | 0.000 | 2.8 | 1.04 | 1.42 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| 1.2 m ホバー (tof_silent_hover_1p2) | hover.thrust_corr 1.0 | 63/63 | 0.01 | 0.03 | 0.002 | 2.7 | 1.00 | 1.37 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| 1.2 m ホバー (tof_silent_hover_1p2) | hover.thrust_corr 1.2 | 36/63 | 0.00 | 0.03 | 0.000 | 2.3 | 0.90 | - | 0 | 0 | 衝撃(ToF なし接地) 36 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | hover.thrust_corr 0.9 | 63/63 | 0.13 | 0.03 | 0.000 | 3.6 | 1.02 | 2.02 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | hover.thrust_corr 1.0 | 63/63 | 0.13 | 0.01 | 0.000 | 3.5 | 1.04 | 2.02 | 0 | 0 | 衝撃(ToF なし接地) 63 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | hover.thrust_corr 1.2 | 36/63 | 0.14 | 0.02 | 0.000 | 3.8 | 1.10 | - | 0 | 0 | 衝撃(ToF なし接地) 36 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | hover.thrust_corr 0.9 | 29/63 | 0.05 | 0.05 | 0.000 | 3.1 | 0.76 | - | 0 | 0 | 衝撃(ToF なし接地) 27、衝撃のあとの静止 2 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | hover.thrust_corr 1.0 | 29/63 | 0.05 | 0.05 | 0.000 | 3.1 | 0.76 | - | 0 | 0 | 衝撃(ToF なし接地) 27、衝撃のあとの静止 2 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | hover.thrust_corr 1.2 | 29/63 | 0.05 | 0.05 | 0.000 | 3.1 | 0.76 | - | 0 | 0 | 衝撃(ToF なし接地) 29 |

### 加速度計バイアス別（hover.thrust_corr 0.9 / 1.0 / 1.2 を合わせた最悪）

| 場面 | 条件 | 有効/全 | 出力停止時からの上昇増分 [m] | LANDING 以後の余分な上昇 [m] | 再上昇 [m] | 出力停止→接地 [s]（ノイズなし最大） | 接地速度 [m/s]（ノイズなし・n0 最大） | 接地速度 [m/s]（n2 最大） | 空中停止 | 床から再離陸 | 終わり方 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 自動離陸の上昇初期 (tof_silent_takeoff) | バイアス -0.2 m/s²（ToF 停止と同時） | 21/21 | 0.95 | 0.13 | 0.000 | 1.6 | 1.14 | 2.02 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| 自動離陸の上昇初期 (tof_silent_takeoff) | バイアス -0.2 m/s²（離陸開始から） | 21/21 | 0.94 | 0.13 | 0.000 | 1.5 | 1.22 | 2.08 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| 自動離陸の上昇初期 (tof_silent_takeoff) | バイアス -0.1 m/s²（ToF 停止と同時） | 21/21 | 0.96 | 0.14 | 0.000 | 1.6 | 1.10 | 1.98 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| 自動離陸の上昇初期 (tof_silent_takeoff) | バイアス -0.1 m/s²（離陸開始から） | 21/21 | 0.95 | 0.13 | 0.000 | 1.6 | 1.14 | 2.01 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| 自動離陸の上昇初期 (tof_silent_takeoff) | バイアス +0.0 m/s²（なし） | 21/21 | 0.96 | 0.14 | 0.000 | 1.7 | 1.05 | 1.95 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| 自動離陸の上昇初期 (tof_silent_takeoff) | バイアス +0.1 m/s²（ToF 停止と同時） | 21/21 | 0.97 | 0.14 | 0.000 | 1.8 | 1.01 | 1.93 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| 自動離陸の上昇初期 (tof_silent_takeoff) | バイアス +0.1 m/s²（離陸開始から） | 21/21 | 0.97 | 0.14 | 0.000 | 1.8 | 0.99 | 1.90 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| 自動離陸の上昇初期 (tof_silent_takeoff) | バイアス +0.2 m/s²（ToF 停止と同時） | 21/21 | 0.97 | 0.14 | 0.000 | 1.8 | 0.99 | 1.90 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| 自動離陸の上昇初期 (tof_silent_takeoff) | バイアス +0.2 m/s²（離陸開始から） | 21/21 | 0.98 | 0.14 | 0.000 | 1.9 | 0.96 | 1.87 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| API 離陸中 (tof_silent_api_takeoff) | バイアス -0.2 m/s²（ToF 停止と同時） | 21/21 | 1.16 | 0.21 | 0.000 | 1.5 | 1.10 | 2.05 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| API 離陸中 (tof_silent_api_takeoff) | バイアス -0.2 m/s²（離陸開始から） | 21/21 | 1.16 | 0.21 | 0.000 | 1.6 | 1.07 | 2.03 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| API 離陸中 (tof_silent_api_takeoff) | バイアス -0.1 m/s²（ToF 停止と同時） | 21/21 | 1.17 | 0.21 | 0.000 | 1.6 | 1.05 | 2.00 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| API 離陸中 (tof_silent_api_takeoff) | バイアス -0.1 m/s²（離陸開始から） | 21/21 | 1.17 | 0.21 | 0.000 | 1.6 | 1.03 | 1.99 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| API 離陸中 (tof_silent_api_takeoff) | バイアス +0.0 m/s²（なし） | 21/21 | 1.17 | 0.21 | 0.000 | 1.6 | 1.00 | 1.97 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| API 離陸中 (tof_silent_api_takeoff) | バイアス +0.1 m/s²（ToF 停止と同時） | 21/21 | 1.18 | 0.22 | 0.000 | 1.7 | 0.96 | 1.93 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| API 離陸中 (tof_silent_api_takeoff) | バイアス +0.1 m/s²（離陸開始から） | 21/21 | 1.18 | 0.22 | 0.000 | 1.6 | 0.98 | 1.94 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| API 離陸中 (tof_silent_api_takeoff) | バイアス +0.2 m/s²（ToF 停止と同時） | 21/21 | 1.19 | 0.22 | 0.000 | 1.8 | 0.93 | 1.90 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| API 離陸中 (tof_silent_api_takeoff) | バイアス +0.2 m/s²（離陸開始から） | 21/21 | 1.19 | 0.22 | 0.000 | 1.6 | 0.98 | 1.89 | 0 | 0 | 衝撃(ToF なし接地) 21 |
| 0.5 m ホバー (tof_silent_hover) | バイアス -0.2 m/s²（ToF 停止と同時） | 18/21 | 0.02 | 0.08 | 0.000 | 1.3 | 0.75 | 1.38 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 0.5 m ホバー (tof_silent_hover) | バイアス -0.2 m/s²（離陸開始から） | 18/21 | 0.01 | 0.07 | 0.000 | 1.0 | 0.91 | 1.53 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 0.5 m ホバー (tof_silent_hover) | バイアス -0.1 m/s²（ToF 停止と同時） | 18/21 | 0.02 | 0.07 | 0.000 | 1.4 | 0.71 | 1.37 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 0.5 m ホバー (tof_silent_hover) | バイアス -0.1 m/s²（離陸開始から） | 18/21 | 0.02 | 0.09 | 0.000 | 1.2 | 0.79 | 1.00 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 0.5 m ホバー (tof_silent_hover) | バイアス +0.0 m/s²（なし） | 18/21 | 0.02 | 0.07 | 0.000 | 1.4 | 0.66 | 1.36 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 0.5 m ホバー (tof_silent_hover) | バイアス +0.1 m/s²（ToF 停止と同時） | 18/21 | 0.02 | 0.07 | 0.000 | 1.5 | 0.63 | 1.36 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 0.5 m ホバー (tof_silent_hover) | バイアス +0.1 m/s²（離陸開始から） | 18/21 | 0.01 | 0.07 | 0.000 | 1.8 | 0.56 | 1.37 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 0.5 m ホバー (tof_silent_hover) | バイアス +0.2 m/s²（ToF 停止と同時） | 18/21 | 0.03 | 0.07 | 0.000 | 1.6 | 0.60 | 1.35 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 0.5 m ホバー (tof_silent_hover) | バイアス +0.2 m/s²（離陸開始から） | 18/21 | 0.02 | 0.06 | 0.000 | 2.6 | 0.48 | 1.44 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 1.2 m ホバー (tof_silent_hover_1p2) | バイアス -0.2 m/s²（ToF 停止と同時） | 18/21 | 0.02 | 0.03 | 0.000 | 2.0 | 1.00 | 1.34 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 1.2 m ホバー (tof_silent_hover_1p2) | バイアス -0.2 m/s²（離陸開始から） | 18/21 | 0.01 | 0.03 | 0.000 | 1.9 | 1.04 | 1.42 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 1.2 m ホバー (tof_silent_hover_1p2) | バイアス -0.1 m/s²（ToF 停止と同時） | 18/21 | 0.02 | 0.03 | 0.000 | 2.1 | 0.96 | 1.32 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 1.2 m ホバー (tof_silent_hover_1p2) | バイアス -0.1 m/s²（離陸開始から） | 18/21 | 0.02 | 0.03 | 0.000 | 2.1 | 0.96 | 1.33 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 1.2 m ホバー (tof_silent_hover_1p2) | バイアス +0.0 m/s²（なし） | 18/21 | 0.02 | 0.03 | 0.000 | 2.3 | 0.91 | 1.31 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 1.2 m ホバー (tof_silent_hover_1p2) | バイアス +0.1 m/s²（ToF 停止と同時） | 18/21 | 0.02 | 0.03 | 0.000 | 2.4 | 0.87 | 1.29 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 1.2 m ホバー (tof_silent_hover_1p2) | バイアス +0.1 m/s²（離陸開始から） | 18/21 | 0.01 | 0.03 | 0.000 | 2.5 | 0.85 | 1.25 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 1.2 m ホバー (tof_silent_hover_1p2) | バイアス +0.2 m/s²（ToF 停止と同時） | 18/21 | 0.02 | 0.03 | 0.000 | 2.6 | 0.83 | 1.26 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| 1.2 m ホバー (tof_silent_hover_1p2) | バイアス +0.2 m/s²（離陸開始から） | 18/21 | 0.01 | 0.03 | 0.002 | 2.8 | 0.79 | 1.15 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | バイアス -0.2 m/s²（ToF 停止と同時） | 18/21 | 0.14 | 0.02 | 0.000 | 2.1 | 0.91 | 1.84 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | バイアス -0.2 m/s²（離陸開始から） | 18/21 | 0.13 | 0.02 | 0.000 | 1.7 | 1.10 | 2.00 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | バイアス -0.1 m/s²（ToF 停止と同時） | 18/21 | 0.14 | 0.02 | 0.000 | 2.2 | 0.87 | 1.83 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | バイアス -0.1 m/s²（離陸開始から） | 18/21 | 0.14 | 0.02 | 0.000 | 2.0 | 0.98 | 2.02 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | バイアス +0.0 m/s²（なし） | 18/21 | 0.14 | 0.02 | 0.000 | 2.4 | 0.82 | 1.81 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | バイアス +0.1 m/s²（ToF 停止と同時） | 18/21 | 0.14 | 0.02 | 0.000 | 2.5 | 0.78 | 1.80 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | バイアス +0.1 m/s²（離陸開始から） | 18/21 | 0.14 | 0.02 | 0.000 | 3.0 | 0.65 | 1.97 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | バイアス +0.2 m/s²（ToF 停止と同時） | 18/21 | 0.14 | 0.02 | 0.000 | 2.7 | 0.74 | 1.79 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | バイアス +0.2 m/s²（離陸開始から） | 18/21 | 0.13 | 0.03 | 0.000 | 3.8 | 0.55 | 2.02 | 0 | 0 | 衝撃(ToF なし接地) 18 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | バイアス -0.2 m/s²（ToF 停止と同時） | 3/21 | 0.04 | 0.04 | 0.000 | 2.8 | 0.71 | - | 0 | 0 | 衝撃(ToF なし接地) 3 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | バイアス -0.2 m/s²（離陸開始から） | 9/21 | 0.04 | 0.04 | 0.000 | 2.8 | 0.71 | - | 0 | 0 | 衝撃(ToF なし接地) 7、衝撃のあとの静止 2 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | バイアス -0.1 m/s²（ToF 停止と同時） | 3/21 | 0.04 | 0.04 | 0.000 | 2.8 | 0.71 | - | 0 | 0 | 衝撃(ToF なし接地) 3 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | バイアス -0.1 m/s²（離陸開始から） | 12/21 | 0.04 | 0.04 | 0.000 | 2.8 | 0.71 | - | 0 | 0 | 衝撃(ToF なし接地) 10、衝撃のあとの静止 2 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | バイアス +0.0 m/s²（なし） | 12/21 | 0.05 | 0.04 | 0.000 | 3.1 | 0.65 | - | 0 | 0 | 衝撃(ToF なし接地) 12 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | バイアス +0.1 m/s²（ToF 停止と同時） | 12/21 | 0.05 | 0.04 | 0.000 | 3.1 | 0.74 | - | 0 | 0 | 衝撃(ToF なし接地) 12 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | バイアス +0.1 m/s²（離陸開始から） | 12/21 | 0.05 | 0.04 | 0.000 | 3.1 | 0.76 | - | 0 | 0 | 衝撃(ToF なし接地) 12 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | バイアス +0.2 m/s²（ToF 停止と同時） | 12/21 | 0.05 | 0.05 | 0.000 | 3.1 | 0.75 | - | 0 | 0 | 衝撃(ToF なし接地) 12 |
| STABILIZE 21 s 飛行 → 通信断 (tof_silent_stabilize_commloss) | バイアス +0.2 m/s²（離陸開始から） | 12/21 | 0.05 | 0.04 | 0.000 | 3.1 | 0.76 | - | 0 | 0 | 衝撃(ToF なし接地) 12 |

Sweeps (noise n0 / n2, scenes hover 0.5 m, hover 1.2 m, stick climb, early climb; offset -0.2 / 0 / +0.2 m/s^2 at the silence; `hover.thrust_corr` 1.0): thrust ceiling ratio 0.95 / 0.97 / 0.98 / 0.99 / 1.00: no re-rise up to 0.99, 0.05 m at 1.00 (hover 1.2 m, +0.2); touchdown speed from 1.2 m with n2 1.17 / 1.02 / 0.93 / 0.84 / 0.72 m/s; with the offset +0.2 m/s^2 from the take-off and the stick climb the ratio 0.97 re-rose 0.07 m (0.98: 0.54 m, 2.8 m/s) and 0.95 / 0.96 did not, so 0.95 was chosen. Velocity trust time 1 / 2 / 3 / 5 s: identical except the early climb at 1 s (rise 0.93-1.00 m instead of 0.74-0.76 m). Real-log replay of the calibration (`front_tof_poshold_yaw360`, 5.6 s hover): calibrated 4.1 s after the take-off began at +0.7 % of the mean hover thrust, -1.1 .. +1.5 % afterwards; the other two logs have no hover long enough (uncalibrated). Existing scenarios (the 79 of the previous suite with an `.expect`; 84 with the five new ones; final run: PASS 79, KNOWN-FAIL 4 = `commloss_land_level`, `landing_dead_zone`, `pos_yaw`, `yaw_hold` as before, SKIP 1 = `workshop_acro`): 67 keep their bounds; the descent-speed bounds of eleven (`tof_silent_takeoff`, `_api_takeoff`, `_hover`, `_hover_1p2`, `_alt_climb`, `_est_high`, `_takeoff_est_high`, `_api_takeoff_est_high`, `_takeoff_noise`, `_touchdown_detect`, `tof_invalid_alt_hover`) were raised (each `.expect` has the reason: the descent speed after the velocity target is reached is set by the 0.95 ceiling), and `tof_silent_hover_timed` also switches off the floor path (`--param safety.blind_touchdown.floor_s=100`) and its log line changed (`Downward ToF silent after ... ms of LANDING`). Of 306 metric values compared old build vs new build 277 are identical; 14 `tof_*` scenarios moved (the descent without a ToF) and four others moved without a logic cause that was found (`boot_motion` duty_max 0.670 -> 0.692, `calib` att_rmse 0.52 -> 0.09 deg and tilt_max 1.19 -> 0.61 deg, `crash_refly` alt 1.386 -> 1.380 / alt_min 0.90 -> 0.94 m, `commloss_land_level` (KNOWN-FAIL) alt_max 0.25 -> 0.32 m): each build is deterministic, the console logs of `calib` are identical except for a sign in the last digit of the first printed quaternion (-0.0000 / 0.0000) and the parameter count (141 -> 144), so the cause is probably a numerical difference of the rebuilt controller code that the closed loop amplifies - not confirmed bit by bit. Not verified: real hardware; the open-loop STABILIZE scenario is tuned for noise n0 seed 12345 only.

**Updated 2026-11-09 (third pass) — the descent without a ToF is a closed loop on an accelerometer-based vertical velocity; touchdown = impact, then stillness; new scenarios `tof_silent_takeoff_est_high`, `tof_silent_api_takeoff_est_high`, `tof_silent_takeoff_noise`, `tof_silent_touchdown_detect`.** The independent review reproduced a fatal case of the open-loop thrust ramp (the support thrust is not calibrated within 2 s of the start of the flight, so a take-off climb silenced at 1.3 m/s kept climbing: truth peak 4.1 m with `hover.thrust_corr` 1.1, 11 m with 1.2, 9.2 m with noise n2). The descent is now `sf::BlindDescent` (PI around the accelerometer-calibrated support thrust on the vertical velocity of `sf::VerticalVelocityEstimator`; late fall as the last resort, zero thrust at 8.33 s, time limit 10 s), and the touchdown detector needs an impact (2 g x 2 samples) followed by 0.25 s of stillness (detailed_design §3.1 note 12 has the real-log table behind the thresholds). Existing scenario metrics and bounds that changed (the others are bit-identical in truth / posvel / ctrl_output between the old and new builds: 60 of the 73 vehicle scenarios with an .expect that were compared; the 13 that differ are all `tof_*` scenarios that reach the blind descent): `tof_silent_takeoff` `alt_max` 0.528 -> 0.231 m (bound 0.70 -> 0.30), `descent_speed_max` 0.825 -> 0.43 m/s (1.1 -> 0.6); `tof_silent_api_takeoff` 0.405 -> 0.177 m (0.60 -> 0.30), 0.71 -> 0.43 m/s (1.0 -> 0.6); `tof_silent_hover` 0.71 -> 0.48 m/s (0.9 -> 0.6); `tof_silent_hover_1p2` 1.30 -> 0.49 m/s (1.5 -> 0.6); `tof_silent_alt_climb` 2.07 -> 1.79 m (2.4 -> 2.0), 1.86 -> 0.47 m/s (2.2 -> 0.6); `tof_silent_est_high` 2.15 -> 1.96 m (2.5 -> 2.2), 1.93 -> 0.52 m/s (2.4 -> 0.7); `tof_invalid_alt_hover` 0.69 -> 0.49 m/s (0.9 -> 0.6); the log line `Blind landing: ToF silent, thrust ramp` became `... velocity loop`. Acceptance matrix (80 runs per build, truth from `truth.csv`; rise = max altitude after the ToF stops minus the altitude when it stops; time = silence to touchdown; speed = touchdown speed; end = what ended LANDING first; n2 = worst of seeds 1-3; 'air stop' = DISARM above 0.10 m):

New build:

| 場面 | corr | n0: 増分[m] | 出力停止→接地[s] | 接地速度[m/s] | 終わり方 | n2 最悪: 増分 | 出力停止→接地 | 接地速度 | 終わり方 |
|---|---|---|---|---|---|---|---|---|---|
| 自動離陸の上昇初期で出力停止 (tof_silent_takeoff) | 0.9 | 0.04 | 0.5 | 0.36 | 衝撃 | 0.59 | 2.3 | 0.51 | 衝撃 |
|  | 1.0 | 0.16 | 1.0 | 0.43 | 衝撃 | 0.76 | 3.1 | 0.57 | 衝撃 |
|  | 1.1 | 0.38 | 2.0 | 0.36 | 衝撃 | 0.85 | 3.7 | 0.64 | 衝撃 |
|  | 1.2 | 0.59 | 3.1 | 0.36 | 衝撃 | 0.91 | 4.2 | 0.63 | 衝撃 |
| API 離陸中に出力停止 (tof_silent_api_takeoff) | 0.9 | 0.03 | 0.5 | 0.25 | 衝撃 | 0.48 | 2.3 | 0.53 | 衝撃 |
|  | 1.0 | 0.13 | 0.8 | 0.43 | 衝撃 | 0.83 | 3.6 | 0.53 | 衝撃 |
|  | 1.1 | 0.31 | 1.6 | 0.36 | 衝撃 | 1.00 | 4.5 | 0.50 | 衝撃 |
|  | 1.2 | 0.60 | 2.9 | 0.35 | 衝撃 | 1.18 | 5.3 | 0.61 | 衝撃 |
| 0.5 m ホバー (tof_silent_hover) | 0.9 | 0.00 | 1.3 | 0.48 | 衝撃 | 0.01 | 1.8 | 0.48 | 衝撃 |
|  | 1.0 | 0.00 | 1.3 | 0.48 | 衝撃 | 0.00 | 1.9 | 0.49 | 衝撃 |
|  | 1.1 | 0.00 | 1.1 | 0.48 | 衝撃 | 0.00 | 1.9 | 0.48 | 接地判定/衝撃 |
|  | 1.2 | 0.00 | 1.1 | 0.47 | 衝撃 | 0.34 | 5.6 | 0.66 | 衝撃 |
| 1.2 m ホバー (tof_silent_hover_1p2) | 0.9 | 0.00 | 2.7 | 0.48 | 衝撃 | 0.01 | 3.3 | 0.59 | 衝撃 |
|  | 1.0 | 0.00 | 2.6 | 0.48 | 衝撃 | 0.00 | 3.3 | 0.60 | 衝撃 |
|  | 1.1 | 0.00 | 2.5 | 0.46 | 衝撃 | 0.18 | 3.8 | 0.60 | 衝撃 |
|  | 1.2 | 0.00 | 2.6 | 0.44 | 衝撃 | 0.00 | 3.4 | 0.96 | 衝撃 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | 0.9 | 0.23 | 4.3 | 0.44 | 衝撃 | 0.18 | 5.8 | 0.57 | 衝撃 |
|  | 1.0 | 0.24 | 4.4 | 0.44 | 衝撃 | 0.20 | 6.2 | 0.60 | 衝撃 |
|  | 1.1 | 0.22 | 4.2 | 0.47 | 衝撃 | 0.01 | 5.3 | 0.65 | 衝撃 |
|  | 1.2 | 0.22 | 4.4 | 0.49 | 衝撃 | 0.08 | 4.9 | 0.78 | 衝撃 |

Old build (the working tree before this change; for comparison):

| 場面 | corr | n0: 増分[m] | 出力停止→接地[s] | 接地速度[m/s] | 終わり方 | n2 最悪: 増分 | 出力停止→接地 | 接地速度 | 終わり方 |
|---|---|---|---|---|---|---|---|---|---|
| 自動離陸の上昇初期で出力停止 (tof_silent_takeoff) | 0.9 | 0.05 | 0.8 | 0.28 | 衝撃 | 2.14 | 4.5 | 2.70 | 衝撃 |
|  | 1.0 | 0.46 | 2.7 | 0.82 | 衝撃 | 9.57 | 7.5 | 9.62 | 衝撃 |
|  | 1.1 | 4.01 | 6.4 | 5.09 | 衝撃 | 24.61 | 9.5 | 20.70 | 他 **空中停止3** |
|  | 1.2 | 10.79 | 8.2 | 10.67 | 衝撃 | 37.56 | 10.5 | 26.90 | 他/接地判定 **空中停止3** |
| API 離陸中に出力停止 (tof_silent_api_takeoff) | 0.9 | 0.04 | 0.7 | 0.19 | 衝撃 | 1.85 | 4.3 | 2.45 | 衝撃 |
|  | 1.0 | 0.36 | 2.4 | 0.70 | 衝撃 | 11.52 | 7.7 | 10.83 | 衝撃 |
|  | 1.1 | 3.40 | 6.3 | 4.50 | 衝撃 | 27.53 | 0.0 | 0.00 | 他 |
|  | 1.2 | 11.49 | 8.3 | 11.10 | 衝撃 | 46.77 | 2.4 | 8.67 | 他/接地判定 **空中停止2** |
| 0.5 m ホバー (tof_silent_hover) | 0.9 | 0.00 | 1.5 | 0.77 | 衝撃 | 0.01 | 1.3 | 1.05 | 衝撃 |
|  | 1.0 | 0.00 | 1.4 | 0.74 | 衝撃 | 0.00 | 1.2 | 1.01 | 衝撃 |
|  | 1.1 | 0.00 | 1.2 | 0.63 | 衝撃 | 0.00 | 1.2 | 1.11 | 衝撃 |
|  | 1.2 | 0.00 | 1.2 | 0.67 | 衝撃 | 0.00 | 1.5 | 5.98 | 衝撃 |
| 1.2 m ホバー (tof_silent_hover_1p2) | 0.9 | 0.00 | 1.9 | 1.34 | 衝撃 | 0.01 | 1.9 | 1.83 | 衝撃 |
|  | 1.0 | 0.00 | 1.9 | 1.28 | 衝撃 | 0.00 | 1.8 | 1.80 | 衝撃 |
|  | 1.1 | 0.00 | 1.8 | 1.23 | 衝撃 | 0.00 | 0.3 | 0.19 | 衝撃 |
|  | 1.2 | 0.00 | 1.9 | 1.20 | 衝撃 | 0.00 | 0.3 | 0.00 | 衝撃 |
| ALT_HOLD 手動上昇中 (tof_silent_alt_climb) | 0.9 | 0.52 | 3.6 | 1.81 | 衝撃 | 0.35 | 3.0 | 2.21 | 衝撃 |
|  | 1.0 | 0.52 | 3.6 | 1.85 | 衝撃 | 0.39 | 3.1 | 2.25 | 衝撃 |
|  | 1.1 | 0.44 | 3.5 | 1.81 | 衝撃 | 0.11 | 2.4 | 2.24 | 衝撃 |
|  | 1.2 | 0.45 | 3.5 | 1.88 | 衝撃 | 0.04 | 0.3 | 0.42 | 衝撃 |

Old build: 8 of 80 runs stopped the motors in the air (1.1-36 m); new build: 0 of 80. Pass conditions: no motor stop in the air (met), rise <= 0.3 m (met except where the craft is climbing fast when the silence is judged: `tof_silent_takeoff` / `tof_silent_api_takeoff` with `hover.thrust_corr` >= 1.1, 0.31-0.60 m with noise off; the cause is the 0.32 s between the ToF stopping and LANDING (the Failsafe threshold) at the climb speed, 1.0-1.3 m/s, and the stopping distance after LANDING is 0.01-0.27 m), touchdown speed <= 0.6 m/s with noise off (met in all 20 runs: 0.25-0.49 m/s; with noise n2 the worst is 0.96 m/s, 1.2 m hover, `hover.thrust_corr` 1.2, after a flight that had overshot to 2.3 m beyond the ToF range). The IMPACT cut (3 G, 2 samples) ends 79 of 80 runs before the stillness test can run (SILS contact 6-9 g; the real touchdown log peaks at 3.39 g); with the impact cut disabled (`--param safety.impact.accel_g=10`) the detector ends all 80 runs 0.38-0.91 s after the contact (median 0.44 s) and never in the air. Not verified: real hardware.

**Added 2026-11-01 — `calib_supersede` (vehicle, `--duration 22000000`):** regression for the supersede rule of the still-calibration (detailed_design §3 note 10). The boot calibration applies an injected accel bias of 0.2 m/s^2; after the injection is removed and the hand places the craft, the negligible recalibration replaces the earlier seed (`previous seed superseded`). It fails on a build without the rule (`estimator left untouched`, the 0.2 seed stays).

**Updated 2026-11-09 (second pass) — the ToF-silent scenarios follow the new policy (`tof_silent_takeoff`, `tof_silent_api_takeoff`, `tof_silent_hover`, `tof_silent_hover_timed`, `tof_silent_prearm`, `tof_silent_armed_ground`; new: `tof_silent_hover_1p2`, `tof_silent_alt_climb`, `tof_silent_stabilize`, `tof_silent_mode_refused`):** Failsafe detects the silence and raises `TOF_SILENT` (+ the level `failsafe_status`), StateManager decides (detailed_design §3.1, note 12). ALT_HOLD / POS_HOLD / auto-takeoff -> LANDING with a blind open-loop thrust ramp; the touchdown is the accelerometer (or the 3 G impact), the 10 s limit (the blind thrust profile is zero at 8.9 s from any start) is the last resort; ACRO / STABILIZE are NOT landed by force (warning only); ALT_HOLD / POS_HOLD switches are refused while silent; ARM is refused. New metric `descent_speed_max` (max downward truth speed in a window). Measured touchdown speeds (truth): early climb 0.84 m/s, API takeoff 0.73, 0.5 m hover 0.72, 1.2 m hover 1.25, manual climb through 1.55 m 1.77 m/s (a normal landing with a working ToF: 0.30 m/s). The earlier note follows.

**Added 2026-11-09 (first pass, superseded by the paragraph above) — `tof_silent_takeoff`, `tof_silent_api_takeoff` (the accident chain through the API: `error takeoff aborted`, then `error tof silent - check the sensor`), `tof_silent_hover`, `tof_silent_hover_timed`, `tof_silent_prearm`, `tof_silent_armed_ground` (vehicle):** fault injection `tof_silent <0|1>` stops the downward ToF's sample output (the chip model never reports "data ready"; no bus error). The state machine must notice the silence by itself (no new sample for 0.3 s): TAKEOFF/FLYING -> LANDING (abort the climb), ARMED_GROUND -> DISARM, ARM refused on the ground with `ArmBlock::TofSilent`, and a LANDING that continues blind ends by time after 6 s (detailed_design §3 note 12). Measured: `tof_silent_takeoff` ToF stops 8.389 s, LANDING 8.702 s, motors stop 8.722 s, `alt_max` 0.219 m (the same run without the response climbs to 42 m); `tof_silent_hover` ToF stops 13.999 s, LANDING 14.302 s, IDLE_GROUND 15.782 s (impact cut); `tof_silent_hover_timed` (`--param safety.impact.accel_g=10`) IDLE_GROUND 20.322 s by time.

**Added 2026-11-09 (2nd independent review) — `tof_recover_frozen_ground`, `tof_recover_edge_consumed`, `tof_recover_climb_alt`, `tof_recover_reflip`, `tof_recover_invalid_first`, `tof_recover_prearm`, `tof_invalid_alt_hover`, `tof_invalid_stabilize`, `tof_silent_flip_end` (vehicle; new SILS fault injection `tof_invalid <0|1>`: every downward-ToF sample is "no target", the samples keep arriving on time).** The recovery from a ToF silence is judged on what the ESTIMATOR did with the samples, not on their arrival: the silence EPISODE ends when `kTofReacquireSamples` (10) consecutive samples were adopted with an innovation <= 0.10 m and the ToF has been alive for 1 s. D1/D2 (`tof_recover_frozen_ground`: truth alt_max 10.9 m -> 2.5 m, motors no longer stop at altitude; `tof_recover_climb_alt`: altitude RMS error 1.19 m -> 0.02 m), D3 (`tof_recover_edge_consumed`, `tof_recover_reflip`: in flight the refused mode-switch edge is consumed, nothing switches by itself at the recovery; the old build entered POS_HOLD / ALT_HOLD at the return and the pilot's OFF then switched it back), D1 (`tof_recover_invalid_first`), the ARM / mode permission (`tof_recover_prearm`), D4 (`tof_invalid_*`: only invalid samples while flying are a silence episode, ALT/POS land, ACRO/STABILIZE only warn; on the ground an invalid ToF is normal, the takeoff is cut after ~0.5 s and the craft rises ~0.16 m), D6 (`tof_silent_flip_end`: decided the moment the flip ends). The first two are the reviewer's reproductions (recover_frozen2, rv_stab_pending_pos) with `.expect`; they FAIL on the previous build and PASS now. D5 (a stale `failsafe_status` reads as silent) has no SILS scenario (there is no injection that stops Failsafe); it is covered by the host test `test_tof_health`.

**Added 2026-11-09 — `api_takeoff_emergency`, `api_land_emergency`, `api_land_stop`, `api_move_stop`, `api_wait_deferred` (vehicle):** while an API command blocks (`takeoff`, `land`, moves, `flip`: `waitUntil` in `api_task.cpp`), the wait now keeps reading the command sources every 50 ms; `emergency` is executed at once (and ends the wait: `error interrupted by emergency`), `stop` is executed at once through the normal `cmdStop` (accepted only while FLYING with a guidance target; then it ends the wait: `error interrupted by stop`), every other line is parked and run afterwards in arrival order (unchanged behaviour). Measured, SILS (injection to last armed motor sample): `emergency` during `takeoff` 7.000 s -> 7.040 s (before the fix 7.760 s, i.e. the rest of the wait); during `land` 16.600 s -> 16.640 s (before: 18.119 s, touchdown). `stop` during `forward 150`: peak horizontal travel 0.14 m (before: 1.48 m, the whole move). The UDP read path is not exercised by SILS (the socket shim never delivers data); only the injection ring is.

### 7 回目の見直し（2026-11-09）— 6 回目の修正に対する独立レビュー 2 本の指摘（C1・I2・I3・I4）

**8 回目の注（2026-11-10）:** この節の I2（上向き加速の証拠）・I3（加速度計に基づく中止）・I4（workshop の `ControllerStatus` 発行）と、それに伴う `tof_recover_*`・`tof_invalid_alt_hover` の入力・閾値の変更は、8 回目で取り下げた（下の「8 回目の見直し」）。C1（天井ブレーキの上昇率、`alignToEskf()`）は残る。以下は 7 回目の記録として残す。

| 指摘 | 変更 | 検証 |
|---|---|---|
| C1（致命）天井ブレーキが古い加速度積分で推力を半分に落とす | ブレーキが積分を使うのは自動離陸の上昇と、ESKF の鉛直状態を信用できない間だけ。信用できる間は ESKF 自身の上昇率。ESKF が ToF を取り込み直した周期に、積分を ESKF の速度へ合わせる（`alignToEskf()`） | 新シナリオ `ceiling_brake_stale_velocity`・`_alt`（6 回目のビルドで不合格）、R1 の再測定 |
| I2（重要）床を離れた証拠が加速度オフセットで床の上でも成立 | 上昇量 `riseM()` は、ろ波した加速度が自分の遅い基準値を `kVerticalVelocityLiftEvidenceMps2`（0.4 m/s²）超えた後だけ、その時点から（加速度 − 基準値）の 2 重積分で数える | 床の上の誤成立の掃引（下）、新シナリオ `weak_spool_alt_offset`・`weak_spool_stabilize_offset`（6 回目のビルドで不合格） |
| I3（重要）浮けない自動離陸が終わらない | 離陸の開始から 5 s（`kAutoTakeoffNoLiftAbortUs`）床を離れた証拠が無い ALT/POS/API の自動離陸は DISARM で中止（StateTask の `updateAutoTakeoff()`） | `weak_spool_alt`（期待値を更新）、新シナリオ `weak_spool_api` |
| I4（重要）workshop は `controller_status` を発行しない | workshop の制御タスクが最小限の `ControllerStatus` を毎周期発行 | workshop ファームのビルド（成功）。実機・SILS の workshop の動作は未確認（`workshop_acro` は SKIP） |
| M（軽微） | `takeoff_elapsed_cycles_` の飽和、`...Mps` への改名、`kManualTakeoffMinRiseM = kTofInvalidTakeoffRiseM`、根拠の数値、`controller_status.latest()` の 1 回読み、関数 3 本を 50 行以内に分割、用語（「出力停止」「正直に」） | host ユニットテスト（`test_blind_landing` 48 本、旧 45 本。8 ファイル計 205 本が合格）、SILS 全件 |

#### C1: 天井ブレーキ（新シナリオ、6 回目のビルド → 7 回目のビルド）

| シナリオ | 内容 | 6 回目のビルド | 7 回目のビルド |
|---|---|---|---|
| `ceiling_brake_stale_velocity` | 長い出力停止（オフセット −0.4 m/s²、積分 3.7 m/s）の直後、1.6 m で ALT_HOLD | `alt_min` 0.010 m（床へ 2.57 m/s）、不合格 | `alt_min` 1.06 m、降下速度の最大 0.19 m/s、合格 |
| `ceiling_brake_stale_velocity_alt` | 同じ状況で 1.24 m（天井）で ALT_HOLD | `alt_min` 0.59 m（降下速度の最大 1.17 m/s）、不合格 | `alt_min` 1.18 m、`alt_max` 1.26 m、降下速度の最大 0.12 m/s、合格 |
| 対照（オフセット 0、`rv10_recov_alt0`） | 正常な ALT_HOLD への切り替え | 正常 | 正常（`alt_min` 1.23 m、`alt_max` 1.26 m） |

自動離陸・API 離陸の真値の最高高度（R1、`hover.thrust_corr` 0.9 / 1.0 / 1.2 × ノイズ n0 / n2（seed 12345・1・2）、`hover.thrust.learn=0`）: 36 本の最大は **1.282 m**（合格: 1.35 m 以下を維持）。6 回目から 0.003 m を超えて変わったのは 9 本で、最大 0.033 m（原因は、天井の近くで ESKF の上昇率だけを見るブレーキになったためと推定。個別の切り分けは未実施）。

| 場面 | hover.thrust_corr | ノイズ | 6 回目の最高 [m] | 7 回目の最高 [m] | ≤ 1.35 m |
|---|---|---|---|---|---|
| ALT_HOLD 自動離陸 | 0.9 | n0 | 0.59 | 0.59 | ○ |
| ALT_HOLD 自動離陸 | 0.9 | n2（3 seed の最大） | 0.83 | 0.83 | ○ |
| ALT_HOLD 自動離陸 | 1.0 | n0 | 0.84 | 0.84 | ○ |
| ALT_HOLD 自動離陸 | 1.0 | n2（3 seed の最大） | 1.17 | 1.17 | ○ |
| ALT_HOLD 自動離陸 | 1.2 | n0 | 1.25 | 1.25 | ○ |
| ALT_HOLD 自動離陸 | 1.2 | n2（3 seed の最大） | 1.25 | 1.28 | ○ |
| API 離陸 | 0.9 | n0 | 0.60 | 0.60 | ○ |
| API 離陸 | 0.9 | n2（3 seed の最大） | 0.82 | 0.82 | ○ |
| API 離陸 | 1.0 | n0 | 0.84 | 0.84 | ○ |
| API 離陸 | 1.0 | n2（3 seed の最大） | 1.14 | 1.14 | ○ |
| API 離陸 | 1.2 | n0 | 1.26 | 1.26 | ○ |
| API 離陸 | 1.2 | n2（3 seed の最大） | 1.27 | 1.28 | ○ |
| POS_HOLD 自動離陸 | 0.9 | n0 | 0.59 | 0.59 | ○ |
| POS_HOLD 自動離陸 | 0.9 | n2（3 seed の最大） | 0.83 | 0.83 | ○ |
| POS_HOLD 自動離陸 | 1.0 | n0 | 0.84 | 0.84 | ○ |
| POS_HOLD 自動離陸 | 1.0 | n2（3 seed の最大） | 1.17 | 1.17 | ○ |
| POS_HOLD 自動離陸 | 1.2 | n0 | 1.25 | 1.25 | ○ |
| POS_HOLD 自動離陸 | 1.2 | n2（3 seed の最大） | 1.26 | 1.28 | ○ |

#### I2: 床の上の誤成立と、離陸で証拠が立つまでの遅れ（掃引）

方法: `rv10_ws_m`（STABILIZE、離陸の閾値より上でホバーより下のスロットルが約 3.5 s）と `rv10_wa_m`（ALT_HOLD の自動離陸）を雛形に、モータの健全度 0.85 / 0.9 × 加速度計オフセット −0.3 / −0.2 / −0.1 / 0 / +0.1 / +0.2 / +0.3 m/s²（ARM の 0.3 s 後に注入）× ノイズ n0 / n2 の 56 本を 1 規則ごとに走らせ、床の上にいる（真値が静止高度 + 2 cm 未満で、その後 0.2 s 以内に浮かない）のに FLYING（STABILIZE）または LANDING（ALT_HOLD）に入った数を数えた（`sweep12.py`）。

| 規則 | STABILIZE 健全度 0.85 | STABILIZE 0.9 | ALT_HOLD 0.85 | ALT_HOLD 0.9 | 計 |
|---|---|---|---|---|---|
| 6 回目の規則（旧） | 6/14 | 3/14 | 4/14 | 3/14 | 16/56 |
| 基準値つき 0.3 | 2/14 | 0/14 | 0/14 | 0/14 | 2/56 |
| 基準値つき 0.4（採用） | 1/14 | 0/14 | 0/14 | 0/14 | 1/56 |
| 基準値つき 0.5 | 0/14 | 0/14 | 0/14 | 0/14 | 0/56 |
| 絶対しきい値 0.7（基準値なし） | 0/14 | 0/14 | 0/14 | 0/14 | 0/56 |

離陸側（健全度 0.95 / 1.0 の離陸、ALT / POS 自動離陸と STABILIZE のスティック離陸 × オフセット −0.3 / 0 / +0.3 × n0 / n2）: 機体が浮いてから上昇量が 10 cm に達するまでの遅れ。「証拠なし」は、そのログで上昇量が 10 cm に達しなかった数（ToF が健全なら ToF の空中判定が先に立つので、この表は ToF が使えない場合の下限）。

| 規則 | ALT / POS 自動離陸（12 本） | STABILIZE スティック離陸（6 本） |
|---|---|---|
| 6 回目の規則（旧） | 証拠なし 0/12、最大の遅れ 0.57 s | 証拠なし 0/6、最大の遅れ 0.35 s |
| 基準値つき 0.3 | 証拠なし 2/12、最大の遅れ 0.37 s | 証拠なし 3/6、最大の遅れ 0.19 s |
| 基準値つき 0.4（採用） | 証拠なし 2/12、最大の遅れ 0.43 s | 証拠なし 3/6、最大の遅れ 0.21 s |
| 基準値つき 0.5 | 証拠なし 4/12、最大の遅れ 0.29 s | 証拠なし 3/6、最大の遅れ 0.21 s |
| 絶対しきい値 0.7（基準値なし） | 証拠なし 2/12、最大の遅れ 1.13 s | 証拠なし 3/6、最大の遅れ 0.23 s |

判断: 基準値つきの規則の 0.4 を採用した。理由: (a) 絶対しきい値は、床の上の誤成立を 0 にするのに 0.7 が要り、そこでは普通の ALT 自動離陸の証拠が最大 1.1 s 遅れる。(b) 基準値つきで 0.5 にすると誤成立は 0 本だが、ToF が使えない自動離陸（`tof_invalid_alt_hover` の 2 回目の離陸）が上昇として認識されず、4.3 m まで上昇した（0.4 では 0.32 m で打ち切り）。(c) 0.4 の誤成立 1 本は、モータの健全度 0.85、オフセット −0.3、ノイズ n2 の組み合わせ。**両側とも余裕が薄い（床の上の側は評価表のオフセット 0.3 に対し 0.1 m/s²、離陸の側は 0.05 m/s²）ことは未解決の点であり、加速度計に依らない独立の備え（気圧計）が望ましい。** 実機のモータの振動は、実機ログ 1 本で床の上のろ波した超過が最大約 1.0 m/s² になった（基準値に対しては小さいが、実機での誤成立の有無は未確認）。

代償（既存シナリオ）: 加速度が基準値を 0.4 m/s² 超えない緩やかな上昇（ToF が使えないとき）はこの証拠では認識されない。`tof_recover_invalid_first`・`tof_recover_reflip` は、開始のスロットルを 3171 → 3250（0.3 s）に強めた（離陸の加速度が証拠に届く）。`tof_invalid_alt_hover` は 2 回目の離陸の打ち切りが 0.12 m 遅れ（`alt_max` 0.20 → 0.32 m）、閾値を 0.30 → 0.40 m にした。

#### I3: 浮けない自動離陸

通常の自動離陸（`alt_auto_takeoff`・`pos_auto_takeoff`・API 離陸）は、TAKEOFF の開始から 0.49 s で床を離れ、0.86 s で高さ 0.15 m（ToF の空中）に達する。5 s（`kAutoTakeoffNoLiftAbortUs`）はその 5 倍以上。`weak_spool_alt`（健全度 0.85）は約 11 s（TAKEOFF の開始の 5 s 後）に DISARM し、床の上の最大高度は 0.021 m（6 回目のビルドでは 0.385 m まで浮き、`Auto-takeoff complete`）。`weak_spool_api` は `error takeoff aborted`、14.0〜17.5 s の `duty_max` 0（6 回目のビルドでは 0.77 でモータが回り続けた）。

#### 6 回目の評価表を最終ビルドで測り直した差分（972 本、旧 = 6 回目の最終ビルド、新 = 7 回目の最終ビルド）

| 場面 | corr | 成立/全 旧→新 | 出力停止時からの上昇増分 [m] 旧→新 | LANDING 以後の余分な上昇 [m] 旧→新 | 再上昇 [m] 旧→新 | 接地速度 [m/s]（ノイズなし・n0 最大）旧→新 | 接地速度 [m/s]（n2 最大）旧→新 | 空中停止 旧→新 | 床から再離陸 旧→新 |
|---|---|---|---|---|---|---|---|---|---|
| 自動離陸の上昇初期 | 0.9 | 54/54 → 54/54 | 0.49 → 0.49 | 0.09 → 0.09 | 0.000 → 0.000 | 0.54 → 0.54 | 1.45 → 1.45 | 0 → 0 | 0 → 0 |
| 自動離陸の上昇初期 | 1.0 | 54/54 → 54/54 | 0.65 → 0.65 | 0.11 → 0.11 | 0.000 → 0.000 | 0.86 → 1.00 | 1.61 → 1.61 | 0 → 0 | 0 → 0 |
| 自動離陸の上昇初期 | 1.2 | 54/54 → 54/54 | 0.90 → 0.94 | 0.12 → 0.13 | 0.000 → 0.000 | 0.89 → 0.89 | 1.61 → 1.67 | 0 → 0 | 0 → 0 |
| API 離陸中 | 0.9 | 54/54 → 54/54 | 0.43 → 0.43 | 0.08 → 0.08 | 0.000 → 0.000 | 0.39 → 0.39 | 1.28 → 1.28 | 0 → 0 | 0 → 0 |
| API 離陸中 | 1.0 | 54/54 → 54/54 | 0.63 → 0.63 | 0.11 → 0.11 | 0.000 → 0.000 | 0.84 → 0.84 | 1.47 → 1.47 | 0 → 0 | 0 → 0 |
| API 離陸中 | 1.2 | 54/54 → 54/54 | 0.95 → 0.95 | 0.17 → 0.17 | 0.000 → 0.000 | 0.72 → 0.72 | 1.58 → 1.58 | 0 → 0 | 0 → 0 |
| 0.5 m ホバー | 0.9 | 54/54 → 54/54 | 0.02 → 0.02 | 0.03 → 0.03 | 0.000 → 0.000 | 0.88 → 0.88 | 1.31 → 1.31 | 0 → 0 | 0 → 0 |
| 0.5 m ホバー | 1.0 | 54/54 → 54/54 | 0.01 → 0.01 | 0.07 → 0.07 | 0.000 → 0.000 | 0.88 → 0.88 | 1.22 → 1.22 | 0 → 0 | 0 → 0 |
| 0.5 m ホバー | 1.2 | 54/54 → 54/54 | 0.01 → 0.00 | 0.08 → 0.08 | 0.000 → 0.000 | 0.93 → 0.92 | 1.85 → 1.84 | 0 → 0 | 0 → 0 |
| 1.2 m ホバー | 0.9 | 54/54 → 54/54 | 0.02 → 0.02 | 0.02 → 0.02 | 0.000 → 0.000 | 1.02 → 1.02 | 1.43 → 1.43 | 0 → 0 | 0 → 0 |
| 1.2 m ホバー | 1.0 | 54/54 → 54/54 | 0.01 → 0.01 | 0.03 → 0.03 | 0.002 → 0.002 | 1.00 → 1.00 | 1.38 → 1.38 | 0 → 0 | 0 → 0 |
| 1.2 m ホバー | 1.2 | 54/54 → 54/54 | 0.01 → 0.01 | 0.04 → 0.05 | 0.000 → 0.003 | 0.92 → 0.93 | 1.41 → 1.36 | 0 → 0 | 0 → 0 |
| ALT_HOLD 手動上昇中 | 0.9 | 54/54 → 54/54 | 0.13 → 0.13 | 0.03 → 0.03 | 0.000 → 0.000 | 1.03 → 1.03 | 1.68 → 1.68 | 0 → 0 | 0 → 0 |
| ALT_HOLD 手動上昇中 | 1.0 | 54/54 → 54/54 | 0.13 → 0.13 | 0.02 → 0.02 | 0.000 → 0.000 | 1.03 → 1.03 | 1.75 → 1.75 | 0 → 0 | 0 → 0 |
| ALT_HOLD 手動上昇中 | 1.2 | 54/54 → 54/54 | 0.12 → 0.12 | 0.02 → 0.02 | 0.000 → 0.002 | 1.19 → 1.19 | 2.04 → 2.02 | 0 → 0 | 0 → 0 |
| STABILIZE 21 s 飛行 → 通信断 | 0.9 | 20/54 → 20/54 | 0.00 → 0.00 | 0.00 → 0.00 | 0.000 → 0.000 | 0.76 → 0.76 | - → - | 0 → 0 | 0 → 0 |
| STABILIZE 21 s 飛行 → 通信断 | 1.0 | 20/54 → 20/54 | 0.00 → 0.00 | 0.00 → 0.00 | 0.000 → 0.000 | 0.76 → 0.76 | - → - | 0 → 0 | 0 → 0 |
| STABILIZE 21 s 飛行 → 通信断 | 1.2 | 20/54 → 20/54 | 0.00 → 0.00 | 0.00 → 0.00 | 0.000 → 0.000 | 0.76 → 0.76 | - → - | 0 → 0 | 0 → 0 |

成立した実行: 旧 870 本 / 新 870 本。空中停止: 旧 0 / 新 0; 再上昇>0.01 m: 旧 0 / 新 0; 床から再離陸: 旧 0 / 新 0; 未終了: 旧 0 / 新 0

#### 再確認試験（SILS 全件）と既存シナリオの metric の変化（7 回目）

`.expect` 付き 91 本（6 回目の 86 本 + 新規 5 本: `weak_spool_alt_offset`・`weak_spool_stabilize_offset`・`weak_spool_api`・`ceiling_brake_stale_velocity`・`ceiling_brake_stale_velocity_alt`）: **PASS 87 / FAIL 0 / KNOWN-FAIL 3（`commloss_land_level`・`pos_yaw`・`yaw_hold`）/ SKIP 1（`workshop_acro`）**。基準（86 本で PASS 82 / FAIL 0 / KNOWN-FAIL 3 / SKIP 1）の 82 本はすべて PASS のまま。6 回目のビルド（開始時の作業ツリー）で同じ 91 本を走らせた結果は PASS 81 / FAIL 6 / KNOWN-FAIL 3 / SKIP 1 で、不合格は新規 5 本と、期待値を更新した `weak_spool_alt`。`disarm_long_idle` は単独で 5 回走らせた（下）。

既存シナリオの metric（329 件）のうち、6 回目のビルドから変わったのは次の 8 件（ほかは小数 3 桁まで同一）:

| シナリオ | 値の変化 | 理由 |
|---|---|---|
| `calib` | `att_rmse` 0.0071 → 0.0049 rad | 1.2 m 付近を飛ぶ飛行で、天井ブレーキが ESKF の上昇率だけを見るようになった（C1）ためと推定（個別の切り分けは未実施）。小さくなる側 |
| `pos_flight` | `tilt_max` 0.2362 → 0.2382 rad（閾値 0.314） | 同上（推定） |
| `tof_invalid_alt_hover` | `alt_max` 0.2005 → 0.3179 m（閾値 0.30 → 0.40） | I2 の代償: 床を測れない 2 回目の離陸で、上昇の証拠が約 0.12 m 遅れる（上の「代償」） |
| `tof_silent_est_high_noise` | `alt_max` 1.252 → 1.2545 m | 天井ブレーキの判定が ESKF のみになった（C1）。0.003 m |
| `tof_silent_takeoff` | `alt_max` 0.2309 → 0.2288 m、`descent_speed_max` 0.5389 → 0.9242 m/s（閾値 1.15） | 床を離れた証拠が上向きの加速の後になったので、離陸の直後の出力停止では支持推力の窓が開くのが遅れ、降下の開始が「低くした見積もり」になった（推定。個別の切り分けは未実施）。閾値内 |
| `weak_spool_alt`（新規扱い） | 床の上の `alt_max` 0.385 → 0.021 m | I3 の仕様変更 |
| `tof_recover_invalid_first`・`tof_recover_reflip` | シナリオの入力を変更（上の「代償」）。`alt_rmse` 0.0254 → 0.0268、0.0242 → 0.018、`alt_max`（reflip）1.38 → 1.63 m（閾値 1.7） | 離陸の加速度を強めたため |

`disarm_long_idle`（2300 s の長いシナリオ）: 6 回目の評価で並行実行中に 1 回不合格だった。単独で 5 回走らせると **5 回とも合格し、metric の値も 5 回とも同一**（`alt_min` 0.6357 m、`alt_max` 0.0211 m）で、揺れは無かった。並行実行（12 本）では 7 回目の途中の 3 回の全件実行すべてで合格。不合格の原因は、確認できた範囲では、走行中・走行済みの実行ファイルを `cp` で上書きした直後に OS が実行ファイルを強制終了する（終了コード −9、フライトログ無し）こと（7 回目の途中で再現。評価用のスクリプトは `rm` してから `cp` するようにした）で、6 回目の 1 回の不合格が同じ原因かは確定できない（未確認）。シナリオ側の揺れの原因は見つからなかった。

#### 関数の長さ（コメント込み）

`StateManager::transition` 56 → 33 行（`enterState()` 13 行を分離）、`processEstimatorCommands` 53 → 48 行（`reloadEstimatorParams()` を分離）、`DataStream::appendEntries` 137 → 8 行（`append400HzEntries()` 46 行、`appendFlightPhase400Entry()` 18 行、`appendFlightFlagsEntry()` 25 行、`appendPilotAndReferenceEntries()` 50 行に分離。パケット内の順序は不変）。新規の `ceilingBrakeClimbRate()` 6 行、`updateAutoTakeoff()` 15 行、`accumulateRise()` 16 行。

#### 未確認の項目

- 実機: C1・I2・I3・I4 のいずれも実機は未確認。実機ログ 3 本の再生ハーネス（`rv10_replay.cpp` など）は今回の変更後に再実行していない（未確認）。実機ログの床の上のろ波した超過は約 1.0 m/s² に達した（上の判断）ので、実機での上昇の証拠の誤成立の有無は未確認。
- `kBlindLandingMinSilentUs`（4 s）の「ToF が生きたまま 10 s を超える LANDING の後に出力停止」の SILS シナリオは作れなかった（天井 1.2 m からの降下は 0.3 m/s で約 4 s で終わり、ToF が生きたまま LANDING を 10 s 以上続ける入力が無い）。host テスト `landing_after_a_long_live_tof_gets_the_minimum_silent_time_before_the_motors_are_cut`・`landing_clock_counts_the_continuous_silence_of_this_landing` で規範表の行を検証している。
- workshop: ファームのビルドは成功。SILS の workshop ターゲットと実機での動作は未確認。
- 上昇の証拠のしきい値 0.4 m/s² の両側の余裕が薄い（上の判断）。

### 8 回目の見直し（2026-11-10）— 7 回目の修正に対する独立レビュー 2 本の指摘（根本原因・D1〜D6・E1〜E5）

**根本原因:** 「床を離れた」の判定を加速度計だけ（上向き加速の証拠 0.4 m/s² の後から数える上昇量）で作り、安全判断の門にした。ゆっくり浮く機体では証拠が立たず、逆に床の上でも立ち上がりと同時に入るオフセットで誤成立する。**原則: 証拠が「無い」ことを、床にいる証拠として扱わない。加速度計だけの判定を安全判断の門にしない。**

| 項目 | 変更 | 検証 |
|---|---|---|
| D1 ToF が使えない自動離陸が上昇し続ける（3.8〜7.6 m） | TAKEOFF 中の無効サンプル計数の 2 s の時間の備えから `left_floor` の条件を外した（HEAD に戻す） | 下の表（m 行列 90 本、`tof_invalid_alt_hover` の `--thrust-eff`）、`weak_spool_alt`・`weak_spool_api`（床の上の LANDING を許容する期待値に書き直し） |
| D2 手動の TAKEOFF が出力停止で FLYING にならない | HEAD に戻す（床の上で FLYING 表示になるのは許容。`operation_manual.md` に記載） | `weak_spool_stabilize`（期待値を書き直し）、`u_*` 14 本 |
| D3 TAKEOFF 中の通信喪失の自動着陸が永久に保留 | 猶予が過ぎたら TAKEOFF からも LANDING（`decideCommLossLanding()`） | 新シナリオ `commloss_takeoff`、`u_0_3095` に通信断を足した SILS |
| D4 浮けない自動離陸の中止が空中で DISARM する | 中止は、健全な ToF が床の近く・増えないと示す積極的な証拠（`TakeoffFloorWatch`）があるときだけ | 新シナリオ `weak_spool_alt_floor`（中止）・`weak_spool_alt_slow_lift`（中止しない）、`w_*` 18 本、実機ログ 3 本の再生 |
| D5 加速度計の上向きの証拠を安全判断の門から外す | 証拠・`left_floor`・`leftFloor()`・workshop の発行・`kManualTakeoffMinRiseM`・`craftIsOffTheFloor()` を削除。`tof_recover_*`・`tof_invalid_alt_hover` を HEAD の入力・閾値に戻した | 全件の SILS（`tof_recover_*`・`tof_invalid_alt_hover` は HEAD の入力で合格）、ビルド 2 種 |
| D6 判断を純関数に | `takeoff_policy.hpp`（`decideTakeoff()`・`decideCommLossLanding()`・`TakeoffFloorWatch`）、`architecture.md` の INV-4 | `test_takeoff_policy`（12 本）、規範表の全セル |
| E1 API の `land` の後の再上昇 | 着陸のループの基準を、較正した支持推力の 1.08 倍までに抑える（`landingBaseThrust()`） | 下の E1（45 本） |
| E2 自動離陸の行き過ぎ | 余裕 `kTakeoffVelocityDisagreeMps` の掃引（120 本）。**採らない** | 下の E2 |
| E3 `kBlindLandingMinSilentUs` | 4 s → 5.5 s | 972 本の評価表（最長 4.7 s → 8 回目の最終ビルドで 4.0 s） |
| E4 天井ブレーキの断続作動 | 止まる高さの予測の超過に比例する上限（`kAltitudeCeilingBrakeRampM` 0.1 m） | 下の E4（60 本 + 別案の試行） |
| E5 軽微 | 文章の食い違い、50 行、作業用の名前、言い切り、用語「沈黙」→「出力停止」 | 目視、関数の長さの検査 |

#### D1・D2・D3・D4: 7 回目のレビューの再現条件を最終ビルドで再実行（HEAD / 7 回目の最終ビルド / 8 回目の最終ビルド）

`m_*`・`u_*`・`w_*`・`Na_*` と、`tof_invalid_alt_hover` を `--thrust-eff` で走らせる条件は、レビューの再現条件を作業用に複製したシナリオで、リポジトリには置かない（入力は表の見出しの記述どおり: `m_*` は `tof_invalid_alt_hover` の ARM 後に加速度計オフセットを注入した変種、`u_*` は STABILIZE でスロットル 3088〜3105 の離陸の直後に `tof_silent 1`、`w_*` は ALT_HOLD の自動離陸でモータの健全度を 0.855〜0.88 にした変種）。

| 条件 | 本数 | HEAD | 7 回目 | 8 回目 |
|---|---|---|---|---|
| m 行列（`tof_invalid_alt_hover` 型。健全度 0.95 / 0.97 / 1.0 × 加速度オフセット 5 通り × `hover.thrust_corr` なし・0.9・1.2 × ノイズ n0 / n2） | 90 | 最高 0.93 m、1.35 m 超 0 本、空中停止 0 本 | **最高 7.57 m、1.35 m 超 17 本**、空中停止 0 本 | 最高 0.93 m、1.35 m 超 0 本、空中停止 0 本 |
| 同: 接地速度が 1.5 m/s 超 / 1.0 m/s 超の本数 | 90 | 4 / 46 | 20 / 39 | 4 / 13（4 本は HEAD と同じ実行で 1.59〜1.86 m/s） |
| `tof_invalid_alt_hover`（`--thrust-eff` 0.9 / 0.95、ノイズ n0）の最高高度 | 2 | 0.44 / 0.69 m | **2.09 / 3.91 m**（2 回目の離陸が空中で DISARM） | 0.44 / 0.69 m（HEAD と同じ） |
| 同（ノイズ n2） | 2 | 0.95 / 1.05 m | 0.95 / 1.05 m | 0.95 / 1.05 m |
| `u_*`（STABILIZE、スロットル 3088〜3105、途中で ToF 出力停止。通信断を足さない） | 14 | 14 本とも FLYING → LANDING（最高 0.02〜2.82 m） | **14 本とも TAKEOFF のまま**（最高 0.02〜5.29 m、着陸しない） | HEAD と同じ（14 本とも一致） |
| `u_0_3095` に通信断（8 s の無音）を足したもの | 1 | 最高 1.07 m、FLYING 5.6 s、LANDING 11.8 s、接地 0.76 m/s | **TAKEOFF のまま 2.76 m まで上昇し、16.6 s に高さ 2.73 m でモータが止まり、7.3 m/s で衝突** | HEAD と同じ |
| `w_*`（ALT_HOLD、健全度 0.855〜0.88、オフセット −0.3 / 0 / +0.3、不感帯あり） | 18 | 18 本とも床の上で LANDING → IDLE | 18 本とも TAKEOFF → IDLE（DISARM）。健全度 0.875〜0.88 の 4 本は 0.11〜0.18 m まで浮いてから DISARM し、1.2〜1.63 m/s で落下 | HEAD と同じ（18 本とも一致。最高 0.02 m、空中の DISARM 0 本） |
| 健全度 0.85〜0.9 × 不感帯（切る / あり）の ALT_HOLD 自動離陸 | 10 | 不感帯を切ると 0.36〜0.37 m まで浮いて FLYING（中止なし）。あると床の上で LANDING | 健全度 0.85〜0.88 は 5 s で DISARM（0.87 は 0.07 m まで浮いて 0.8 m/s、0.88 は 0.18 m まで浮いて 1.63 m/s で落下）。0.9 は飛ぶ | 不感帯を切ると、0.85・0.86 は 5 s で DISARM（床の上）、0.87 以上は離陸を完了して飛ぶ。不感帯があると全て床の上で LANDING（HEAD と同じ） |

D4 の注: 実機のセンサは床の上で無効を返す（実機ログ 3 本の TAKEOFF の最初の 186〜344 ms が無効）ので、実機で床の積極的な証拠が立つのは ToF が有効に読める床（マット、傾き）に限られる。実機ログ 3 本の TAKEOFF の ToF サンプルを `TakeoffFloorWatch` に通すと、3 本とも証拠は一度も立たない（TAKEOFF の ToF は無効 → 0.031〜0.041 m → 上昇で、1 s の保持に届かない）。

#### E1: API の `land` の後の再上昇（`hover.thrust_corr` × seed 1〜3、ノイズ n2、1.2 m まで上げてから `land`）

LANDING に入った後の最大上昇 [m]（HEAD / 7 回目 / 8 回目）: corr 1.0: 0.03〜0.05 / 0.03〜0.05 / 0.02〜0.04、1.05: 0.08〜0.13 / 0.08〜0.11 / 0.01〜0.03、1.1: 0.17〜0.24 / 0.17〜0.26 / 0.03〜0.09、1.15: 0.27〜0.32 / 0.28〜0.29 / 0.07、1.2: （HEAD は離陸で 2.0 m まで行き過ぎ、LANDING の前に 2.5 m/s で接地するので比較できない）/ 0.60〜0.65 / 0.00〜0.04。

原因: ToF 生存中の着陸の速度ループは hover 推力の「見積もり」から始まり、積分が遅い。見積もりが支持推力の 1.24 倍のとき（corr 1.2）、推力は数秒間、機体の重さを上回った。HEAD でも起きる。

試した案と結果:

| 案 | 上昇 [m] | `landing_dead_zone` | 採否 |
|---|---|---|---|
| (a) 上向き速度が 0 / 0.03 / 0.1 m/s を超える間だけ、推力を支持推力の 0.95 倍に制限 | 0.04〜0.57 | 0.1 m/s は合格（0・0.03 は未実施） | 不十分 |
| (b) 目標の降下速度に達するまで推力を支持推力の 0.95 倍に制限、または基準を支持推力そのものに制限 | 0.00〜0.05 / 0.00〜0.01 | **不合格**（後者は接触が約 0.2 → 0.34 m/s） | 採らない |
| (c) 基準を支持推力の 1.05 倍までに制限 | 0.00〜0.04 | **不合格**（3.2 G の衝撃、接触 0.27 m/s。見積もりが支持推力の 1.057 倍で、推力 0.4 % の差で経路が変わる） | 採らない |
| (d) 基準を支持推力の 1.08 倍までに制限 | 0.00〜0.09 | 合格（変化なし。見積もり 1.057 倍が上限の内側） | **採用** |
| 同 1.10 / 1.15 倍 | 0.05〜0.11 / 0.17〜0.29 | 合格 | 効きが足りない |

`landing_dead_zone` は、わずかな推力の差で接地の経路（柔らかい接触か、3 G の衝撃か）が入れ替わる境目にあるシナリオで、1.08 は、その 1.057 の外側にある最小の値として選んだ（根拠の数値は `flight_limits.hpp` の `kLandingBaseOverSupportMax`）。

#### E2: 自動離陸の行き過ぎ（API の離陸、`hover.thrust_corr` 0.9 / 1.0 / 1.1 / 1.2 × ノイズ n0 / n2 × seed 1〜3 の 24 本、真値の最高 [m]）

| 余裕 [m/s] | 0.9 n0 | 0.9 n2 | 1.0 n0 | 1.0 n2 | 1.1 n0 | 1.1 n2 | 1.2 n0 | 1.2 n2 | 最悪 |
|---|---|---|---|---|---|---|---|---|---|
| 0.6（現行） | 0.60 | 0.77〜0.82 | 0.83 | 1.11〜1.14 | 1.20 | 1.26〜1.29 | 1.30 | 1.29〜1.29 | 1.296 |
| 0.45 | 0.60 | 0.77〜0.79 | 0.83 | 1.06〜1.13 | 1.20 | 1.26〜1.28 | 1.29 | 1.29〜1.29 | 1.295 |
| 0.3 | 0.60 | 0.76〜0.78 | 0.83 | 1.03〜1.10 | 1.16〜1.16 | 1.26〜1.26 | 1.29 | 1.29〜1.31 | 1.311 |
| 0.2 | 0.56 | 0.69〜0.77 | 0.80 | 1.01〜1.11 | 1.15 | 1.25〜1.26 | 1.28 | 1.29〜1.31 | 1.314 |
| 0.1 | 0.57 | 0.66〜0.77 | 0.79 | 0.99〜1.06 | 1.11 | 1.26〜1.28 | 1.28 | 1.29〜1.30 | 1.303 |

採らない理由: (1) 最悪の `hover.thrust_corr` 1.2 は 1.28〜1.31 m のまま変わらない（行き過ぎを決めるのは天井ブレーキで、余裕ではない）。(2) 改善は 1.0 の n2（1.11〜1.14 → 0.99〜1.06 m）、0.9 の n2（0.77〜0.82 → 0.66〜0.77 m）、1.1 の n0（1.20 → 1.11 m）の最大約 0.09 m だけで、0.5 m の目標に対する行き過ぎは残る。(3) 余裕を小さくすると、普通の離陸の最初の 1 s（積分と ESKF が最大 0.5 m/s 違う）で積分が速度ループに入り、余裕の小さい `alt_recenter_gate`・`landing_dead_zone` の挙動が変わるおそれがある。余裕 0.6〜0.1 のどれでも `alt_recenter_gate`・`alt_auto_takeoff`・`pos_auto_takeoff`・`alt_takeoff_steer`・`alt_flight`・`api_flight`・`pos_flight` は合格した（`landing_dead_zone` は、この掃引の時点で E1 の途中の案が入っていて 0.6 を含む全てで不合格だったため、この掃引では未確認）。余裕は 0.6 のままにする。

#### E4: 天井ブレーキの断続作動（1.2 m のホバーと離陸の行き過ぎ、`hover.thrust_corr` 0.9 / 1.0 / 1.2 × ノイズ n0 / n2 × seed 1〜2 の 12 本、ブレーキが深さの半分より深くなった回数〔seed 1 / 2〕と真値のピーク）

| 幅 `kAltitudeCeilingBrakeRampM`（0 = 硬いブレーキ） | 0.9 n0 | 0.9 n2 | 1.0 n0 | 1.0 n2 | 1.2 n0 | 1.2 n2 | 最悪のピーク |
|---|---|---|---|---|---|---|---|
| 0（旧） | 136 / 134 | 9 / 12 | 18 / 18 | 0 / 5 | 2 / 2 | 38 / 38 | 1.288 m |
| 0.03 m | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 2 / 2 | 9 / 9 | 1.279 m |
| 0.06 m | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 1 / 1 | 3 / 10 | 1.290 m |
| 0.1 m（採用） | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 1 / 1 | 3 / 3 | 1.296 m |

比較した別案（seed 1・2）: 上昇率の下限（0.1 / 0.15 / 0.2 / 0.3 m/s 以下ではブレーキしない）は、回数を 89 → 3 / 3 / 2 / 1 回などに減らすが、ピークが 1.363〜1.375 m（0.9 n0）、1.348〜1.693 m（1.2 n2）に増え、1.35 m の限界を超える。解除側のヒステリシス（0.02〜0.06 m）は 89 → 27〜32 回にしか減らない（上昇率が 0 以下になると解除されるため）。比例させる案は回数を 0〜3 に減らし、ピークは 0.008 m しか増えない。文書の「天井でのホバーには触れない」は、「予測の止まる高さが 1.25 m 未満なら触れない。それより上では比例して働く」に直した。

#### 7 回目の評価表を最終ビルドで測り直した差分（972 本、旧 = 7 回目の最終ビルド、新 = 8 回目の最終ビルド）

| 場面 | corr | 成立/全 旧→新 | 出力停止時からの上昇増分 [m] 旧→新 | LANDING 以後の余分な上昇 [m] 旧→新 | 再上昇 [m] 旧→新 | 接地速度 [m/s]（ノイズなし・n0 最大）旧→新 | 接地速度 [m/s]（n2 最大）旧→新 | 空中停止 旧→新 | 床から再離陸 旧→新 |
|---|---|---|---|---|---|---|---|---|---|
| 自動離陸の上昇初期 | 0.9 | 54/54 → 54/54 | 0.49 → 0.49 | 0.09 → 0.09 | 0.000 → 0.000 | 0.54 → 0.54 | 1.45 → 1.45 | 0 → 0 | 0 → 0 |
| 自動離陸の上昇初期 | 1.0 | 54/54 → 54/54 | 0.65 → 0.65 | 0.11 → 0.11 | 0.000 → 0.000 | 1.00 → 0.86 | 1.61 → 1.61 | 0 → 0 | 0 → 0 |
| 自動離陸の上昇初期 | 1.2 | 54/54 → 54/54 | 0.94 → 0.95 | 0.13 → 0.14 | 0.000 → 0.000 | 0.89 → 0.89 | 1.67 → 1.67 | 0 → 0 | 0 → 0 |
| API 離陸中 | 0.9 | 54/54 → 54/54 | 0.43 → 0.43 | 0.08 → 0.08 | 0.000 → 0.000 | 0.39 → 0.39 | 1.28 → 1.28 | 0 → 0 | 0 → 0 |
| API 離陸中 | 1.0 | 54/54 → 54/54 | 0.63 → 0.63 | 0.11 → 0.11 | 0.000 → 0.000 | 0.84 → 0.84 | 1.47 → 1.47 | 0 → 0 | 0 → 0 |
| API 離陸中 | 1.2 | 54/54 → 54/54 | 0.95 → 0.95 | 0.17 → 0.17 | 0.000 → 0.000 | 0.72 → 0.72 | 1.58 → 1.58 | 0 → 0 | 0 → 0 |
| 0.5 m ホバー | 0.9 | 54/54 → 54/54 | 0.02 → 0.02 | 0.03 → 0.03 | 0.000 → 0.000 | 0.88 → 0.88 | 1.31 → 1.31 | 0 → 0 | 0 → 0 |
| 0.5 m ホバー | 1.0 | 54/54 → 54/54 | 0.01 → 0.01 | 0.07 → 0.07 | 0.000 → 0.000 | 0.88 → 0.88 | 1.22 → 1.22 | 0 → 0 | 0 → 0 |
| 0.5 m ホバー | 1.2 | 54/54 → 54/54 | 0.00 → 0.00 | 0.08 → 0.08 | 0.000 → 0.000 | 0.92 → 0.93 | 1.84 → 1.71 | 0 → 0 | 0 → 0 |
| 1.2 m ホバー | 0.9 | 54/54 → 54/54 | 0.02 → 0.02 | 0.02 → 0.02 | 0.000 → 0.000 | 1.02 → 1.02 | 1.43 → 1.43 | 0 → 0 | 0 → 0 |
| 1.2 m ホバー | 1.0 | 54/54 → 54/54 | 0.01 → 0.01 | 0.03 → 0.03 | 0.002 → 0.002 | 1.00 → 1.00 | 1.38 → 1.38 | 0 → 0 | 0 → 0 |
| 1.2 m ホバー | 1.2 | 54/54 → 54/54 | 0.01 → 0.01 | 0.05 → 0.05 | 0.003 → 0.002 | 0.93 → 0.91 | 1.36 → 1.37 | 0 → 0 | 0 → 0 |
| ALT_HOLD 手動上昇中 | 0.9 | 54/54 → 54/54 | 0.13 → 0.13 | 0.03 → 0.03 | 0.000 → 0.000 | 1.03 → 1.03 | 1.68 → 1.68 | 0 → 0 | 0 → 0 |
| ALT_HOLD 手動上昇中 | 1.0 | 54/54 → 54/54 | 0.13 → 0.13 | 0.02 → 0.02 | 0.000 → 0.000 | 1.03 → 1.03 | 1.75 → 1.75 | 0 → 0 | 0 → 0 |
| ALT_HOLD 手動上昇中 | 1.2 | 54/54 → 54/54 | 0.12 → 0.11 | 0.02 → 0.02 | 0.002 → 0.000 | 1.19 → 1.17 | 2.02 → 1.98 | 0 → 0 | 0 → 0 |
| STABILIZE 21 s 飛行 → 通信断 | 0.9 | 20/54 → 20/54 | 0.00 → 0.00 | 0.00 → 0.00 | 0.000 → 0.000 | 0.76 → 0.76 | - → - | 0 → 0 | 0 → 0 |
| STABILIZE 21 s 飛行 → 通信断 | 1.0 | 20/54 → 20/54 | 0.00 → 0.00 | 0.00 → 0.00 | 0.000 → 0.000 | 0.76 → 0.76 | - → - | 0 → 0 | 0 → 0 |
| STABILIZE 21 s 飛行 → 通信断 | 1.2 | 20/54 → 20/54 | 0.00 → 0.00 | 0.00 → 0.00 | 0.000 → 0.000 | 0.76 → 0.76 | - → - | 0 → 0 | 0 → 0 |

成立した実行: 旧 870 本 / 新 870 本。空中停止: 旧 0 / 新 0; 床から再離陸: 旧 0 / 新 0; 未終了: 旧 0 / 新 0

出力停止から最初の接触までの最長（E3）: 7 回目の最終ビルド 4.66 s（ALT_HOLD 手動上昇中の 4.7 s）→ 8 回目の最終ビルド 4.02 s（同 4.0 s）。`kBlindLandingMinSilentUs` 5.5 s は両方を 0.8 s 以上の余裕で覆い、後半の低下（5 s から推力が下がり、8.33 s でゼロ）の前に収まる。

#### 再確認試験（SILS 全件）と既存シナリオの metric の変化（8 回目）

`.expect` 付き 94 本（7 回目の 91 本 + 新規 3 本: `weak_spool_alt_floor`・`weak_spool_alt_slow_lift`・`commloss_takeoff`）: **PASS 90 / FAIL 0 / KNOWN-FAIL 3（`commloss_land_level`・`pos_yaw`・`yaw_hold`）/ SKIP 1（`workshop_acro`）**。基準（91 本で PASS 87 / FAIL 0 / KNOWN-FAIL 3 / SKIP 1）に、新規 3 本の合格を足した値と一致。

同じ 94 本を 7 回目の最終ビルドで走らせると PASS 79 / FAIL 11 / KNOWN-FAIL 3 / SKIP 1: 新規 3 本、書き直した `weak_spool_*` 5 本、HEAD の入力・閾値に戻した `tof_invalid_alt_hover`・`tof_recover_invalid_first`・`tof_recover_reflip` の 3 本が不合格（`tof_invalid_alt_hover` は `alt_max` 0.3179 m > 0.30）。HEAD のビルドでは新規 3 本のうち `commloss_takeoff`・`weak_spool_alt_floor` が不合格で、`weak_spool_alt_slow_lift` は合格する（HEAD は中止しない）。

既存シナリオの metric（上で挙げた入力・期待値を変えたシナリオを除く 316 件）のうち、7 回目の最終ビルドから変わったのは 46 件（すべて合格のまま）。変わった理由（推定。個別の切り分けは未実施）:

| シナリオ | 値の変化 | 理由 |
|---|---|---|
| `tof_silent_takeoff` | `descent_speed_max` 0.9242 → 0.5389 m/s、`alt_max` 0.2288 → 0.2309 m | 上向き加速の証拠を消したので、離陸の直後の出力停止でも支持推力の窓が早く開く（6 回目の値 0.5389 に戻った） |
| `tof_silent_est_high_noise` | `alt_max` 1.2545 → 1.2977 m（閾値 1.55） | E4（天井ブレーキが比例する） |
| `ceiling_brake_stale_velocity_alt` | `alt_min` 1.177 → 1.198 m、`alt_max` 1.259 → 1.269 m、降下速度の最大 0.125 → 0.088 m/s | E4 |
| `calib`・`pos_flight`・`api_flight`・`boot_motion`・`crash_refly`・`flip_*`・`api_flip_*`・`tof_silent_est_high` | `att_rmse`・`tilt_max`・`alt_*`・`flip_*_deg`・`settle_time_s` などが 3 桁目から変わる（例: `calib` の `att_rmse` 0.0049 → 0.0082 rad、`api_flip_pitch` の `tilt_max` 0.0421 → 0.0846 rad、`crash_refly` の `alt_max` 1.2249 → 1.2637 m） | 1.2 m 付近を飛ぶ飛行で、E4 の比例ブレーキが硬いブレーキとは異なる推力を出す（推定） |
| `weak_spool_alt`・`weak_spool_api`・`weak_spool_stabilize`（+ `_offset` 2 本） | 期待値を書き直した（上の D1・D2） | 8 回目の判断（7 回目の仕組みの取り下げ）。HEAD に無かったシナリオなので、HEAD の合否は変わらない（HEAD でも合格） |
| `tof_invalid_alt_hover`・`tof_recover_invalid_first`・`tof_recover_reflip` | 7 回目で変えた入力・閾値を HEAD の値に戻した | D5。HEAD の入力で合格 |

#### 関数の長さ・ビルド・ホスト試験

- 50 行（コメント込み）を超える関数で、HEAD からの変更を含むもの: `PidController::compute`（740 行）と `StateTask`（501 行）の 2 本のみ（いずれも HEAD から 50 行超で、今回の変更は数行）。新規・変更した関数（`decideTakeoff`・`decideCommLossLanding`・`TakeoffFloorWatch`・`updateTakeoff`・`brakeAboveCeiling`・`landingBaseThrust`・`handleCommLossGrace`・`publishControllerStatus`〔50 行〕）は 50 行以内、ネスト 2 段以内。
- `sf build vehicle` 成功（バイナリ 1177.1 KB）。workshop ファーム（`firmware/workshop` を作業用の複製で、`vehicle` と `common` は実体を指して）のビルド成功。
- ホスト試験 9 ファイル 215 本合格（`test_takeoff_policy` 12 本を追加、`test_blind_landing` は上昇の証拠の試験 2 本を削除して 46 本）。

#### 未確認の項目

- 実機: 8 回目の変更はいずれも実機は未確認。実機ログ 3 本の再生は、`TakeoffFloorWatch` に TAKEOFF の ToF サンプルを通す形だけ行った（上の D4 の注）。上昇の証拠を消したので、7 回目の床の上のオフセットの階段の試験（`rv13_lift`）は対象が無い（再実行不可）。
- E1 の 1.08・E4 の 0.1 m・E3 の 5.5 s は SILS で決めた候補値。E1 の 1.08 は `landing_dead_zone` の境目に合わせた値で、実機の着陸が同じ境目を持つかは未確認。
- D1 で受け入れた「床の上の ToF なし LANDING」は、モータが `kNoTofThrustCapRatio` 以下の推力で数秒（`weak_spool_alt` で 8.1 s から 14.5 s の約 6 s）回り、接地検出（床が機体を支える）で止まる。実機は未確認。
- m 行列の 90 本のうち 4 本は、接地速度が 1.59〜1.86 m/s（ToF なしの降下の最初の離陸の上昇中 0.64〜0.93 m からの接地）で、HEAD と同じ値。8 回目では未解決。
- 既存シナリオの metric が変わった理由（E4・上昇の証拠の削除）は推定で、個別の切り分けは未実施。
- E2 の掃引では `landing_dead_zone` は未確認（E1 の途中の案が入っていたため）。最終ビルドの `landing_dead_zone` は合格。
- workshop: ファームのビルドは成功。SILS の workshop ターゲットと実機での動作は未確認（`workshop_acro` は SKIP）。HEAD の workshop は `controller_status` を発行しない（`takeoff_reached` が立たず、ALT_HOLD / POS_HOLD の自動離陸が完了しない: HEAD と同じ。workshop は高度モードを使わない前提）。

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
| `alt_band` of `alt_auto_takeoff` and `pos_auto_takeoff` (window 12-16 s) | < 0.20 -> < 0.55 (SILS value 0.2668); lower bound > 0.05 added 2026-11-01 by user decision | (b) The old bound was tuned to the old plant's value 0.008; all 14 real take-offs (5 ALT_HOLD, 9 POS_HOLD; ESKF altitude, take-off +4.2 to +8.2 s) are at or above 0.062. The real maximum 0.521 is rounded up. The p90 (0.443) is not used because it would reject 2 of the 14 real take-offs (0.500, 0.521) | 14 real take-offs: p50 0.164, p90 0.443, max 0.521 |
| `alt_mean` lower bound of `api_flight` (window 14-17.5 s) | > 0.42 -> > 0.30 (SILS value 0.4130) | (b) The old bound was tuned to the old plant's value 0.518; 4 of 14 real take-offs are below it. The real minimum 0.326 is rounded down | 14 real take-offs (take-off +7.7 to +11.2 s): min 0.326, p10 0.348, p50 0.473 |
| xfail marker of `calib.expect` | Removed | The xfail reason (0.6-0.7 Hz self-oscillation) no longer appears: XPASS in two regression runs, PASS in two direct runs (att_rmse 0.55 deg) | No real counterpart |
| Maximum distance of the SILS bottom ToF model (`MAX_MM` in `simulator/sils/devices/vl53_device.cpp`) | 1400 -> 1630 mm | (b) In real logs (121 logs + flip_test1/2, about 150k samples) the ToF is valid (status 0) 95-99 % of the time at 1.4-1.7 m, 91 % at 1.7-1.8 m, 66-70 % at 1.8-2.0 m and 22 % at 2.0-2.1 m (the invalid status is mostly 255, as in SILS). The model's ceiling is set by the 24-bin buffer at 1636 mm; 1630 keeps the two-bin split inside the buffer (probed: valid with error <= 17 mm up to 1.63 m). The 1.63-2.0 m range stays unmodelled | Real validity rates (height = ESKF altitude) |
| `alt_band` upper bound of `alt_inflight_switch.expect` (window 14.7-18.7 s) | < 0.32 -> < 0.38 (SILS value 0.351) | The damped oscillation after the STABILIZE->ALT_HOLD capture, which the bound's own comment records as a fixed amplitude independent of the initiating disturbance. R_out value 0.351 plus about 8 % | No real counterpart, unverified (needs a hardware log of an in-flight switch to ALT_HOLD) |
| `crash_refly` (`.scn` rows J and L, two windows in `.expect`) | Row J 2500->500 ms, row L 8000->10000 ms (times from the re-calibration dwell on unchanged). `.expect`: crash-rest window 14-16 s -> 12-14 s, lift window 16.8-18.4 s -> 14.8-16.4 s (thresholds unchanged, shifted by the same -2.0 s as the hand's arrival) | (a) The R_out crash leaves the craft lying on its side (roll 90 deg); the 3.5 s boot calibration that starts about 0.2 s after impact completes on its side and leaves level_offset 1.5708 rad in the estimator, so even after the hand levels the craft the ARM guard (attitude estimate disagrees with gravity; correct behaviour, unchanged) rejected every re-ARM. The handling trajectory already levels any crash attitude, so the hand now arrives during the calibration (14.0 s) and the calibration restarts on the levelled craft. The re-fly really flies (climb to 1.33 m, final hover 0.87 m) | No real counterpart |
| xfail marker of `pos_flight.expect` | Removed | The yaw torque-authority saturation (duty ceiling lock) no longer appears on the R_out plant (lower hover duty leaves yaw headroom). XPASS in two regression runs. The markers of `pos_yaw` and `yaw_hold` stay | No real counterpart |

**Not changed (including unverified items):** the xfail markers of `commloss_land_level`, `pos_yaw` and `yaw_hold` (still KNOWN-FAIL; `commloss_land_level` fails on `alt_max` 0.156 m > 0.1 m after touchdown).

**Added 2026-11-09 (4th review: C2, C3, C5) — the ToF close-range dead zone in the SILS chip model and the altitude ceiling on the stick climb.**

| Scenario | What it checks (measured) |
|----------|---------------------------|
| every scenario | The VL53 model has a close-range dead zone (`vl53_device.cpp` `MIN_MM` = 31 mm, the smallest valid distance of the real logs; the SILS craft rests 21 mm above the floor): the ground reads "no target" as the real craft does, and the first valid sample comes ~0.4 s after the TAKEOFF entry. `tof_deadzone 0` turns it off. Compared with the build before it (72 scenarios present in both), the truth trajectory of 56 differs from the first takeoff on (the first valid ToF sample comes later, which shifts the ESKF ground-to-air handoff: metrics move by a few cm, e.g. a 0.03 m higher hover mean); 16 are identical (no takeoff, or an API takeoff that is cut at the ground). Every pass/fail is unchanged except the scenarios retuned below |
| `alt_ceiling_stick`, `alt_ceiling_stick_noise` | The ALT_HOLD stick held UP stops at the 1.2 m ceiling (tapered climb, truth 1.17-1.18 m; noise n2, 3 seeds: 1.185-1.187 m; 1.97 m without the ceiling) |
| `tof_recover_landing_invalid` | Samples return during the blind landing, all invalid: the episode lasts to the touchdown (before the fix it ended after ~1 s) |
| `tof_invalid_alt_hover` | Takeoff with a permanently invalid ToF is cut 0.15 s after the estimated rise reaches 5 cm: peak 0.20 m (0.15-0.41 m over n2 / thrust error) |
| `landing_dead_zone` | A soft ToF landing (contact below 3 G) with the close-range dead zone modelled (was KNOWN-FAIL; PASS since 2026-11-09): the dead-zone touchdown ends it, 0.2 s after the last valid reading (0.085 m), the craft at rest on the floor (detailed_design §3 note 7) |
| `alt_disarm_land` | Dead zone off (event `tof_deadzone 0`): tests only the routing of the DISARM |
| `tof_silent_hover_1p2`, `tof_silent_alt_climb`, `rc_flip`, `rc_flip_latch`, `alt_inflight_switch`, `alt_recenter_gate` | Retuned because the ceiling bounds the stick climb at 1.2 m (reasons in each `.scn` / `.expect`): the hover is ~1.1 m (silence 20.5 s), the climb silence is at ~0.8 m (0.92 m peak, 0.40 m/s touchdown), the flip scenarios run with `flip.min_height_m` 0.8 / 0.75 m, the two stick-climb scenarios have lower bounds |

The 80-run table above (blind descent, 5 scenes x `hover.thrust_corr` x noise) was measured BEFORE the dead zone and the ceiling and has NOT been re-measured: the dead zone moves the hover scenes by ~0.03-0.06 m and 0.05 m/s, and the 1.2 m hover and manual-climb scenes are now different conditions.
