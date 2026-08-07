# 空中ホバー（ALTITUDE_HOLD）レビュー動画 — 再開ノート

> 作業再開用の自己完結メモ。新しいセッション（空コンテキスト）でも、このファイルだけで
> 「無改変の旧ファーム（`firmware/vehicle` = `emu_vehicle`）をエミュレータ上で空中ホバーさせ、
> `sf sils scenario <scn> --video` でレビュー動画を出す」ところまで進められることを目標にする。
> Self-contained resume note: a fresh session can produce an airborne stable-hover review
> video from this file alone.

> **【2026-06-02 更新・本ノートは上位ノートに引き継ぎ】** 本ノート §1 の「閉ループが 1.12 バイアスを
> 吸収してホバーする」は **ESKF が正しい前提**だった。実際は初の空中試験で ESKF 鉛直が発散（runaway）し、
> 真因は **VL53 モデルが飛行中に valid を返さない**ことと判明（推力物理は正しい＝検証済み）。続きは
> **`vl53_dynamic_validity_resume.md`**（ToF モデルの動的 valid 性の作り込み）と auto-memory
> `project_eskf_vertical_divergence.md` を参照。アームタイミング/throttle 写像/DSL alt トークン等の
> 本ノートの裏取り事実は引き続き有効。

最終更新: 2026-06-02 / 直近コミット: `d03532c`（VL53 M2 完了＝ToF が status0・±17mm で ESKF POS_Z に供給）。
**前提**: 全参照は調査ワークフローで実コード裏取り済み（M2 ノート §7 が未検証の誤りを含んだ反省を踏まえた）。

---

## 0. 現状（State）

- **VL53 M2 = 完了。** ToF が status=0・±17mm の有効距離を返す → ESKF `updateToF` が POS_Z に融合
  （`eskf.cpp` MASK_TOF=POS_Z|VEL_Z|BA_Z）→ ALTITUDE_HOLD の高度源が生きた。出荷 config `USE_TOF=true`
  （`config.hpp:119`、`init.cpp:356` で `eskf_config.sensor_enabled[SENSOR_TOF]=USE_TOF`）。
- **次のゴール = 空中安定ホバーのレビュー動画。** arm → 離陸 → ALTITUDE_HOLD 係留 → ホバー → 着陸を
  決定論的 `*.scn` で注入し `sf sils scenario --video` で MP4 を出す。

---

## 1. 結論先取り：ホバーするか？ = する（閉ループ ALT_HOLD で）

数値解析（Plant ↔ ファーム、同一 Ct=1.0e-8、相互検証済み）:

| 項目 | 値 | 出典 |
|------|----|----|
| 重量 mass·g | 0.037×9.81 = **0.3630 N** | `config.hpp:527` MASS=0.037 |
| Plant の hoverDuty（重力釣合 duty/モータ） | **0.6517 (65.2%)** | `plant.cpp:126` |
| ファーム hover feedforward = mass·9.81·1.12 | **0.4065 N** (per-motor 0.1016 N) | `altitude_controller.hpp:43` |
| ファーム hover duty（Vbat=3.7, thrustToDuty） | **0.6976 (69.8%)** | `motor_model.hpp:127` |
| 開ループ純推力 NET | 0.4065 − 0.3630 = **+0.0435 N → +1.18 m/s²（≈+0.12g）上昇** | — |

**根本**: `HOVER_THRUST_CORRECTION=1.12`（`config.hpp:533`）は**実機の電池サグ**（負荷で電圧降下→duty 増し）
向けに較正された factor。emu の INA3221 は**一定 3.7V**（`virtual_board.cpp:214`→`plant.hpp:143`
`batteryVoltage()=cfg_.v_batt=3.7`、サグ無し）ゆえ 12% のサグ余裕が**純粋な過剰推力 → 上昇**になる。
M2 ノート §7 の「too high → climb」予測は**正しい**（+0.12g と定量化）。

### ★ M2 ノート §7 の誤りを2点訂正

1. **ALT_HOLD のスロットルは「絶対高度」でなく「上昇/下降レート」**。
   `altitude_controller.hpp:178` `altitude_setpoint += climb_rate_cmd*dt`。中央（raw 2048, ±0.1 deadzone）= 保持。
   目標高度へ上げるには「スティックを一定時間**上げ続ける**」（位置に動かして戻すのでは到達しない）。
2. **1.12 の climb 懸念は「emu Vbat 一定」結合ではない**。既定の `hoverThrustConstant` は vbat を**完全に無視**
   （`altitude_controller.hpp:43` `(void)vbat`）。固定フィードフォワードバイアスで、内側 VEL PID が積分で吸収する。
   （vbat 依存は未使用の `hoverThrustVoltageCorrected` スタブにのみ存在。）

### 対策 = HOVER_THRUST_CORRECTION は変更しない（Code/Param Identity 厳守）

閉ループ ALT_HOLD が自己補正する。内側 VEL PID が必要とする補正は **−0.0435 N**、対して
`VEL_OUTPUT_MAX=±0.15 N`（`config.hpp:537`）＝**71% 余裕**（減速権限 4.05 m/s² vs 上昇 1.18 m/s²）。
PID は back-calculation アンチワインドアップ（`pid.hpp`）、定常 −0.0435 N はクランプ内で飽和しない。
推力クランプ（0.4065≪MAX_TOTAL_THRUST 0.672N）も duty クランプ（0.698<0.95）も発火しない。

**→ 唯一やるべきは「ALT_HOLD を確実に engage させ captureAltitude させる」こと。** 制御パラメータは触らない。

---

## 2. ALT_HOLD の動作仕様（実コード裏取り）

| 項目 | 仕様 | 出典 |
|------|------|----|
| モード選択 | `ctrl_flags & CTRL_FLAG_ALT_MODE(0x08)` かつ POS 無し → ALTITUDE_HOLD | `control_task.cpp:387` |
| 高度センサ無効時 | ALT_HOLD は **STABILIZE に降格**（ToF/Baro 必須）→ ToF が生きていることが**ハードゲート** | `control_task.cpp:407` |
| 高度キャプチャ | モード遷移時に現在の ESKF 高度（`-position.z`）を setpoint に。`captureAltitude` が [0.10, 3.0]m にクランプし `stick_unlocked_=false` にリセット | `control_task.cpp:432`, `altitude_controller.hpp:135` |
| スティックロック解除 | キャプチャ後、スロットルが中央 deadzone（`\|(raw-2048)/2048\|<STICK_DEADZONE=0.1` ⇒ raw≈1843..2253）に**一度入る**まで `stickToClimbRate` は 0（保持）を返す | `altitude_controller.hpp:223` |
| スロットル=レート | 中央=保持、上=上昇（最大 0.5 m/s）、下=下降（最大 0.3 m/s）。レートは setpoint を積分 | `altitude_controller.hpp:178`, `config.hpp:551` |
| デバウンス | モード切替は `MODE_SWITCH_DEBOUNCE_COUNT=10` 連続サイクル(@400Hz=25ms)。フラグは**連続アサート**必須 | `control_task.cpp:41` |
| 閉ループ実行条件 | `(mode==ALT_HOLD\|\|POS_HOLD) && altitude_captured` のときのみ。さもなくば開ループ `throttle*MAX_TOTAL_THRUST`（=ここが +1.18m/s² 上昇する分岐） | `control_task.cpp:793` |
| デバッグログ | `ALT_HOLD: sp=.. alt=.. vz=.. thrust=.. hover=.. vbat=..`（engage したか・alt が sp 追従するかの確認に使う） | `control_task.cpp:812` |

---

## 3. 実装：*.scn DSL に CTRL_FLAG_ALT_MODE(0x08) を通す（7編集）

`build_control_packet`/`inject_rc` は**既に `flags` 引数を取り p[11]=flags を書く**（`scenario_inject.cpp:31-44`）。
欠けているのは「`*.scn` で alt を指定し、その bit を flags に OR する」配線だけ。

```
編集(a) scenario_inject.hpp:38 — kFlagArm の隣に追加:
    constexpr uint8_t kFlagAltMode = 0x08;   // CTRL_FLAG_ALT_MODE (controller_comm.hpp:38 と一致)

編集(b) scenario.cpp:43 — Event 構造体に追加（arm の隣）:
    uint8_t  alt = 0;   // 1 => CTRL_FLAG_ALT_MODE

編集(c) scenario.cpp:234 — rc パーサに OPTIONAL alt トークン（hold_ms/rate_hz の後）:
    long alt = 0;
    if (iss >> alt) { if (alt!=0 && alt!=1) { err(path,lineno,"rc <alt> must be 0 or 1"); return -1; } }
    e.alt = (uint8_t)alt;
  ※ alt は4番目の OPTIONAL トークン。alt を出す行は hold_ms と rate_hz も必須:
    `rc <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz> <alt>`

編集(d) scenario.cpp:249 — rc_ramp パーサに OPTIONAL alt トークン（必須6個の後 = 7番目）:
    long alt = 0;
    if (iss >> alt) { if (alt!=0 && alt!=1) { err(path,lineno,"rc_ramp <alt> must be 0 or 1"); return -1; } }
    e.alt = (uint8_t)alt;

編集(e) scenario.cpp:299 — Rc 分岐の flags に OR:
    const uint8_t flags = (e.arm ? sils::kFlagArm : 0) | (e.alt ? sils::kFlagAltMode : 0);

編集(f) scenario.cpp:316 — RcRamp 分岐の flags に同じ OR
```

ビルド: `sf sils build`（emu_vehicle が `--target vehicle` 既定）。回帰確認:
`sf sils scenario simulator/sils/scenarios/hover_espnow.scn` が依然 PASS（alt 省略→0）。

---

## 4. ドラフト `simulator/sils/scenarios/hover_alt.scn`

```
# hover_alt.scn — 空中 ALTITUDE_HOLD ホバー（旧ファーム / emu_vehicle）
# スティックは raw 12bit ADC 0..4095, 中央2048。
# rc:      <t> rc      <thr> <roll> <pitch> <yaw> <arm> [hold_ms] [rate_hz] [alt]
# rc_ramp: <t> rc_ramp <field> <from> <to> <step> <rate_hz> <arm> [alt]
# alt=1 で CTRL_FLAG_ALT_MODE(0x08) → ALTITUDE_HOLD。'+' = 直前イベント終了直後。
# 開ループ(STABILIZE): throttle01=clamp((raw-2048)/2048,0,1); thrust=throttle01*0.672N。重量0.363N。
#   raw3300→0.611→0.411N(>重量→離陸上昇)、raw2048→開ループ0N かつ ALT_HOLD 中央/保持 deadzone。
#
#  <t>  ch       <args...>
   0    rc       2048 2048 2048 2048 0   2000  50      # A: disarm 中央 2s
   +    rc       2048 2048 2048 2048 1   1000  50      # B: ARM 立上り 1s（スロットル0）
   +    rc       3300 2048 2048 2048 1   1200  50      # C: 開ループ離陸 ~1.2s → ~0.3-0.5m（STABILIZE, alt=0）
   +    rc       2048 2048 2048 2048 1   20000 50 1    # D: ALT_HOLD(alt=1)+中央スロットル → capture & hold 20s
   +    rc       2048 2048 2048 2048 1   1500  50      # E: ALT_HOLD 解除(alt=0)+スロットル0 → 下降/着陸 1.5s
   +    rc       0    2048 2048 2048 0   1000  50      # F: スロットル0, DISARM
#
# 期待: C で ~0.3-0.5m へ離陸（開ループ）。D 突入で現在高度を capture（[0.10,3.0]m）し保持。raw2048 は
# ±0.1 deadzone 内ゆえ stick_unlocked_=true→climb_cmd=0→保持。定常推力は 0.4065N でなく ~0.363N に
# 落ち着く（VEL PID が +0.0435N の 1.12 バイアスを除去）。
#
# TUNING: raw3300 と 1.2s は初期推測。C が MAX_ALTITUDE=3.0m を超える/MIN 0.10m 未満なら raw(3180..3600)と
#   hold_ms を trajectory.csv の alt 列から数値調整。HOVER_THRUST_CORRECTION は触らない（§1）。
```

---

## 5. 検証レシピ（M2 完了の判定に相当）

```bash
sf sils scenario simulator/sils/scenarios/hover_alt.scn --duration 30000000 --video
# バンドル: simulator/sils/viz/out_scn_hover_alt/{trajectory.csv, events.jsonl, console.log, results.json, scn_hover_alt.mp4}
```

**主判定（数値・画像読込不要）= `trajectory.csv`**（20列, ヘッダ `emu_trajectory.cpp:76`:
`t,px,py,pz,qw,qx,qy,qz,alt[8],roll,pitch,yawrate,yawcmd,alt_est[13],roll_est,pitch_est,m0[16],m1,m2,m3[19]`）:

1. **離陸**: 位相 C で `alt` 列が ~0.20m 超（接地脱出。cf. hover_espnow は alt~0.013m で接地のまま）。
2. **capture+保持**: 位相 D で `alt` が一定値に**落ち着き上昇が止まる**。D 末尾10秒の `|d(alt)/dt| < ~0.02 m/s` = ホバー
   （engage 失敗なら +0.5 m/s 上昇が見える）。
3. **duty 有界**: m0..m3 が hover 域（~0.60-0.70）、0.95+/1.00 に張り付かない（暴走無し=E3回帰）。
4. **姿勢安定**: roll/pitch ~0。

**コンソール判定（`hover_alt.expect`, hover_espnow.expect と同形式）**:
- `log_contains any 'scenario] driver online'`
- `log_contains any 'Motors ARMED'`
- `log_contains any 'ALT_HOLD: sp='` ← **mode が engage した証拠**（STABILIZE 降格してない）
- `log_absent any 'duties[FR=1.00'` ← 飽和無し

**climb vs hover の決定的判別**: 位相 D の `alt` 列が**平坦**であること。D で上昇していたら ALT_HOLD が engage して
いない（開ループ +1.18m/s²）か capture 失敗 → **engagement をデバッグ（gain は触らない）**。
推力が 0.4065N のままで alt が上昇 = engage 失敗。0.363N 付近に落ちる = 成功。

---

## 6. リスクと即時 de-risk（優先順）

1. **【最優先】ToF が valid を返さないと ALT_HOLD は STABILIZE に降格**（`control_task.cpp:407`、has_altitude ゲート）。
   → 着手前に `simulator/sils/build/vl53_probe 500 6` で ToF が status0 を返すこと、emu の console に ToF init/ESKF
   POS_Z 更新が出ることを確認。M2 完了済みなので通る見込みだが**最初に確認**。
2. **capture がクランプ外を掴む**: [0.10, 3.0]m にクランプ。C が <0.10m なら setpoint が 0.10m に張り付く（沈む）、
   >3.0m なら 3.0m。→ C/D 境界の `alt` を trajectory.csv で読み raw3300/1.2s を ~0.3-0.8m に数値調整。
3. **stick-unlock 不成立**: capture が `stick_unlocked_=false` にリセット。D のスロットルを**正確に raw 2048**に保つ
   （>2253 だと残留上昇レートが命令される）。
4. **デバウンス**: ALT_MODE は10連続サイクル必要。D の rc hold(20000ms@50Hz)は毎フレーム alt=1 ゆえ自明に満たす。
   単発フレームの alt では engage しない。
5. **パーサのトークン順**: rc で alt を出すには hold_ms と rate_hz も必須（alt は4番目の optional）。
   `rc ... <arm> 1` は `1` を hold_ms と誤解釈。常に完全形 `... <arm> <hold_ms> <rate_hz> <alt>` で書く（ドラフト準拠）。
6. **開ループ離陸が伸びる**: +1.18m/s² が 1.2s で ~0.85m まで行く可能性。emu は決定論的ゆえ trajectory.csv で
   raw/duration を厳密に詰める。`alt` 列を唯一の真実とする。
7. **--video レンダ失敗**（viz venv + 非空 trajectory.csv 必要、PASS 時のみ）。→ まず `--video` 無しで .expect が
   PASS し trajectory.csv が埋まることを確認、その後 `--video`。MP4 は提示用、判定は trajectory/.expect。

---

## 7. 関連

- **VL53 M2**: `simulator/sils/docs/vl53_m2_resume.md`（§0 に M2 完了、§7 は本ノートで2点訂正済み）。
- ツール: オフライン gen4 probe = `simulator/sils/build/vl53_probe <mm> <frames>`（CMake `SILS_BUILD_VL53_PROBE`）。
  軌跡 = `emu_trajectory.cpp`（`SILS_EMU_TRAJ`）、`sf sils scenario --video` がバンドルに設定。
- メモリ: `project_stampfly_emulator.md`（全体経緯）、`feedback_control_simulation.md`（制御変更は数値裏付け必須）。
- **CLAUDE.md 厳守**: 制御パラメータ（HOVER_THRUST_CORRECTION 等）は**変更しない**。§1 の数値解析が「閉ループで
  吸収・パラメータ変更不要」を裏付けている。
- 画像（MP4 フレーム）を見る必要があればサブエージェント内で完結（CLAUDE.md 画像ルール）。判定は数値で足りる。
```
