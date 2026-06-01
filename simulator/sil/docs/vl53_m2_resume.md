# VL53L3CX ToF エミュレーション — Milestone 2 再開ノート

> 作業再開用の自己完結メモ。新しいセッション（空コンテキスト）でも、このファイルだけで M2 を続行できることを目標にする。
> Self-contained resume note: a fresh session should be able to continue M2 from this file alone.

最終更新: 2026-06-02 / 直近コミット: `d858191`（M1 完了, M2 WIP）。**本更新で M2 コア達成**（下記 §0）。

---

## 0. M2 コア = 達成・検証済み（2026-06-02）

**結論: emu_vehicle の ToFTask が `bottom≈target status=0`（VALID）を返すようになった。** 距離掃引（実エミュレータ）:

| target | bottom | status |
|--------|--------|--------|
| 100mm | 96mm | 0 |
| 300mm | 289mm | 0 |
| 500mm | 481mm | 0 |
| 700mm | 674mm | 0 |
| 1000mm | 1059mm | 0 |
| 1300mm | 1251mm | 0 |

誤差は**対称ピークの±96mm 量子化**（1bin=192.5mm）の範囲内。status は frame 0,1=6(NO_WRAP_CHECK)→
frame 2=4(一過性)→**frame 3 以降ずっと 0**（14フレームで安定確認）。

**resume ノート §2/§4/§5 の前提は誤りだった。** 実際は「ピーク未検出（NumberOfObjectsFound=0）」ではなく、
**ピークは検出されていた（=1）が 3 つの別問題でレンジが無効だった**。オフライン probe（§3 Option B、実装済み）が
即座に暴いた。真の原因と修正（実コード裏取り済み）:

### 修正1: zero_distance_phase = 22528（zdp≠0）★最重要
- §5 は「phasecal_result__reference_phase=0 → zdp=0」と仮定していたが**誤り**。
  `VL53LX_hist_calc_zero_distance_phase`（vl53lx_core_support.c:165）は
  `zdp = (period + ref_phase + 2048·vcsel_start − 2048·cal_vcsel_start) % period`。
  我々の ref_phase=0, vcsel_start=0 でも、HISTOGRAM_LONG の vcsel period(9)・cal_config__vcsel_start により
  **zdp = 22528 = 11 bin**（決定論的）。
- driver は `range = (peak_phase − zdp) × 0.094`。ピークを phase=R×10.639（zdp=0前提）に置くと
  **負レンジ → status 4(OUT_OF_BOUNDS)**。f_017（gen3.c:786）の valid window は
  `[zdp−(valid_phase_low<<8), zdp+(valid_phase_high<<8)] = [20480, 57344]`。
- **修正**: ピーク phase = `ZERO_DIST_PHASE(22528) + R×10.639` に置く。

### 修正2: ambient-bin strip = 4
- HISTOGRAM_LONG の bin_seq=[7,0,1,2,3,4]。0x07 コード→`number_of_ambient_bins=4`。
  gen4 は `VL53LX_hist_remove_ambient_bins`（core_support.c:244）で**先頭4bin を strip し左シフト**してから
  ピーク検出。よって raw bin に置いたピークは output で 4 bin 左にずれる。
- 実証: raw bin 5 のピーク → output bin 1 → phase 3072 → range (3072−22528)×0.094 = **−1828mm**（probe と完全一致）。
- **修正**: RAW buffer には `output_bin + AMBIENT_BIN_STRIP(4)` に置く。

### 修正3: result__stream_count を進める ★status 0 昇格の鍵
- 旧モデルは `g_vl.ptr == 0x0086` 完全一致で stream++。だが driver の measurement 再開
  （`init_and_start_range`）は **MODE_START(0x0087) を末尾に含むブロック書き込み**（StartMeasurement=0x0001..0x0088,
  ClearInterruptAndStartMeasurement=0x0044..0x0088）。先頭ポインタは下位アドレスなので**完全一致では捕捉できず
  stream が 0 固定**だった。
- stream=0 固定 → driver は毎フレームを「最初のフレーム」と誤認（multi_bins_rec を毎回リセット, api_core.c:2642）→
  rolling 再結合・`VL53LX_hist_phase_consistency_check`（core.c:1729, prev フレームが ranging 状態でないと早期 return）が
  破綻 → status が 6,6,4,7 と振動し**永遠に 0 にならない**。
- **修正**: 書き込みが MODE_START(0x0087) を含めば stream++（init=0xFF で frame 0 が 0）。これで stream が
  0,1,2,... と進み、phase-consistency が成立 → frame 3 以降 status 0。

実装場所: **`simulator/sil/devices/vl53_device.cpp`**（virtual_board.cpp から分離。probe と共有）の
`sil_vl53::xfer` / `fill_histogram` / `target_mm` と定数 `ZERO_DIST_PHASE` / `AMBIENT_BIN_STRIP` / `REG_MODE_START`。
virtual_board.cpp の `sil_board_i2c_xfer` は 0x29/0x30 を `sil_vl53::set_distance_mm`+`xfer` へ委譲。

### skew 較正 = 完了（サブbin精度 ±96mm→±17mm）
- §5 の「肩 skew」ではなく**2-bin split**を採用: 目標を `b0` と `b0+1` に frac 比で分割し、gen4 の復号重心を
  距離に連続追従させる（`fill_histogram` の `main_lo`/`main_hi`）。probe 細粒度掃引で検証:
  50..1400mm 全域で誤差 **±17mm 以内**・全 status=0・定常安定（量子化の 192mm ステップを解消）。
- **外側 shoulder は付けない**: 当初 b0-1/b0+2 に raised shoulder を置いたが、frac≈0/1（分割が退化し片方の
  main bin が floor まで落ちる）で peak と shoulder の間に1bin gap → gen4 が **wrap-target(status7)** 誤判定
  （例: 接地時の Plant ToF≈0..30mm が status254 化）。周囲の ambient floor 自体が pulse edge ゆえ shoulder 不要。
  除去で全 frac 域 status0。
- 残差 ±17mm は gen4 の f_022（半幅2窓）重心フィルタの系統的サザナミ。実センサのノイズと同等の実用域ゆえ
  これ以上の較正は過剰（必要なら frac 依存の補正項で詰められるが diminishing returns）。
- **接地時 ToF≈0 は status254（無効）が正**: ラッパー MIN_VALID_DISTANCE_MM=30mm 未満を弾く。skew 前の対称
  ピークは量子化で 96mm と過大報告していた（離陸し高度>30mmで valid）。

### 残課題
- **MAX_MM=1400 の制約**: zdp=11bin + strip 4 で usable output bin は 11..21（~1540mm まで）。ホバー(<1m)は十分だが、
  高高度は phase wrap 処理が要る（M2 範囲外）。

---

## 3. オフライン gen4 probe（実装済み・検証ツール）

`simulator/sil/smoke/vl53_probe.cpp`（CMake `SIL_BUILD_VL53_PROBE`, OFF 既定）。無改変 ST ドライバを
チップモデル（`devices/vl53_device.cpp`）に直結（FreeRTOS/MuJoCo/scheduler 無し、自前 platform port）。
毎フレーム `MultiRangingData` 全体＋ decoded `histogram_bin_data`（number_of_ambient_bins, total_periods_elapsed,
vcsel_width, bin_seq, zero_distance_phase, bins）＋ post-process config を printf。ビルド・実行:
```bash
cmake -S simulator/sil -B simulator/sil/build -DSIL_BUILD_VL53_PROBE=ON
cmake --build simulator/sil/build --target vl53_probe -j
./simulator/sil/build/vl53_probe 700 6   # target_mm frames
```
これが「合成 histogram → gen4 の生の判定」を 1 ビルドで何十距離も見せ、上記3原因を即座に暴いた。

---

## 5. 確定済みの土台（再導出不要・実コードで裏取り済み）

- **距離は peak phase の完全線形**: `range_mm = 0.093994 × (median_phase − zero_distance_phase)`
  （`vl53lx_core_support.c:403 VL53LX_range_maths`、runtime 定数 fast_osc=0xBCCC, gain_factor=1987,
  range_offset=0）。**1 phase 単位 = 0.094mm、1 bin = 2048 phase = 192.5mm**。逆算: `phase = R_mm × 10.639`。
- **対称ピーク**（左右肩が等しい）→ 重心位相 = `b0×2048 + 1024`（bin 中心）。任意位相は肩を skew:
  `phase ≈ b0×2048 + 1024 + 1024×(右肩−左肩)/(peak−ambient)`。skew は上流フィルタ `f_022`（半幅2窓+ambient減算）
  を通るので**近似**→ harness で実測較正（1-2回）。
- **init を通すゲートは 3 つだけ**（他は ACK or ゼロ返しで driver 自己修復）:
  - `0x00E5` FIRMWARE__SYSTEM_STATUS → bit0=1（boot 完了）
  - `0x0031` GPIO__TIO_HV_STATUS → **ACTIVE_LOW**（bit0=0 が ready）。preset が
    `gpio_hv_mux__ctrl=ACTIVE_LOW(0x10)`（api_preset_modes.c:752 で確認）。
  - `0x00DE` RESULT__OSC_CALIBRATE_VAL → **非ゼロ必須**（0 だと `set_inter_measurement_period_ms` が
    −15 DIVISION_BY_ZERO; api_core.c:967）。現在 0x0600 を返している。
- **fast_osc 自己修復**: NVM の osc<0x1000 を返すと driver が 0xBCCC に強制（api_core.c:711）→ 距離スケールが固定。
- **16bit BE レジスタポインタ**（INA3221/BMP280 の 8bit とは違う）。write_buf[0]=MSB, [1]=LSB。
- **histogram 読み出し = 1 回の ReadMulti(0x0088, 83 バイト)**。バイト配置（`VL53LX_get_histogram_bin_data`,
  api_core.c:2526 で照合済み）:
  - off0(0x88) interrupt_status / off1(0x89) **range_status=0x09**(RANGECOMPLETE, abort 回避) /
    off2 report / off3 stream_count / off4-5(0x8C-0x8D) dss_spads u16 BE
  - **bins: off6(0x8E) から 24個 × 3 バイト BE**（bin k = off 6+3k）。`HISTOGRAM_BIN_0_2=0x008E` 確認済み。
  - phasecal_result__reference_phase u16 BE at off78(0x00D6) = 0 → **zdp = 22528（≠0!）** §0 修正1 参照
  - phasecal_result__vcsel_start u8 at off80(0x00D8) = 0
  - **bin23 修復**: driver が `buf[0x00D5(off77)] = (buf[0x00D9(off81)]<<2) + buf[0x00DA(off82)]` で
    bin23 の低バイトを上書き（api_core.c:2606-2622, **bin ループ前**）。整合させること。
- **アドレス**: bottom は power-on 0x29 → reg `0x0001` に 0x30 を書いて再アドレス → 以後 0x30。
  front は 0x31（未モデル → graceful 無効化、放置可）。`sil_board_i2c_xfer` は 0x29 と 0x30 両方を `vl53_xfer` へ。
- **RangeStatus 0 は 2 フレーム目から**（1 フレーム目は NO_WRAP_CHECK → status 6）。距離を滑らかに動かせば
  phase-consistency で 0 に昇格（|Δphase| < 2048 ≈ 192mm/frame）。
- **位相窓**: valid_phase_high で単峰の使用域 ~0..3273mm（hover では問題なし）。`b0` は [1,16] にクランプ済み。

---

## 6. 検証手順（M2 完了の判定）

```bash
source setup_env.sh   # or: PYTHONPATH=lib
cmake --build simulator/sil/build --target emu_vehicle -j

# 距離掃引（現状は全部 status=255。M2 完了で ~target が status=0 で出るべき）
for mm in 100 289 500 1000 1500; do
  out=$(SIL_VL53_TEST_MM=$mm ./simulator/sil/build/emu_vehicle \
        simulator/sil/models/stampfly.xml 22000000 2>&1 | grep "bottom=" | tail -1)
  echo "target=${mm} -> $out"
done
# 期待（M2完了時）: bottom≈target(±量子化), status=0
```

**M2 Definition of Done**:
1. `SIL_VL53_TEST_MM=R` で ToFTask が `bottom≈R status=0`（2 フレーム目以降）。誤差は対称ピークで ±96mm、
   肩 skew 較正後は数 mm。
2. `SIL_VL53_TEST_MM` を外して `vl53_target_mm()` が `g_plant->tof()×1000` を返す本番経路で、
   Plant 距離追従を確認（craft を非接地高度に置く手段が要る — テスト override か、後述の ALT_HOLD）。
3. 回帰なし: 全 SIL ビルド緑 / emu_vehicle・emu_vehicle_new 決定論 / hover_espnow(14)・console_cli(8) PASS。
4. 敵対的レビュー（実コード裏取り。サブエージェントは [[feedback_checklist_discipline]] の通り過信しない）。

---

## 7. M2 後の道（空中ホバー動画）

> **→ 検証済みの完全な手順は `simulator/sil/docs/hover_resume.md` を正とする**（実コード裏取り済み）。
> 以下の §7 原案は**2点誤りがあり hover_resume.md で訂正済み**: ①ALT_HOLD のスロットルは絶対高度でなく
> 上昇/下降**レート**（目標へは上げ続ける）。②1.12 の climb は「Vbat 一定」結合でなく固定 FF バイアス
> （既定 hover 推定は vbat 無視）→ 閉ループが71%余裕で吸収、**HOVER_THRUST_CORRECTION は変更不要**。
> また `hover_espnow.scn` はサブホバー接地維持なので**新規 `hover_alt.scn`**（alt=1+離陸→中央保持）が要る。

ToF が距離を返せたら:
1. ファーム高度推定（ESKF SENSOR_TOF, 出荷 config で既に USE_TOF=true）が有意になる。
2. **ALTITUDE_HOLD をシナリオで起動**: `CTRL_FLAG_ALT_MODE=0x08`（ControlPacket byte[11] bit3）を
   ESP-NOW 経路で送る。**`scenario.cpp` の rc/rc_ramp に mode/flags フィールドを追加**し
   `scenario_inject` に `kFlagAltMode=0x08` を足す（`control_task.cpp:383` が getControlFlags() で mode 選択）。
3. throttle を一旦 deadzone(中央 ~2048)に戻して stick-unlock → 上げて目標高度へ → 中央で保持
   （`altitude_controller.hpp` captureAltitude/update）。
4. `sf sil scenario hover_espnow.scn --video` で**空中安定ホバー動画**。
5. 注意: `HOVER_THRUST_CORRECTION=1.12` は実機の電池サグ前提。emu は INA3221 で Vbat 一定ゆえ
   緩い climb/sink の可能性 → 数値裏付けの上で要調整（CLAUDE.md 制御パラメータ規約）。

**config 判断 = 解決**: M2 が通ったので出荷 config（USE_TOF=true）を維持する（baro flip 不要、Code Identity 維持）。

---

## 8. 関連
- メモリ: `project_stampfly_emulator.md`（全体経緯）, `project_sil_reset.md`（方針）, `feedback_checklist_discipline.md`。
- トレース成果物（生）: `~/.claude/.../tasks/w7h0qzund.output`（VL53 init/histogram/gen4 の詳細トレース。
  ただし誤りを含む: 「非現実的」「ACTIVE_HIGH」「osc-cal 0でOK」「read 0x0080/91B」は**実コードで訂正済み**。本ノートが正）。
- 距離式の数値確認・recipe は §5 を正とする。
