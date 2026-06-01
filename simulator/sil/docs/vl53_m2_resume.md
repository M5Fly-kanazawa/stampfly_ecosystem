# VL53L3CX ToF エミュレーション — Milestone 2 再開ノート

> 作業再開用の自己完結メモ。新しいセッション（空コンテキスト）でも、このファイルだけで M2 を続行できることを目標にする。
> Self-contained resume note: a fresh session should be able to continue M2 from this file alone.

最終更新: 2026-06-02 / 直近コミット: `9bd685d`（M1 完了, M2 WIP）

---

## 1. 現状（State）

**Milestone 1 = 完了・検証済み。** 無改変の ST BareDriver 1.2.14（HISTOGRAM モード）がエミュレータ `emu_vehicle` で
boot → DataInit → 0x29→0x30 再アドレス → 測距ループ完走する。実機ログ:
```
VL53L3CX (BOTTOM) initialized at address 0x30
Bottom ToF sensor initialized successfully
Bottom ToF initialized and ranging
ToFTask: ToFTask alive: bottom=0mm status=255
```

**Milestone 2 = WIP（未達）。** histogram 合成は実装済みだが、gen4 が **`NumberOfObjectsFound=0`（ピーク未検出）**
→ ラッパー `getDistance` が status=255 / distance=0 を返す（`vl53l3cx_wrapper.cpp:309-311`）。
距離が出ない。**これを解くのが M2 のゴール。**

実装場所: `simulator/sil/devices/virtual_board.cpp` の `vl53_*`（`vl53_xfer` / `vl53_fill_histogram` /
`vl53_target_mm` / `vl53_pack_bin` と定数）+ `sil_board_i2c_xfer` の `0x29||0x30` 分岐。

---

## 2. M2 の唯一の壁（The blocker）

gen4 パイプライン（`VL53LX_ipp_hist_process_data` → `VL53LX_f_025`, gen4.c）が、合成した histogram から
**ピークを「物体」として検出していない** → `NumberOfObjectsFound=0`。

確認済み: status=255 は「物体0個」を意味する（254=物体ありだが全無効, 6=no-wrap-check）。つまり
**まだ検出器の手前で落ちている**（RangeStatus の段階まで到達していない）。

---

## 3. 再開の初手 = gen4 の可視化（最優先）

盲目反復（リビルド＋20秒起動で status だけ見る）は遅すぎる。**まず gen4 の内部値を見えるようにする。**
2 案。**Option B（オフライン harness）を推奨**（init 状態を実 driver で確立でき、Plant 不要、反復が速い）。

### Option A: ST トレースを有効化
- `vl53lx_platform_log.h` は `#ifdef VL53LX_LOG_ENABLE` でゲート。有効時 `VL53LX_trace_print_module_function`,
  `_trace_level`(extern), `VL53LX_get/set_trace_functions`, `VL53LX_clock` が必要（**src/ に実装なし** → 自前で用意）。
- 手順: emu_vehicle ターゲットに `target_compile_definitions(emu_vehicle PRIVATE VL53LX_LOG_ENABLE)` を追加し、
  小さな backend（printf 版 `VL53LX_trace_print_module_function` + `uint32_t _trace_level;` + get/set スタブ）を
  `devices/` に新規作成してリンク。`VL53LX_clock` は `devices/vl53_platform_glue.c` 由来の tick があれば流用、無ければ 0 返し。
- trace レベル/モジュールを CORE+DEBUG（`VL53LX_TRACE_MODULE_CORE | _ALL`, `_trace_level=VL53LX_TRACE_LEVEL_ALL`）に。
- **リスク**: 巨大 ST コードで一斉に展開 → format-string 警告や未定義シンボルが出る可能性。OFF 既定の CMake option で隔離する。

### Option B: オフライン gen4 harness（推奨）
- 新規 `simulator/sil/smoke/vl53_probe.cpp`: ST driver 一式 + `devices/virtual_board.cpp`（の vl53 model）+
  `devices/vl53_platform_glue.c` をリンクし、`VL53LX_DEV` を作って
  `WaitDeviceBooted → DataInit → SetDistanceMode(LONG) → StartMeasurement → (loop) GetMultiRangingData` を直接呼ぶ。
- **各フレームで `VL53LX_MultiRangingData_t` 全体を printf**: `NumberOfObjectsFound`, 各 target の
  `RangeStatus`, `RangeMilliMeter`, `SignalRateRtnMegaCps`, `AmbientRateRtnMegaCps`, `SigmaMilliMeter`。
- 距離は `SIL_VL53_TEST_MM` で与える（`vl53_target_mm()` が拾う。g_plant 不要）。
- I2C 配線: ST driver の `VL53LX_ReadMulti/WriteMulti`（vl53lx_platform.c）が ESP-IDF `i2c_master_*` を使うか確認し、
  使うなら既存 shim 経由で `sil_board_i2c_xfer`→`vl53_xfer` に届く。届かない場合は platform を直接 `sil_board_i2c_xfer` に繋ぐ薄いブリッジを harness 側で用意。
- これで「合成 histogram → gen4 の生の判定」が**1 ビルドで何十距離も**見える。CMake に `SIL_BUILD_VL53_PROBE` option（OFF 既定）で追加。

---

## 4. ピーク未検出の根本原因 候補（可視化後にこの順で潰す）

1. **信号ゲート未通過**: pulse net events `VL53LX_p_010`（pulse 幅でピーク−ambient の和）が
   `signal_total_events_limit=100` 未満 → active_results=0。現状 peak=2000 で十分なはずだが、bin の
   読み取り or ambient 減算が想定とズレている可能性。harness で `SignalRateRtnMegaCps` を見る。
2. **gen4 設定依存（最有力）**: `bin_seq` / `vcsel period`(`VL53LX_p_005`) / `number_of_ambient_bins` /
   `total_periods_elapsed` 等が init の RAM 状態由来。NVM ゼロ返しや config-write を「ACK だけで保存しない」ため、
   gen4 が期待する内部状態と不整合の可能性。特に `number_of_ambient_bins`（bin_seq の 0x07 ニブルで増える）が
   非0だと **読んだ 24 bin と論理 bin の対応がズレる**（`VL53LX_hist_remove_ambient_bins` がバッファをシフト）。
   → harness で bin_seq / number_of_ambient_bins を確認。
3. **rolling/stream の不整合**: gen4 は `pdev->ll_state.rd_stream_count`（driver 内部カウンタ）で
   even/odd bin 列を選ぶ（私が出す 0x008B stream_count とは別物）。`stream_count==0` の最初のフレームは
   `multi_bins_rec` をリセット（api_core.c:2642）。検出には複数フレームの蓄積が要る可能性。
4. **histogram のスケール/符号**: bin 値 24bit BE は正しいと確認済みだが、ambient 推定（低 bin の平均）が
   floor=40 を「全部信号」と誤認 or 閾値計算で潰れている可能性。ambient bin を明確に低く、peak を鋭くする。

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
  - phasecal_result__reference_phase u16 BE at off78(0x00D6) = 0 → zdp 0
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

**保留中のユーザー判断**: ToF 完遂で出荷 config（USE_TOF=true）維持 vs baro config flip
（USE_BAROMETER=true/USE_TOF=false, Code Identity 例外）。M2 が通れば前者で行ける見込み。

---

## 8. 関連
- メモリ: `project_stampfly_emulator.md`（全体経緯）, `project_sil_reset.md`（方針）, `feedback_checklist_discipline.md`。
- トレース成果物（生）: `~/.claude/.../tasks/w7h0qzund.output`（VL53 init/histogram/gen4 の詳細トレース。
  ただし誤りを含む: 「非現実的」「ACTIVE_HIGH」「osc-cal 0でOK」「read 0x0080/91B」は**実コードで訂正済み**。本ノートが正）。
- 距離式の数値確認・recipe は §5 を正とする。
