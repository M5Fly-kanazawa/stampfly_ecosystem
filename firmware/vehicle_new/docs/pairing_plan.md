# vehicle_new ⇄ コントローラ ペアリング — 調査結果と実装計画

最終更新: 2026-06-08（調査のみ。実装は未着手）

> **結論（調査）: vehicle_new のペアリングは「部品のみ存在・未配線（実質未実装）」。**
> 一方で **コントローラ側・プロトコル仕様（SSOT）・旧 vehicle には実装/定義が揃っており**、
> vehicle_new 側に「迎え撃つ」ロジックを足すのが本計画。

---

## 1. なぜ要るか（背景）

landing page の売りは「**1教室で最大30機が同時に飛ぶ**」。同一空間で複数機・複数送信機が
ESP-NOW を使うと、**混信**（他人の送信機のパケットで自分の機体が動く）が起きる。これを防ぐのが
**ペアリング＝送信機と機体を1対1に束ねる**仕組み。現状の vehicle_new は誰のパケットでも受けて
動くため、30機運用に耐えない。

---

## 2. 現状の通信（vehicle_new）

| 項目 | 現状 | 出典 |
|------|------|------|
| 受信方式 | **ブロードキャスト無差別受信**（FF:FF:FF:FF:FF:FF を peer 登録） | `components/sf_comm/comm.cpp`（peer を 0xFF で memset） |
| 送信元フィルタ | **なし**（src MAC は未使用） | comm.cpp `(void)info; // src MAC unused (Phase 2a)` |
| チャンネル | 固定 ch1 | comm.cpp `kWifiChannel = 1` |
| ペア状態 | 無し（FlightState に PAIRING 無し） | `sf_state/include/flight_state.hpp` |
| 相手 MAC の永続化 | 無し | — |

### ★ 重要な発見：SSOT パケットは既に宛先 MAC を持っている

`protocol/spec/messages.yaml` の **ControlPacket(14B)** は **offset 0-2 = `drone_mac`（宛先
ドローン MAC の下位3バイト）**。コントローラはここに「送りたい機体の MAC」を入れて送っている
（`firmware/controller/.../espnow_tdma.c` が `drone_mac[3:5]` を先頭3Bに格納）。
**vehicle_new はこの3バイトを読まずに捨てている**。

```
ControlPacket(14B): [drone_mac(0-2)] [thr(3-4)][roll(5-6)][pitch(7-8)][yaw(9-10)] [flags(11)] [reserved(12)] [checksum(13)]
                     ↑ ここで宛先を判別できるのに、今は無視している
```

→ **「受信時に bytes0-2 が自分の MAC 下位3Bと一致するか」を見るだけで、混信の核を断てる**
（フィールドは既に在るので、追加は1チェックぶん）。

---

## 3. 既に在る素材（流用元）

| 素材 | 場所 | 内容 |
|------|------|------|
| **PairingPacket(11B) 仕様** | `protocol/spec/messages.yaml:428` | `channel`(0) + `drone_mac[6]`(1-6) + `signature[4]`(7-10, 値 `0xAA 0x55 0x16 0x88`) |
| **コントローラのペアリング** | `firmware/controller/components/espnow_tdma/espnow_tdma.c:496-623` | `peering_process()`＝ch1-13 をスキャンして signature 待ち→ `peer_info_save/load()`＝ドローン MAC+ch を SPIFFS に保存→保存 MAC へ送信 |
| **旧 vehicle のペア状態管理** | `firmware/vehicle/components/sf_svc_comm/include/controller_comm.hpp`, `sf_svc_state/stampfly_state.cpp` | `PairingState{NOT_PAIRED,PAIRING,PAIRED}` + `enter/exitPairingMode()` + NVS 保存 + CLI `pair start/stop`・`unpair` |
| **UI 部品（呼ぶだけ）** | `sf_hal_led/led.cpp` `showPairing()`（青速点滅）、`sf_hal_buzzer/buzzer.cpp` `pairingTone()`、`sf_hal_button`（長押し3s検出済） | いずれも**定義済みだが未呼び出し** |
| **ボタン長押し** | `tasks/state_task.cpp:306-315` | 長押しは「pairing/system reset 用に予約・ここでは無視」と明記（＝配線先が無い） |

---

## 4. 設計方針

1. **アドレッシング＝ドローン MAC 下位3バイト**（ControlPacket 0-2）。SSOT を尊重し、新パケットを
   増やさない。
2. **最小の混信対策＝「自分宛フィルタ」**（受信時に 0-2 が自 MAC[3:5] と不一致なら破棄）。
   ブロードキャスト宛（FF FF FF）は後方互換で受理（SIL/ベンチ・未ペア時のため）。
3. **ペアリング UX ＝ コントローラのスキャンに応答**：機体が PAIRING モードで自分の MAC と
   channel を `PairingPacket` で広告 → コントローラが発見して bind（コントローラ側は実装済み）。
4. **R5（Pub-Sub 疎結合）/ StateManager 単一所有**を守る：PAIRING も状態遷移なら StateManager が
   所有。comm は「事実（受信パケット・ペア状態）」を publish、判断は state。

---

## 5. 実装フェーズ（提案）

### P1: 自分宛フィルタ（即・低コスト・最高価値）
- comm 受信で `ControlPacket.drone_mac`(0-2) を取り出し、**自 MAC 下位3B**（`esp_wifi_get_mac` /
  efuse、SIL は固定値）と照合。不一致は破棄。`FF FF FF`（broadcast 宛）は受理。
- これだけで「他人の送信機で自分が動く」が止まる（コントローラは既にペア相手 MAC を入れて送る）。
- **SIL 検証**: scenario で「誤 drone_mac の ControlPacket」を注入→ motor 不動、「正 MAC / FF」→
  通常飛行。scenario DSL に MAC 付き rc 注入を足す（既存 `inject_rc` を MAC 引数で拡張）。
- 影響: comm.cpp の受信ハンドラに数行。R5・SSOT 遵守。**まず P1 だけでも価値が大きい。**

### P2: ペアリングモード（送信機からの発見に応答）
- ボタン長押し3s → `pilot_request`（または新トピック）で PAIRING 要求 → StateManager が PAIRING へ。
- PAIRING 中: `PairingPacket`（自 channel + 自 MAC + signature）を broadcast で周期送信。
  `LED.showPairing()`（青速点滅）・`Buzzer.pairingTone()` を notify 経由で発火（R5）。
- コントローラが scan で発見・bind（`peering_process` 実装済み）。一定時間で通常へ復帰。

### P3: 状態機械統合＋永続化
- 状態: PAIRING を FlightState に追加（or 地上サブ状態）。requirements §2 / architecture / detailed_design §3
  にペアリング遷移を追記（**現状これらに PAIRING は無い＝設計文書の追記が必要**）。
- 永続化: ペア済みコントローラ MAC・割当 channel を NVS（`sf_params` とは別 namespace）に保存し
  起動時 load。未ペアなら起動時 PAIRING へ。
- CLI: `pair` / `unpair`（旧 vehicle の `pair start/stop`・`unpair` を移植参考）。

### P4: per-drone チャンネル割当（30機スケール）
- ペア時に機体ごとに channel を割当（PairingPacket は channel を持つ）。同時飛行時の airtime 分散。
- 運用ポリシー（手動割当 / 自動空きch探索）は教材設計と合わせて決める。

---

## 6. SIL 検証方針

- **P1**: scenario で `drone_mac` 付き ControlPacket を注入できるよう `scenario_inject` を拡張。
  「誤 MAC → 無視（ARM もしない）」「正 MAC / broadcast → 飛行」をゲート化（log/metric）。
- **P2-P3**: emu で PAIRING 遷移・PairingPacket 送出・NVS 保存/復元を発火確認。エミュレータの
  ESP-NOW shim に「機体が送出したパケット」を観測する経路が要る（送信側 capture の追加）。
- 既存の決定論・byte-identical 原則を維持（未ペア既定はブロードキャスト受理で従来と一致）。

---

## 7. 着手順の推奨

1. **P1（自分宛フィルタ）を最初に**。低コスト・即効・SSOT 準拠で、混信対策の本質。実機 Phase 2/3
   （ブリングアップ・初飛行）の前に入れておくと、複数機環境でも安全に飛ばせる。
2. P2-P3 はペアリング UX。実機で複数機を運用する段になったら。
3. P4 は 30機ワークショップ運用の直前。

---

## 8. 設計文書への反映（要対応）

- `requirements.md §2`・`architecture.md`・`detailed_design.md §3` に **PAIRING 状態と遷移**が無い。
  実装前に「状態モデルにペアリングをどう位置づけるか」をユーザーと確認し、設計文書を先に更新する
  （CLAUDE.md: 設計矛盾は実装前に報告・議論）。
- `detailed_design.md §7.6` の `espnow_pair/`（サンプルコード予約・実体なし）と本機能の関係を整理。
  教育例（`coding_and_education.md` の `09_espnow_pair`）と本体機能は別物として扱う。
