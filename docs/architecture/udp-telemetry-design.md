# UDP テレメトリ設計記録

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 背景と問題

TCP WebSocket テレメトリ (840B × 100Hz = 83.2 KB/s) が ESP-NOW 制御信号を阻害し、
飛行中に操縦不能になった。ESP32-S3 は WiFi ラジオが1つしかなく、
テレメトリ送信中は ESP-NOW パケットを受信できない。

## 2. 失敗したアプローチと教訓

### 方式A: センサ独立パケット（sendto 265回/秒）

各センサ型を個別パケットで送信。帯域計算では問題なかったが、
**sendto() のブロック時間がパケットサイズに依存しない** ことを見逃した。

```
ベンチマーク実測:
  117B: avg=3075μs
  325B: avg=3031μs
  840B: avg=2854μs
  → サイズに関係なく ~2.5ms/回
```

265回/秒 × 2.5ms = 662ms/秒 = **ラジオ占有率 66%**。
ESP-NOW が使える時間は 34% しかなく、制御パケットがドロップした。

**教訓: sendto() のオーバヘッドは per-call であり per-byte ではない。
小さいパケットを多数送るのは最悪の戦略。**

### 制御ループ内での直接 sendto()

400Hz IMU ループ内で sendto() を呼んだところ、IMU レートが 200Hz に低下。
sendto() の 2.5ms ブロックが 2.5ms のIMUセマフォ周期を超過した。

**教訓: ブロッキング I/O をリアルタイムループ内で呼ばない。**
→ Producer-Consumer パターン（FreeRTOS キュー）で送信を分離。

### data_ready フラグ競合

imu_task と telemetry_task が同じ `volatile bool` フラグを消費。
先に読んだタスクがフラグを false にし、後のタスクはデータを見逃す。
ToF 0Hz、Flow 56Hz の原因だった。

**教訓: volatile フラグは複数のコンシューマ間で共有できない。**
→ タイムスタンプ変化検出（非破壊的）に変更。

### ToF タイムスタンプ共有

bottom と front の ToF が同じ `g_tof_last_timestamp_us` を更新。
電池駆動時に front ToF が初期化成功し、1ループで2回更新 → 2倍のレートに見えた。

**教訓: 1データソース = 1変数。共有しない。**
→ `g_tof_bottom_last_timestamp_us` / `g_tof_front_last_timestamp_us` に分離。

### スタックオーバーフロー（複数回）

WiFi CLI タスクのスタック 4096B に対して SendItem (342B) + ws_batch_pkt (840B) をスタックに確保。

**教訓: 組込みタスクのスタックは小さい。大きな構造体は module-static にする。**

## 3. 最終解決: 統合パケット方式

8 IMU サイクル分のデータを 1 パケット (~1000B) にまとめて 50Hz で送信。

```
パケット構造:
  [Header 4B]
  [IMU+ESKF × 8]   640B (固定)
  [PosVel × 8]     224B (固定)
  [RateRef × 8]     48B (固定)
  [entry_count 1B]
  [SensorEntries]   可変 (Control, Flow, ToF, Baro, Mag)
  [Checksum 1B]
  合計: ~920-1000B
```

### 結果

| 指標 | TCP(旧) | UDP方式A | UDP統合(最終) |
|------|---------|---------|-------------|
| sendto() | 100/秒 | 272/秒 | **50/秒** |
| ラジオ占有 | >70% | 68% | **12.5%** |
| パケットロス | 19% | 5% | **0%** |
| ESP-NOW操縦 | 不可 | 困難 | **正常** |

### 核心的知見

**ESP32 の lwIP sendto() は1回あたり ~2.5ms の固定オーバヘッドを持つ。**
ペイロードサイズ（117B〜950B）による差はない。
ESP-NOW との共存品質を決めるのはデータ量ではなく **sendto() の呼び出し回数**。

## 4. テキストログとステータス v4（統合パケットの外に足したもの）

統合パケット（0x50）は数値の 400Hz ストリームで、最悪でも 1377 バイト（上限 1400 バイトまで 23 バイト）しか余裕が無い（2026-11-23 に 50 Hz の `ctrl_ref` を 30 → 51 バイトにして目標 5 欄と推力励振の状態を載せたため、1334 バイトから +21 バイト）。フローの標本（100 Hz、1 件 11 バイト）は遅れが無ければ 1 パケットに 2 件だが、遅れた TelemetryTask はリング（8 件）に最大 8 件を見つける。全件を 1 度に載せると、7 件以上で最後の磁気などが落ちる（実測の表は `test_main.cpp` の `wire_unified_packet_size_by_flow_samples`）。そこで **1 パケットに載せるフローは最大 4 件**（`kMaxFlowEntriesPerPacket`、`data_stream_wire.hpp`）とし、超過分はリングに残して次のパケットへ回す（古い順なので時刻の順は保たれる）。4 件の根拠: 全種類のエントリが載る最悪のパケットが 1377 バイトで上限に収まること（`static_assert` で検査。エントリを大きくして超えるとビルドが止まる）、かつ排出が 4 件／20 ms で到着の 2 件／20 ms（100 Hz）を上回ること。リングが満杯のとき最古の標本を捨てる動き（`TopicRing`）は変わらず、捨てが起きる条件は「TelemetryTask が約 80 ms を超えて遅れる」ままで（制御の記録のリング 32 標本 = 約 80 ms と同じ）、溜まりが一度 8 件に達した後は 1 パケットごとに 2 件ずつ減るので、回復の途中で再び遅れたときの余裕は従来より少ない。フローのリングの捨ては `entry_drops` には数えない。機体の `ESP_LOG` の行（状態遷移・着陸判定・フェイルセーフ・拒否の理由）と診断値は、統合パケットを大きくせずに、同じソケット・同じ TelemetryTask から別のパケットで届ける。

| パケット | 内容 | 送る頻度・大きさ |
|---------|------|----------------|
| 0x4F ステータス（v4） | v3（電池・状態・レート PID ゲイン・電流、57 バイト）の後ろに、ToF の診断値（下向き・前方: range status・信号レート・環境光レート・σ）、2 つの 400Hz タスクのループ周期の数え上げ、有限でない数の数え上げ、ログ経路の欠落の計数（`log_stream` の溢れ・`sendto` 失敗・統合パケットのエントリ棄却・テキストログの断った行数）を追記 | 1 Hz、224 バイト。PC は全長で版を選ぶ: 17・53・57・224 |
| 0x51 テキストログ | `ESP_LOG` の行を、機体時刻・切り詰めのフラグ・直前に失われた行数つきで、順序どおりに。[Header 4B][レコード（8B ヘッダ＋本文）…][Checksum 1B]。ヘッダの `sample_count` はレコード数、`sequence` はテキストパケット専用の通し番号 | 待つ行があるときだけ、最大 100 ms ごと（毎秒 10 個まで）、最大 1029 バイト |

**sendto の回数の見積り（§3 の核心的知見に従う）:** 電波の費用は `sendto()` の呼び出し回数で決まる（約 2.5 ms/回）ので、テキストパケットは 50 Hz の周期 5 回に 1 回までに間引く。統合パケットの毎秒 50 回（電波の 12.5 %）に対し、最悪でも毎秒 10 回の追加（2.5 %）で、行が無い間は 0 回。1 パケットのレコードは 1024 バイトまで（最長のレコード 392 バイトが 2 つ入る）。ログの嵐でもテキスト側の負担は 10.3 KB/s で、取得の数値ストリーム（約 48〜66 KB/s）を圧迫しない。

**欠落を見えるようにする:** ① 行を断った数は、次に入る行の `dropped_before`（255 で飽和）として欠落の位置で伝え、正確な合計はステータス v4 の `text_dropped_lines`。② テキストパケットが途中で失われたことは通し番号の飛びから PC が数え、バンドルの `meta.json` の `capture.text_packets_lost` に書く。③ 1 行が上限（384 バイト）を超えて切られたことはレコードのフラグで伝える。④ 取得が無い間はリングへ書かず、欠落とも数えない（取得は「今」から始まる）。数値ストリームの欠落は従来どおり統合パケットの通し番号の飛びで見る。

**PC 側:** `udp_capture.py` が 0x51 とステータス v4 をデコードし、`sf log wifi` がバンドル（`.sflog.zip`）の `text_log`・`tof_diagnostics`・`loop_timing`・`nonfinite`・`link_health` ストリームとして保存する（列は `protocol/spec/flight_log.yaml`）。テキストは `sf log info --text` で読む。順序の入れ替わりと重複には、テキストパケットの通し番号（16 ビット、最大値を基準に展開）で対応する。電文のバイトの並びは `data_stream_wire.hpp` と `udp_capture.py` が定義し、ホスト試験（`firmware/vehicle/test`）と `tools/log_analyzer/test_udp_capture_text_log.py` が、両者の共有ファイル（`firmware/vehicle/test/golden/`）とバイト単位で突き合わせる。

## 5. 関連ファイル

> **注記:** 本設計は当初の旧 `firmware/vehicle`（現 `firmware/vehicle_old`、凍結・新規開発なし）で確立された。ファームウェア昇格リファクタ（`vehicle_new` → `vehicle`）後は、電文互換の「Data Stream」として `firmware/vehicle/components/sf_telemetry/` 配下に再実装されており、`sf log wifi` / `sf log viz` は無改造で動作する。

| ファイル | 役割 |
|---------|------|
| `firmware/vehicle/components/sf_telemetry/include/data_stream_wire.hpp` | パケット構造体定義（`WireHeader`/`WireImuEskf`/`WirePosVel` 等、統合パケット 0x50） |
| `firmware/vehicle/components/sf_telemetry/data_stream.cpp` | `sf::DataStream` 実装（UDPログサーバ相当。ステータス v4・テキストパケットの送信を含む） |
| `firmware/vehicle/components/sf_core/log_sink.cpp` | ログシンク（`ESP_LOG` の入口 1 つ、コンソールと WiFi の出口 2 つ） |
| `firmware/vehicle/tasks/telemetry_task.cpp` | 統合パケット構築・送信（`TelemetryTask` が `sf::DataStream` を所有） |
| `tools/log_analyzer/udp_capture.py` | Python UDP 受信・統計・バンドル保存（0x51・ステータス v4 のデコードを含む） |
| `protocol/spec/flight_log.yaml` | バンドルのストリーム・列の基準（`text_log` ほか 4 ストリーム） |
| `tools/log_analyzer/visualize_interactive.py` | インタラクティブビューワー |

---

<a id="english"></a>

## 1. Background

TCP WebSocket telemetry (840B × 100Hz) blocked ESP-NOW control,
making flight impossible. ESP32-S3 has a single WiFi radio shared
between telemetry and ESP-NOW.

## 2. Failed Approaches and Lessons

### Method A: Individual sensor packets (sendto 265/sec)

Bandwidth calculations showed feasibility, but missed the key fact:
**sendto() block time is independent of packet size** (~2.5ms/call
whether 117B or 840B).

265 calls/sec × 2.5ms = 66% radio occupancy → ESP-NOW starved.

**Lesson: sendto() overhead is per-call, not per-byte.**

### Direct sendto() in control loop

Calling sendto() in the 400Hz IMU loop halved the rate to 200Hz.

**Lesson: Never call blocking I/O in a real-time loop.**
→ Fixed with producer-consumer (FreeRTOS queue).

### Shared data_ready flags

imu_task and telemetry_task consumed the same volatile bool.
First reader wins, second misses data.

**Lesson: Volatile flags cannot be shared between consumers.**
→ Fixed with timestamp-change detection.

### Shared ToF timestamp

Bottom and front ToF updated the same variable → 2× detected rate.

**Lesson: One variable per data source.**

### Stack overflows

Large structures on 4KB task stacks caused reboots.

**Lesson: All large structures must be module-static.**

## 3. Final Solution: Unified Packet

8 IMU cycles packed into one ~1000B packet at 50Hz.

| Metric | TCP | UDP-A | UDP Unified |
|--------|-----|-------|-------------|
| sendto() | 100/s | 272/s | **50/s** |
| Radio use | >70% | 68% | **12.5%** |
| Loss | 19% | 5% | **0%** |
| Control | impossible | difficult | **normal** |

**Key insight:** ESP32 lwIP sendto() has ~2.5ms fixed overhead per call.
Minimizing call count determines ESP-NOW coexistence quality.

## 4. Text Log and Status v4 (added outside the unified packet)

The unified packet (0x50) is the numeric 400Hz stream and has only 23 bytes of margin under its 1400-byte cap in the worst case (1377 bytes; since 2026-11-23 the 50 Hz `ctrl_ref` is 51 bytes instead of 30 so that it carries the 5 targets and the thrust-excitation state, +21 bytes over 1334). The flow samples (100 Hz, 11 bytes each) are 2 per packet when nothing is late, but a late TelemetryTask finds up to 8 in the ring (8 deep). Putting them all in one packet drops the last entries (mag, ...) from 7 samples on (the measured table is `wire_unified_packet_size_by_flow_samples` in `test_main.cpp`). So **one packet carries at most 4 flow samples** (`kMaxFlowEntriesPerPacket` in `data_stream_wire.hpp`); the surplus stays in the ring for the next packet (oldest first, so the time order holds). Why 4: the worst packet, with every kind of entry present, is 1377 bytes and fits the cap (checked by a `static_assert`, so enlarging an entry past the cap stops the build), and the drain of 4 per 20 ms outpaces the arrival of 2 per 20 ms (100 Hz). The ring still drops its oldest sample when full (`TopicRing`) and the condition for that is unchanged, a TelemetryTask more than about 80 ms late (the same as the 32-sample control ring, about 80 ms); but once the backlog has reached 8 it shrinks by only 2 per packet, so a second delay during the recovery has less room than before. Drops of the flow ring are not counted in `entry_drops`. The ESP_LOG lines of the vehicle (state transitions, landing decisions, failsafe, reasons for refusals) and the diagnostics reach the PC without growing it, as separate packets from the same socket and the same TelemetryTask.

| Packet | Content | Rate and size |
|--------|---------|---------------|
| 0x4F status (v4) | After v3 (battery, state, rate-PID gains, current; 57 bytes) it appends the ToF diagnostics (downward and front: range status, signal rate, ambient rate, sigma), the loop-timing counters of both 400Hz tasks, the non-finite counters and the loss counters of the log paths (`log_stream` overflow, `sendto` failures, entries refused by the unified packet, lines refused by the text-log ring) | 1 Hz, 224 bytes. The PC picks the version by the total length: 17, 53, 57, 224 |
| 0x51 text log | The `ESP_LOG` lines, in order, with the vehicle time, a truncation flag and the number of lines lost just before. [Header 4B][record (8B header + text) ...][Checksum 1B]. `sample_count` of the header is the number of records, `sequence` is a counter of its own for text packets | Only while lines wait, at most every 100 ms (up to 10 per second), at most 1029 bytes |

**Estimate of sendto calls (following the key insight of section 3):** the radio cost is set by the number of `sendto()` calls (about 2.5 ms each), so a text packet is sent at most on every 5th 50 Hz cycle. Against the 50 calls/s of the unified packet (12.5 % of the radio), the worst case adds 10 calls/s (2.5 %), and none while no line waits. The records of one packet are at most 1024 bytes (two longest records of 392 bytes fit). Even in a log storm the text exit costs 10.3 KB/s and does not squeeze the numeric stream of the capture (about 48 to 66 KB/s).

**Making loss visible:** (1) The number of refused lines travels with the next line that enters as its `dropped_before` (saturating at 255), at the place of the loss; the exact total is the status v4 field `text_dropped_lines`. (2) The PC counts text packets lost on the way from jumps of the sequence and writes `capture.text_packets_lost` into `meta.json` of the bundle. (3) A line cut at the limit (384 bytes) is flagged in the record. (4) While no capture is active nothing is written to the ring and nothing is counted as lost (a capture starts from "now"). Loss of the numeric streams is still seen as jumps of the unified-packet sequence.

**PC side:** `udp_capture.py` decodes 0x51 and the status v4, and `sf log wifi` stores them as the `text_log`, `tof_diagnostics`, `loop_timing`, `nonfinite` and `link_health` streams of the bundle (`.sflog.zip`; columns in `protocol/spec/flight_log.yaml`). The text is read with `sf log info --text`. Reordered and duplicated packets are handled by the text-packet sequence (16 bits, unwrapped relative to the highest value seen). The byte layout is defined by `data_stream_wire.hpp` and `udp_capture.py`; the host tests (`firmware/vehicle/test`) and `tools/log_analyzer/test_udp_capture_text_log.py` compare both byte for byte with the files they share (`firmware/vehicle/test/golden/`).

## 5. Related Files

| File | Role |
|------|------|
| `firmware/vehicle/components/sf_telemetry/include/data_stream_wire.hpp` | Wire structs and builders (unified 0x50, status v4, text log 0x51) |
| `firmware/vehicle/components/sf_telemetry/data_stream.cpp` | `sf::DataStream` (sends the status v4 and the text packets) |
| `firmware/vehicle/components/sf_core/log_sink.cpp` | The log sink (one `ESP_LOG` entry, two exits: console and WiFi) |
| `tools/log_analyzer/udp_capture.py` | Python receiver (decodes 0x51 and status v4), statistics, bundle writer |
| `protocol/spec/flight_log.yaml` | Streams and columns of the bundle (`text_log` and four more) |
