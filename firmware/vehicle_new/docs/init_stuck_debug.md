# 実機 INIT 停止バグ — 根本原因調査の引き継ぎ

最終更新: 2026-06-10
状態: **調査中（根本原因を行単位で特定する最終フラッシュ待ち）**
方針: **ユーザー強い指示＝場当たり対処禁止・根本原因を特定して抜本対処。SIL で動くこと自体が盲点（記憶済 `project_sil_hardware_concurrency_gap`）。**

> このバグが解けるまで実機ブリングアップ（development_roadmap Phase 2/3）は進めない。本書がブロッカーの単一情報源。

---

## 1. 事象（Symptom）

- vehicle_new を実機（M5StampFly, ESP32-S3）にフラッシュ。**ペア済み復元起動（`Pairing restored: DC:54:75:EE:75:2C`）で 300 秒以上 LED 白＝INIT のまま**。`Init complete → IDLE_GROUND` が出ない＝INIT→IDLE_GROUND 遷移が起きない。
- 一方 **ImuTask は 400Hz で回り続ける**（chi2 ログが 1 秒で 400 増）。`sensor_imu` は出力されている。
- **コントローラは OFF**（受信割込みゼロ＝「コントローラ送信が core0 を飽和させる」説は否定済）。
- **未ペア起動（ただし旧 UI 前ファーム）は ~2.2 秒で IDLE_GROUND 到達**。ペア済み＋UI 版で stuck。

> **重要な交絡（未解決）**: 「速い起動」は**ペアリング実装版（UI 変更前）＋未ペア**、「stuck」は**UI 変更版＋ペア復元**。
> 「ペア済みだから遅い」と断定できず、**UI 変更自体が原因の可能性**も残る。下記マーカーで切り分く。

---

## 2. これまでに確定した事実（DIAG instrumentation の結果）

ImuTask が毎 ~1.1 秒 raw printf で出す `DIAG:` 行（commit 94125d8）の実機ログより:

```
DIAG: state=0 armed=0 imu_ts=... | state_loops=1 stage=9 | notify_upd=23   (1.4s)
DIAG: state=0 ...                | state_loops=1 stage=9 | notify_upd=60   (2.6s)
DIAG: state=0 ...                | state_loops=2 stage=1 | notify_upd=94   (3.7s)
DIAG: state=0 ...                | state_loops=2 stage=1 | notify_upd=1457 (48s) ← 以後ずっと固定
```

- **`state_loops` が 2 で永久停止、`stage=1` で固定** ＝ StateTask は**ループ 2 回目の「stage 1（先頭）と stage 2（pilot_request 読込）の間」で永久ブロック**。
- その区間にあるコードは `state_task.cpp:281` の **INIT 判定 → `g_state_manager.notifyInitComplete()`** だけ（`sensor_imu.latest()` は RingBuffer ＝ lock-free・非ブロッキング）。
- `imu_ts`/`notify_upd` は増え続ける ＝ **ImuTask・NotifyTask は生存**。StateTask だけが固まっている。

### stage マーカーの位置（state_task.cpp 現状）
| stage | 位置 |
|------|------|
| 1 | ループ先頭（`ulTaskNotifyTake` 直後） |
| 2 | `pilot_request.latest()` 直前 |
| 4 | `command_setpoint` 読込直前（takeoff） |
| 6 | `pairing_complete.latest()` 直前 |
| 9 | `g_state_manager.update()` 直前 |

→ **stage=1 固定なので stage 1〜2 間（＝ notifyInitComplete）で確定。**

---

## 3. 解析上の矛盾（なぜ難しいか）

`notifyInitComplete()` の中身（`state_manager.cpp:78`）:
```cpp
if (state_ != INIT) return;
ESP_LOGI(TAG, "Init complete → IDLE_GROUND");   // (a)
transition(FlightState::IDLE_GROUND);            // (b)
```
`transition()`（INIT→IDLE_GROUND 経路）で mutex/ブロッキングを取り得る呼び出し:
1. `ESP_LOGI("Transition: ...")` — s_log_mutex
2. exit callbacks — **登録ゼロ**（`exit_callback_count_==0`、空回り）
3. enter callback ×1 — `estimator_command.publish({Recalibrate})` ＝ `xQueueSend(...,0)` **非ブロッキング**
4. `publishMode()` → `system_mode.publish()` — `xSemaphoreTake(mutex, portMAX_DELAY)`

**ところが、ブロックし得る候補が全て「空いている」ことを実機ログが示している:**
- **`ESP_LOGI` ではない**: Telemetry の `ESP_LOGW`（sendto failed）が stuck 後も毎秒出続けている。もし StateTask が ESP_LOGI 内で `s_log_mutex` を握ってブロックなら Telemetry も道連れで止まるはずだが止まっていない ⇒ s_log_mutex は空き。
- **`publishMode` の system_mode mutex でもない**: `notify_upd` が 30Hz で増加 ＝ NotifyTask が `system_mode.latest()`（同じ mutex）を毎秒 30 回取得できている ⇒ system_mode mutex は空き。
- **enter callback の Queue publish は非ブロッキング**（`xQueueSend(...,0)`、満杯でもドロップして返る）。

→ **理屈上 `notifyInitComplete`/`transition` 内の全呼び出しが非ブロッキングなのに、StateTask はそこで永久に止まっている。** この矛盾を物理的に解くため、行単位 raw printf マーカーを入れた（次節）。

### 補足（観測された別事象）
- 1 回目の起動試行は ~718ms で**リブート**（`wifi:flush txq` 直後にブートローダへ）。2 回目の起動が stuck になる。2 回目に panic/backtrace は無い ＝ StateTask は crash でなく純粋にブロック。
- この起動で PMW3901（光学フロー）init 失敗（Optional, Flow disabled）。BMM150（mag）も時々失敗。いずれも Optional で本筋とは無関係と判断（BMI270 は正常、ESKF/chi2 は 400Hz 稼働）。

---

## 4. 次にやること（最優先・1 アクション）

**行単位マーカー入りファーム（commit 7141216, ビルド済み）をフラッシュして `ROOT:` 行を採取する。**

```bash
source setup_env.sh
sf flash vehicle_new -m
```

起動後に出る `ROOT:` 行を**全部**採取。正常なら以下が**全部**順に出る:
```
ROOT: ST pre-notifyInitComplete
ROOT: NIC-1 pre-esplog
ROOT: NIC-2 post-esplog pre-transition
ROOT: TR-1 enter (0->1) pre-esplog
ROOT: TR-2 post-esplog, exit_cb_cnt=0
ROOT: TR-4 state set, enter_cb_cnt=1
ROOT: TR-5 enter_cb[0] pre
ROOT: TR-5 enter_cb[0] post
ROOT: TR-6 pre-publishMode
ROOT: TR-7 post-publishMode (transition done)
ROOT: NIC-3 post-transition
ROOT: ST post-notifyInitComplete
```

### 判定表（最後に出た `ROOT:` の「次」がブロック箇所）
| 最後に出た行 | ブロック箇所＝真因 |
|------------|------------------|
| **どの ROOT も出ない** | `ST pre` の raw printf 自体が出ない＝StateTask 固有のコンソール出力経路がブロック（ImuTask の DIAG が出続けるなら StateTask だけの問題＝タスク文脈/コア/ロック） |
| `ROOT: ST pre` | `notifyInitComplete` 入口〜NIC-1 printf の間（ほぼあり得ない、要再検討） |
| `ROOT: NIC-1` | **`ESP_LOGI("Init complete")` がブロック**（コンソール書込み or s_log。Telemetry 生存と矛盾するので、その場合 s_log_mutex の理解を改める） |
| `ROOT: NIC-2` または `TR-1` | `transition()` の `ESP_LOGI("Transition")` 周り |
| `ROOT: TR-5 enter_cb[0] pre` | **enter コールバック内**（`estimator_command.publish` 経路、Queue が想定外にブロック？ or ラムダ内別処理） |
| `ROOT: TR-6 pre-publishMode` | **`publishMode()` の system_mode mutex**（NotifyTask 生存と矛盾するので、その場合 mutex の理解を改める） |
| 全部出る | notifyInitComplete は通っている＝**stage マーカーの読み違い**。別の場所（要再調査） |

確定したら**その行の根本原因を抜本対処**し、最後に DIAG/ROOT 計装を全て revert（下記）。

---

## 5. 交絡を切るための補助テスト（任意・余裕があれば）

「UI 変更が原因」か「ペア復元が原因」かを切り分けるため、**ペアリングを消して未ペアで同じ（マーカー）ファームを起動**する:
- 方法 A: `idf.py erase-flash` 後に再フラッシュ（NVS 全消去 ＝ 未ペア）。ただし全 NVS が消える。
- 方法 B: NVS の `sf_pair` namespace だけ消す手段があれば（CLI `unpair` は StateTask がブロック中ゆえ届かない可能性大）。
- 結果: 未ペアで IDLE_GROUND 到達 ⇒ **ペア復元経路が原因**。未ペアでも stuck ⇒ **UI 版ファーム自体が原因**。

---

## 6. 計装の所在（root-cause 確定後に revert するもの）

| commit | 内容 |
|--------|------|
| 94125d8 | `g_diag_state_loops`/`g_diag_state_stage`（state_task）、`g_diag_notify_updates`（notify_task）、ImuTask の `DIAG:` printf（imu_task） |
| 7141216 | `notifyInitComplete`/`transition` の `ROOT:` 行マーカー（state_manager）、`ST pre/post`（state_task） |

revert 手順（確定後）: 上記 2 commit の追加分を取り除く（`git revert` ではなく該当行削除＋fix コミットが綺麗。`<cstdio>` include も不要なら除去）。**抜本対処の本コミットとは分ける。**

---

## 7. 関連文書・記憶

- 記憶 `project_sil_hardware_concurrency_gap` — SIL が実機並行性バグを構造的に見逃す盲点（今後の課題）。
- 記憶 `project_pairing_status` — ペアリング P1〜P3 実装＋SIL 検証完了。
- `development_roadmap.md §4 Phase 2/3` — このバグ解決後に戻る本道。
- `next_session_plan.md` — 全体の優先順（本書はその最優先タスクの詳細）。
