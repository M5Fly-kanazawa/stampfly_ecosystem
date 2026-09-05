# 実機検証チェックリスト（講師用）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

> 現行の workshop ファームは 2026-07-18 に vehicle のコンポーネント基盤上へ再構築された
> （`firmware/vehicle/docs/workshop_migration.md` §8）。この再構築以降、実機ベンチ・飛行レッスン
> （L5〜L8, L13）の実機検証は行われていない。本チェックリストは、9/10 のチュートリアル本番前に
> 講師が実機で確認する手順である。書式は DXH 講座の
> `docs/events/dxh2026/day2-morning-checklist.md` に倣う。**各項目は「操作 → 期待される結果」**
> **を確認し、チェックボックスで記録する。NG が出た場合は該当節の「NG時の代替」に従う。**

## 1. ベンチ確認（プロペラを外した状態で実施）

- [ ] `sf doctor` を実行 → エラーなく完了すること
- [ ] `sf lesson switch sci2026:1` → `sf lesson build` → `sf lesson flash` → 起動音・緑常灯まで到達すること
- [ ] 起動音が授業チャイムであること（workshop ファーム共通の識別音。vehicle の標準起動音 C5→E5→G5 とは異なる。`sf flash vehicle` に戻すと標準音に戻る）
- [ ] `sf lesson switch sci2026:3 --solution` → `sf lesson build` → `sf lesson flash` → プロペラなしでモータが回転すること（duty を上げるとゆっくり回転数が上がる）
- [ ] `sf lesson switch sci2026:4 --solution` → コントローラのスティックを倒すと `rc_roll()`/`rc_pitch()` 等の値がシリアル出力で追従すること
- [ ] USB 接続中は本体ボタン ARM が拒否されること（安全仕様）
- [ ] USB を抜きバッテリ駆動 → 本体ボタンで ARM → DISARM が機能すること
- [ ] `sf lesson switch sci2026:2 --solution` → Teleplot でジャイロ・加速度値が手で傾けると変化すること
- [ ] `sf telemetry` でライブダッシュボードが表示されること
- [ ] `sf log wifi -o bench.csv` でテレメトリ取得 → `sf log convert`（USB経由の場合）でCSV変換が成功すること

**NG時の代替:** ベンチ確認で問題が出た場合、`sf flash vehicle`（標準ファーム）に戻して S1〜S3 の実機デモは標準ファームで行う。workshop ファーム固有の問題（実習5以降）は「見るだけ」に切り替え、実演できない場合の代替として事前取得済みのログ・動画（§5）を使う。

## 2. 飛行確認（プロペラガード装着，保護メガネ着用，低スロットルから）

- [ ] `sf lesson switch sci2026:5 --solution` → 離陸しホバリングが安定すること（実習5 の $K_p=0.5$ 設定）
- [ ] `sf lesson switch sci2026:8 --solution` → PID 化した状態で離陸し，P のみと比べて定常偏差が減ること
- [ ] `sf lesson switch sci2026:9 --solution` → Teleplot で `cf_roll`（相補フィルタ）と `eskf_roll` がおおむね一致すること

**NG時の代替:** 特定の実習のみ飛行が不安定な場合，その実習のデモは事前取得済み動画（§5）に切り替え，他の実習は実演を続ける。全実習で不安定な場合，S4 全体を動画中心の進行に切り替える。

## 3. システム同定の確認

- [ ] 実習7 の手順（`ws::set_rate_target()` を呼ぶ学習者コード）でログを取得 → `sf sysid fit flight.csv --kp 0.5 --plot` が実行でき，Roll/Pitch の $K$, $\tau_m$ が実習6 の理論値（$K_{roll}=102$, $K_{pitch}=70$）に近い値で出ること
- [ ] 同じログで `sf sysid rate-fit flight.csv --axis roll` が実行でき，$(b, L, T)$ が妥当な値（$L$ が数ms〜十数ms程度）で出ること

**NG時の代替:** 同定が収束しない場合，事前取得済みの参照ログ（`analysis/reports/rate_sysid_reference/` 等）を使ったデモに切り替え，当日ログでの実演は「一緒に打つ」ではなく「見るだけ」にする。

## 4. 本番ファームでのデモ確認

- [ ] `sf flash vehicle`（標準ファーム）で POS_HOLD デモ（S1）が安定してホバリングすること
- [ ] 標準ファームで取得したログに対し `sf sysid rate-fit`/`sf sysid rate-tune`（S4）が実行でき，出力される `param set rate.roll.kp ...` が現在のパラメータと大きく乖離しないこと

**NG時の代替:** POS_HOLD が不安定な場合，S1 のデモは ALT_HOLD または STABILIZE に切り替える。rate-tune の出力が非現実的な場合，事前取得済みの結果例をスライドで示すのみにとどめる。

## 5. 実演できない場合の代替：動画・ログの取得

各セッションのデモについて，リハーサル時に以下を取得しておく。実演できない場合の代替として使う。

- [ ] S1: POS_HOLD ホバリングの動画（1分程度）
- [ ] S4: 実習5（P制御）→ 実習8（PID）の飛行動画，および `flight.csv`（`sf sysid fit`/`rate-fit` の入力に使える状態）
- [ ] S4: `sf sysid rate-fit`/`rate-tune` の実行ログ・出力（テキストで保存）
- [ ] S5: `sf sils gui` でのシナリオ実行のスクリーン録画
- [ ] 全動画・ログを講師 PC のローカルとクラウド（会場 WiFi 不通に備え）の両方に保存する

**実演できない場合の代替として使える SILS 由来の素材は生成済み:** S1/S4/S5 の SILS（MuJoCo）グラフ・動画・テキストサマリは `docs/events/sci_tutorial_2026/fallback/`（索引は同ディレクトリの `README.md`）に PNG/MP4 として生成済み。上記チェックボックスが指す**実機由来**の動画・ログ（S1 の POS_HOLD 実写ホバリング、S4 の実習5→8 実飛行、`rate-fit`/`rate-tune` の実ログ出力）は対象外で、講師が別途リハーサルで取得すること。

---

<a id="english"></a>

## 1. Overview

> The current workshop firmware was rebuilt onto the vehicle component base on
> 2026-07-18 (`firmware/vehicle/docs/workshop_migration.md` §8). Since that
> rebuild, the hardware bench and flight lessons (L5-L8, L13) have not been
> verified on real hardware. This checklist is the instructor's pre-tutorial
> (2026-09-10) hardware verification procedure, modeled on the DXH workshop's
> `docs/events/dxh2026/day2-morning-checklist.md`. **Each item states an**
> **action and its expected result; check the box once confirmed. If an item**
> **fails, follow that section's fallback.**

## 1. Bench Check (propellers removed)

- [ ] `sf doctor` completes with no errors
- [ ] `sf lesson switch sci2026:1` -> `sf lesson build` -> `sf lesson flash` -> reaches the boot chime and steady green LED
- [ ] The boot sound is the school chime (the workshop firmware's common identity sound, distinct from the vehicle's standard C5-E5-G5 chime; `sf flash vehicle` restores the standard sound)
- [ ] `sf lesson switch sci2026:3 --solution` -> `sf lesson build` -> `sf lesson flash` -> motors spin with no propellers (speed rises gradually with duty)
- [ ] `sf lesson switch sci2026:4 --solution` -> `rc_roll()`/`rc_pitch()` etc. track the controller sticks in the serial output
- [ ] Button ARM is rejected while USB is connected (safety behavior)
- [ ] Unplug USB, run on battery -> button ARM and DISARM both work
- [ ] `sf lesson switch sci2026:2 --solution` -> gyro/accel values change in Teleplot as you tilt the vehicle by hand
- [ ] `sf telemetry` shows the live dashboard
- [ ] `sf log wifi -o bench.csv` captures telemetry; `sf log convert` (USB path) succeeds

**Fallback:** if the bench check fails, flash the production firmware (`sf flash vehicle`) and run the S1-S3 hardware demos on it instead. For workshop-firmware-specific issues (Exercise 5 onward), switch to "watch only" and use the pre-recorded log/video from §5.

## 2. Flight Check (prop guards on, eye protection, start at low throttle)

- [ ] `sf lesson switch sci2026:5 --solution` -> takes off and hovers stably (Exercise 5's $K_p=0.5$)
- [ ] `sf lesson switch sci2026:8 --solution` -> takes off with PID enabled, steady-state error visibly smaller than P-only
- [ ] `sf lesson switch sci2026:9 --solution` -> `cf_roll` (complementary filter) and `eskf_roll` broadly agree in Teleplot

**Fallback:** if only one exercise flies unstably, switch that exercise's demo to the pre-recorded video (§5) and keep flying the others live. If all exercises are unstable, run S4 primarily from video.

## 3. System Identification Check

- [ ] Using Exercise 7's procedure (student code calling `ws::set_rate_target()`), `sf sysid fit flight.csv --kp 0.5 --plot` runs and returns roll/pitch $K$, $\tau_m$ close to the Exercise 6 theoretical values ($K_{roll}=102$, $K_{pitch}=70$)
- [ ] The same log works with `sf sysid rate-fit flight.csv --axis roll`, returning a plausible $(b, L, T)$ ($L$ on the order of single-digit to low-double-digit ms)

**Fallback:** if identification does not converge, demo with a pre-captured reference log (e.g. under `analysis/reports/rate_sysid_reference/`) and present the day's own log as "watch only" rather than live.

## 4. Production-Firmware Demo Check

- [ ] `sf flash vehicle` (production firmware): POS_HOLD (S1) hovers stably
- [ ] `sf sysid rate-fit`/`sf sysid rate-tune` (S4) run on a production-firmware log and the resulting `param set rate.roll.kp ...` values are not wildly off from the current parameters

**Fallback:** if POS_HOLD is unstable, switch the S1 demo to ALT_HOLD or STABILIZE. If rate-tune's output is unrealistic, show a pre-captured example on the slide instead of running it live.

## 5. Fallback for When a Live Demo Fails: Video and Logs

Capture the following during rehearsal, for every session's demo. Use these as the fallback whenever a live demo cannot be run.

- [ ] S1: a short (about 1 minute) video of POS_HOLD hovering
- [ ] S4: flight video for Exercise 5 (P-control) through Exercise 8 (PID), plus `flight.csv` (usable as input to `sf sysid fit`/`rate-fit`)
- [ ] S4: the run log and output of `sf sysid rate-fit`/`rate-tune` (saved as text)
- [ ] S5: a screen recording of a `sf sils gui` scenario run
- [ ] Save every video and log both locally on the instructor laptop and in the cloud (in case venue WiFi is down)

**Fallback material derived from SILS, usable when a live demo fails, is already generated:** the S1/S4/S5 SILS (MuJoCo) graphs, videos, and text summaries live in `docs/events/sci_tutorial_2026/fallback/` as PNG/MP4 files (see that directory's `README.md` for the index). The **hardware**-derived video/logs the checkboxes above refer to (S1's real POS_HOLD hover footage, S4's real Exercise 5->8 flight, the `rate-fit`/`rate-tune` run logs) are out of scope there and still need to be captured by the instructor during a separate rehearsal.
