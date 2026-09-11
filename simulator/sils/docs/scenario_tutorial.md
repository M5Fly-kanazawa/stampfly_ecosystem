# SILS シナリオ（.scn）と合否判定（.expect）の書き方チュートリアル
# SILS Scenario (.scn) and Assertion (.expect) Authoring Tutorial

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。
>
> 文法の正本（Single Source of Truth）はコードである。`.scn` は
> `simulator/sils/devices/scenario.cpp` のパーサ、`.expect` は
> `lib/sfcli/commands/sils.py` の `_eval_expect`/`_traj_metric` が唯一の正。
> 本書はそれを読んで書いたもので、齟齬があればコードを信じること。

## 1. 概要

### このドキュメントについて

SILS（Software-in-the-Loop＝実機ファームを無改変のままPC上の物理シミュレーションと閉ループで動かす仕組み）で機体に流す入力とその合否判定は、2種類のテキストファイルで書く。

| ファイル | 役割 | 中身 |
|---------|------|------|
| `<name>.scn` | 入力の台本 — 「いつ・何を機体に与えるか」 | スティック値・API コマンド・風・故障などのイベントを時刻順に並べた行 |
| `<name>.expect` | 合否判定 — 「何をもって合格とするか」 | ログ文字列の有無・順序・終了コード・実行結果のフライトログ一式（`.sflog.zip`。1回の実行の信号をまとめた zip 形式のログファイル。仕様の正本は `protocol/spec/flight_log.yaml`）由来の数値しきい値 |

どちらも UTF-8 のプレーンテキストで、`simulator/sils/scenarios/` に同名（拡張子違い）で並べて置く。`.expect` が無い場合は「入力が実際に注入されたか」と「終了コードが 0 か」だけで合否が決まる（後述）。

### 対象読者

`sf sils scenario` で既存シナリオを走らせたことはあるが、自分でシナリオと合否判定を新規に書きたい人。スキーマ（データの構造・形式の定義）を一から読み解く必要はなく、本書と手元の実例（`simulator/sils/scenarios/`）を見比べれば書けるようにする。

## 2. `.scn` の書き方

### 行の構造

1行が1イベント。空行と `#` 以降のコメントは無視される。

```
<時刻>  <種別>  <引数...>   # コメント
```

### 時刻の3通り

| 書き方 | 意味 |
|--------|------|
| `0` | 絶対時刻 0 ms（シナリオ開始と同時） |
| `<数値ms>`（例: `5000`） | 絶対時刻（そのシナリオ全体の開始からのミリ秒） |
| `+`（プラス単体） | 直前イベントが終わった直後（`+0` と同じ） |
| `+<数値ms>`（例: `+500`） | 直前イベントが終わってから指定ミリ秒後 |

**注意:** 絶対時刻は「直前イベントの終了時刻より前」を指定するとパースエラーになる（1行ずつ順に発火する単一スレッドのドライバのため、追い越しは許さない）。`rc` の `hold_ms` は保持時間ぶんだけそのイベントの終了時刻を後ろへ押すので、絶対時刻を混ぜるときは前のイベントがいつ終わるかを意識すること。迷ったら `+` を使えば自動的に辻褄が合う。

### イベント種別

| 種別 | 引数 | 単位・備考 |
|------|------|-----------|
| `rc` | `<thr> <roll> <pitch> <yaw> <arm> [hold_ms] [rate_hz] [alt] [acro] [pos]` | 送信機スティックの模擬。詳細は次項 |
| `rc_foreign` | `rc` と同じ引数 | **別の（ペアリングされていない）送信機 MAC** から同じ内容を送る。混信フィルタの検証用 |
| `rc_ramp` | `<throttle\|roll\|pitch\|yaw> <from> <to> <step> <rate_hz> <arm> [alt] [acro]` | 1軸だけを `from`→`to` へ `step` 刻みで掃引（他の軸は中央値2048）。`step` は符号不要（方向は from/to から自動判定）、`rate_hz` は 1〜1000 |
| `key` | `"<text>"` | ファームのコンソール（CLI）へ文字列をそのまま流し込む。`\n \t \r \\ \"` エスケープ可 |
| `api` | `"<command line>"` | Tello 風 API コマンド1行をファームの ApiTask パーサへ直接注入（例: `"forward 50"`）。ApiTask を持つ `vehicle`/`workshop` ターゲットのみ対応、他ターゲットでは「未対応」として記録されるだけで無視される |
| `wind` | `<fx> <fy> <fz> [dur_ms]` | 機体に加える外乱力、NED座標 [N]。`dur_ms` を指定すると突風パルス（保持後に自動で 0,0,0 へ戻る）、省略すると定常的に加わり続ける |
| `fault` | `<motor 0-3> <gain 0-1>` | 指定モータの推力健全度を劣化させる（1.0=健全、0.0=完全停止）。以後持続 |
| `bias` | `<ax> <ay> <az> <gx> <gy> <gz>` | 生IMUに決定論的なバイアスを注入（機体 FRD＝前右下座標系）。加速度 [m/s²]・角速度 [rad/s] |
| `handle` | `<carry_alt_m> <place_x> <place_y> <lift_ms> <carry_ms> <place_ms>` | 人が持ち上げて運ぶ動作の模擬（lift→right→carry→place）。`carry_alt_m>0`、各所要時間 `>0` 必須。タイムライン上は瞬時に発火し、その後の disarmed な `rc` 保持が実時間を占有する |
| `btn` | （未実装・予約） | 現状は警告を出して読み飛ばされるだけ（GPIOボタンの模擬は将来実装） |

### `rc` 行の詳細（最重要）

```
<t>  rc  <thr> <roll> <pitch> <yaw> <arm> [hold_ms] [rate_hz] [alt] [acro] [pos]
```

- `thr`/`roll`/`pitch`/`yaw`: 12bit ADC 生値、**範囲 0〜4095、中立 2048**。送信機のスティックそのもの
- `arm`: 0 か 1。ファームの状態機械は「ARM ビットの立ち上がり→立ち下がり」をトグルとして扱う（実機の押しボタンと同じ規約）ので、ARM させたいときは 0→1 の1回の遷移を作ればよく、DISARM させたいときはもう一度 0→1 を作る（＝1度離してもう一度押す）
- `hold_ms`（省略時 0）: このスティック値を何ミリ秒保持するか。0 なら単発フレーム
- `rate_hz`（省略時 20）: 送信周期 [Hz]、1〜1000。`hold_ms` はこの周期に丸められ、正の保持は必ず最低1フレームは送られる
- `alt`（省略時 0）: 1 で ALTITUDE_HOLD（高度保持）フラグを立てる
- `acro`（省略時 0）: 1 で ACRO（角速度）モードフラグを立てる
- `pos`（省略時 0）: 1 で POSITION_HOLD（位置保持）フラグを立てる

**位置引数であることに注意:** `alt`/`acro`/`pos` は `hold_ms`・`rate_hz` の**後**にしか置けない。たとえば `acro` だけを 1 にしたい行でも、`hold_ms` と `rate_hz` を省略はできない（完全な形で書く）。

```
# 良い例: acro を立てたいので hold_ms/rate_hz を明示し、alt=0, acro=1 と並べる
+  rc  2048 2048 2048 2048  1  1000  50   0  1
```

### イベントの終了時刻（`+` の基準）

`+` は「直前イベントが終わった時刻」を基準にする。何をもって「終わり」とするかはイベント種別で決まる:

- `rc`: `hold_ms` ぶん先（0 なら単発フレームで時間を消費しない）
- `rc_ramp`: 掃引に要するフレーム数 × 周期
- `wind`: `dur_ms` を指定したときだけその時間ぶん（突風パルス）。省略時（定常ステップ）は瞬時
- それ以外（`key`/`api`/`fault`/`bias`/`handle`/`btn`）: タイムライン上は瞬時（0ms）

## 3. `.expect` の書き方

### 行の種類

| 種類 | 書式 | 意味 |
|------|------|------|
| `xfail: <理由>` | ファイルの**先頭の非空・非コメント行のみ**有効 | 既知の未解決課題であることを明示するマーカー。`sf sils scenario` 単体の合否表示には影響しない（常に素の PASS/FAIL）が、`sf sils regression` はこれを `[KNOWN-FAIL]` として集計対象外にし、直ったら `[XPASS]`（要マーカー削除）で知らせる |
| `exit <code>` | 例: `exit 0` | プロセスの終了コードが一致するか |
| `log_contains <out\|err\|any> "<text>"` | 例: `log_contains any "ARM accepted"` | 標準出力/標準エラー/両方のいずれかに文字列が含まれるか |
| `log_absent <out\|err\|any> "<text>"` | | 含まれ**ない**ことを確認（異常ログが出ていないことの確認等） |
| `order "<a>" "<b>"` | | `a` の最初の出現位置が `b` の最初の出現位置より前にあるか（両方の標準出力+標準エラーの結合テキストで判定） |
| `metric <name> <op> <value> [in <t0> <t1>]` | 例: `metric tilt_max < 0.314 in 7.0 13.0`（≈18°） | 実行結果のフライトログ一式（`truth.csv` 等のストリーム）から計算した数値でのしきい値判定。`op` は `< <= > >=` の4種類のみ（`==`/`!=` は無い）。`in <t0> <t1>` を付けると窓（**秒単位**）でフェーズを絞れる。省略すると全区間 |
| `skip <理由...>` | | この行を「合格扱いだが評価はしていない」として記録する（ハード依存の機能等で使う） |
| `# コメント` | | 無視される |

**空の `.expect`（実質評価行が1つも無い）は自動的に FAIL になる。**「何も書かなければ通る」にはならないよう意図的にそう作られている。

### `metric` の名前一覧

実行結果のフライトログ一式（zip 形式のログファイル `<...>.sflog.zip`。中身は `sf log check` 等でも読めるパケット種別ごとの CSV）の中の `truth.csv`（シミュレータの物理真値: 位置・姿勢クォータニオン・速度・角速度）、`attitude.csv`／`posvel.csv`（ファーム推定値）、`motor.csv`（モータ duty）の各ストリームから計算される。**角度系の値（`roll_rmse`/`pitch_rmse`/`att_rmse`/`tilt_max`/`yaw_band`）は SI 単位のラジアンで扱う**（`protocol/spec/flight_log.yaml` 準拠。度ではない）。姿勢はクォータニオン列 `quat_w/x/y/z` で記録されており、ロール・ピッチ・ヨー角へは内部でオイラー角変換してから算出する。`sf sils scenario` のコンソール出力には、ラジアン値に加えて読みやすさのための `(x deg)` 換算値も添えられる。

| 名前 | 単位 | 意味 |
|------|------|------|
| `alt_mean` | m | 窓内の高度平均 |
| `alt_min` | m | 窓内の高度最小値 |
| `alt_max` | m | 窓内の高度最大値 |
| `alt_band` | m | 窓内の高度の最大−最小（ピークtoピーク、高度保持の変動幅） |
| `alt_rmse` | m | 推定高度（`posvel.csv`）と真値高度（`truth.csv`）の二乗平均平方根誤差 |
| `roll_rmse` | rad | 推定ロール（`attitude.csv`）と真値ロール（`truth.csv`）の RMSE |
| `pitch_rmse` | rad | 推定ピッチ（`attitude.csv`）と真値ピッチ（`truth.csv`）の RMSE |
| `att_rmse` | rad | roll_rmse と pitch_rmse を合成した姿勢誤差の大きさ（`hypot`） |
| `tilt_max` | rad | 窓内の真値の傾き `hypot(roll, pitch)` の最大値（転倒していないことの確認） |
| `yaw_band` | rad | 窓内の真値方位（クォータニオンから算出、±180°の継ぎ目でアンラップ済み）のピークtoピーク |
| `duty_max` | 比（0〜1） | 窓内の4モータ duty の最大値（飽和していないことの確認） |
| `horizontal_drift_max` | m | 窓の**開始時点**からの水平面内の最大距離（位置保持の逸脱量） |

未知の名前・フライトログ一式に対象ストリームが無い・窓内にデータが無い、のいずれかでは判定不能扱いで **FAIL** になる（`None` を返し、しきい値比較をしない）。

### 決定論性への依拠

`.scn`/`.expect` の判定は「同じ入力なら出力が完全に一致する（バイト同一）」という SILS の決定論性の上に成り立っている。ログ文字列の有無・出現順序も、フライトログ一式の数値も、乱数ノイズを混ぜない限り（`--noise off` が既定）毎回同一になるので、しきい値をゆるく持たせる必要はない（既存シナリオのしきい値は「実測値＋わずかな余裕」で決めてある。後述の手順4で同じやり方をする）。

## 4. 手順の実例 — ロールステップ応答の行き過ぎと整定を見る

STABILIZE モード（自己水平化。姿勢角に追従するモードで、鉛直方向の推定精度に依存しない）でロール・スティックへ矩形波状のステップを与え、傾きが指令値をどれだけ行き過ぎ、その後どこまで整定するかを合否判定にする例。以下は実際に `sf sils scenario` を実行して得た数値。

### (1) `my_roll_step.scn` を書く

`stab_flight.scn`（`simulator/sils/scenarios/`）の ARM→離陸→ステップの流れを土台に、ロール1軸だけに絞って短縮した。

```
# my_roll_step.scn — roll step response (STABILIZE)
#
#  <t>  ch  <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz>
   0    rc  2048  2048   2048    2048  0     4000      50   # A: disarmed 4s, boot calibration
   +    rc  2048  2048   2048    2048  1     500       50   # B: ARM
   +    rc  3243  2048   2048    2048  1     2000      50   # C: throttle up -> TAKEOFF -> FLYING, climb
   +    rc  3176  2600   2048    2048  1     1000      50   # D: ROLL STEP +8 deg (right)
   +    rc  3176  2048   2048    2048  1     1500      50   # E: back to centre, overshoot/settle window
   +    rc  2048  2048   2048    2048  0     200       50   # F: release ARM button
   +    rc  2048  2048   2048    2048  1     200       50   # G: press again -> DISARM
   +    rc  2048  2048   2048    2048  0     600       50   # H: settle disarmed
```

- A: 4秒間 disarmed で待ち、起動時センサ較正を完了させる
- B: ARM の立ち上がりエッジ（0.5秒 IDLE で保持してから）
- C: スロットルを離陸閾値（norm 0.5、raw 2048+2048×0.5=3072）超の raw 3243 に上げ、ARMED_GROUND→TAKEOFF→FLYING/STABILIZE へ遷移させつつ上昇
- D: スロットルをほぼホバー相当の raw 3176 に落としつつ、ロール・スティックを raw 2600（≈+8°相当）へ1秒間ステップ
- E: ロール・スティックを中央（0°指令）へ戻し、1.5秒かけて行き過ぎ・整定を見る
- F〜G: ARM ボタンを離して再度押す＝トグルで DISARM
- H: DISARM 後の落下・接地を見送る

`thr` の raw 値（3243=離陸バースト、3176=ほぼホバー）は `stab_flight.scn` から流用した実測値。自分のプロジェクトで別のモータ・機体設定を使う場合は、まず `rc` をスティック中央のまま数秒保持するシナリオを走らせ、フライトログ一式の `truth.csv` の高度（NED座標の `pos_z` を反転した値）が概ね一定に留まる `thr` を実測してから合わせ込むこと（「まず走らせて実測してから」は手順(4)でも繰り返す）。

### (2) 走らせて出力を見る

```bash
source setup_env.sh
sf sils scenario simulator/sils/scenarios/my_roll_step.scn --target vehicle
```

この時点では `.expect` が無いので、判定は「入力が注入されたか」と「終了コード0か」だけになる（実行結果）:

```
[INFO] (no .expect at my_roll_step.expect — verdict = injection + exit code)
[INFO] scenario my_roll_step.scn: PASS (exit 0, 2 checks, events=events.jsonl)
  [PASS] input injected (events.jsonl non-empty)  (38441 bytes)
  [PASS] exit == 0  (got 0)
[OK] bundle: simulator/sils/viz/out_scn_my_roll_step
```

結果一式（コンソールログ・フライトログ一式 `sils_my_roll_step_<YYYYMMDD>T<HHMMSS>.sflog.zip`・`events.jsonl`）は `simulator/sils/viz/out_scn_my_roll_step/` に書き出される。一式内の `truth.csv`（姿勢はクォータニオン `quat_w/x/y/z` で記録、ロール角へはオイラー角変換して求める）を見ると、D イベント（scn上の絶対時刻 6500ms=6.5s）の直後からロール角が立ち上がり、指令の+8°（≈0.140 rad）を行き過ぎて **約0.194 rad（≈11.1°）まで達してから**戻り、E で中央に戻した後は **約0.078 rad（≈4〜5°）の残留偏差を残したまま** DISARM（scn上9200〜9400ms=9.2〜9.4s）を迎える、という実測が確認できた（`tilt_max`/`roll_rmse` 等の角度系メトリクスは SI 単位のラジアンで扱う）。

### (3) `my_roll_step.expect` を書く

(2) で得た実測値（行き過ぎ量・整定後の残留量）に余裕を持たせてしきい値化する。

```
exit 0
log_contains any "ARM accepted"
log_contains any "Takeoff detected"
log_contains any "Takeoff complete"
log_contains any "DISARM accepted"
order "ARM accepted" "Takeoff detected"
order "Takeoff detected" "Takeoff complete"
order "Takeoff complete" "DISARM accepted"
metric tilt_max > 0.140 in 6.5 7.5     # ステップ窓: 指令8度(≈0.140rad)を行き過ぎていること
metric tilt_max < 0.349 in 6.5 7.5     # ただし有界（転倒していない、≈20°）
metric tilt_max < 0.105 in 8.7 9.0     # DISARM直前には数度まで整定していること（≈6°）
metric duty_max  < 0.90 in 6.5 9.0     # モータ飽和なし
```

`tilt_max` はラジアン単位のしきい値であること（`protocol/spec/flight_log.yaml` 準拠、コンソール出力には `(x deg)` の換算値が添えられる）に注意。`in` の窓は**秒単位**であることに注意（`.scn` 側の時刻はミリ秒だが、こちらは秒）。D イベントは scn 上 6500〜7500ms なので `in 6.5 7.5`、E の終盤（DISARM 直前）は 8700〜9000ms 相当なので `in 8.7 9.0` とした。

### (4) 再実行して PASS を確認する

```bash
sf sils scenario simulator/sils/scenarios/my_roll_step.scn --target vehicle
```

実測結果（実際に得られた出力）:

```
[INFO] scenario my_roll_step.scn: PASS (exit 0, 13 checks, events=events.jsonl)
  [PASS] input injected (events.jsonl non-empty)  (38441 bytes)
  [PASS] exit == 0  (got 0)
  [PASS] log_contains any 'ARM accepted'  (found)
  [PASS] log_contains any 'Takeoff detected'  (found)
  [PASS] log_contains any 'Takeoff complete'  (found)
  [PASS] log_contains any 'DISARM accepted'  (found)
  [PASS] order 'ARM accepted' before 'Takeoff detected'  (idx_a=10577 idx_b=10860)
  [PASS] order 'Takeoff detected' before 'Takeoff complete'  (idx_a=10860 idx_b=11074)
  [PASS] order 'Takeoff complete' before 'DISARM accepted'  (idx_a=11074 idx_b=11181)
  [PASS] metric tilt_max > 0.14 in [6.5,7.5]  (tilt_max=0.1944 (11.14 deg))
  [PASS] metric tilt_max < 0.349 in [6.5,7.5]  (tilt_max=0.1944 (11.14 deg))
  [PASS] metric tilt_max < 0.105 in [8.7,9.0]  (tilt_max=0.0784 (4.49 deg))
  [PASS] metric duty_max < 0.9 in [6.5,9.0]  (duty_max=0.7098)
[OK] bundle: simulator/sils/viz/out_scn_my_roll_step
```

`--video` を付ければ、PASS 時に MuJoCo 3D＋状態グラフのレビュー動画が生成される（`sf sils scenario ... --video`）。

### (5) FAIL したときの見方

しきい値をきつくしすぎた例として、`tilt_max < 0.087 in 6.5 7.5`（≈5°。実測11.1°に対して明らかに厳しすぎる）だけを書いた `.expect` で走らせると:

```
[ERROR] scenario FAILED — see .../console.log
[INFO] scenario my_roll_step.scn: FAIL (exit 0, 3 checks, events=events.jsonl)
  [PASS] input injected (events.jsonl non-empty)  (38441 bytes)
  [PASS] exit == 0  (got 0)
  [FAIL] metric tilt_max < 0.087 in [6.5,7.5]  (tilt_max=0.1944 (11.14 deg))
```

のように `[FAIL]` 行に**実測値**（`tilt_max=0.1944`、括弧内は deg 換算）が添えられるので、しきい値をどちらへどれだけ動かせばよいかがそのまま読める。終了コードは PASS/FAIL に関わらず 0（ここでは `exit 0` が別途 PASS）だが、`sf sils scenario` コマンド自体の終了コードは **FAIL のとき 2** になる（CI 等でそのまま使える）。

しきい値の決め方の基本は「まず `.expect` 無し、または緩い値で1回走らせて実測値を見てから、実測に余裕（マージン）を持たせて確定する」— 期待値を先に決め打ちしてから帳尻を合わせない。

## 5. GUI との関係

`sf sils gui`（ブラウザで動く SILS 実験環境）の「シナリオ作成」タブでもイベントを表形式で組んで `.scn` を保存できる（保存先は `scenarios/`、`.scn` のみ）。**`.expect` を作成する画面は無い** — GUI サーバ（`gui/server.py`）は保存済みシナリオを走らせるとき、同名の `<name>.expect` が `scenarios/` に存在すればそれを自動適用するだけで、`.expect` の生成・編集機能は持たない。合否判定を作りたいときは本書の手順どおり手書きする。

GUI 上で「保存せずに編集中のイベント列のまま実行」した場合（カスタム実行）は一時ファイルに書き出されるため対応する `.expect` が存在せず、判定は「入力注入＋終了コード」のみになる（3節の「`.expect` が無い場合」と同じ扱い）。

## 6. よくある間違い

コードを読んで確認できたものだけを挙げる。

| 間違い | 何が起きるか |
|--------|------------|
| 絶対時刻が直前イベントの終了時刻より前 | パースエラーで起動しない（`event starts before the previous one ends`）。基本は `+` を使い、絶対時刻は本当に必要なときだけにする |
| `rc` の `alt`/`acro`/`pos` を、`hold_ms`/`rate_hz` を省略したまま指定しようとする | 位置引数なので届かない（`alt` は6番目のトークン）。フラグを立てたい行は `hold_ms`・`rate_hz` まで含めて完全な形で書く |
| `rc`/`rc_ramp`/`fault`/`bias`/`wind` の引数の並びを間違える（例: `roll`と`pitch`を逆に置く） | パーサはトークンの型（数値かどうか）しかチェックしないので、数値として妥当なら黙って通り、意図と違う軸が動く。ヘッダコメントの列見出し（`#  <t>  ch  <thr> <roll> <pitch> <yaw> ...`）と付き合わせて書く |
| `.expect` の `metric ... in <t0> <t1>` の単位を `.scn` と同じミリ秒だと思い込む | `.scn` の時刻は**ミリ秒**、`.expect` の `in` の窓は**秒**。6500ms のつもりで `in 6500 7500` と書くと、フライトログ一式の `timestamp_us` 列（マイクロ秒の絶対仮想クロック。`in` の秒数を ×1e6 して照合）と単位が合わず範囲外＝空窓で FAIL になる |
| `.expect` の文字列アサーションで引用符を書き忘れる、または閉じ忘れる | `log_contains`/`log_absent`/`order` の引数はクォート付き文字列が前提（`shlex` で分割）。閉じ忘れると `ValueError` になり `bad assertion` として FAIL する |
| `metric` の名前を綴り間違える（例: `tiltmax`、`Alt_Mean` の大文字化） | 名前は完全一致・大文字小文字を区別。一致しないと「未知のメトリクス」扱いで `None` が返り、常に FAIL する |
| `xfail:` 行を先頭以外（他のアサーションの後など）に書く | `_read_xfail` はファイル中で最初に現れる非空・非コメント行だけを見るので、そこが `xfail:` でなければ既知失敗マーカーとして認識され**ない**。一方で `_eval_expect` 側は `xfail:` で始まる行を見つけるたびにアサーションとしての評価をスキップするので、「エラーにはならないが `sf sils regression` の KNOWN-FAIL 集計にも乗らない」という気づきにくい状態になる。`xfail:` は必ずファイルの一番上（コメント行より後でもよいが、他のアサーション行より前）に置く |
| 空の `.expect`（コメントだけ、または `skip` だけ）を書く | 評価対象のアサーションが1つも無いと自動的に FAIL になる（`all([])==True` の落とし穴を避けるための仕様） |

---

<a id="english"></a>

## 1. Overview

### About this document

In the SILS (Software-in-the-Loop — running the unmodified vehicle firmware in closed loop with a PC-side physics simulation) bench, the input fed to the vehicle and the pass/fail judgment on the result are each a plain text file:

| File | Role | Content |
|------|------|---------|
| `<name>.scn` | Input script — "what to feed the vehicle, and when" | Timed events (stick values, API commands, wind, faults, …) |
| `<name>.expect` | Pass/fail assertions — "what counts as passing" | Log-string presence/order, exit code, and numeric thresholds from the run's flight-log bundle (`.sflog.zip` — a zip-format log file bundling one run's signals; authoritative spec `protocol/spec/flight_log.yaml`) |

Both are UTF-8 plain text, kept as a matching pair (same stem, different extension) under `simulator/sils/scenarios/`. Without an `.expect` file, the verdict is only "was input actually injected" plus "did the process exit 0" (see below).

### Audience

Readers who have already run an existing scenario with `sf sils scenario` and now want to author their own scenario and its assertions. This document plus the real examples under `simulator/sils/scenarios/` should be enough — no need to reverse-engineer the schema (the structure/format of the data) from scratch.

## 2. Writing a `.scn` file

### Line structure

One event per line. Blank lines and anything after `#` are ignored.

```
<time>  <channel>  <args...>   # comment
```

### The three time forms

| Form | Meaning |
|------|---------|
| `0` | Absolute time 0 ms (scenario start) |
| `<number ms>` (e.g. `5000`) | Absolute time, milliseconds from the scenario's own start |
| `+` (bare plus) | Immediately after the previous event ends (same as `+0`) |
| `+<number ms>` (e.g. `+500`) | That many milliseconds after the previous event ends |

**Note:** an absolute time earlier than the previous event's end time is a parse error (the driver is single-threaded and fires events strictly in order — it cannot "catch up" to a missed absolute time). An `rc` event's `hold_ms` pushes its own end time forward by that much, so when mixing absolute times, keep track of when the prior event actually ends. When in doubt, use `+` — it always stays consistent automatically.

### Event channels

| Channel | Args | Units / notes |
|---------|------|---------------|
| `rc` | `<thr> <roll> <pitch> <yaw> <arm> [hold_ms] [rate_hz] [alt] [acro] [pos]` | Simulated transmitter sticks — see next section |
| `rc_foreign` | same as `rc` | Injects the same content from a **different (unpaired) transmitter MAC** — used to test the crosstalk filter |
| `rc_ramp` | `<throttle\|roll\|pitch\|yaw> <from> <to> <step> <rate_hz> <arm> [alt] [acro]` | Sweeps ONE axis from `from` to `to` in `step` increments (other axes stay centred at 2048). `step` is unsigned (direction is inferred from from/to); `rate_hz` is 1..1000 |
| `key` | `"<text>"` | Feeds the text straight into the firmware's console (CLI). Supports `\n \t \r \\ \"` escapes |
| `api` | `"<command line>"` | Injects one Tello-style API command line directly into the firmware's ApiTask parser (e.g. `"forward 50"`). Only works on targets with an ApiTask (`vehicle`/`workshop`); on other targets it is silently recorded as unsupported |
| `wind` | `<fx> <fy> <fz> [dur_ms]` | External disturbance force on the craft, NED [N]. With `dur_ms` it is a gust pulse (reverts to 0,0,0 afterward); without it, the force stays applied |
| `fault` | `<motor 0-3> <gain 0-1>` | Degrades one motor's thrust health (1.0 = healthy, 0.0 = dead), sustained |
| `bias` | `<ax> <ay> <az> <gx> <gy> <gz>` | Injects a deterministic raw IMU bias (body FRD = Forward-Right-Down). Accel [m/s²], gyro [rad/s] |
| `handle` | `<carry_alt_m> <place_x> <place_y> <lift_ms> <carry_ms> <place_ms>` | Simulates a human hand-carry maneuver (lift→right→carry→place). `carry_alt_m>0` and each duration `>0` are required. Fires instantaneously on the timeline; a following disarmed `rc` hold occupies the real time it takes |
| `btn` | (reserved, not implemented) | Currently just warns and is skipped (GPIO button simulation is a future feature) |

### `rc` line in detail (the most important one)

```
<t>  rc  <thr> <roll> <pitch> <yaw> <arm> [hold_ms] [rate_hz] [alt] [acro] [pos]
```

- `thr`/`roll`/`pitch`/`yaw`: raw 12-bit ADC values, **range 0..4095, centre 2048** — exactly what a real transmitter stick sends
- `arm`: 0 or 1. The firmware's state machine treats a rising-then-falling edge on the ARM bit as a toggle (the same convention a real momentary push button uses), so to ARM you make one 0→1 transition, and to DISARM you make another one (i.e. release then press again)
- `hold_ms` (default 0): how long to hold this stick value, in ms. 0 = a single frame
- `rate_hz` (default 20): send rate [Hz], 1..1000. `hold_ms` is rounded to this period; any positive hold always emits at least one frame
- `alt` (default 0): 1 sets the ALTITUDE_HOLD flag
- `acro` (default 0): 1 sets the ACRO (rate) mode flag
- `pos` (default 0): 1 sets the POSITION_HOLD flag

**These trailing fields are positional:** `alt`/`acro`/`pos` can only appear AFTER `hold_ms` and `rate_hz`. To set only `acro`, you must still spell out `hold_ms` and `rate_hz` explicitly (write the full form).

```
# good: hold_ms/rate_hz are explicit so acro (0 1) lands in the right slot
+  rc  2048 2048 2048 2048  1  1000  50   0  1
```

### When an event "ends" (the basis for `+`)

`+` resolves relative to when the previous event ends, which depends on its channel:

- `rc`: `hold_ms` later (0 consumes no time — single frame)
- `rc_ramp`: frame count needed for the sweep × its period
- `wind`: only if `dur_ms` was given (gust pulse); a sustained step (no `dur_ms`) is instantaneous
- everything else (`key`/`api`/`fault`/`bias`/`handle`/`btn`): instantaneous (0 ms) on the timeline

## 3. Writing a `.expect` file

### Line kinds

| Kind | Syntax | Meaning |
|------|--------|---------|
| `xfail: <reason>` | Valid **only as the file's first non-blank, non-comment line** | Marks a scenario as a known-tracked failure. Does not change `sf sils scenario`'s own raw PASS/FAIL display, but `sf sils regression` reports it as `[KNOWN-FAIL]` (excluded from the gate count) and flags `[XPASS]` if it starts passing while the marker is still there |
| `exit <code>` | e.g. `exit 0` | Process exit code matches |
| `log_contains <out\|err\|any> "<text>"` | e.g. `log_contains any "ARM accepted"` | The named stream (stdout / stderr / both) contains the text |
| `log_absent <out\|err\|any> "<text>"` | | The text is **not** present (e.g. confirming no error was logged) |
| `order "<a>" "<b>"` | | `a`'s first occurrence precedes `b`'s first occurrence (checked against the merged stdout+stderr text) |
| `metric <name> <op> <value> [in <t0> <t1>]` | e.g. `metric tilt_max < 0.314 in 7.0 13.0` (≈18°) | Numeric threshold on a value computed from the run's flight-log bundle (streams such as `truth.csv`). `op` is one of `< <= > >=` only (no `==`/`!=`). Optional `in <t0> <t1>` restricts to a window in **seconds**; omitted = the whole run |
| `skip <reason...>` | | Records the line as passing but not actually evaluated (for hardware-gated checks) |
| `# comment` | | Ignored |

**An `.expect` with zero real assertions to evaluate FAILS automatically** — this is deliberate, to avoid the "empty file passes vacuously" trap (`all([]) == True` in Python).

### Metric names

Computed from the streams inside the run's flight-log bundle (the zip-format log file `<...>.sflog.zip`; the same packet-type CSVs also readable with `sf log check` etc.): `truth.csv` (the simulator's physical ground truth — position, attitude quaternion, velocity, angular rate), `attitude.csv`/`posvel.csv` (firmware estimates), and `motor.csv` (motor duty). **The angular values (`roll_rmse`/`pitch_rmse`/`att_rmse`/`tilt_max`/`yaw_band`) are in SI radians** here (per `protocol/spec/flight_log.yaml`, not degrees). Attitude is stored as the quaternion columns `quat_w/x/y/z` and converted to Euler roll/pitch/yaw internally before these metrics are computed. `sf sils scenario`'s console output appends a `(x deg)` conversion alongside the radian value for readability.

| Name | Unit | Meaning |
|------|------|---------|
| `alt_mean` | m | mean altitude over the window |
| `alt_min` | m | min altitude over the window |
| `alt_max` | m | max altitude over the window |
| `alt_band` | m | peak-to-peak altitude (max − min) over the window |
| `alt_rmse` | m | RMSE between the estimate (`posvel.csv`) and the truth altitude (`truth.csv`) |
| `roll_rmse` | rad | RMSE between the estimate (`attitude.csv`) and the truth roll (`truth.csv`) |
| `pitch_rmse` | rad | RMSE between the estimate (`attitude.csv`) and the truth pitch (`truth.csv`) |
| `att_rmse` | rad | combined attitude error magnitude (`hypot` of the two RMSEs above) |
| `tilt_max` | rad | max true tilt magnitude `hypot(roll, pitch)` over the window (no-tumble check) |
| `yaw_band` | rad | peak-to-peak true heading (from the truth quaternion, unwrapped across the ±180° seam) over the window |
| `duty_max` | ratio (0..1) | max of the four motors' duty over the window (saturation check) |
| `horizontal_drift_max` | m | max planar distance from the window's **start** point |

An unknown name, a bundle missing the needed stream, or an empty window all resolve to "unjudgeable" and **FAIL** (the underlying function returns `None`, and no comparison is made).

### Relying on determinism

The whole `.scn`/`.expect` scheme rests on SILS's determinism: the same input yields byte-identical output every time. Log text/order and the flight-log bundle's numbers are therefore exactly reproducible (as long as noise stays off, the default), so thresholds don't need slack for run-to-run variance — existing thresholds in this repo are set from "the measured value plus a small margin," the same method demonstrated in Step 4 below.

## 4. Worked example — roll step response, overshoot and settling

Below is a real run: a STABILIZE-mode (self-levelling — the craft tracks a commanded attitude angle, independent of vertical-axis estimation accuracy) roll-stick step, judged on how far the attitude overshoots the commanded angle and how well it settles afterward. All numbers below came from actually running `sf sils scenario`.

### (1) Write `my_roll_step.scn`

Based on `stab_flight.scn`'s (`simulator/sils/scenarios/`) ARM→takeoff→step flow, trimmed down to a single roll axis.

```
# my_roll_step.scn — roll step response (STABILIZE)
#
#  <t>  ch  <thr> <roll> <pitch> <yaw> <arm> <hold_ms> <rate_hz>
   0    rc  2048  2048   2048    2048  0     4000      50   # A: disarmed 4s, boot calibration
   +    rc  2048  2048   2048    2048  1     500       50   # B: ARM
   +    rc  3243  2048   2048    2048  1     2000      50   # C: throttle up -> TAKEOFF -> FLYING, climb
   +    rc  3176  2600   2048    2048  1     1000      50   # D: ROLL STEP +8 deg (right)
   +    rc  3176  2048   2048    2048  1     1500      50   # E: back to centre, overshoot/settle window
   +    rc  2048  2048   2048    2048  0     200       50   # F: release ARM button
   +    rc  2048  2048   2048    2048  1     200       50   # G: press again -> DISARM
   +    rc  2048  2048   2048    2048  0     600       50   # H: settle disarmed
```

- A: 4 s disarmed, letting boot-time sensor calibration finish
- B: ARM rising edge (after 0.5 s idle)
- C: throttle up past the takeoff threshold (norm 0.5, raw 2048+2048×0.5=3072) to raw 3243 — drives ARMED_GROUND→TAKEOFF→FLYING/STABILIZE and climbs
- D: throttle down to near-hover raw 3176 while stepping the roll stick to raw 2600 (≈+8°) for 1 s
- E: roll stick back to centre (0° command), 1.5 s to watch overshoot and settling
- F–G: release then press the ARM button = toggle DISARM
- H: let it fall/settle after DISARM

The `thr` raw values (3243 = climb burst, 3176 ≈ hover) are the measured values reused from `stab_flight.scn`. If you're using a different motor/airframe setup, first run a scenario that holds the sticks centred for a few seconds, measure the `thr` at which the altitude in the flight-log bundle's `truth.csv` (NED, so altitude is `-pos_z`) stays roughly flat, and use that (the same "run it first, then measure" method repeats in step 4).

### (2) Run it and look at the output

```bash
source setup_env.sh
sf sils scenario simulator/sils/scenarios/my_roll_step.scn --target vehicle
```

With no `.expect` yet, the verdict is only injection + exit code (actual output):

```
[INFO] (no .expect at my_roll_step.expect — verdict = injection + exit code)
[INFO] scenario my_roll_step.scn: PASS (exit 0, 2 checks, events=events.jsonl)
  [PASS] input injected (events.jsonl non-empty)  (38441 bytes)
  [PASS] exit == 0  (got 0)
[OK] bundle: simulator/sils/viz/out_scn_my_roll_step
```

The full result bundle (console log, flight-log bundle `sils_my_roll_step_<YYYYMMDD>T<HHMMSS>.sflog.zip`, `events.jsonl`) lands in `simulator/sils/viz/out_scn_my_roll_step/`. In the bundle's `truth.csv` (attitude stored as the quaternion `quat_w/x/y/z`, converted to Euler roll here), the roll angle starts rising right after event D (absolute scn time 6500 ms = 6.5 s), overshoots the commanded +8° (≈0.140 rad) up to **about 0.194 rad (≈11.1°)** before coming back, and after E returns the stick to centre it settles with a **residual offset of about 0.078 rad (≈4–5°)** by the time DISARM fires (scn time 9200–9400 ms = 9.2–9.4 s). (Angular metrics such as `tilt_max`/`roll_rmse` are SI radians.)

### (3) Write `my_roll_step.expect`

Turn the measured overshoot and residual settling values from (2) into thresholds with margin.

```
exit 0
log_contains any "ARM accepted"
log_contains any "Takeoff detected"
log_contains any "Takeoff complete"
log_contains any "DISARM accepted"
order "ARM accepted" "Takeoff detected"
order "Takeoff detected" "Takeoff complete"
order "Takeoff complete" "DISARM accepted"
metric tilt_max > 0.140 in 6.5 7.5     # step window: overshoots the commanded 8 deg (≈0.140 rad)
metric tilt_max < 0.349 in 6.5 7.5     # but stays bounded (no tumble, ≈20 deg)
metric tilt_max < 0.105 in 8.7 9.0     # settled to within a few degrees before DISARM (≈6 deg)
metric duty_max  < 0.90 in 6.5 9.0     # motors not saturated
```

Note that `tilt_max` takes a threshold in radians (per `protocol/spec/flight_log.yaml`; the console output appends a `(x deg)` conversion). Also note that `in` windows are in **seconds** (the `.scn` timings are in milliseconds). Event D runs 6500–7500 ms on the scn timeline, hence `in 6.5 7.5`; the tail of E (just before DISARM) is around 8700–9000 ms, hence `in 8.7 9.0`.

### (4) Re-run and confirm PASS

```bash
sf sils scenario simulator/sils/scenarios/my_roll_step.scn --target vehicle
```

Actual output obtained:

```
[INFO] scenario my_roll_step.scn: PASS (exit 0, 13 checks, events=events.jsonl)
  [PASS] input injected (events.jsonl non-empty)  (38441 bytes)
  [PASS] exit == 0  (got 0)
  [PASS] log_contains any 'ARM accepted'  (found)
  [PASS] log_contains any 'Takeoff detected'  (found)
  [PASS] log_contains any 'Takeoff complete'  (found)
  [PASS] log_contains any 'DISARM accepted'  (found)
  [PASS] order 'ARM accepted' before 'Takeoff detected'  (idx_a=10577 idx_b=10860)
  [PASS] order 'Takeoff detected' before 'Takeoff complete'  (idx_a=10860 idx_b=11074)
  [PASS] order 'Takeoff complete' before 'DISARM accepted'  (idx_a=11074 idx_b=11181)
  [PASS] metric tilt_max > 0.14 in [6.5,7.5]  (tilt_max=0.1944 (11.14 deg))
  [PASS] metric tilt_max < 0.349 in [6.5,7.5]  (tilt_max=0.1944 (11.14 deg))
  [PASS] metric tilt_max < 0.105 in [8.7,9.0]  (tilt_max=0.0784 (4.49 deg))
  [PASS] metric duty_max < 0.9 in [6.5,9.0]  (duty_max=0.7098)
[OK] bundle: simulator/sils/viz/out_scn_my_roll_step
```

Add `--video` to render a review MP4 (MuJoCo 3D + state graphs) on PASS (`sf sils scenario ... --video`).

### (5) Reading a FAIL

As a demonstration, running with an `.expect` containing only an overly tight `tilt_max < 0.087 in 6.5 7.5` (≈5°, clearly too strict against the measured 11.1°):

```
[ERROR] scenario FAILED — see .../console.log
[INFO] scenario my_roll_step.scn: FAIL (exit 0, 3 checks, events=events.jsonl)
  [PASS] input injected (events.jsonl non-empty)  (38441 bytes)
  [PASS] exit == 0  (got 0)
  [FAIL] metric tilt_max < 0.087 in [6.5,7.5]  (tilt_max=0.1944 (11.14 deg))
```

The `[FAIL]` line carries the **measured value** (`tilt_max=0.1944`, with the deg conversion in parentheses), so you can see directly which way — and by how much — to move the threshold. The exit code of the process itself is 0 either way here (`exit 0` passed separately), but `sf sils scenario`'s own process exit code is **2 on FAIL** (usable directly in CI).

The rule of thumb for setting thresholds: run once first (with no `.expect`, or a loose one) to see the measured value, THEN add margin and fix the threshold — never guess the expected value first and try to make the run match it.

## 5. Relationship with the GUI

`sf sils gui` (the browser-based SILS workbench) can also build events in a table and save a `.scn` from its "シナリオ作成" (scenario builder) tab (saved to `scenarios/`, `.scn` only). **There is no screen for creating an `.expect`** — the GUI server (`gui/server.py`) only auto-applies a same-named `<name>.expect` in `scenarios/` if one already exists when running a saved scenario; it has no `.expect` generation/editing feature. Write assertions by hand, following this document.

Running the GUI's in-progress (unsaved) event list ("custom run") writes to a scratch file with no matching `.expect`, so the verdict is injection + exit code only — the same fallback described in Section 3.

## 6. Common mistakes

Only mistakes confirmed by reading the code are listed.

| Mistake | What happens |
|---------|---------------|
| An absolute time earlier than the previous event's end | Parse error, won't run (`event starts before the previous one ends`). Default to `+`; use absolute times only when you really need to |
| Setting `rc`'s `alt`/`acro`/`pos` while omitting `hold_ms`/`rate_hz` | Doesn't reach them — they're positional (`alt` is the 6th token). Write the full form (through `hold_ms`/`rate_hz`) whenever you need a trailing flag |
| Swapping argument order in `rc`/`rc_ramp`/`fault`/`bias`/`wind` (e.g. `roll` and `pitch` reversed) | The parser only checks that a token is numeric, so a swap silently passes and moves the wrong axis. Cross-check against the header comment's column labels (`#  <t>  ch  <thr> <roll> <pitch> <yaw> ...`) |
| Assuming `.expect`'s `metric ... in <t0> <t1>` uses the same milliseconds as `.scn` | `.scn` times are **milliseconds**; `.expect`'s `in` window is **seconds**. Writing `in 6500 7500` meaning 6500 ms doesn't match the flight-log bundle's `timestamp_us` column (a microsecond absolute virtual clock — the `in` seconds are multiplied by 1e6 to compare) — the window ends up empty and FAILs |
| Forgetting or mismatching quotes in `.expect` string assertions | `log_contains`/`log_absent`/`order` arguments must be quoted strings (parsed with `shlex`). An unterminated quote raises `ValueError`, recorded as a `bad assertion` FAIL |
| Misspelling a `metric` name (e.g. `tiltmax`, or capitalizing `Alt_Mean`) | Names are matched exactly, case-sensitive. A mismatch is treated as "unknown metric" → `None` → always FAILs |
| Placing `xfail:` somewhere other than the first line (e.g. after other assertions) | `_read_xfail` only looks at the file's first non-blank, non-comment line — if that isn't `xfail:`, the marker is **not** recognized as a known-fail marker. Meanwhile `_eval_expect` still skips any line starting with `xfail:` as a non-assertion wherever it appears, so the result is an easy-to-miss state: no error, but also no `[KNOWN-FAIL]` credit in `sf sils regression`. Always put `xfail:` at the very top (comment lines before it are fine; other assertion lines must come after it) |
| An empty `.expect` (comments only, or `skip` only) | Automatically FAILs when there are zero real assertions to evaluate (a deliberate guard against the `all([]) == True` trap) |
