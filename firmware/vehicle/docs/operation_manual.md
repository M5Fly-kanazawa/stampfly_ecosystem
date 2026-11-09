# StampFly vehicle 操作マニュアル / Operation Manual

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

vehicle ファームウェアの**起動シーケンス・LED/音の意味・コントローラ操作・ペアリング手順・
CLI コマンド**をまとめた実機運用マニュアル。UI（LED/ブザー）と操作系は旧 vehicle ファームを踏襲する。

### 対象読者

実機（M5StampFly）を起動・操作する人、ブリングアップ（development_roadmap Phase 2/3）を行う人。

### 安全上の注意

- **モータ単体テスト（CLI `motor`）は必ずプロペラを外すか機体を保持して行う。**
- ARM/離陸は周囲の安全を確認してから。**ACRO/STABILIZE 飛行中・地上**はコントローラの DISARM（または機体ボタン）で即停止。**ALT_HOLD/POS_HOLD 飛行中**の DISARM は自動着陸（緩降下）になるため、即停止したいときは**もう一度 DISARM**（または緊急停止）する。

## 2. 起動シーケンス

電源 ON 後、ファームは Phase 0〜5 を順に実行し、LED と音で状態を知らせる。

### 起動フェーズと LED/音

| 段階 | 内容 | StampS3 LED（状態, GPIO21） | 音 |
|------|------|---------------------------|-----|
| Phase 0 | NVS 初期化 | — | — |
| Phase 1 | BSP（sf_board）: I2C/SPI/LEDC バス | — | — |
| Phase 2 | Pub-Sub トピック初期化 | — | — |
| Phase 3 | パラメータ読込（NVS or 既定） | — | — |
| Phase 4 | 16 タスク起動 → NotifyTask 起動 | **白 常灯**（INIT） | **startTone**（ドミソ: C5→E5→G5）|
| Phase 5 | アプリフック `sf::app::start()`（自作アプリの追加タスク起動。アプリ無しなら何もしない） | — | — |
| — | INIT → IDLE_GROUND（IMU が有効値を出す）| 白 → 緑/マゼンタへ | — |
| 校正 | 起動バイアス校正（静止確認済 2.5 秒分を平均。動かすとやり直し）| **マゼンタ 低速点滅** | （無音）|
| 完了 | 校正完了 → ARM 可能 | **緑 常灯** | **readyTone**（ピッ×3）|
| ペア | 未ペアなら自動ペアリング待機 | **青 高速点滅** | **pairingTone**（C5→G5）|

**ポイント:**
- 起動音（ドミソ）が鳴り**白点灯**＝電源 ON・初期化開始。
- **マゼンタ点滅中は静止**（バイアス校正中）。動きを検出すると蓄積を破棄してやり直すため、機体を置いて LED が緑になる（＋ピッ×3）まで待つ。
- **緑常灯＋ピッ×3**＝校正完了・ARM 可能。
- **青高速点滅**＝送信機を探索中（ペアリング待ち、§5 参照）。
- Critical なハード故障時は同じ StampS3 LED が**赤 高速点滅**して停止する（sf_board の致命停止経路）。

## 3. LED / 音 リファレンス

LED は**2 チャネル**に分かれる:

| チャネル | LED | 役割 |
|---------|-----|------|
| **モード色** | 本体 LED（基板の表裏 ×2、常に同一表示） | フライトモードの色。**常に実モード**を表示: 地上（DISARM/ARM とも）＝常灯、飛行中＝低速点滅 |
| **システム状態** | StampS3 内蔵 LED（GPIO21 ×1） | INIT/校正/ペアリング/IDLE/ARMED/離陸/着陸などの状態表示 |

基板の表裏は機体姿勢でどちらかが見えなくなるため同一表示とし、StampS3 LED を独立した状態表示に使う。

### モード色（本体 LED, GPIO39×2）

| モード | 色 | 地上（DISARM/ARM とも） | 飛行中 |
|--------|-----|------------------------|--------|
| ACRO | 青 | 常灯 | 低速点滅 |
| STABILIZE | 黄緑 | 常灯 | 低速点滅 |
| ALT_HOLD | オレンジ | 常灯 | 低速点滅 |
| POS_HOLD | マゼンタ | 常灯 | 低速点滅 |
| （INIT 中）| 白 | 常灯 | — |
| **低電圧**（最優先）| シアン | 高速点滅 | 高速点滅 |

地上での ARM/DISARM の区別は StampS3 LED（下記: ARMED_GROUND＝緑低速点滅、IDLE_GROUND＝緑常灯）が担う。

- **モード変更は地上（DISARM/ARM 中）でも受理される**（仕様 2026-06-11）。スイッチを切り替えると実モードと LED が即座に変わる。
- ALT_HOLD/POS_HOLD で ARM してもプロペラは回らない（制御器が接地中の推力をゼロに遮断）。これらのモードの飛行開始は**自動離陸**（下記 §4）。

### システム状態（StampS3 内蔵 LED, GPIO21）

優先度は上から（上位が下位を上書き）: 低電圧 > **ARM 中の下向き ToF 出力停止（手動飛行の警告）** > ペアリング > ARM 阻害表示（地上のみ。傾き不一致 / 待機 / 下向き ToF 出力停止）> 飛行状態。

| 状態 | 色 | パターン |
|------|-----|---------|
| 低電圧（電池電圧が `safety.battery.low_v` 以下）| シアン | 低速点滅 |
| ペアリング中（探索中）| 青 | 高速点滅 |
| **傾き不一致**（地上で姿勢推定が重力とずれている。**持ち上げて水平な地面に置き直す**）| **赤** | **低速点滅** |
| **待機**（地上。校正中〔**静止するまで完了しない**〕・傾き判定待ち〔リセット/DISARM 後の約 1 秒〕・モータテスト後の再水平化待ち〔約 1.5 秒〕。**動かさず待つ**）| マゼンタ | 低速点滅 |
| **下向き ToF 出力停止**（地上。高度センサ〔ToF〕が新しい測定値を出さない、または出力が戻った後に推定器が取り込み直す前〔約 1 秒〕。**センサを確認し、電源を入れ直す。復帰後も約 1 秒待つ**）| **黄** | **高速点滅** |
| **下向き ToF 出力停止**（ARM 中。新しい測定値が来ない・空中で無効な測定値だけが続く〔飛行中は 0.5 秒、離陸中は 5 cm 上がってから 0.15 秒（上昇量が見えないときは、離陸の開始から 2 秒たったとき）〕・復帰後に推定器が取り込み直す前〔約 1 秒〕。**ACRO・STABILIZE では強制着陸されない**ので自分で着陸させる。ALT_HOLD・POS_HOLD では自動で着陸に入る。本体 LED・MCU LED の両方）| **黄** | **高速点滅**（警告音も鳴る）|
| INIT（初期化中）| 白 | 常灯 |
| IDLE_GROUND（校正済・ARM 可）| 緑 | 常灯 |
| IDLE_HELD（手持ち検出）| シアン | 高速点滅 |
| ARMED_GROUND（ARM 済・地上）| 緑 | 低速点滅 |
| TAKEOFF（離陸シーケンス）| 白 | 高速点滅 |
| FLYING（飛行中）| 緑 | 常灯 |
| LANDING（着陸シーケンス）| オレンジ | 低速点滅 |
| Critical ハード故障（停止）| 赤 | 高速点滅 |

赤の**低速**点滅（傾き不一致＝置き直せば復帰）と赤の**高速**点滅（Critical 故障＝停止、または autotune 失敗の合図）は別物。緑常灯（IDLE_GROUND）は ARM 前判定を全て通過＝ARM 可能を意味する。

### イベント → 音（ブザー）

| イベント | 音 | 音列 |
|---------|-----|------|
| 起動 | startTone | C5→E5→G5（ドミソ）|
| 校正完了（ARM 可能）| readyTone | C5 ×3（ピッピッピッ）|
| ARM | armTone | E5→G5（上昇）|
| DISARM | disarmTone | G5→E5（下降）|
| ペアリング開始 | pairingTone | C5→G5 |
| 低電圧警告 | lowBatteryWarning | A4 …（繰返し）|
| エラー（フェイルセーフ）| errorTone | C4（長く低い）|
| **ARM 拒否**（ARM 操作が ARM 前判定で拒否された。理由の別なく同じ音。理由は LED と CLI `status` の `prearm` 行）| errorTone | C4（長く低い）|

ブザーは CLI `sound off` で無効化できる（NVS 保存）。LED 輝度は `led <0-255>`。

## 4. コントローラ操作

コントローラ（送信機）は固定ファーム。機体は SSOT プロトコル（`protocol/spec/messages.yaml` の
ControlPacket 14B）に準拠してデコードする。**操作は旧 vehicle と同一。**

### スティック割当

スロットルは**バネ復帰式**（離すと中央 raw 2048 に戻る）。**モードで解釈が変わる**:

| スティック | 役割 | 値（12bit ADC, 中央2048）|
|-----------|------|------------------------|
| スロットル（STABILIZE/ACRO）| 推力 | 中央2048=推力0（離すと降下）、上端4095=最大。上半分のみ使用（下半分は0クリップ）|
| スロットル（ALT_HOLD/POS_HOLD）| 目標高度を上下 | **中央2048=現在高度ホールド**、上=上昇（`altitude.climb_rate`）、下=降下（`altitude.descent_rate`）。対称。|
| ロール | 左右傾斜 | 中央2048、±でロール |
| ピッチ | 前後傾斜 | 中央2048、±でピッチ |
| ヨー | 旋回 | 中央2048、±でヨーレート |

中央付近はデッドバンド（±0.05）で 0 に丸める。

### ARM / DISARM

| 操作 | 動作 |
|------|------|
| コントローラの ARM スイッチ ON（flags bit0 立ち上がり）| IDLE_GROUND → ARMED_GROUND（ARM）。校正完了・電圧正常が条件 |
| DISARM 操作（地上 / ACRO・STABILIZE 飛行中）| → IDLE_GROUND。**即カット**（モータ停止）|
| DISARM 操作（**ALT_HOLD/POS_HOLD で飛行中**）| → **自動着陸**（FLYING → LANDING）。空中でモータを切らず緩降下（0.3 m/s）で接地し、接地検出で本当の DISARM。**着陸中にもう一度 DISARM すると即カット（中断）** |
| 機体ボタン クリック（地上）| ARM/DISARM トグル（飛行中のクリックは無視）|

**ARM できない時**: ARM 操作（RC ボタン・機体ボタン・API）が拒否されると**エラー音（errorTone）が 1 回**鳴り、拒否中は LED が理由を示す（赤低速点滅＝傾き不一致、マゼンタ低速点滅＝待機、青高速点滅＝ペアリング中、シアン低速点滅＝低電圧、黄高速点滅＝下向き ToF 出力停止）。CLI `status` の `prearm` 行は `prearm  : ready` または `prearm  : BLOCKED (tilt mismatch - place level)` のように理由を 1 行で示す（WiFi CLI でも可）。拒否の対象: 校正中（マゼンタ点滅）・低電圧/USB 給電・**ペアリング中**・ベンチ用モータテスト中/テスト後の再水平化待ち（約 1.5 秒）・**下向き ToF（高度センサ）の出力停止のエピソード中**（新しい測定値が 0.3 秒以上来ない、空中で無効な測定値だけが続いた〔飛行中は 0.5 秒、離陸中は 5 cm 上がってから 0.15 秒（上昇量が見えないときは、離陸の開始から 2 秒たったとき）〕、または出力が戻っても推定器がまだ ToF を取り込み直していない〔測定値が 1 秒以上途切れず届き、かつ推定が ToF と 10 回連続で一致するまで、復帰の約 1 秒後〕。**ACRO を含む全モードで拒否**。測定値が「届いた」だけでは許可されない）（`ARM rejected: downward ToF is silent (理由)`、CLI `prearm  : BLOCKED (downward ToF silent / not re-acquired - ...)`。飛行中に出力停止した場合は、**ALT_HOLD・POS_HOLD・自動離陸では上昇を中止して自動で着陸する**（降下は高度を使わずに降りる。**推力の上限は、機体を支えていた推力（支持推力。加速度計で較正する）の 0.95 倍で、重さより小さいので、機体は上昇できない**。降下は、速度を 0.4 m/s に保つ速度制御ではなく、実質は**推力の上限による一定の下向き加速度（約 0.05 g = 0.49 m/s²）の降下**で、降りるほど速くなる（1.2 m の高さから約 1.1 m/s で接地する）。加速度計から推定した鉛直速度（最後のリセットから 2 秒だけ信用する）は、上昇を止める（推力を下げる）ためと降下の最初の加速のために使い、降下が速すぎても、推力を上限より上げて止めには行かない（機体が上昇しうるため）。さらに、降下が 5 秒続いたら推力を絞り始め、8.3 秒でゼロにする。接地は加速度計: 2 g 超の衝撃のあと 0.25 秒間、加速も降下もしていなければ着陸完了、または床が機体を支えている状態（推力が支持推力の 0.7 倍未満なのに加速度計が 1 g を示す）が 0.5 秒続けば着陸完了。ただし通常は 3 G の衝撃カットが先に働いて DISARM する（アラートの種別は ToF 出力停止中の LANDING では非常にならず、接地として扱う）。LANDING に入ってから 10 秒たち、かつ ToF の出力停止が 4 秒続いても接地を検出できないときは、時間切れで着陸完了とし、モータを止める（この時点で推力はすでにゼロで、機体は接地しているか自由落下中。ToF が生きたまま 10 秒を超えた LANDING の後に止まったときも、少なくとも 4 秒は降下させてからモータを止める）。接地速度は SILS の実測（6 回目の見直し。場面が成立した 870 本）で、センサノイズなしの最大が 0.89 m/s（離陸直後に出力停止した場合）、ホバー中の出力停止で 0.93〜1.02 m/s、手動上昇中で 1.19 m/s、センサノイズ n2 の最悪が 2.04 m/s と、通常着陸の約 0.3 m/s よりずっと速いので、屋外や人の近くでは避ける。ALT_HOLD・POS_HOLD ではスティックで上げても高度 1.2 m で止まる（高度上限。自動離陸は 0.5 m）ので、落下距離は 1.2 m までに抑えられる。SILS の値で実機は未検証）。**ARMED_GROUND（ARM 済みで地上）で ToF が出力停止したときは、モードを問わず自動で DISARM する**（機体は飛んでいないので、飛ばさない）。**ACRO・STABILIZE では強制着陸せず**、本体の LED（2 系統）が黄の高速点滅になり警告音が鳴るので、パイロットが自分で着陸させる。出力停止中は ALT_HOLD・POS_HOLD への切り替えも拒否され、CLI `status` に `modeblock: REFUSED (...)` と `tof     : SILENT - 理由 ...` が出る。**飛行中に拒否したスイッチは「消費」される**: ToF が復帰しても自動では ALT_HOLD・POS_HOLD に切り替わらないので、復帰後（黄の点滅が消えてから）にスイッチを入れ直す。地上で拒否した切り替えは、エピソードが終わると自動で適用される）・**地上で姿勢推定が重力とずれている間**（`ARM rejected: attitude estimate disagrees with gravity`）は ARM 拒否（CLI `status`
で確認可）。ずれは IDLE_GROUND 中に自動で（滞在ごとに1回）推定器リセットで再水平化される。起動・リセット直後は判定に約 1 秒かかり、その間は `ARM rejected: attitude check not passed yet` で拒否される。ずれが残る（機体が傾いて置かれている等）場合は、LED が**赤の低速点滅**になる。機体を持ち上げて水平な地面に置き直す（置き直しで再校正される）。リセット/DISARM 直後の約 1 秒（傾き判定待ち）とモータテスト後の約 1.5 秒（再水平化待ち）は**マゼンタの低速点滅**で、動かさずに待つと緑に戻る。

**地上での自動 DISARM**: ARM したまま地上（ARMED_GROUND）でプロペラを回し続け、モータ振動で姿勢推定が重力方向から約 10° 以上ずれて 0.5 秒続くと、機体は自動で DISARM（モータ停止）し、ブザーが DISARM 音を鳴らす（ログ: `Disarmed on ground: attitude estimate disagrees with gravity`）。ACRO/STABILIZE ではスロットルを上げれば状態に関係なく浮くため、傾いた推定で離陸させないための措置。DISARM 後は推定器をリセットして再水平化する（再校正はしない）。判定が通るまでの約 1 秒は ARM できない。しきい値は暫定（`safety.tilt_check.max_deg/persist_s/lpf_s`）。


### フライトモード切替（flags）

| flags ビット | モード | 制御 |
|-------------|--------|------|
| （なし）| STABILIZE | 姿勢安定（既定）|
| bit2 = MODE | ACRO | 角速度制御 |
| bit3 = ALT_MODE | ALTITUDE_HOLD | 高度保持 |
| bit4 = POS_MODE | POSITION_HOLD | 位置保持 |

**優先順位: POS_HOLD > ALT_HOLD > ACRO > STABILIZE**（複数立っていれば上位を採用）。

#### ヘディングホールド（STABILIZE 以上）

ヨースティックが中立の間、離した瞬間の機首方位を自動保持する（推定ヨー角への P ループ。地磁気不要）。ヨー外乱があっても向きが流れず、振られても元の方位へ戻る。ヨースティックを動かすと即解除（パイロット優先）、次に中立へ戻した方位を再捕捉する。ACRO では無効（純レート制御）。

| パラメータ | 既定値 | 意味 |
|-----------|--------|------|
| `attitude.yawhold.kp` | 3.0 | 方位 P ゲイン [1/s]。**0 で機能無効** |
| `attitude.yawhold.rate_max` | 2.0 | 補正回頭率の上限 [rad/s] |

### 離陸・着陸

| 操作 | 動作 |
|------|------|
| ARMED_GROUND でスロットルを上げる（>0.5）— STABILIZE/ACRO | **手動離陸**: スロットル＝推力。TAKEOFF → FLYING（ToF が空中検出で確定）|
| **ALT_HOLD/POS_HOLD で ARM する**（スロットル操作不要）| **自動離陸**（再設計 2026-06-14）: ARM 自体がトリガ。短いスプール（0.3秒, プロペラはまだ回らない）の後、**目標高度 0.5m まで**固定 0.3 m/s で上昇（**鉛直のみ自動**）。**上昇中も姿勢はパイロット操縦可** — スティック中立なら水平で真上、ロール/ピッチを倒せば上昇中も傾けて操縦できる（POS は発進点保持）。**0.5m 到達で完了**（行き過ぎでなく目標値 0.5m を保持）。**スロットルを一度中央（raw 2048=バネ静止）に戻すまで**スティックで高度を動かせない（再センターロック＝暴発防止。バネ式は離せば中央に戻り解除）。以降は 中央=保持/上=上昇/下=降下 |
| スロットルを下げる（ALT/POS, 中央より下）| その場から降下（中央で再び保持）|
| **DISARM（ALT/POS 飛行中）**| **自動着陸**: 緩降下で接地まで（FLYING → LANDING → IDLE_GROUND）。途中でもう一度 DISARM すれば即カット |

ALT/POS の自動離陸は**鉛直（高度）だけが自動**で、上昇中も**ロール/ピッチ/ヨーはパイロットが操縦できる**（スティック中立なら水平で真上）。**自動着陸（DISARM 起動）の降下も同じ — 降下中もロール/ピッチ/ヨーで操縦できる**（鉛直だけ自動降下）。ただし**通信途絶のフェイルセーフ着陸**はパイロット不在のため水平降下になる（リンク途絶を検出して自動で水平化）。**飛行中に STABILIZE/ACRO から ALT_HOLD/POS_HOLD へ切り替えた場合も**、切替時の高度を捕捉し、再センターロックが掛かる（スロットルを中央に通すまで高度ジャンプしない）。

### 操作フロー全体

```
電源ON →（自動ペアリング待機: 青点滅）
  → コントローラをペアリングモードに → 成立（緑常灯）
  → 校正完了（ピッ×3, 緑常灯）
  → モードスイッチで飛行モード選択（本体LED が選択モード色で常灯）
  → ARM（armTone, 本体LED モード色常灯のまま / StampS3 が緑点滅に）
  → 離陸: STAB/ACRO=スロットルUP（手動）、ALT/POS=ARM で自動離陸（0.3 m/s で 0.5m まで。飛行中は本体LED 点滅）
  → 飛行中もモードスイッチで切替可（ACRO/STAB/ALT/POS）
  → スロットルDOWN / DISARM → 着陸 → IDLE_GROUND（緑常灯）
```

### Python からのプログラム飛行（Tello 風 API）

機体は UDP :8889 で Tello SDK 互換のテキストコマンドを受け付ける（`tools/stampfly_py/`）:

```python
from stampfly import StampFly
with StampFly("192.168.10.1") as fly:  # connect() = SDK モード（SoftAP は Tello 互換の 192.168.10.1）
    fly.takeoff()                       # POS_HOLD 自動離陸 → 0.5 m ホバー（手動離陸と統一）
    fly.forward(50); fly.cw(90)         # 各コマンドは「到達後」に返る
    print(fly.battery(), fly.height(), fly.attitude())
    fly.land()
```

| コマンド | 動作 |
|---------|------|
| `command` | SDK モード（これが全ての前提）|
| `takeoff` / `land` / `emergency` / `stop` | 自動離陸（POS_HOLD・0.5m、手動 RC と統一）/ 自動着陸 / 即時モータ停止 / その場停止 |
| `up/down/left/right/forward/back <cm>`、`cw/ccw <deg>`、`go <x> <y> <z> <speed>` | 相対移動（10〜300cm、目標合成・到達後 ok）|
| `battery?` `height?` `attitude?` `speed?` | クエリ |

**待機中のコマンド:** `takeoff`・`land`・移動・`flip` などは目的を達成するまで応答を返さずに待つが、その間も機体はコマンドを読み続ける（50 ms ごと）。待機中に `emergency` を送ると**待機の残りを待たず**モータを止め、待機していたコマンドは `error interrupted by emergency` を返す。待機中の `stop` は飛行中（FLYING）で誘導目標があれば即座に受理され（現在位置と向きで止まる）、待機していたコマンドは `error interrupted by stop` を返す。`takeoff` / `land` / `flip` の待機中は飛行中でない（または FLIP 中）ため `stop` は通常どおり拒否され（`error not flying or guidance released ...`）、待機は続く。それ以外のコマンドは待機が終わってから到着順に実行される（従来どおり）。実機での遅延は未確認（SILS では注入からモータ停止まで 40 ms）。

**浮かないときの自動離陸の扱い（ALT_HOLD / POS_HOLD の ARM 起動、API `takeoff`）:** 浮けない機体（モータが弱すぎる・電池が減っている）の扱いは、高度センサ（ToF）が床の上で何を返すかで 2 通りになる。(1) ToF が床の上で**有効な低い値**（5 cm 以下）を返し、その値が増えないまま離陸の開始から 5 秒（`kAutoTakeoffNoLiftAbortUs`）たつと、機体は離陸を中止して **DISARM（モータ停止、IDLE_GROUND）** する（ログ `Auto-takeoff aborted: the healthy ToF has shown the craft on the floor for 5000 ms`）。この間、制御器が目標高度を捕捉したと言っても TAKEOFF のまま保つ（床の上で FLYING にしない）。(2) ToF が床の上で**無効**を返す場合（実機の多くの床。最小距離 3 cm 未満では測れない）は、離陸の開始から約 2 秒 + 0.15 秒で ToF の出力停止として扱われ、ALT_HOLD・POS_HOLD では自動で**着陸に入る**。機体はまだ床の上にいるが、ToF なしの降下の推力は機体の重さより小さいので浮かず、床が機体を支えていると検出されるとモータが止まる。「証拠が無い」ことは中止の理由にしない: ゆっくり浮く機体（ToF の読みが増える）は中止されず、離陸を続ける。加速度計だけで「床を離れた」とは判定しない（ゆっくり浮く機体と床が支える機体を区別できないため）。もう一度飛ばすには ARM し直す前に、電池の残量とプロペラ・モータを確認する。**手動（ACRO / STABILIZE）の離陸には適用しない**（スロットルはパイロットのもの）: ToF が床の上で無効を返す機体は、離陸の 2 秒 + 0.15 秒後に ToF の出力停止になり、床の上でも FLYING と表示されることがある（強制着陸はされない。パイロットがスロットルを握っているため）。

**API `takeoff` の結果と、その後の機体の状態:** 床を離れず中止された場合（上の (1) の DISARM、または (2) の自動の着陸）、`takeoff` は `error takeoff aborted` を返す。(1) ではそのとき機体は DISARM 済み（モータ停止）。(2) では床の上で着陸動作に入っており、床が機体を支えると検出されてモータが止まる（数 s かかる）。以前は 12 秒後に `error takeoff timeout` を返してもモータは止まらなかった。`error takeoff timeout`（12 秒）が出るのは、機体が床を離れたのに目標高度（0.5 m）を捕捉できないままの場合だけで、このときも離陸は取り消されず、機体は空中で TAKEOFF（制御器の時間切れの備え〔3 秒〕で FLYING へ進む）か FLYING のまま飛んでいる: 続けて `land` または `emergency` を送る。

**システム同定（レートループ）:** ホバー中に `sysid <roll|pitch|yaw> <chirp|doublet> <amp_dps> <dur_s>` で1軸のレート目標に励振を加算できる（振幅≤86dps・10s にクランプ、Airborne 限定）。ホスト側は `sf log wifi` でキャプチャ → `sf sysid rate-fit` でプラント同定（b,T,L）→ `sf sysid rate-tune --wc <rad/s> --pm <deg>` で位相余裕・交差周波数仕様を満たす PID を自動算出（`param set` 行を出力）。励振飛行は `sf sysid rate-excite --takeoff --land` で自動化可。

**オンボード自動チューン:** ホバー中に `autotune <roll|pitch|yaw> [ωc rad/s] [PM deg]`（既定 25/60）で、機体が単独で ①ステップドサイン9点掃引（2〜35Hz）②プラント同定 G(s)=b·e^{-Ls}/(s(Ts+1)) ③仕様を満たす PID 設計＋余裕の数値検証 ④**ライブ適用**（NVS 非保存 — 着陸後に確認飛行してから `param save`）を行う。安全判定: フィット残差・余裕誤差±5°・param 範囲・ゲイン跳躍 [1/4,4]× — どれかに掛かると**旧ゲインのまま** "error … gains unchanged" を返す。応答例: `ok kp=9.3e-04 ti=0.167 td=0.0056 wc=60.0 pm=50.0 gm=27.8`。

**安全則:** ①ペアリング済み送信機を中立で保持（スティックを動かすと API 誘導は即解除＝パイロット優先。モードスイッチはエッジ適用なので置いたままの位置は API を妨げない）②通信断フェイルセーフ（自動着陸）は API の下で常に有効 ③移動は1回 3m、目標高度は 0.2〜1.2m（高度上限 `kAltitudeCeilingM`）にクランプ。上限や距離を超える指示は、**クランプした値で実行し `ok` を返す**（移動は要求より短く、高さは上限で止まる。機体側のログに警告 `target altitude clamped to the 1.20 m ceiling` が 1 行出る）。

**宙返り（Flip）は既定で無効（`flip.enable` = 0）。** 実機での初回の宙返り（2026-10-02）が失敗し、原因解析中のためである。無効の間は API `flip` が `error flip: disabled` を返し、FLIP ボタンも拒否される（機体側のログに `Flip button: rejected (Disabled)`）。有効にするのは、ネットを張った監督下の試験に限る: `param set flip.enable 1`（即時反映、再起動不要。NVS に保存するなら `param save`）、試験後は `param set flip.enable 0`。

**宙返り（Flip）:** `flip <l/r/f/b>` は ALT_HOLD/POS_HOLD・対地高度 **1.0m 以上**・ほぼ水平で定常・電池電圧 **3.6V 以上**（負荷時）・前回の flip から 2 秒以上のときだけ実行される（不成立時は `error flip: <理由>`）。機体に上向きセンサが無いため天井までの距離は判定できない — **開始高度 + 0.6m 以上の天井余裕は操縦者の責任**。屋内では最初はネットのある場所・高天井（目安 2m 以上）で1方向ずつ試し、電池残量に余裕がある状態で行うこと。実行条件・応答語彙の詳細は [`docs/architecture/tello-api-reference.md`](../../../docs/architecture/tello-api-reference.md) を参照。

**FLIP ボタン:** コントローラの FLIP ボタンでも同じ宙返りを起動できる（全 FLYING モード対応、ACRO/STABILIZE を含む）。方向はボタンを押した瞬間のロール/ピッチスティックの向きで決まる — 大きく倒している軸（閾値超）があればその方向、両軸とも中立に近ければ既定方向（工場出荷時 Right）になる。実行条件は API の `flip` と同じ。不成立で拒否されると**エラー音（errorTone）が 1 回**鳴り（機体側のログにも1行）、CLI `status` の `flip  ` 行が `flip    : ready` または `flip    : BLOCKED (not steady - hold still ...)` のように理由を示す。理由が一過性の「不安定（NotSteady: 傾き/角速度/速度が範囲外）」のときだけは即拒否せず、押してから **1 秒（`flip.button_wait_ms`、0 で即拒否）** の間、機体が落ち着くのを待ち、落ち着けばそのまま宙返りを始める（1 秒経っても落ち着かなければエラー音）。Disabled・高度不足・電池低下などはすぐ拒否される。ボタンを押し続けても再要求はされない — 1回の押下につき1回だけ要求し、次に起動するには離してもう一度押す。

## 5. ペアリング手順

機体とコントローラを1対1に束ね、複数機・複数送信機の混信を防ぐ。詳細は
[`pairing_plan.md`](pairing_plan.md) と、教室での取り違え対策の検討経緯は
[`docs/plans/pairing-methods-plan.md`](../../../docs/plans/pairing-methods-plan.md)。

### 手順

1. 機体の電源を入れる。**未ペアなら自動でペアリング待機**（青 高速点滅 ＋ pairingTone）。
   機体は自 MAC を含む PairingPacket を 500ms 周期で broadcast する。
2. **コントローラをペアリングモードにする**（CH1-13 をスキャンし、聞こえた機体を候補表に集める。
   最初の1通を採用するのではなく、候補が集まる間 LCD の一覧を更新し続ける）。
3. コントローラの LCD に候補（機体 MAC 下4桁 + チャンネル、受信強度順）が一覧表示される。利用者が
   一覧から機体を選んでボタンで確定すると、その機体宛に ControlPacket を送信する。
4. **機体は、自機宛（`drone_mac` が自 MAC の下3バイトと一致）の ControlPacket だけをペア候補に
   する。** 別の組のコントローラが別の機体を選んで送る電文は、この機体ではペア候補にならず棄却
   される（棄却数は `pair status` の `rejected` に積算される）。
5. 自機宛の電文の送信元 MAC を相手側として確定 → **Paired**（青点滅停止 → 緑常灯）。
   相手側の MAC は NVS に保存され、次回起動時に自動復元される。
6. 以降、相手側以外の送信機のパケットは破棄される（混信対策、従来どおり）。

### 機体 ID（ラベル、MAC 下4桁）

コントローラの候補一覧は各機体を「MAC 下4桁」（例 `A1B2`）で表示する。取り違えを防ぐため、
機体ごとにこの下4桁を印字したラベル（シール等）を本体に貼っておく。ラベルの値は USB CLI の
`mac` コマンド（`sf monitor` で接続）で確認できる:

```
> mac
MAC: XX:XX:XX:XX:XX:XX
Label: XXYY
SoftAP SSID: StampFly-XXYY
SoftAP BSSID: XX:XX:XX:XX:XX:ZZ (= MAC + 1, ESP32 rule)
```

起動時のログにも `Own MAC: .. Label: XXYY` の1行が毎回出力される（モニタを開いたまま電源を
入れれば確認できる）。

**関係式（ESP32 の仕様）:**

| 値 | 由来・関係 |
|----|-----------|
| 機体 ID（ラベル、コントローラの候補一覧、SoftAP の SSID `StampFly-XXYY` の末尾、Tello 互換 API の `sn?`） | ステーション側 MAC（ESP-NOW の送信元）の下 4 桁。**機体の識別子はこれ 1 つ** |
| SoftAP の BSSID（Wi-Fi スキャンで見えるアクセスポイントの MAC） | ステーション側 MAC + 1（末尾 1 バイト）。ESP32 は 2 つのインターフェースに同じ MAC を割り当てられないため、SSID の末尾（= 機体 ID）とは 1 違う |

### 再ペアリング / 解除

| 操作 | 動作 |
|------|------|
| 機体ボタン 長押し3秒（地上でも手持ちでも可）| 既存バインドを破棄して再ペアリング |
| CLI `unpair` | 同上（バインド破棄＋ペアリング再突入）|
| CLI `pair status` | 自 MAC/ラベル・PairingState・バインド済み相手側の MAC・棄却カウンタを表示（下記例） |

```
> pair status
own mac : XX:XX:XX:XX:XX:XX (label XXYY)
pairing : Paired
bound   : XX:XX:XX:XX:XX:XX
rejected: 3 (packets addressed to a different vehicle)
```

### 注意（重要）

- **複数組を同時にペアリングモードにしてよい。** 機体は自機宛の電文だけをペア候補にするため
  （上記手順4）、隣の組のコントローラが送る電文が原因で取り違えが成立することはない。
- ただし**コントローラの候補一覧には、電波が届く範囲の他機体のラベルも一緒に並ぶ**。一覧から
  選ぶ操作そのものは人手なので、**必ず自分の機体に貼ったラベルと一致する行を選ぶこと**。他の
  組のラベルを誤って選ぶと、その機体との間でペアリングが成立してしまう（機体側の宛先確認は
  「別の組が誤って選んだ場合」を防ぐものではなく、「何も選んでいない機体が誤って拾われる」こと
  を防ぐ仕組みである点に注意）。
- 最終確認は**機体の LED が緑色に変わること**（ペア成立の合図）。誤って別のラベルを選んで
  しまった場合は、選んだ側の機体の LED が緑になり、本来ペアリングしたかった自分の機体は青点滅
  のまま残るので気づける。

## 6. CLI コマンド一覧

USB シリアル（USB Serial/JTAG）で接続し、`stampfly>` プロンプトにコマンドを入力する。`help` で一覧、
TAB で補完。

### vehicle の現行コマンド

| コマンド | 引数 | 説明 |
|---------|------|------|
| `param` | `[list\|get <name>\|set <name> <val>\|save]` | パラメータ読み書き（範囲検証・NVS 保存）|
| `status` | — | 状態/モード/ARM・ペアリング・姿勢・高度・電池・センサ |
| `sensor` | `[imu\|mag\|baro\|tof\|flow\|power\|all]` | センサ最新値表示 |
| `version` | — | ファーム名・ビルド日時 |
| `mac` | — | 自 MAC とラベル用の下4桁を表示（機体ラベル用） |
| `pair` | `[start\|status]` | ペアリング再突入 / バインド状態表示 |
| `unpair` | — | バインド破棄＋ペアリング再突入 |
| `sound` | `[on\|off]` | ブザー有効/無効（NVS 保存）|
| `led` | `<0-255>` | 本体 LED 輝度（NVS 保存）|
| `motor` | `[test <1-4> <0-100> [秒]\|all <0-100> [秒]\|stop]` | **モータ単体テスト（disarmed 限定・要プロペラ除去）**。M1=FR M2=RR M3=RL M4=FL。最後の引数で回転時間（秒、0.1〜60、省略時 2 秒）。例: `motor test 1 30 5`。講習用ファームでも使える |
| `magcal` | `[start\|stop\|status\|save\|clear]` | 地磁気のハード/ソフトアイアン校正（下記手順）|
| `reboot` | — | 再起動 |

**テレメトリの受信**: 機体は 50Hz のモニタ用テレメトリ（104B バイナリ、UDP ブロードキャスト :5005）を常時放送している。機体と同一ネットワークの PC で:

```bash
sf telemetry              # ターミナルのライブダッシュボード
sf telemetry --web        # ブラウザ表示（http://localhost:5006、グラフ付き）
sf telemetry --csv f.csv  # CSV 記録（ターミナル/--web どちらでも併用可）
```

グラフ・解析には Data Stream（400Hz・全センサ）を使う: `sf log wifi` → `sf log viz`。
なお `sf log wifi` キャプチャ中は排他ログ方針によりテレメトリ放送は停止する。

**地磁気校正の手順**（初回・搭載機器変更時・移設時に実施）:
1. disarmed のまま `magcal start`
2. 機体を手に持ち**全方位にゆっくり回す**（8の字、各軸一回転ずつ、20〜30秒。`magcal status` でサンプル数確認、100以上を目安）
3. `magcal stop` — 球面フィットを計算し offset/scale/fitness を表示
4. `magcal save` — NVS に永続化
5. 補正は即時有効。**ESKF のヨー補助は次回起動時**（静止校正完了時の磁気参照捕捉）に係合。融合に使うには `param set eskf.use_mag 1`（既定 off）

未校正のままだと: 磁気サンプルは生値（`calibrated=false`）で配信され、**ImuTask が磁気を融合から自動除外**する（ハードアイアンオフセットは磁場と同程度の大きさになり得るため、未校正で融合するとヨー参照が汚染される）。

**実装方針:** 各コマンドは状態/HAL に直接触れず、トピックへ「事実」を publish して所有タスクが
適用する（例: `motor` → `motor_test` トピック → ControlTask、`sound`/`led` → `ui_command` → NotifyTask、
`unpair` → `button_event` → StateTask）。これにより**将来 WiFi/UDP から同じトピックへ注入すれば
WiFi 経由の制御を追加できる**（§7）。

### 旧 vehicle のコマンド（参考・将来移植候補）

旧 vehicle には約50コマンドがある（移植は対応する内部機能が要るため順次）。主なもの:

| カテゴリ | コマンド例 | 説明 |
|---------|-----------|------|
| 飛行 | `takeoff [m]` / `land` / `hover [m] [s]` / `jump [m]` | 自律離着陸・ホバー |
| 移動 | `up/down/forward/back/left/right <cm>` / `cw/ccw <deg>` | 相対移動（Tello 風）|
| クエリ | `battery?` / `height?` / `tof?` / `attitude?` | 値問い合わせ（Tello 互換）|
| 制御 | `trim` / `gain <axis> <param> <val>` | トリム・PID ゲイン調整 |
| キャリブ | `magcal start/stop/save` | **→ 移植済み**（上の現行コマンド表参照）|
| 通信 | `comm [espnow\|udp]` / `wifi ...` | 通信モード・WiFi 設定 |

これらは「自律飛行・WiFi 制御・Tello 互換」用。コントローラ手動飛行には不要で、必要に応じて
vehicle に移植する。

## 7. 将来の WiFi 制御（方針）

CLI コマンドはすべて**トピック publish**で実装してあるため、UDP/TCP の受信経路を追加して同じ
トピック（`motor_test` / `ui_command` / `command_setpoint` 等）へ注入すれば、**コマンド実装を変えずに
WiFi 制御を追加できる**。受信層（UDP サーバ）の追加が今後の作業。

---

<a id="english"></a>

## 1. Overview

### About This Document

Operation manual for the vehicle firmware: **boot sequence, LED/sound meanings, controller
operation, pairing procedure, and CLI commands.** The UI (LED/buzzer) and controls follow the
legacy vehicle firmware.

### Safety Notes

- **Always remove the propellers or hold the craft when running the motor test (CLI `motor`).**
- Ensure the area is clear before ARM/takeoff; DISARM (controller or on-board button) cuts motors.

## 2. Boot Sequence

After power-on the firmware runs Phase 0–5 in order, signalling state via LED and sound.

| Stage | Action | StampS3 LED (state, GPIO21) | Sound |
|-------|--------|------------------------------|-------|
| Phase 0 | NVS init | — | — |
| Phase 1 | BSP (sf_board): I2C/SPI/LEDC buses | — | — |
| Phase 2 | Pub-Sub topics | — | — |
| Phase 3 | Parameters (NVS or defaults) | — | — |
| Phase 4 | 16 tasks start → NotifyTask | **white solid** (INIT) | **startTone** (C5→E5→G5) |
| Phase 5 | App hook `sf::app::start()` (starts the user application's extra tasks; does nothing without an app) | — | — |
| — | INIT → IDLE_GROUND (IMU valid) | white → green/magenta | — |
| Calib | Boot bias calibration (averages 2.5 s of VERIFIED-still samples; motion restarts it) | **magenta slow blink** | (silent) |
| Ready | Calibration done → can ARM | **green solid** | **readyTone** (beep ×3) |
| Pair | Auto-pairing if unpaired | **blue fast blink** | **pairingTone** (C5→G5) |

A Critical hardware failure lights the same StampS3 LED **red fast blink** and halts (sf_board).

## 3. LED / Sound Reference

The LEDs form **two channels**:

| Channel | LED | Role |
|---------|-----|------|
| **Mode colour** | Body LEDs (board top+bottom ×2, always identical) | Flight-mode colour. **Always the ACTIVE mode**: on the ground (disarmed or armed) = solid; airborne = slow blink |
| **System state** | StampS3 built-in LED (GPIO21 ×1) | INIT/calibrating/pairing/idle/armed/takeoff/landing status |

The board's top/bottom LEDs show the same content (one of them is hidden depending on the
craft's pose); the StampS3 LED serves as the independent status display.

### Mode colour (body LEDs, GPIO39×2)

| Mode | Colour | On ground (disarmed or armed) | Airborne |
|------|--------|-------------------------------|----------|
| ACRO | blue | solid | slow blink |
| STABILIZE | yellow-green | solid | slow blink |
| ALT_HOLD | orange | solid | slow blink |
| POS_HOLD | magenta | solid | slow blink |
| (during INIT) | white | solid | — |
| **Low battery** (top priority) | cyan | fast blink | fast blink |
| **Downward ToF silent while armed** (below low battery; manual-flight warning) | yellow | fast blink | fast blink |

The ARM/DISARM distinction on the ground lives on the StampS3 LED (ARMED_GROUND = green slow blink, IDLE_GROUND = green solid).

- **Mode changes are accepted on the ground** (disarmed or armed; spec 2026-06-11). Flipping
  the switch changes the actual mode — and the LED — immediately.
- ARMing in ALT_HOLD/POS_HOLD keeps the props stopped (the controller gates thrust to zero on
  the ground). In these modes **ARM ITSELF triggers an auto-takeoff** (redesign 2026-06-14, no
  throttle input): after a short spool dwell (0.3 s, props still stopped) the craft climbs at
  0.3 m/s to the **0.5 m target altitude** and holds the target (not the climb overshoot). In
  ALT_HOLD/POS_HOLD the throttle is a SYMMETRIC altitude command: **centre (raw 2048, the spring
  rest) = hold**, up = climb (`altitude.climb_rate`), down = descend (`altitude.descent_rate`).
  A re-center gate keeps an off-center stick from jumping the altitude until it returns to centre
  (release the spring stick to unlock) after takeoff or an in-flight switch into ALT/POS.
  STABILIZE/ACRO keep the manual throttle takeoff (throttle = thrust, centre = off).

### System state (StampS3 built-in LED, GPIO21), priority: low-battery > **downward ToF silent while armed (manual-flight warning)** > pairing > ARM-blocked (ground only: tilt mismatch / wait / downward ToF silent) > flight state

| State | Colour | Pattern |
|-------|--------|---------|
| Low battery (≤ `safety.battery.low_v`) | cyan | slow blink |
| Pairing (searching) | blue | fast blink |
| **Tilt mismatch** (ground; the estimate disagrees with gravity — **pick the craft up and place it level**) | **red** | **slow blink** |
| **Wait** (ground; calibrating [**never completes while moving**], tilt check pending [~1 s after reset/DISARM], or bench re-level pending [~1.5 s]; **keep still and wait**) | magenta | slow blink |
| **Downward ToF silent** (ground: the altitude sensor [ToF] delivers no new measurement, or its output is back but the estimator has not re-acquired it yet [~1 s]; **check the sensor, power-cycle, and wait ~1 s after it returns**) | **yellow** | **fast blink** |
| **Downward ToF silent** (ARMED: no new measurement, only invalid measurements in the air [0.5 s while flying; 0.15 s in a takeoff once the craft has risen 5 cm, or, when the rise is not visible, once 2 s of the takeoff have passed], or the estimator has not re-acquired the ToF yet [~1 s]. **ACRO / STABILIZE are NOT force-landed**, land yourself; ALT_HOLD / POS_HOLD start an automatic landing. Body LED and MCU LED both) | **yellow** | **fast blink** (with a warning tone) |
| INIT | white | solid |
| IDLE_GROUND (ready) | green | solid |
| IDLE_HELD | cyan | fast blink |
| ARMED_GROUND | green | slow blink |
| TAKEOFF | white | fast blink |
| FLYING | green | solid |
| LANDING | orange | slow blink |
| Critical hardware failure (halt) | red | fast blink |

Red **slow** blink (tilt mismatch: recoverable by re-placing) is distinct from red **fast** blink (Critical failure = halt, or the autotune-failed cue). Solid green (IDLE_GROUND) means all pre-arm gates pass, i.e. ARM is possible.

### Event → Sound

| Event | Sound | Notes |
|-------|-------|-------|
| Boot | startTone | C5→E5→G5 |
| Calibration done (ready) | readyTone | beep ×3 |
| ARM | armTone | E5→G5 |
| DISARM | disarmTone | G5→E5 |
| Pairing start | pairingTone | C5→G5 |
| Low battery | lowBatteryWarning | repeated |
| Error (failsafe) | errorTone | low/long |
| **ARM rejected** (an ARM request was refused by a pre-arm gate; same tone for every reason — the reason is on the LED and in the `status` `prearm` line) | errorTone | low/long |

Mute via `sound off` (NVS-saved); LED brightness via `led <0-255>`.

## 4. Controller Operation

The transmitter is fixed firmware; the vehicle decodes the SSOT ControlPacket (14B). **Operation is
identical to the legacy vehicle.**

- **Sticks** (12-bit ADC, centre 2048): throttle (0..4095, low half clipped to 0), roll/pitch/yaw
  (±about centre, deadband ±0.05).
- **ARM/DISARM**: controller ARM switch (flags bit0) rising = ARM (gated on calibration + voltage),
  falling = DISARM. On-board button click (on ground) toggles ARM/DISARM.
- **Flight modes** (flags): bit2 = ACRO, bit3 = ALT_HOLD, bit4 = POS_HOLD; default STABILIZE.
  Priority **POS_HOLD > ALT_HOLD > ACRO > STABILIZE**.
- **Heading hold** (STABILIZE and above): while the yaw stick is neutral the heading captured at
  stick release is held (P loop on the estimator yaw — no magnetometer). Any yaw input releases it
  instantly. Params: `attitude.yawhold.kp` (3.0, 0 disables), `attitude.yawhold.rate_max` (2.0 rad/s).
  Disabled in ACRO (pure rate control).
- **Takeoff**: throttle > 0.5 in ARMED_GROUND → TAKEOFF → FLYING (ToF airborne). **Landing**: throttle
  down / DISARM.

ARM is refused while calibrating, on low/USB power, while pairing, during/after a bench motor test (re-level pending), or until the attitude check has passed on the ground (about 1 s after boot/reset; refused while it disagrees with gravity), or **during a downward-ToF silence episode** (no new measurement for 0.3 s, only invalid measurements in the air [0.5 s while flying; 0.15 s in a takeoff once the craft has risen 5 cm, or, when the rise is not visible, once 2 s of the takeoff have passed], or the output is back but the estimator has not re-acquired the ToF yet: it needs >= 1 s of continuous measurements AND 10 consecutive agreeing samples, ~1 s after the return; **refused in every mode, ACRO included**; a measurement that merely arrives is not enough; log `ARM rejected: downward ToF is silent (reason)`). A refused ARM request (RC button, on-board click or API) plays the error tone once, and while refused the StampS3 LED shows why (red slow blink = tilt mismatch: place the craft level; magenta slow blink = wait and keep still; blue fast = pairing; cyan slow = low battery; yellow fast = downward ToF silent). `status` prints one line: `prearm  : ready` or `prearm  : BLOCKED (tilt mismatch - place level)`. During a ToF silence episode `status` also prints `tof     : SILENT - <reason>` and, while a mode switch was refused, `modeblock: REFUSED (...)`. ALT_HOLD / POS_HOLD switches are refused during an episode: **in flight the refused switch is CONSUMED** (the mode never switches by itself when the ToF recovers: flip the switch again after the yellow blink has stopped); on the ground the refused switch applies by itself when the episode ends. In flight, a silence in ALT_HOLD / POS_HOLD / auto-takeoff aborts the climb and lands the craft automatically (the descent does not use altitude. **The thrust ceiling is 0.95 x the thrust that was holding the craft (the support thrust, calibrated by the accelerometer), which is below the weight, so the craft cannot climb.** The descent is NOT a regulated 0.4 m/s: in effect it is a descent at a constant downward acceleration set by the thrust ceiling (about 0.05 g = 0.49 m/s^2), so it speeds up as the craft falls (about 1.1 m/s at touchdown from a height of 1.2 m). The vertical velocity estimated from the accelerometer (trusted only for 2 s after its last reset) is used to stop a climb (by lowering the thrust) and for the first acceleration of the descent; it never raises the thrust above the ceiling to slow a fast descent (that could make the craft climb). After 5 s of descent the thrust is also reduced, reaching zero at 8.3 s. The touchdown is detected by the accelerometer: an impact above 2 g followed by 0.25 s with no acceleration and no descent, or the floor holding the craft up (the thrust is below 0.7 x the support thrust yet the accelerometer reads 1 g) for 0.5 s, completes the landing; normally the 3 G impact cut fires first and disarms (during a LANDING with a silent ToF it is treated as the touchdown, not as an emergency). If no touchdown is detected 10 s after LANDING was entered AND the ToF has been silent for 4 s, the time limit completes the landing and stops the motors (the thrust is already zero by then: the craft is on the floor or in free fall; a ToF that dies after a LANDING that already lasted more than 10 s gets at least 4 s of descent before the motors are cut). The touchdown speed measured in SILS (6th pass) over the 870 runs in which the scene held is at most 0.89 m/s without sensor noise (a silence right after take-off), 0.93-1.02 m/s for a silence in a hover, 1.19 m/s in a manual climb, and 2.04 m/s in the worst run with sensor noise n2 - much faster than a normal landing (about 0.3 m/s): avoid flying outdoors or near people when the ToF is unreliable. In ALT_HOLD / POS_HOLD the stick climb stops at 1.2 m (the altitude ceiling; the auto-takeoff target is 0.5 m), which bounds the fall height. SILS values; not validated on a real flight. **A craft that is ARMED on the ground (ARMED_GROUND) is disarmed automatically when the ToF goes silent, in every mode** (it is not flying: it is not launched blind). In ACRO / STABILIZE a silent ToF only WARNS (no forced landing). A craft left ARMED on the ground with spinning props whose estimate drifts ~10 deg or more from gravity for 0.5 s is disarmed automatically (the estimator is then reset to level; no recalibration, since the craft may be tilted); thresholds are provisional (`safety.tilt_check.*`).

**Commands that wait:** `takeoff`, `land`, the moves, `flip` and similar commands do not reply until their goal is reached, but the vehicle keeps reading the command sources meanwhile (every 50 ms). An `emergency` sent while one waits stops the motors **without waiting for the rest of the wait**, and the waiting command replies `error interrupted by emergency` (SILS: 40 ms from the injection to the motors stopping; not measured on a real vehicle). A `stop` sent while a move waits in FLYING with a guidance target is accepted at once (the craft stops at its present position and heading) and the waiting command replies `error interrupted by stop`; during a `takeoff` / `land` / `flip` wait the craft is not in a state `stop` applies to (not flying, or in FLIP), so `stop` is refused as usual (`error not flying or guidance released ...`) and the wait goes on. Every other command is executed after the wait has ended, in the order of arrival, as before.

**An auto-takeoff that does not lift (ALT_HOLD / POS_HOLD started by the ARM, API `takeoff`):** a craft that cannot lift (weak motors, a low battery) is treated in one of two ways, depending on what the downward ToF returns on the floor. (1) If the ToF returns a **valid low reading** (5 cm or less) that does not rise, 5 s (`kAutoTakeoffNoLiftAbortUs`) after the TAKEOFF began the craft aborts the takeoff and **DISARMS (motors stop, IDLE_GROUND)** (log `Auto-takeoff aborted: the healthy ToF has shown the craft on the floor for 5000 ms`); until then the TAKEOFF is held even if the controller says the target height is captured (no FLYING on the floor). (2) If the ToF returns **invalid** samples on the floor (most real floors: it cannot range below about 3 cm), the ToF is treated as silent about 2 s + 0.15 s after the takeoff began and ALT_HOLD / POS_HOLD **start an automatic landing**. The craft is still on the floor but the thrust of the descent without a ToF is below its weight, so it does not lift, and the motors stop once the floor is detected to hold the craft. A missing proof of flight is never a reason to abort: a craft that lifts slowly (the ToF reading rises) is not aborted and keeps taking off. The accelerometer alone never decides "left the floor" (it cannot tell a slow lift from a craft that the floor holds). Check the battery, the propellers and the motors before you ARM again. **The manual takeoff (ACRO / STABILIZE) is not affected** (the throttle is the pilot's): a craft whose ToF reads invalid on the floor becomes ToF-silent 2 s + 0.15 s after the takeoff and may show FLYING on the floor (no forced landing; the pilot holds the throttle).

**API `takeoff`: result and the state of the craft afterwards:** when the takeoff does not leave the floor ((1) above: the DISARM, or (2): the automatic landing), `takeoff` replies `error takeoff aborted`; in (1) the craft is already DISARMED (motors stopped), in (2) it is landing on the floor and the motors stop once the floor is detected to hold it (a few seconds); before, it replied `error takeoff timeout` after 12 s while the motors kept spinning. `error takeoff timeout` (12 s) now appears only when the craft has left the floor but has not captured the target height (0.5 m): the takeoff is NOT cancelled then, and the craft keeps flying in the air, in TAKEOFF (the controller's own time backstop [3 s] moves it to FLYING) or in FLYING: send `land` or `emergency` next.

**Safety rules (API):** (1) Hold the paired transmitter at neutral (moving a stick releases the API guidance at once = pilot priority; mode switches are edge-applied, so a switch left in place does not hinder the API). (2) The comm-loss failsafe (auto-landing) is always active under the API. (3) A move is limited to 3 m per command and the target altitude to 0.2-1.2 m (the altitude ceiling `kAltitudeCeilingM`). A command that asks for more is **executed with the clamped value and still replies `ok`** (the move is shorter than asked, the height stops at the ceiling; the vehicle log carries one warning, `target altitude clamped to the 1.20 m ceiling`).

**Flip is disabled by default (`flip.enable` = 0).** The first hardware flip (2026-10-02) failed and the analysis is still open. While disabled, the API `flip` replies `error flip: disabled` and the FLIP button is rejected too (vehicle log: `Flip button: rejected (Disabled)`). Enable it only for supervised tests over a safety net: `param set flip.enable 1` (applies immediately, no reboot; `param save` to persist) and set it back to 0 afterwards.

**Flip:** sent via the Tello-style network API (`flip <l/r/f/b>`, UDP :8889 — see
`tools/stampfly_py/` and `docs/architecture/tello-api-reference.md`). Requires
ALT_HOLD/POS_HOLD, altitude above ground **>= 1.0 m**, near-level and steady flight,
battery **>= 3.6 V** under load, and >= 2 s since the previous flip; otherwise it
replies `error flip: <reason>`. The vehicle has no upward-facing sensor, so **a
ceiling clearance of start altitude + 0.6 m or more is the pilot's responsibility**.
Indoors, try one direction at a time with a net and a high ceiling (roughly 2 m or
more) first, with battery margin to spare.

**FLIP button:** the controller's FLIP button starts the same maneuver (any FLYING
mode, including ACRO/STABILIZE). The direction is read from the roll/pitch stick
deflection at the moment the button is pressed — whichever axis is held past the
threshold wins that direction; with both sticks near centre it falls back to the
default direction (factory default: Right). Execution conditions are the same as the
API `flip`. A refused press plays the error tone once (and logs one line on the vehicle), and
`status` prints the reason (`flip    : ready` / `flip    : BLOCKED (not steady - hold still ...)`).
When the only reason is the transient NotSteady (tilt/rate/velocity out of bounds), the press is
not refused at once but held for up to 1 s (`flip.button_wait_ms`; 0 = refuse at once) and the
flip starts as soon as the craft settles (error tone if it does not). Disabled, too low, low
battery etc. are refused immediately. Holding the button down does not re-request — one press
requests once; release and press again to fire the next one.

## 5. Pairing Procedure

Binds one transmitter to one vehicle to prevent crosstalk. See [`pairing_plan.md`](pairing_plan.md);
for the background on the classroom cross-pairing fix, see
[`docs/plans/pairing-methods-plan.md`](../../../docs/plans/pairing-methods-plan.md).

1. Power on the vehicle. **If unpaired it auto-enters Pairing** (blue fast blink + tone) and broadcasts
   a PairingPacket (its MAC) every 500 ms.
2. **Put the controller into pairing mode** (it scans CH1-13 and keeps collecting every vehicle it
   hears into a candidate table — it does not adopt the first packet).
3. The controller's LCD lists the candidates (vehicle MAC last 4 hex digits + channel, strongest
   signal first). The user picks one from the list and confirms; the controller then unicasts a
   ControlPacket to that vehicle.
4. **The vehicle only treats a ControlPacket as a bind candidate when it is addressed to itself**
   (`drone_mac` matches this vehicle's own MAC's lower 3 bytes). A packet from a neighboring
   controller that picked a different vehicle is never a candidate here (the count is tallied in
   `pair status`'s `rejected` field).
5. The vehicle fixes the source MAC of a packet addressed to itself as its peer → **Paired** (blue
   blink stops → green solid). The peer MAC is saved to NVS and restored on the next boot.
6. Thereafter ControlPackets from any other transmitter are dropped (unchanged from before).

### Vehicle ID (label, last 4 hex digits of the MAC)

The controller's candidate list shows each vehicle as its "MAC last 4 hex digits" (e.g. `A1B2`).
To avoid mix-ups, put a sticker with these 4 digits on each vehicle. Read the value with the USB
CLI `mac` command (connect with `sf monitor`):

```
> mac
MAC: XX:XX:XX:XX:XX:XX
Label: XXYY
SoftAP SSID: StampFly-XXYY
SoftAP BSSID: XX:XX:XX:XX:XX:ZZ (= MAC + 1, ESP32 rule)
```

The boot log also prints one `Own MAC: .. Label: XXYY` line every time (visible if the monitor is
already open when power is applied).

**Relation (ESP32 rule):**

| Value | Origin / relation |
|-------|-------------------|
| Vehicle ID (label, controller candidate list, tail of the SoftAP SSID `StampFly-XXYY`, Tello-compatible `sn?`) | last 4 hex digits of the *station* MAC (the ESP-NOW source address). **This is the vehicle's single identity** |
| SoftAP BSSID (the access point's MAC a Wi-Fi scanner shows) | station MAC + 1 (last byte). ESP32 cannot give two interfaces the same MAC, so it differs from the SSID tail (= vehicle ID) by one |

**Re-pair / clear**: on-board button long-press 3 s (on the ground or held in hand), or CLI `unpair`. `pair status`
shows this vehicle's own MAC/label, the PairingState, the bound MAC, and the rejected-packet count:

```
> pair status
own mac : XX:XX:XX:XX:XX:XX (label XXYY)
pairing : Paired
bound   : XX:XX:XX:XX:XX:XX
rejected: 3 (packets addressed to a different vehicle)
```

**Important**:
- **Several pairs may enter pairing mode at the same time.** Because the vehicle only treats
  packets addressed to itself as candidates (step 4), a neighboring controller's packets alone
  cannot cause a cross-pair.
- However, **the controller's candidate list also shows any other vehicle's label that is within
  radio range.** Picking from the list is still a human action, so **always pick the row matching
  the label stuck on your own vehicle.** Picking another pair's label by mistake does complete a
  pairing with that vehicle (the vehicle-side destination check prevents an *unselected* vehicle
  from being picked up by accident — it does not prevent a *deliberately wrong* selection from
  working).
- The final check is **the vehicle's LED turning green** (the sign of a successful pairing). If
  you pick the wrong label, that other vehicle's LED turns green while the vehicle you meant to
  pair keeps blinking blue — so the mistake is noticeable.

## 6. CLI Commands

Connect over USB serial (USB Serial/JTAG); type at the `stampfly>` prompt. `help` lists commands, TAB
completes.

| Command | Args | Description |
|---------|------|-------------|
| `param` | `[list\|get <name>\|set <name> <val>\|save]` | Read/write parameters (validated, NVS) |
| `status` | — | State/mode/arm, pairing, attitude, altitude, battery, sensors |
| `sensor` | `[imu\|mag\|baro\|tof\|flow\|power\|all]` | Print sensor readings |
| `version` | — | Firmware name / build date |
| `mac` | — | Show this vehicle's MAC and its 4-hex-digit label (for the vehicle sticker) |
| `pair` | `[start\|status]` | Re-enter pairing / show bind |
| `unpair` | — | Clear bind and re-enter pairing |
| `sound` | `[on\|off]` | Enable/disable the buzzer (NVS) |
| `led` | `<0-255>` | Body LED brightness (NVS) |
| `motor` | `[test <1-4> <0-100> [sec]\|all <0-100> [sec]\|stop]` | **Bench motor test (DISARMED only — props off!)**. M1=FR M2=RR M3=RL M4=FL. Optional last argument = run time in seconds (0.1–60, default 2). Example: `motor test 1 30 5`. Also works on the workshop firmware |
| `magcal` | `[start\|stop\|status\|save\|clear]` | Magnetometer hard/soft-iron calibration: `start` → rotate the craft in all orientations (figure-8, ~100+ samples) → `stop` (sphere fit) → `save` (NVS). Yaw aiding engages at the next boot; enable fusion with `param set eskf.use_mag 1`. Uncalibrated mag is automatically kept OUT of the fusion |
| `reboot` | — | Restart the flight controller |

Every command publishes a FACT on a topic and the owning task applies it (e.g. `motor` → `motor_test`
→ ControlTask; `sound`/`led` → `ui_command` → NotifyTask; `unpair` → `button_event` → StateTask). So a
future WiFi/UDP receiver can inject the same topics to add **WiFi control without changing the command
implementations** (§7).

The legacy vehicle has ~50 commands (flight: `takeoff`/`hover`/`jump`; Tello-style moves and queries:
`battery?`/`height?`; `trim`/`gain`/`magcal`/`comm`/`wifi`). These are for autonomous flight / WiFi /
Tello compatibility and are ported to vehicle on demand.

## 7. Future WiFi Control (Direction)

Because every CLI command is implemented as a topic publish, adding a UDP/TCP receiver that injects
the same topics (`motor_test` / `ui_command` / `command_setpoint`, …) adds WiFi control **without
changing the command logic**. The receiver layer (UDP server) is future work.
