# StampFly の ToF センサ VL53L3CX の実機の測定記録

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

StampFly の下向きと前方に付く ToF（Time of Flight、光を出して反射が戻るまでの時間から距離を測るセンサ）VL53L3CX について、機体 1 台で測った結果をまとめる。内容は次の 3 点である。

| 内容 | 章 |
|------|----|
| 1 回の読み出しにかかる時間の内訳 | 3 章 |
| 測距の上限（飛行ログの調査と、手に持った機体での実験） | 4 章 |
| クロストークの較正の実験（較正の結果と、補正ありの手持ち測定） | 5 章 |

事実・推論・未確認を分けて書く。測っていないことは書かない。

### 測定の共通条件

| 項目 | 内容 |
|------|------|
| 日付 | 2026-11-21 の夜から 2026-11-22 |
| 機体 | 1 台。床の上または手持ち、ARM していない、プロペラ付き、電池あり（4.16〜4.19 V）、全センサ動作（`status` の `sensors present=0x7F`）、電源スイッチ付きハブ経由の USB |
| ファームウェア | fix/flight-safety の f4b72074（`App version archive/2026-11-13-31-gf4b72074`）を基準に、比較用の変更を Git 管理外の作業ツリーで加えたもの（各章に書く）。測定の後、機体は基準のビルドに戻した（3・4 章の後は `logs/checkout/restore_A_20261122.txt`、5 章の後は `logs/checkout/restore_A_20261122b.txt`） |
| 記録 | `logs/checkout/`（Git 管理外）。作った変更と集計の道具は同じ場所に置いた（下の表） |

| 記録・道具 | 内容 |
|------------|------|
| `toftime_floor_1.txt` | 3 章の記録（75 秒） |
| `range_L33_*.txt`、`range_M33_*.txt`、`range_L33_check.txt` | 4 章の手持ち実験の記録 |
| `tof_timing.patch`、`toftime_parse.py` | 3 章の計測用の変更（Git のパッチ）と集計 |
| `tof_range.patch`、`tofdiag_parse.py`、`range_capture.py`、`move_an.py`、`run_variant.sh` | 4 章の実験用の変更（Git のパッチ）、集計、記録、動かし方の解析、書き込みから測定までの手順 |
| `tof_xtalk.patch`、`xtalk_cal.py` | 5 章の実験用の変更（Git のパッチ。命令 `tofcal` と診断の追加）と、較正の実行・記録の手順 |
| `xtalk_show_before.txt`、`xtalk_cal_w1000.txt`、`xtalk_cal_w1500.txt` | 5 章の較正前の状態と、較正 2 回の記録 |
| `range_Xon_wall_{0.5,1.0,1.5,2.0,2.5,3.0}.txt` | 5 章の補正ありの手持ち測定の記録（補正なしの比較は `range_L33_wall_*.txt`） |
| `restore_A_20261122b.txt` | 5 章の後に基準のビルドへ戻したときの記録 |

### 結論の要約

| 項目 | 内容 | 種別 |
|------|------|------|
| 1 回の読み出しの時間 | 下向き平均 5839 µs、前方 5470 µs。時間の 75〜80 % は I2C の転送。ヒストグラムの計算は 7〜10 % | 事実（床の上、目標 1 個） |
| 開発者の見立て | 「ヒストグラムの計算が遅い」。測定では時間の大半が I2C だった | 事実 |
| 測距モード | 機体のファームウェアは LONG。静止なら白い壁で 3.0 m（部屋の制約で、これより遠くは測っていない）まで、床で約 2.0 m（手の届く高さ）まで有効率 95〜100 % | 事実 |
| MEDIUM | 壁から 1.5 m までは有効、2.0 m で有効率 1.0 %、2.5〜3.0 m では誤った近い距離を有効として返した。使えない | 事実 |
| 飛行ログの有効率 | 気圧高度基準で 1.9〜2.0 m が 64 %、2.0〜2.2 m が 22 % | 事実（計算はサブエージェント、独立の確認なし） |
| 手前の弱い目標 | 距離 30 mm 未満の弱い目標は、クロストークの較正（RAM 上、白い壁で 1500 mm の較正値）を有効にすると、白い壁の 0.5〜3.0 m の 6 点すべてで 0 % になった（補正なしでは 5.8〜76 %）。壁の目標の有効率・信号・距離は変わらなかった。原因がクロストークであることの確定ではない（5 章） | 事実（消えたこと）、推論（原因） |
| クロストークの較正 | 暗い環境ではなく室内の照明下で 2 回（壁から 1000 mm、1500 mm）実行し、2 回とも抽出に成功（`cal_status=0`、代替値ではない）。較正値は電源を切ると消える。飛行中の有効率の低下への効果、床向きの測定、保存の方法は未確認（5・6 章） | 事実、未確認 |
| 飛行中の途切れの原因 | 手持ちの静止・動き・傾きでは再現しなかった。原因は未確認 | 未確認 |

## 2. 構成と設定

### ドライバと測距モード

| 項目 | 内容 | 出どころ |
|------|------|----------|
| コンポーネント | `firmware/vehicle/components/sf_hal_vl53l3cx/`（ST のベアドライバ 1.2.14 と、そのラッパー） | ソース |
| ドライバの版 | 1.2.14.2687 | 実機の `TOFCFG` 行 |
| 測距モード | LONG（`VL53LX_SetDistanceMode` に `VL53LX_DISTANCEMODE_LONG`、`vl53l3cx_wrapper.cpp` 209 行） | ソース、`TOFCFG` 行（`mode=LONG preset=27`） |
| 測定時間 | 33 ms（`timing_budget_ms`、同 217 行付近）。測定は約 30 Hz | ソース、`TOFCFG` 行 |
| クロストーク補正 | 無効（`xtalk_en=0`、較正値 0）。クロストークとは、センサの手前の部品などでの反射が、測りたい物の反射に混ざること | `TOFCFG` 行 |
| 拡張測距 | 有効（`uwr=1`） | `TOFCFG` 行 |
| 高速発振器の値 | `fast_osc=48455` | `TOFCFG` 行 |
| I2C の速度 | 400 kHz（`vl53l3cx_wrapper.cpp` 153 行） | ソース |
| タスク | `TofTask`。コア 0、優先度 14、周期 33 ms（`tasks/tof_task.cpp`、`main/config.hpp`）。下向きを先に、続けて前方を読む | ソース |

### 有効の判定と目標の選び方

センサは 1 回の測定で最大 4 個の目標（反射した物）を返す。ラッパーは次の規則で 1 個を選ぶ。

| 段階 | 内容 | 出どころ |
|------|------|----------|
| 候補 | 状態コードが 0 で、距離が 30 mm を超える目標（`MIN_VALID_DISTANCE_MM = 30`） | `vl53l3cx_wrapper.cpp` 299〜311 行 |
| 選び方 | 候補のうち最も遠い目標。コメントは「高度の測定では床がたいてい最も遠い」 | 同 305〜316 行 |
| 候補が無い | 距離 0、状態は最初の目標の状態（0 なら 254）。目標が 1 個も無ければ状態 255 | 同 |
| 有効の判定 | 選んだ目標の状態が 0 のとき `valid`（`tof_task.cpp` 126 行） | ソース |

### 状態コードの意味

ドライバの定義（`include/vl53lx/vl53lx_def.h` 286〜318 行）と、ラッパーが足す値は次のとおり。

| コード | 名前 | 意味（ドライバの注釈の訳） |
|--------|------|----------------------------|
| 0 | RANGE_VALID | 距離は有効 |
| 1 | SIGMA_FAIL | ばらつき（sigma）の判定に失敗 |
| 2 | SIGNAL_FAIL | 信号の判定に失敗 |
| 3 | RANGE_VALID_MIN_RANGE_CLIPPED | 目標が最小の検出しきい値より近い |
| 4 | OUTOFBOUNDS_FAIL | 位相が有効な範囲の外（折り返しの判定による退出とは別） |
| 5 | HARDWARE_FAIL | ハードウェアの失敗 |
| 6 | RANGE_VALID_NO_WRAP_CHECK_FAIL | 距離は有効だが折り返しの確認をしていない |
| 7 | WRAP_TARGET_FAIL | 折り返した目標。別の発光周期に一致する位相が無い |
| 8 | PROCESSING_FAIL | 内部の演算のあふれ |
| 9 | XTALK_SIGNAL_FAIL | 簡易測距に固有 |
| 10 | SYNCRONISATION_INT | 連続測定の開始直後の最初の割り込み。データを無視する |
| 11 | RANGE_VALID_MERGED_PULSE | 距離は有効だが、複数のパルスが合わさった結果 |
| 12 | TARGET_PRESENT_LACK_OF_SIGNAL | 目標はあるが信号が足りない |
| 13 | MIN_RANGE_FAIL | SPAD（受光素子の配列）の予期しない異常 |
| 14 | RANGE_INVALID | 距離の計算値が負 |
| 254 | （ラッパーが足す） | 全目標が無効 |
| 255 | NONE（ドライバの定義）。ラッパーでは目標なしにも使う | 更新なし。目標が 1 個も無い |

## 3. 読み出し時間の内訳

### 測り方

| 項目 | 内容 |
|------|------|
| 記録 | `logs/checkout/toftime_floor_1.txt`（75 秒、床の上） |
| 計測用のビルド | `vl53lx_platform.c` の I2C の入出力と、`vl53lx_platform_ipp.c` のヒストグラム処理の入口に、`esp_timer_get_time`（マイクロ秒の時刻）の計測を入れた。`TofTask` が 10 秒ごとに `TOFTIME` の行を出す |
| 集計 | `toftime_parse.py`（各行の平均を測定回数で重みづけして平均する） |
| 測定回数 | 下向き 2129 回、前方 2128 回、読み出しの失敗は 0 |

### 結果（事実）

| 項目 | 下向き | 前方 |
|------|--------|------|
| 1 回の合計 平均 / 最大 | 5839 / 6998 µs | 5470 / 6952 µs |
| I2C（3 回の転送、158 B） | 4356 µs（74.6 %） | 4369 µs（79.9 %） |
| ヒストグラム処理 | 555 µs（9.5 %） | 399 µs（7.3 %） |
| それ以外 | 928 µs（15.9 %） | 703 µs（12.8 %） |

下向きの段別の内訳は次のとおり。

| 段 | 時間 | うち I2C |
|----|------|----------|
| データの有無の確認 | 304 µs | 274 µs |
| 結果の読み出しと計算 | 3403 µs | 2135 µs（83 B の読み出し） |
| 割り込みのクリアと次の測定の開始 | 2132 µs | 1946 µs（68 B の書き込み） |

前方は、同じ 3 段が 326 µs（うち I2C 305）、3102 µs（同 2143）、2042 µs（同 1920）である。

| 補足 | 内容 |
|------|------|
| I2C の速度 | 400 kHz |
| 2 個の合計（計算） | 5839 + 5470 = 11309 µs。周期 33 ms の約 34 % |
| ヒストグラム処理 | 整数演算で、64 ビットの除算を含む。コードは約 6.1 KB で、ドライバ全体（約 33 KB）の約 18 %（ビルドのマップからの集計） |
| 条件 | 床の上で、目標が 1 個の場合 |

開発者の見立て（ヒストグラムの計算が遅い）とは逆に、時間の大半は I2C だった。

### 未確認

| 項目 | 状態 |
|------|------|
| 目標が複数のときの計算時間 | 未確認 |
| I2C の転送の間に CPU が空くか（待ちの間に他のタスクが走れるか） | 未確認 |
| ARM 中・飛行中（モータ回転中）の読み出し時間 | 未確認 |

## 4. 測距の上限

### 飛行ログの調査

2026-06-11 以降の飛行ログ 196 本を、気圧高度を基準にして調べた。図と表は作業中の場所に置いたため文書からは参照せず、要点の数値だけを書く。**計算はサブエージェントが行い、独立の確認はしていない。**

新しいファームウェアの 46 本について、気圧高度基準の高さ別の有効率（状態コード 0 の割合）は次のとおり。

| 高さ（気圧高度基準） | 有効率 | 件数 |
|----------------------|--------|------|
| 1.0〜1.5 m | 99.6〜99.8 % | 271〜964 件（0.1 m ごと） |
| 1.5〜1.8 m | 97.9〜98.5 % | 137〜243 件 |
| 1.8〜1.9 m | 96.2 % | 79 件 |
| 1.9〜2.0 m | 64.0 % | 25 件 |
| 2.0〜2.1 m | 22.2 % | 45 件 |
| 2.1〜2.2 m | 22.2 % | 18 件 |

| 項目 | 内容 |
|------|------|
| 床までの距離として確かな最大 | 2.04 m（ログ 20260618T185844） |
| 水平な途切れ（気圧高度が一定のまま起きる無効）の状態コード | 255（目標なし）と 12（目標はあるが信号が足りない） |
| 気圧高度の基準の誤差 | 残差の rms（二乗平均平方根）が 0.23 m 程度。高さの区切りには、この大きさの誤差が入る |

`firmware/vehicle/components/sf_core/include/flight_limits.hpp` の `kAltitudeCeilingM = 1.2 m`（高度の上限の候補値）は、「ToF の有効距離 約 1.6 m（飛行ログで 1.7 m まで 95〜99 % が有効、それ以上で低下）」を根拠の 1 つにしている。上の調査の値は、その根拠と大きくは食い違わない。この文書は上限の変更を提案しない。

### 手持ち実験の方法

| 項目 | 内容 |
|------|------|
| 実験用のビルド | 下向きセンサの読み出しごとに、全目標の距離・状態コード・信号レート・周囲光・sigma を `TOFDIAG` の行で出す。測距モードと測定時間をビルドの設定で選べる（`tof_range.patch`。選ばないときは飛行用の LONG・33 ms）。設定は `TOFCFG` の行で出す。前方センサは変えていない |
| 集計 | `tofdiag_parse.py`。1 点ごとに有効率・距離の中央値・信号レートなどを出す。この文書の数値は、記録から再集計した値である |
| 設定の行 | LONG、測定時間 33 ms、ドライバ 1.2.14.2687、`xtalk_en=0`（較正値 0）、`uwr=1`、`fast_osc=48455` |
| 有効率の定義 | 選ばれた目標の状態が 0 で、距離が 0 より大きい行の割合 |
| 信号レート | 単位は Mcps（メガカウント毎秒、受光素子が数えた光子の数の速さ） |

機体は手で持った。高さは案内した値で、下の表は実際に測れた距離の中央値で整理した。

### (a) 床の上（LONG・33 ms、静止）

| 実際の距離の中央値 | 有効率 | 信号レート [Mcps] |
|--------------------|--------|-------------------|
| 0.52 m | 100 % | 3.88 |
| 0.79 m | 100 % | 1.65 |
| 1.04 m | 100 % | 0.95 |
| 1.31 m | 100 % | 0.63 |
| 1.50 m | 100 % | 0.50 |
| 1.68 m | 100 % | 0.45 |
| 1.90 m | 95.2 % | 0.24 |
| 2.01 m | 100 % | 0.17 |

周囲光は 0.125〜0.133 Mcps だった。手の届く最高点は約 2.0 m で、1.9 m 以上では、手前 0.5 m 付近に別の目標が見えている（持ち手の体と推定。推定であり確認していない）。

### (b) 白い壁（LONG・33 ms、静止）

壁紙は白で、表面に細かい凹凸がある。

| 実際の距離の中央値 | 有効率 | 信号レート [Mcps] |
|--------------------|--------|-------------------|
| 0.51 m | 100 % | 4.82 |
| 1.01 m | 100 % | 1.27 |
| 1.56 m | 100 % | 0.55 |
| 2.03 m | 100 % | 0.30 |
| 2.49 m | 100 % | 0.20 |
| 2.97 m | 100 % | 0.11 |

周囲光は 0.133〜0.148 Mcps だった。

### (c) 白い壁（MEDIUM・33 ms、静止）

| 壁からの案内した距離 | 結果 |
|----------------------|------|
| 0.53 m、0.93 m、1.50 m（実際の距離の中央値） | 有効率 100 % |
| 2.0 m | 有効率 1.0 %（状態は 4 が 105 行、7 が 101 行、0 が 2 行） |
| 2.5 m | 有効率 43.2 %。ただし有効とされた距離の中央値は 118 mm（誤った近い距離） |
| 3.0 m | 有効率 27.2 %。同 44 mm（誤った近い距離） |

6 点の合計で、拡張測距が働いたことを示す印（`TOFDIAG` の `x=1`）の目標は 17 行だった。

### (d) 床の上で動かす（LONG・33 ms）

| 動かし方 | 行数 | 有効率 | 補足 |
|----------|------|--------|------|
| ゆっくり上げ下げ | 1504 | 100 % | |
| 速い上げ下げ | 1174 | 99.2 % | 無効 9 行はすべて状態 7 が 1 行ずつ。距離の変化の速さの最大は約 5.5 m/s（隣り合う行の差から計算） |
| 頭上で前後左右に約 20 度傾ける | 1064 | 99.9 % | 無効 1 行（状態 7） |

距離は 0.66〜1.93 m だった。

### 共通の観察

| 項目 | 内容 |
|------|------|
| 弱い近い目標 | 0.75 m より上の点では、距離 1〜29 mm・信号 0.04〜0.12 Mcps の弱い目標が出る。出る行の割合は点ごとに 2〜59 %。センサの手前からの反射（クロストーク）と推定する。推定であり、この時点では確認していない（5 章で、クロストークの較正により消えた） |
| 選び方との関係 | 2 章の選び方は距離 30 mm 以下を除き、最も遠い有効な目標を選ぶので、LONG では床・壁が選ばれた |
| 信号レート × 距離の 2 乗 | 床で 1.02〜1.28（1.7 m まで）、壁で 0.96〜1.33。壁は床の約 1.2 倍（計算）。距離の 2 乗に反比例して信号が下がる、という予想に合う |

### 発光の周期から計算した範囲

ToF は光の位相から距離を求める。発光の周期より遠い物は、折り返して近い距離に見える。範囲は次の式で計算した（測定を行った側が使った近似式。`fast_osc` を使う。ソースの定数から直接導いたものではなく、確認は 6 章）。

$$d_{\text{pclk}} \approx 198.4 \times \frac{48332}{f_{\text{osc}}}\ \text{[mm]}$$

$d_{\text{pclk}}$ は発光の周期（pclk）の 1 個分に当たる距離、$f_{\text{osc}}$ は `fast_osc` の値（48455 のとき約 197.9 mm）である。

| モード | 発光の周期 | 有効な位相の上限 | 計算した範囲（周期 × 197.9 mm） |
|--------|------------|------------------|--------------------------------|
| LONG | 20 pclk と 24 pclk | 17 pclk | 周期から 3.96 m と 4.75 m、有効の上限から 3.36 m |
| MEDIUM | 12 pclk と 16 pclk | 9 pclk | 周期から 2.37 m と 3.17 m、有効の上限から 1.78 m |

出どころは次のとおり（行番号はこの時点のソースのもの）。

| 内容 | ソース |
|------|--------|
| LONG の周期（レジスタ値 0x09 と 0x0B）と有効位相の上限 0x88 | `src/vl53lx/vl53lx_api_preset_modes.c` 1132 行、1169 行 |
| MEDIUM の周期（レジスタ値 0x05 と 0x07）と有効位相の上限 0x48 | 同 1254 行、1291 行 |
| 周期の復号（(レジスタ値 + 1) × 2） | `src/vl53lx/vl53lx_core_support.c` 465 行 |
| 位相から距離への換算 | 同 403 行 |
| 有効位相の上限による判定（状態 4 になる） | `src/vl53lx/vl53lx_hist_algos_gen3.c` 835〜849 行 |

MEDIUM の有効の上限の計算（約 1.78 m）は、(c) の実測（1.5 m は有効、2.0 m で途切れる）と合う。

### 読み取り

| 項目 | 内容 | 種別 |
|------|------|------|
| LONG の設定 | 誤りではない。静止なら白い壁で 3.0 m（部屋の制約）まで測れた | 事実 |
| MEDIUM | 1.5〜2.0 m の間で途切れ、範囲外では誤った近い距離を有効として返すので使えない。同梱の ST の資料 AN5561 の「室内では MEDIUM のほうが遠い」は、この機体では再現しなかった | 事実 |
| 飛行ログとの食い違い | 飛行ログでは 1.9〜2.0 m で有効率が 64 %、2.0〜2.2 m で 22 % に下がる。手持ちの静止・動き・傾きでは再現しなかった。ただし手持ちで届いた高さは約 2.0 m まで | 事実 |
| 飛行中の途切れの原因 | 候補は、飛行した場所の床、モータ回転中の条件（振動・電源）、クロストーク（4 章の測定は補正なし。5 章の補正ありの測定は手持ちのみで、飛行では未確認） | 未確認 |

## 5. クロストークの較正の実験

### 目的と方法

4 章で、距離 1〜29 mm・信号の弱い目標が出ることを見つけ、センサの手前からの反射（クロストーク。発光した光の一部が目標に届かず、センサの内部や窓から直接受光部に入ること）と推定した。この推定を確かめるため、同じ日にクロストークの較正（クロストークの量を測って、測距の結果から差し引く値を作る操作）を機体で行い、較正の前後で同じ手持ち実験を比べた。

| 項目 | 内容 |
|------|------|
| 実験用ビルド | f4b72074 に、既存の診断（`TOFDIAG`・`TOFCFG` 行）に加えてコンソールの命令 `tofcal xtalk [mm] \| off \| on \| show \| refspad` を足したもの。変更は `logs/checkout/tof_xtalk.patch`（Git 管理外） |
| 命令の動き | 命令は要求を出すだけ。ToF タスクが周期の頭で測距を止め、ST のドライバの `VL53LX_PerformXTalkCalibration` を呼び、測距を再開する。ARM 中は拒否する |
| 較正値の保存 | RAM 上だけ。電源を切ると消える |
| 測距の設定 | LONG・33 ms（4 章と同じ） |
| 較正の姿勢 | 機体を手で持ち、下向きの ToF を白い壁（白い壁紙、細かい凹凸）に正対させた。室内の照明下（暗い環境ではない） |
| 較正の距離 | 壁から 1000 mm と 1500 mm の 2 回（命令に渡した値） |

### ドライバの性質（ソースの読み）

担当がソースで確認した内容である。関数名で示す（`src/vl53lx/vl53lx_api.c`）。

| 内容 | 種別 |
|------|------|
| `VL53LX_PerformXTalkCalibration` の戻り値は、較正の成否を表さない。最後の測距モード・測定時間の再設定の結果で上書きされる | 事実（ソース） |
| 抽出に失敗すると決め打ちの代替値（平面オフセット 50、波形 307/410/410/307）が入る。実際の状態は `LLData.xtalk_results.cal_status` で見る | 事実（ソース） |
| 較正の最後に補正は有効になる | 事実（ソース） |
| 汚れに対する動的な補正（smudge correction）はドライバにあるが、`VL53LX_DataInit` の中で無効（`VL53LX_SMUDGE_CORRECTION_NONE`）にされる。今回も無効のまま | 事実（ソース） |
| 較正の条件について、リポジトリ内の要約 `firmware/vehicle/components/sf_hal_vl53l3cx/docs/VL53L3CX_driver_doc.md` 6 節は「基準 SPAD（受光素子）→ クロストーク → オフセットの順、クロストークは暗い環境・600 mm」とする。ST の関数の説明（同梱の `VL53LX_API.chm`）は「目標は 60 cm 以上、実際の距離を値で渡す」とする。目標の反射率・大きさの指定は同梱資料に見つからなかった | 事実（資料） |
| ST の UM2778（較正の資料）は同梱されておらず、読んでいない | 未確認 |

今回は照明下で較正したので、要約が求める「暗い環境」とは条件が違う。

### 較正の結果

記録は `xtalk_cal_w1000.txt`、`xtalk_cal_w1500.txt`。較正前の状態は `xtalk_show_before.txt`（補正は無効、平面オフセット 0、波形はすべて 0）。

| 項目 | 1000 mm | 1500 mm |
|------|---------|---------|
| 平面オフセット（生の値 / ÷512） | 191 / 0.373 | 220 / 0.430 |
| 波形 `shape_bins`（先頭 5 区間、合計） | 12, 352, 471, 172, 17（1024） | 27, 389, 456, 160, 0（1032） |
| `ext_rate_per_spad` | 1152 | 1160 |
| `ext_signal_events` | 3376 | 3400 |
| 抽出の窓 / 目標の開始区間 | 0..4 / 5 | 0..4 / 8 |
| `cal_status` / `fallback` | 0 / 0 | 0 / 0 |
| 所要時間 `elapsed_ms` | 1508 | 1510 |

事実として次の点が記録から読める。

| 項目 | 内容 | 種別 |
|------|------|------|
| 抽出の成否 | 2 回とも `cal_status=0`、`fallback=0`。代替値の特徴（平面オフセット 50、波形 307/410/410/307）とは一致しない | 事実 |
| 較正の副作用 | 較正後、`nodetect_kcps`（ドライバの調整値 `DYNXTALK_NODETECT_XTALK_OFFSET_KCPS`）が 410 から平面オフセットと同じ値（191、220）に変わった。ドライバが設定している | 事実 |
| 1500 mm の較正の開始状態 | 1000 mm の較正値が有効（`en=1`、平面オフセット 191）のまま始めた。2 回は独立の較正ではなく、2 回目は 1 回目の続きの状態からの較正である | 事実 |
| 測距の中断 | 較正中は測距の出力が約 1.5 秒止まる。再開に 7 ms | 事実 |

2 つの距離で平面オフセット（191 と 220）・波形・`ext_rate_per_spad`（1152 と 1160）がほぼ同じである。壁の戻りが抽出の窓（区間 0〜4）に混ざった結果なら、壁の距離に応じて窓の中身が変わるはずなので、混ざっていないと判断した。これは推論であり、根拠は `ext_rate_per_spad` と波形の位置が同じことだけである。ただし上のとおり 2 回目は 1 回目の補正が有効なまま行ったので、この一致が 2 つの独立な測定の一致とは言えない（独立に較正し直した場合は未確認）。

### 補正ありの手持ち測定

1500 mm での較正値（平面オフセット 220）が有効な状態で、同じ白い壁について 0.5〜3.0 m の 6 点を測り（`range_Xon_wall_{0.5,1.0,1.5,2.0,2.5,3.0}.txt`）、同じ日の補正なし（`range_L33_wall_*.txt`、4 章 (b)）と比べた。集計は `tofdiag_parse.py` の読み込み関数で再計算した。

| 壁までの距離 | 有効率（なし / あり） | 選ばれた距離の中央値 mm | 最遠の目標の信号 Mcps | `sgm` mm（ドライバが出す距離のばらつきの値） | 30 mm 未満の目標を含む行 | 1 行あたりの目標数 |
|---|---|---|---|---|---|---|
| 0.5 m | 100 % / 100 % | 506 / 534 | 4.820 / 4.250 | 1.9 / 2.1 | 5.8 % / 0.0 % | 1.08 / 1.00 |
| 1.0 m | 100 % / 100 % | 1011 / 1010 | 1.266 / 1.258 | 3.2 / 3.1 | 75.9 % / 0.0 % | 1.98 / 1.00 |
| 1.5 m | 100 % / 100 % | 1561 / 1476 | 0.547 / 0.609 | 5.7 / 5.5 | 63.8 % / 0.0 % | 1.95 / 1.00 |
| 2.0 m | 100 % / 100 % | 2032 / 1999 | 0.297 / 0.344 | 9.0 / 7.4 | 76.4 % / 0.0 % | 1.99 / 1.00 |
| 2.5 m | 100 % / 100 % | 2494 / 2505 | 0.195 / 0.188 | 12.9 / 12.8 | 65.0 % / 0.0 % | 1.95 / 1.00 |
| 3.0 m | 100 % / 100 % | 2973 / 2980 | 0.109 / 0.117 | 18.9 / 16.7 | 64.1 % / 0.0 % | 1.88 / 1.00 |

1 点あたり 203〜208 行。「30 mm 未満の目標」は、状態コードによらず距離が 30 mm 未満（負の値を含む）の目標が 1 つでもある行の割合。補正なしの 1.0 m の値は、依頼時の集計の 76.4 % に対し、記録から数えた値は 75.9 % だった（他の 11 点は一致）。差は集計の数え方の違いと考えられるが、原因は確かめていない。

| 項目 | 内容 | 種別 |
|------|------|------|
| 30 mm 未満の目標 | 補正ありでは 6 点すべてで 0 % になり、目標数が 1 になった（補正なしでは 5.8〜76 %） | 事実 |
| 壁の目標 | 有効率は 100 % のまま。信号 × 距離の 2 乗の中央値は補正なしに対して 0.98〜1.12 倍（2.0 m が 1.12 倍、他の 5 点は 0.98〜1.05 倍）。`sgm` は同程度か小さい。選ばれた距離の中央値の差は、2 つの系列を手持ちで別々の位置に立って測ったことを含む | 事実 |
| 周囲光 | 補正なし 0.133〜0.148 Mcps、補正あり 0.117〜0.125 Mcps（点ごとの中央値の範囲） | 事実 |
| 補正の状態 | 1.5〜3.0 m の 4 点の記録には `TOFCFG` の `xtalk_en=1`、`xtalk_plane_kcps=220` がある。0.5 m と 1.0 m の記録には `TOFCFG` がない。この 2 点は、較正の終了（機体の時刻 313 s）の後の測定で、記録の時刻が途切れず続いている（再起動なし）ことから、同じ状態と判断した | 事実と判断 |
| 弱い近い目標の原因 | 補正で消えたことは、4 章で推定した「センサの手前からの反射（クロストーク）」と矛盾しない。ただし、消えたことだけでは原因がクロストークだと確定したとは言えない（補正が別の原因の目標も一緒に除いた可能性を、今回の記録だけでは分けられない） | 推論 |

### 読み取り

| 項目 | 内容 | 種別 |
|------|------|------|
| 手前の弱い目標 | クロストークの較正で、白い壁に対して全距離で消えた。壁の目標の測定結果は変わらない（差は立ち位置と周囲光の差の範囲） | 事実 |
| 較正の成否 | ドライバの戻り値は成否を表さないので、成否は `cal_status` と代替値の特徴で判断する。今回は 2 回とも成功の側 | 事実 |
| 飛行中の 1.9〜2.2 m での有効率の低下への効果 | 補正が効くかは飛行では試していない | 未確認 |

## 6. 未確認の事項と次に確かめること

| 項目 | 状態 | 次に確かめること |
|------|------|------------------|
| 飛行中の途切れ（1.9 m 以上）の原因 | 未確認 | 飛行中の `TOFDIAG` を記録する |
| クロストークの補正が、飛行中の 1.9〜2.2 m での有効率の低下に効くか | 未確認（5 章: 手持ちの壁では 30 mm 未満の目標が消えたが、飛行では試していない） | 補正を有効にして飛行し、`TOFDIAG` を記録する |
| 床に向けた補正ありの測定 | 未確認（5 章の補正ありの測定は白い壁だけ） | 床に向けて 4 章 (a) と同じ測定を補正ありで行う |
| 補正の `off` → `on` で較正値が保たれるか | 未確認 | `tofcal off`、`tofcal on` の前後で `tofcal show` を比べる |
| 基準 SPAD の較正（`tofcal refspad`） | 未実行 | 実行して、クロストークの較正との順序・結果を確かめる（要約資料は「基準 SPAD → クロストーク → オフセット」の順とする） |
| 較正後の ToF タスクのスタックの残り | 未確認 | 較正の後に `stack` で ToF タスクの最小の空きを読む |
| 近距離（0.3 m 未満）への補正の影響 | 未確認 | 0.3 m 未満で補正なし・ありを比べる |
| 較正の条件の影響（暗い環境・目標の反射率と大きさ・独立な再較正） | 未確認（今回は室内の照明下、白い壁 1 種、1500 mm の較正は 1000 mm の較正が有効な状態から開始） | 補正を無効にしてから較正し直し、条件を変えて較正値を比べる。ST の UM2778 を入手して条件を確かめる |
| ファームウェアに取り込む場合の課題 | 未確認（較正値の保存先（NVS: 不揮発メモリ）、較正をいつ・誰が・どの姿勢で行うかの手順は未設計） | 設計を決めて実装する |
| MEDIUM で見られた「範囲外で手前の弱い目標を有効として返す」誤りが、補正で防げるか | 未確認（試していない） | MEDIUM で補正ありの 2.0〜3.0 m を測る |
| 2 m を超える高さでの床の測定 | 未確認（手持ちで届いたのは約 2.0 m まで） | 高い所から床を測る |
| 手前 0.5 m 付近の別の目標が持ち手の体であること | 未確認（推定） | 体が入らない配置で測る |
| 弱い近い目標がセンサの手前からの反射であること | 未確認（推定。5 章で較正により消えたが、原因がクロストークと確定したとは言えない） | 床向きの補正ありの測定、条件を変えた較正の前後の比較 |
| 発光の周期から計算した範囲の式 | 未確認（測定を行った側が使った近似式。ソースの `range_maths` の定数からの直接の導出は、手元の読みでは約 4 倍の差が出て、一致を確かめられなかった。MEDIUM の実測とは合う） | 式の出どころを確かめる |
| 飛行ログの調査の計算 | 独立の確認なし | 別の手段で再計算する |
| 読み出し時間: 目標が複数のとき、I2C の間の CPU の空き、ARM 中・飛行中の値 | 未確認 | 3 章の未確認の表のとおり |
| 機体 1 台だけの測定（較正値の機体ごとの差を含む） | 1 台 | 他の機体でも測る |

---

<a id="english"></a>

# Measurement Record of the VL53L3CX ToF Sensor on StampFly

## 1. Overview

### About This Document

This document summarizes measurements on one vehicle of the ToF sensors (Time of Flight: sensors that measure distance from the time emitted light takes to come back) VL53L3CX, one facing down and one facing forward on StampFly. It covers three things.

| Content | Chapter |
|---------|---------|
| Breakdown of the time of one read-out | 3 |
| Upper limit of ranging (analysis of flight logs and hand-held experiments) | 4 |
| Crosstalk calibration experiment (calibration results and hand-held measurement with the correction) | 5 |

Facts, inferences and unconfirmed items are written separately. Nothing that was not measured is written.

### Common Measurement Conditions

| Item | Content |
|------|---------|
| Date | The night of 2026-11-21 to 2026-11-22 |
| Vehicle | One vehicle. On the floor or hand-held, not armed, propellers fitted, battery connected (4.16-4.19 V), all sensors running (`sensors present=0x7F` in `status`), USB through a hub with a power switch |
| Firmware | Based on f4b72074 of fix/flight-safety (`App version archive/2026-11-13-31-gf4b72074`) with comparison changes added in a working tree outside Git (described in each chapter). After the measurements the vehicle was returned to the baseline build (`logs/checkout/restore_A_20261122.txt` after chapters 3 and 4, `logs/checkout/restore_A_20261122b.txt` after chapter 5) |
| Records | `logs/checkout/` (outside Git). The changes made and the tools for aggregation are kept in the same place (table below) |

| Record or tool | Content |
|----------------|---------|
| `toftime_floor_1.txt` | Record of chapter 3 (75 s) |
| `range_L33_*.txt`, `range_M33_*.txt`, `range_L33_check.txt` | Records of the hand-held experiments of chapter 4 |
| `tof_timing.patch`, `toftime_parse.py` | The measurement change of chapter 3 (a Git patch) and its aggregation |
| `tof_range.patch`, `tofdiag_parse.py`, `range_capture.py`, `move_an.py`, `run_variant.sh` | The experiment change of chapter 4 (a Git patch), aggregation, recording, analysis of the movement, and the procedure from flashing to measuring |
| `tof_xtalk.patch`, `xtalk_cal.py` | The experiment change of chapter 5 (a Git patch; adds the `tofcal` command and diagnostics) and the procedure for running and recording the calibration |
| `xtalk_show_before.txt`, `xtalk_cal_w1000.txt`, `xtalk_cal_w1500.txt` | The state before calibration and the records of the two calibrations of chapter 5 |
| `range_Xon_wall_{0.5,1.0,1.5,2.0,2.5,3.0}.txt` | Records of the hand-held measurement with the correction of chapter 5 (the comparison without it is `range_L33_wall_*.txt`) |
| `restore_A_20261122b.txt` | Record of returning to the baseline build after chapter 5 |

### Summary of Conclusions

| Item | Content | Kind |
|------|---------|------|
| Time of one read-out | Mean 5839 µs for the downward sensor and 5470 µs for the front one. 75-80 % of the time is I2C transfer. The histogram calculation is 7-10 % | Fact (on the floor, one target) |
| Developer's view | "The histogram calculation is slow." In the measurement most of the time was I2C | Fact |
| Ranging mode | The vehicle firmware uses LONG. When stationary, the valid rate was 95-100 % up to 3.0 m on a white wall (limited by the room; farther was not measured) and up to about 2.0 m on the floor (the height the hand could reach) | Fact |
| MEDIUM | Valid up to 1.5 m from the wall, a valid rate of 1.0 % at 2.0 m, and a wrong near distance returned as valid at 2.5-3.0 m. Not usable | Fact |
| Valid rate in flight logs | Against the barometric height, 64 % at 1.9-2.0 m and 22 % at 2.0-2.2 m | Fact (calculated by a sub-agent, no independent check) |
| Weak near target | Weak targets under 30 mm became 0 % at all 6 points from 0.5 to 3.0 m on the white wall (5.8-76 % without it) once the crosstalk calibration (in RAM, the 1500 mm value on a white wall) was enabled. The valid rate, signal and distance of the wall target did not change. This does not establish that the cause is crosstalk (chapter 5) | Fact (that they disappeared), inference (the cause) |
| Crosstalk calibration | Run twice under room lighting rather than in a dark environment (1000 mm and 1500 mm from the wall); the extraction succeeded both times (`cal_status=0`, not the substitute values). The value is lost at power off. The effect on the drop of the valid rate in flight, measurement facing the floor and the way to store the value are unconfirmed (chapters 5 and 6) | Fact, unconfirmed |
| Cause of the dropouts in flight | Not reproduced by hand-held stationary, moving or tilted cases. The cause is unconfirmed | Unconfirmed |

## 2. Configuration and Settings

### Driver and Ranging Mode

| Item | Content | Source |
|------|---------|--------|
| Component | `firmware/vehicle/components/sf_hal_vl53l3cx/` (ST bare driver 1.2.14 and its wrapper) | Source |
| Driver version | 1.2.14.2687 | `TOFCFG` line from the real device |
| Ranging mode | LONG (`VL53LX_DISTANCEMODE_LONG` passed to `VL53LX_SetDistanceMode`, line 209 of `vl53l3cx_wrapper.cpp`) | Source, `TOFCFG` line (`mode=LONG preset=27`) |
| Timing budget | 33 ms (`timing_budget_ms`, around line 217 of the same file). About 30 Hz | Source, `TOFCFG` line |
| Crosstalk correction | Disabled (`xtalk_en=0`, calibration value 0). Crosstalk means reflection from parts in front of the sensor mixing into the reflection of the object to be measured | `TOFCFG` line |
| Extended ranging | Enabled (`uwr=1`) | `TOFCFG` line |
| Fast oscillator value | `fast_osc=48455` | `TOFCFG` line |
| I2C speed | 400 kHz (line 153 of `vl53l3cx_wrapper.cpp`) | Source |
| Task | `TofTask`. Core 0, priority 14, period 33 ms (`tasks/tof_task.cpp`, `main/config.hpp`). Reads the downward sensor first, then the front one | Source |

### Validity Judgement and Target Selection

One measurement returns at most 4 targets (objects that reflected the light). The wrapper chooses one with the following rule.

| Step | Content | Source |
|------|---------|--------|
| Candidates | Targets with status code 0 and a distance above 30 mm (`MIN_VALID_DISTANCE_MM = 30`) | `vl53l3cx_wrapper.cpp` lines 299-311 |
| Selection | The farthest of the candidates. The comment says "for altitude measurement the ground is usually the farthest" | Same, lines 305-316 |
| No candidate | Distance 0, status is that of the first target (254 if that is 0). If there is no target, status 255 | Same |
| Validity | `valid` when the status of the selected target is 0 (line 126 of `tof_task.cpp`) | Source |

### Meaning of the Status Codes

The definitions of the driver (`include/vl53lx/vl53lx_def.h`, lines 286-318) and the values added by the wrapper are as follows.

| Code | Name | Meaning (translation of the driver's comment) |
|------|------|-----------------------------------------------|
| 0 | RANGE_VALID | The range is valid |
| 1 | SIGMA_FAIL | The sigma (spread) check failed |
| 2 | SIGNAL_FAIL | The signal check failed |
| 3 | RANGE_VALID_MIN_RANGE_CLIPPED | The target is below the minimum detection threshold |
| 4 | OUTOFBOUNDS_FAIL | The phase is outside the valid limits (different from an exit by the wrap check) |
| 5 | HARDWARE_FAIL | Hardware failure |
| 6 | RANGE_VALID_NO_WRAP_CHECK_FAIL | The range is valid but the wrap check has not been done |
| 7 | WRAP_TARGET_FAIL | Wrapped target: no matching phase in the other emission period |
| 8 | PROCESSING_FAIL | Internal arithmetic overflow |
| 9 | XTALK_SIGNAL_FAIL | Specific to simple ranging |
| 10 | SYNCRONISATION_INT | The first interrupt after the start of continuous ranging. Ignore the data |
| 11 | RANGE_VALID_MERGED_PULSE | The range is valid but is the result of several merged pulses |
| 12 | TARGET_PRESENT_LACK_OF_SIGNAL | A target is present but the signal is insufficient |
| 13 | MIN_RANGE_FAIL | Unexpected error in the SPAD array (the array of light-receiving elements) |
| 14 | RANGE_INVALID | The calculated range is negative |
| 254 | (added by the wrapper) | All targets invalid |
| 255 | NONE (driver definition); the wrapper also uses it for no target | No update; no target at all |

## 3. Breakdown of the Read-out Time

### How It Was Measured

| Item | Content |
|------|---------|
| Record | `logs/checkout/toftime_floor_1.txt` (75 s, on the floor) |
| Measurement build | Timing with `esp_timer_get_time` (a microsecond clock) was added to the I2C input/output of `vl53lx_platform.c` and to the entry of the histogram processing in `vl53lx_platform_ipp.c`. `TofTask` prints a `TOFTIME` line every 10 s |
| Aggregation | `toftime_parse.py` (the mean of each line weighted by the number of measurements) |
| Number of measurements | 2129 for the downward sensor and 2128 for the front one, with 0 read failures |

### Result (Fact)

| Item | Downward | Front |
|------|----------|-------|
| Total of one read-out, mean / maximum | 5839 / 6998 µs | 5470 / 6952 µs |
| I2C (3 transfers, 158 B) | 4356 µs (74.6 %) | 4369 µs (79.9 %) |
| Histogram processing | 555 µs (9.5 %) | 399 µs (7.3 %) |
| Everything else | 928 µs (15.9 %) | 703 µs (12.8 %) |

The breakdown by step for the downward sensor is as follows.

| Step | Time | Of which I2C |
|------|------|--------------|
| Checking whether data is ready | 304 µs | 274 µs |
| Reading and calculating the result | 3403 µs | 2135 µs (reading 83 B) |
| Clearing the interrupt and starting the next measurement | 2132 µs | 1946 µs (writing 68 B) |

For the front sensor the same three steps are 326 µs (I2C 305), 3102 µs (2143) and 2042 µs (1920).

| Note | Content |
|------|---------|
| I2C speed | 400 kHz |
| Sum of the two (calculated) | 5839 + 5470 = 11309 µs, about 34 % of the 33 ms period |
| Histogram processing | Integer arithmetic including 64-bit division. The code is about 6.1 KB, about 18 % of the whole driver (about 33 KB) (aggregated from the build map) |
| Condition | On the floor, with one target |

Contrary to the developer's view (the histogram calculation is slow), most of the time was I2C.

### Unconfirmed

| Item | Status |
|------|--------|
| Calculation time with several targets | Unconfirmed |
| Whether the CPU is free between the I2C transfers (whether other tasks can run while waiting) | Unconfirmed |
| Read-out time while armed and in flight (motors running) | Unconfirmed |

## 4. Upper Limit of Ranging

### Analysis of Flight Logs

196 flight logs from 2026-06-11 onward were examined against the barometric height. The figures and tables were kept in a working place and are not referred to from this document; only the key numbers are written. **The calculation was done by a sub-agent and has not been checked independently.**

For the 46 logs of the newer firmware, the valid rate (fraction of status code 0) by height against the barometric height is as follows.

| Height (barometric) | Valid rate | Count |
|---------------------|------------|-------|
| 1.0-1.5 m | 99.6-99.8 % | 271-964 (per 0.1 m) |
| 1.5-1.8 m | 97.9-98.5 % | 137-243 |
| 1.8-1.9 m | 96.2 % | 79 |
| 1.9-2.0 m | 64.0 % | 25 |
| 2.0-2.1 m | 22.2 % | 45 |
| 2.1-2.2 m | 22.2 % | 18 |

| Item | Content |
|------|---------|
| Largest reliable distance to the floor | 2.04 m (log 20260618T185844) |
| Status codes of the flat dropouts (invalid while the barometric height stays constant) | 255 (no target) and 12 (target present but signal insufficient) |
| Error of the barometric reference | The rms (root mean square) of the residual is about 0.23 m. An error of this size is in the height bins |

`kAltitudeCeilingM = 1.2 m` (a candidate value of the altitude ceiling) in `firmware/vehicle/components/sf_core/include/flight_limits.hpp` takes "the ToF valid distance of about 1.6 m (in flight logs, 95-99 % valid up to 1.7 m, dropping beyond)" as one of its bases. The values above do not differ greatly from that basis. This document does not propose a change of the ceiling.

### Method of the Hand-held Experiments

| Item | Content |
|------|---------|
| Experiment build | On every read of the downward sensor it prints the distance, status code, signal rate, ambient light and sigma of all targets in a `TOFDIAG` line. The ranging mode and timing budget can be chosen in the build settings (`tof_range.patch`; when not chosen, the flight values LONG and 33 ms). The settings are printed in a `TOFCFG` line. The front sensor was not changed |
| Aggregation | `tofdiag_parse.py`. It gives the valid rate, the median distance, the signal rate and so on per point. The numbers in this document are values re-aggregated from the records |
| Settings line | LONG, timing budget 33 ms, driver 1.2.14.2687, `xtalk_en=0` (calibration value 0), `uwr=1`, `fast_osc=48455` |
| Definition of valid rate | The fraction of lines whose selected target has status 0 and a distance above 0 |
| Signal rate | Unit Mcps (mega counts per second, the rate of photons counted by the light-receiving elements) |

The vehicle was held by hand. The heights were the guided ones; the tables below are organized by the median of the distance actually measured.

### (a) On the Floor (LONG, 33 ms, Stationary)

| Median of the actual distance | Valid rate | Signal rate [Mcps] |
|-------------------------------|------------|--------------------|
| 0.52 m | 100 % | 3.88 |
| 0.79 m | 100 % | 1.65 |
| 1.04 m | 100 % | 0.95 |
| 1.31 m | 100 % | 0.63 |
| 1.50 m | 100 % | 0.50 |
| 1.68 m | 100 % | 0.45 |
| 1.90 m | 95.2 % | 0.24 |
| 2.01 m | 100 % | 0.17 |

The ambient light was 0.125-0.133 Mcps. The highest point the hand could reach was about 2.0 m; at 1.9 m and above another target is seen near 0.5 m in front (presumed to be the body of the person holding it; this is a presumption and was not confirmed).

### (b) White Wall (LONG, 33 ms, Stationary)

The wallpaper is white with a finely uneven surface.

| Median of the actual distance | Valid rate | Signal rate [Mcps] |
|-------------------------------|------------|--------------------|
| 0.51 m | 100 % | 4.82 |
| 1.01 m | 100 % | 1.27 |
| 1.56 m | 100 % | 0.55 |
| 2.03 m | 100 % | 0.30 |
| 2.49 m | 100 % | 0.20 |
| 2.97 m | 100 % | 0.11 |

The ambient light was 0.133-0.148 Mcps.

### (c) White Wall (MEDIUM, 33 ms, Stationary)

| Guided distance from the wall | Result |
|-------------------------------|--------|
| 0.53 m, 0.93 m, 1.50 m (median of the actual distance) | Valid rate 100 % |
| 2.0 m | Valid rate 1.0 % (status 4 in 105 lines, 7 in 101 lines, 0 in 2 lines) |
| 2.5 m | Valid rate 43.2 %, but the median of the distances accepted as valid is 118 mm (a wrong near distance) |
| 3.0 m | Valid rate 27.2 %, the same median 44 mm (a wrong near distance) |

In the sum of the 6 points, 17 lines had a target with the mark showing that extended ranging acted (`x=1` in `TOFDIAG`).

### (d) Moving above the Floor (LONG, 33 ms)

| Movement | Lines | Valid rate | Note |
|----------|-------|------------|------|
| Slow up and down | 1504 | 100 % | |
| Fast up and down | 1174 | 99.2 % | The 9 invalid lines are all status 7, one line each. The maximum rate of change of the distance is about 5.5 m/s (calculated from adjacent lines) |
| Tilting about 20 degrees forward, back, left and right overhead | 1064 | 99.9 % | 1 invalid line (status 7) |

The distance was 0.66-1.93 m.

### Common Observations

| Item | Content |
|------|---------|
| Weak near target | At points above 0.75 m, a weak target at 1-29 mm with a signal of 0.04-0.12 Mcps appears. The fraction of lines where it appears is 2-59 % per point. It is presumed to be reflection from in front of the sensor (crosstalk). This is a presumption and was not confirmed at this point (in chapter 5 it disappeared with the crosstalk calibration) |
| Relation to the selection | The selection of chapter 2 excludes distances of 30 mm or less and takes the farthest valid target, so the floor or wall was chosen in LONG |
| Signal rate × distance squared | 1.02-1.28 on the floor (up to 1.7 m) and 0.96-1.33 on the wall. The wall is about 1.2 times the floor (calculated). This agrees with the expectation that the signal falls in inverse proportion to the square of the distance |

### Range Calculated from the Emission Period

ToF obtains the distance from the phase of the light. An object beyond the emission period wraps around and appears at a near distance. The range was calculated with the following expression (an approximation used by the side that made the measurement, using `fast_osc`; it was not derived directly from the constants in the source; see chapter 6).

$$d_{\text{pclk}} \approx 198.4 \times \frac{48332}{f_{\text{osc}}}\ \text{[mm]}$$

$d_{\text{pclk}}$ is the distance corresponding to one emission period unit (pclk) and $f_{\text{osc}}$ is the value of `fast_osc` (about 197.9 mm at 48455).

| Mode | Emission periods | Upper limit of the valid phase | Calculated range (periods × 197.9 mm) |
|------|------------------|--------------------------------|----------------------------------------|
| LONG | 20 pclk and 24 pclk | 17 pclk | 3.96 m and 4.75 m from the periods, 3.36 m from the valid upper limit |
| MEDIUM | 12 pclk and 16 pclk | 9 pclk | 2.37 m and 3.17 m from the periods, 1.78 m from the valid upper limit |

The sources are as follows (line numbers are those of the source at this time).

| Content | Source |
|---------|--------|
| LONG periods (register values 0x09 and 0x0B) and valid phase upper limit 0x88 | `src/vl53lx/vl53lx_api_preset_modes.c` lines 1132 and 1169 |
| MEDIUM periods (register values 0x05 and 0x07) and valid phase upper limit 0x48 | Same, lines 1254 and 1291 |
| Decoding of the period ((register value + 1) × 2) | `src/vl53lx/vl53lx_core_support.c` line 465 |
| Conversion from phase to distance | Same, line 403 |
| Judgement by the valid phase upper limit (gives status 4) | `src/vl53lx/vl53lx_hist_algos_gen3.c` lines 835-849 |

The calculated valid upper limit of MEDIUM (about 1.78 m) agrees with the measurement of (c) (valid at 1.5 m, dropout at 2.0 m).

### Reading

| Item | Content | Kind |
|------|---------|------|
| LONG setting | Not a mistake. When stationary it measured up to 3.0 m (the limit of the room) on a white wall | Fact |
| MEDIUM | It drops out between 1.5 and 2.0 m and returns a wrong near distance as valid out of range, so it is not usable. The statement "MEDIUM reaches farther indoors" in the bundled ST document AN5561 was not reproduced on this vehicle | Fact |
| Disagreement with flight logs | In flight logs the valid rate falls to 64 % at 1.9-2.0 m and to 22 % at 2.0-2.2 m. This was not reproduced by hand-held stationary, moving or tilted cases. The height reached by hand, however, was only about 2.0 m | Fact |
| Cause of the dropouts in flight | Candidates are the floor of the place where it flew, conditions with the motors running (vibration, power supply) and crosstalk (the measurements of this chapter have no correction; the measurement with the correction in chapter 5 was hand-held only, and not tried in flight) | Unconfirmed |

## 5. Crosstalk Calibration Experiment

### Purpose and Method

Chapter 4 found weak targets at 1-29 mm and presumed them to be reflection from in front of the sensor (crosstalk: part of the emitted light that does not reach the target but enters the receiver directly through the inside of the sensor or its window). To test this presumption, a crosstalk calibration (an operation that measures the amount of crosstalk and produces a value to subtract from the ranging result) was performed on the vehicle on the same day, and the same hand-held experiments were compared before and after it.

| Item | Content |
|------|---------|
| Experiment build | f4b72074 plus the console command `tofcal xtalk [mm] \| off \| on \| show \| refspad`, in addition to the existing diagnostics (`TOFDIAG` and `TOFCFG` lines). The change is `logs/checkout/tof_xtalk.patch` (outside Git) |
| Behavior of the command | The command only files a request. The ToF task stops ranging at the head of its period, calls `VL53LX_PerformXTalkCalibration` of the ST driver, and restarts ranging. It is refused while armed |
| Storage of the calibration value | RAM only. It is lost at power off |
| Ranging setting | LONG, 33 ms (same as chapter 4) |
| Posture during calibration | The vehicle was held by hand with the downward ToF facing a white wall squarely (white wallpaper, fine unevenness). Indoors under room lighting (not a dark environment) |
| Calibration distance | Twice, at 1000 mm and 1500 mm from the wall (the value given to the command) |

### Properties of the Driver (Reading of the Source)

This is what the person in charge confirmed in the source. Shown by function name (`src/vl53lx/vl53lx_api.c`).

| Content | Kind |
|---------|------|
| The return value of `VL53LX_PerformXTalkCalibration` does not show whether the calibration succeeded. It is overwritten by the result of the final resetting of the distance mode and the timing budget | Fact (source) |
| If the extraction fails, fixed substitute values (plane offset 50, shape 307/410/410/307) are put in. The actual state is seen in `LLData.xtalk_results.cal_status` | Fact (source) |
| The correction is enabled at the end of the calibration | Fact (source) |
| The driver has a dynamic correction for smudge (smudge correction), but `VL53LX_DataInit` disables it (`VL53LX_SMUDGE_CORRECTION_NONE`). It stayed disabled this time too | Fact (source) |
| On the calibration conditions, section 6 of the in-repository summary `firmware/vehicle/components/sf_hal_vl53l3cx/docs/VL53L3CX_driver_doc.md` says "reference SPAD (light-receiving element) -> crosstalk -> offset in this order; crosstalk in a dark environment at 600 mm". The description of the ST function (bundled `VL53LX_API.chm`) says "the target should be 60 cm or more, and the actual distance is passed as the value". A specification of the target's reflectance or size was not found in the bundled material | Fact (documents) |
| ST UM2778 (the calibration document) is not bundled and was not read | Unconfirmed |

The calibration was done under room lighting, so the condition differs from the "dark environment" the summary asks for.

### Calibration Results

Records: `xtalk_cal_w1000.txt` and `xtalk_cal_w1500.txt`. The state before calibration is in `xtalk_show_before.txt` (correction disabled, plane offset 0, shape all 0).

| Item | 1000 mm | 1500 mm |
|------|---------|---------|
| Plane offset (raw value / divided by 512) | 191 / 0.373 | 220 / 0.430 |
| Shape `shape_bins` (first 5 bins, sum) | 12, 352, 471, 172, 17 (1024) | 27, 389, 456, 160, 0 (1032) |
| `ext_rate_per_spad` | 1152 | 1160 |
| `ext_signal_events` | 3376 | 3400 |
| Extraction window / start bin of the target | 0..4 / 5 | 0..4 / 8 |
| `cal_status` / `fallback` | 0 / 0 | 0 / 0 |
| Duration `elapsed_ms` | 1508 | 1510 |

The following can be read from the records as facts.

| Item | Content | Kind |
|------|---------|------|
| Success of the extraction | `cal_status=0` and `fallback=0` both times. The values do not match the signature of the substitute values (plane offset 50, shape 307/410/410/307) | Fact |
| Side effect of the calibration | After calibration `nodetect_kcps` (the driver parameter `DYNXTALK_NODETECT_XTALK_OFFSET_KCPS`) changed from 410 to the same value as the plane offset (191, 220). The driver sets it | Fact |
| Start state of the 1500 mm calibration | It started with the 1000 mm calibration value still enabled (`en=1`, plane offset 191). The two are not independent calibrations; the second one is a calibration from the state left by the first | Fact |
| Interruption of ranging | The ranging output stops for about 1.5 s during calibration. Restart takes 7 ms | Fact |

At the two distances the plane offsets (191 and 220), the shapes and `ext_rate_per_spad` (1152 and 1160) are almost the same. If the return from the wall had mixed into the extraction window (bins 0-4), the contents of the window would change with the wall distance, so it was judged not to have mixed in. This is an inference; its only basis is that `ext_rate_per_spad` and the position of the shape are the same. However, as above, the second calibration was run with the first correction still enabled, so this agreement cannot be called agreement of two independent measurements (calibrating again independently is unconfirmed).

### Hand-held Measurement with the Correction

With the 1500 mm calibration value (plane offset 220) enabled, 6 points from 0.5 to 3.0 m were measured on the same white wall (`range_Xon_wall_{0.5,1.0,1.5,2.0,2.5,3.0}.txt`) and compared with the correction-off measurement of the same day (`range_L33_wall_*.txt`, (b) of chapter 4). The aggregation was recalculated with the reading function of `tofdiag_parse.py`.

| Distance to the wall | Valid rate (off / on) | Median selected distance mm | Signal of the farthest target Mcps | `sgm` mm (distance-spread value given by the driver) | Lines containing a target under 30 mm | Mean number of targets per line |
|---|---|---|---|---|---|---|
| 0.5 m | 100 % / 100 % | 506 / 534 | 4.820 / 4.250 | 1.9 / 2.1 | 5.8 % / 0.0 % | 1.08 / 1.00 |
| 1.0 m | 100 % / 100 % | 1011 / 1010 | 1.266 / 1.258 | 3.2 / 3.1 | 75.9 % / 0.0 % | 1.98 / 1.00 |
| 1.5 m | 100 % / 100 % | 1561 / 1476 | 0.547 / 0.609 | 5.7 / 5.5 | 63.8 % / 0.0 % | 1.95 / 1.00 |
| 2.0 m | 100 % / 100 % | 2032 / 1999 | 0.297 / 0.344 | 9.0 / 7.4 | 76.4 % / 0.0 % | 1.99 / 1.00 |
| 2.5 m | 100 % / 100 % | 2494 / 2505 | 0.195 / 0.188 | 12.9 / 12.8 | 65.0 % / 0.0 % | 1.95 / 1.00 |
| 3.0 m | 100 % / 100 % | 2973 / 2980 | 0.109 / 0.117 | 18.9 / 16.7 | 64.1 % / 0.0 % | 1.88 / 1.00 |

There were 203-208 lines per point. "A target under 30 mm" is the fraction of lines that contain at least one target whose distance is under 30 mm (including negative values), whatever its status code. For correction-off at 1.0 m, the requester's aggregation gave 76.4 % and the value counted from the record was 75.9 % (the other 11 points agree). The difference is probably a difference in how it was counted, but the cause was not checked.

| Item | Content | Kind |
|------|---------|------|
| Targets under 30 mm | With the correction they were 0 % at all 6 points and the number of targets was 1 (5.8-76 % without it) | Fact |
| Wall target | The valid rate stayed at 100 %. The median of signal × distance squared was 0.98-1.12 times that without the correction (1.12 times at 2.0 m, 0.98-1.05 times at the other 5 points). `sgm` was the same or smaller. The differences in the median selected distance include the fact that the two series were measured with the hand standing at different positions | Fact |
| Ambient light | 0.133-0.148 Mcps without the correction, 0.117-0.125 Mcps with it (range of the per-point medians) | Fact |
| State of the correction | The records of the 4 points from 1.5 to 3.0 m contain `TOFCFG` with `xtalk_en=1` and `xtalk_plane_kcps=220`. The records of 0.5 m and 1.0 m contain no `TOFCFG`. These two points were measured after the end of the calibration (vehicle time 313 s) and the time in the records continues without a break (no restart), so the state was judged to be the same | Fact and judgement |
| Cause of the weak near target | That it disappeared with the correction does not contradict the presumption of chapter 4 that it is reflection from in front of the sensor (crosstalk). However, its disappearance alone does not establish that the cause is crosstalk (whether the correction also removed targets of another cause cannot be separated by this record alone) | Inference |

### Reading

| Item | Content | Kind |
|------|---------|------|
| Weak near target | With the crosstalk calibration it disappeared at all distances on the white wall. The measured wall target did not change (the differences are within the differences of position and ambient light) | Fact |
| Success of the calibration | The return value of the driver does not show success, so success is judged from `cal_status` and the signature of the substitute values. Both times it was on the success side | Fact |
| Effect on the drop of the valid rate at 1.9-2.2 m in flight | Whether the correction helps was not tried in flight | Unconfirmed |

## 6. Unconfirmed Items and What to Check Next

| Item | Status | What to check next |
|------|--------|--------------------|
| Cause of the dropouts in flight (1.9 m and above) | Unconfirmed | Record `TOFDIAG` in flight |
| Whether the crosstalk correction helps with the drop of the valid rate at 1.9-2.2 m in flight | Unconfirmed (chapter 5: targets under 30 mm disappeared on the hand-held wall, but it was not tried in flight) | Fly with the correction enabled and record `TOFDIAG` |
| Measurement with the correction facing the floor | Unconfirmed (the measurement with the correction in chapter 5 was on a white wall only) | Repeat (a) of chapter 4 facing the floor with the correction |
| Whether the calibration value is kept across `off` -> `on` of the correction | Unconfirmed | Compare `tofcal show` before and after `tofcal off` and `tofcal on` |
| Reference SPAD calibration (`tofcal refspad`) | Not run | Run it and check the order and the result with the crosstalk calibration (the summary document gives the order reference SPAD -> crosstalk -> offset) |
| Remaining stack of the ToF task after the calibration | Unconfirmed | Read the minimum free stack of the ToF task with `stack` after a calibration |
| Effect of the correction at short range (under 0.3 m) | Unconfirmed | Compare without and with the correction under 0.3 m |
| Effect of the calibration conditions (dark environment, reflectance and size of the target, independent re-calibration) | Unconfirmed (this time: room lighting, one kind of white wall, and the 1500 mm calibration started from the state with the 1000 mm calibration enabled) | Calibrate again after disabling the correction and compare the values under changed conditions. Obtain ST UM2778 and check the conditions |
| Issues for taking it into the firmware | Unconfirmed (where to store the calibration value (NVS: non-volatile memory), and the procedure of when, by whom and in what posture to calibrate, are not designed) | Decide the design and implement it |
| Whether the correction prevents the error seen in MEDIUM of returning a weak near target as valid when out of range | Unconfirmed (not tried) | Measure 2.0-3.0 m in MEDIUM with the correction |
| Measurement of the floor above 2 m | Unconfirmed (the hand reached only about 2.0 m) | Measure the floor from a higher place |
| That the other target near 0.5 m in front is the body of the person holding it | Unconfirmed (presumption) | Measure with an arrangement where the body is not in view |
| That the weak near target is reflection from in front of the sensor | Unconfirmed (presumption; it disappeared with the calibration in chapter 5, but that does not establish crosstalk as the cause) | Measure facing the floor with the correction, and compare before and after calibrations under changed conditions |
| The expression for the range calculated from the emission period | Unconfirmed (an approximation used by the side that made the measurement; a direct derivation from the constants of `range_maths` in the source gave about a 4-fold difference in a reading at hand, so agreement could not be confirmed. It agrees with the measurement of MEDIUM) | Check where the expression comes from |
| Calculation of the flight-log analysis | No independent check | Recalculate by another means |
| Read-out time: with several targets, CPU idle between I2C transfers, values while armed and in flight | Unconfirmed | As in the unconfirmed table of chapter 3 |
| Measurement of one vehicle only (including vehicle-to-vehicle differences of the calibration value) | One vehicle | Measure other vehicles too |
