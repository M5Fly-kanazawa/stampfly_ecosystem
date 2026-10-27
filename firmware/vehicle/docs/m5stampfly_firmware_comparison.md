# M5StampFly 旧ファームウェアと vehicle の比較レビュー（Comparison with the M5StampFly Firmware）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

作成日: 2026-10-28。ファイルの場所は `firmware/vehicle` からの相対パスで書く（`components/` と `tasks/` は省くことがある）。

- **観点**：M5Stack が販売している StampFly の機体ファームウェアを、stampfly_ecosystem の vehicle ファームウェアに置き換えられるか。
- **並べ方**：置き換えを妨げる点、現ユーザーが失うものや操作が変わるもの、vehicle で良くなる点の順に書く。vehicle を勧める資料に偏らないよう、問題点を先に置いた。
- **比較対象**
  - 旧：`m5stack/M5StampFly` の main（コミット fa7b6a5）。PR #6（Arduino-ESP32 3.x 対応）はまだ取り込まれていないので、取り込む前の状態を見た。
  - vehicle：`M5Fly-kanazawa/stampfly_ecosystem` の `firmware/vehicle`（コミット 884f2af、2026-10-24 時点の main と同じ）。
- **方法**：ソースコードを読んで比べた。ビルドも実機での試験もしていない。
  - 次の根拠は、該当するソースの行を直接開いて確かめた。
    - A1〜A3、A5
    - A4 のうち SNN と autotune の起動経路
  - それ以外は調査エージェントが読んだ結果で、ファイル名と行番号を付けている。
- **用語**
  - NVS：ESP32 のフラッシュ上にある、設定値を電源を切っても保存しておく領域。
  - タスク：FreeRTOS（ESP32 上で動く小さな OS）で、並行して動く処理の単位。
  - CI：変更のたびに自動でビルドや検査を実行する仕組み。
  - CLI：文字でコマンドを打ち込んで操作する画面。
  - API：外部のプログラムから機体を操作するための窓口（ここでは UDP で受けるテキストコマンド）。
  - Tello：DJI 社などの小型ドローン。vehicle はそのコマンド体系と互換の API を持つ。
  - SNN：スパイキングニューラルネットワーク。神経細胞の発火（パルス）を模したニューラルネットワーク。
  - AHRS：姿勢・方位の推定。
  - Data Stream：vehicle が Wi-Fi で PC へ送る高速の計測データ（400 Hz のサンプルを 50 Hz でまとめて送る）。

### 結論

1. **機体ファームウェアとしての機能・安全機能・保守性は、vehicle が大きく上回る。**
   - ピン割り当ては両者で同じ。
   - パーティション構成は、StampS3（フラッシュ 8 MB、PSRAM なし）に収まる。
2. **置き換えは機体だけでは完結しない。**
   - vehicle は 14 バイトの制御パケット以外を受け付けない。
   - 旧世代のコントローラ（AtomJoy2024June 系、25 バイト・float 形式）とはペアリングも操縦もできない。
   - そのため、コントローラのファームウェアも同時に入れ替える必要がある。
3. **出荷前に直すか、方針を決める必要がある点が 5 つある。**
   - BMI270 のライセンス表記の不整合
   - 研究用の機能（未学習の SNN 制御器など）が既定のビルドに入っていること
   - 飛行中の DISARM 操作の意味が変わること
   - 通信が途切れた直後の 3 秒間の動作が、要件（ホバー維持）と実装（最後のスティック入力を保持）で食い違っていること
   - StampFly V1.1 での動作確認

### 比較表

| 項目 | 旧（M5StampFly） | vehicle |
|---|---|---|
| 開発環境 | Arduino（PlatformIO、Arduino-ESP32 のバージョンは指定なし） | ESP-IDF 5.5.2（`dependencies.lock` で固定）、`sf` コマンドで導入 |
| 自前コードの規模 | src/ 約 3,500 行 | tasks 約 7,200 行＋各コンポーネント（数万行。ST の ToF ドライバは除く） |
| 処理の構成 | Arduino の loop 1 本。タイマ割り込みのフラグを待つ空ループで 400 Hz | FreeRTOS タスク 16 個。制御は core 1、センサは core 0。IMU タイマ駆動で 400 Hz |
| 使うセンサ | IMU（BMI270）、底面 ToF、電源モニタ（INA3221） | 上記＋オプティカルフロー（PMW3901）、気圧（BMP280）、地磁気（BMM150）。気圧と地磁気は既定では融合しない |
| 状態推定 | Madgwick（姿勢）＋高度の 3 状態カルマンフィルタ | 15 状態の誤差状態カルマンフィルタ（ESKF） |
| 飛行モード | 角度／角速度 × 手動スロットル／高度維持、フリップ | ACRO／STABILIZE／ALT_HOLD／POS_HOLD（オプティカルフローで位置保持）。フリップは既定で無効 |
| I2C ドライバ | Arduino の Wire1 と ESP-IDF の旧ドライバを同じポートで併用。戻り値は未確認のまま | ESP-IDF の新しい `i2c_master` ドライバに統一。エラーは上位に返す |
| ARM 前の確認 | なし | 電池電圧、起動時の校正の完了、姿勢と重力方向の一致、ペアリング中でないこと |
| 通信断 | 100 ms で自動着陸へ（壊れたパケットでも「接続中」扱い） | 500 ms で検出し、最後のスティック入力のまま 3 秒飛び続けてから着陸へ（A5） |
| 電池電圧の低下 | LED 表示だけ | 3.4 V で警告、3.0 V で着陸へ |
| 衝撃・異常回転 | 2 g 超でモータ停止 | 3 g 超、または角速度 800 deg/s 超でモータ停止 |
| 設定の保存 | なし（ゲインはすべて定数） | NVS にパラメータ、ペアリング相手、地磁気の校正値を保存 |
| 外部との接続 | ESP-NOW のみ（コントローラへ 40 Hz でテレメトリを送る） | ESP-NOW（制御の受信のみ）、USB／TCP の CLI、Tello 互換の UDP API、Data Stream、UDP テレメトリ |
| 試験 | なし | PC 上の単体試験 約 3,100 行（CI からの実行は見つからず）、CI での SILS 再確認試験 |
| ライセンス表記 | Bosch・ST のヘッダを保持 | BMI270 の表記に不整合あり（A3） |

## 2. 置き換えを妨げる点・出荷前に決めるべき点

### A1. コントローラとの互換性がない（最重要）

両者の制御パケットは形式がまったく異なる。

| | 旧（機体 `src/rc.cpp:56-143`、コントローラ AtomJoy2024June `senddata[25]`） | vehicle（`common/protocol/include/espnow_protocol.hpp:45-56`） |
|---|---|---|
| 長さ | 25 バイト（長さの確認なし） | 14 バイトちょうどでないと捨てる（`sf_comm/comm.cpp:323`） |
| 宛先の確認 | 先頭 3 バイト = 機体 MAC の下位 3 バイト | 同じ |
| スティック | float 4 個（各 4 バイト） | uint16 4 個（12 ビットの AD 値、2048 が中央） |
| ボタン類 | 1 バイトずつ（ARM、FLIP、モード、高度モード、AHRS リセット） | 1 バイトのビットフラグ |
| チェックサム | 先頭 24 バイトの和（byte 24） | 先頭 13 バイトの和（byte 13） |

- **ペアリングの手順と保存が違う。**
  - 機体が一斉送信するペアリング用パケット（11 バイト：チャンネル、MAC、`AA 55 16 88`）は、両者でバイト単位まで同じ。
  - 旧
    - 機体が起動のたびに 2.5 秒間このパケットを送る。
    - 最初に届いたパケットの送信元を通信相手として覚える。
    - 覚えた相手は保存しない。
    - チャンネルは 3 に固定。
  - vehicle
    - 相手が未登録なら自動でペアリング状態に入り、500 ms ごとにパケットを送る。
    - 相手の MAC は NVS に保存する。
    - チャンネルはパラメータで変えられ、既定値は 1。
  - stampfly_ecosystem のコントローラ
    - 起動時に BtnA を押していたときだけペアリング画面に入る。
    - 受信強度の順に並んだ候補から機体を選ぶ。
- **テレメトリが届かなくなる。** 旧の機体は 120 バイトのテレメトリを ESP-NOW でコントローラへ 40 Hz で送る（`rc.cpp:219`）。vehicle が ESP-NOW で送るのはペアリング用パケットだけ（`comm.cpp:501`）。
- **影響**
  - 置き換えは、機体とコントローラの 2 つのファームウェアを同時に入れ替える作業になる。
  - 片方だけ更新した利用者は、ペアリングできない組み合わせを持つことになる。
  - M5Burner（M5Stack のファームウェア書き込みツール）での配布方法と手順書の書き換えが必要になる。
- **選択肢**
  - 移行期間のあいだ、vehicle に旧形式（25 バイト）の受信処理を加える案がある。
  - ペアリング用パケットと宛先確認の方式が共通なので、原理的には実現できそうに見える。
  - ただし、旧コントローラがチャンネル 1 で動くかどうかなどは未確認。

### A2. 飛行中の DISARM（ボタン操作）の意味が変わる

| モード | 旧 | vehicle |
|---|---|---|
| 全モード | 飛行中のボタン操作で自動着陸（`flight_control.cpp:339`） | （下の 2 行に分かれる） |
| ALT_HOLD／POS_HOLD | — | 自動着陸（毎秒 0.3 m で降下） |
| STABILIZE／ACRO | — | **その場でモータを停止**（`sf_state/state_manager.cpp:253-296`） |

vehicle の動作は操作説明書（`operation_manual.md:134`）に書かれており、意図した仕様である。それでも、旧ファームウェアに慣れた利用者が STABILIZE で同じ操作をすると、機体は高い位置から落ちる。製品としてこの仕様でよいか決め、手順書で周知する必要がある。

### A3. ライセンス表記の不整合（BMI270）

- `sf_hal_bmi270/src/bmi270_config_file.c` の中身は、Bosch が配布している BMI270 の設定データ（8,192 バイト）そのものである（ファイル内のコメント:29-31 に記載）。
  - しかしファイル先頭の表記は `SPDX: MIT, Copyright (c) 2025 Kouhei Ito` になっている。
  - Bosch の BMI270_SensorAPI は BSD-3-Clause で、再配布するときは著作権表示を残す必要がある。
  - コンポーネントの `LICENSE` も MIT だけで、Bosch の表示が見当たらない。
- `NOTICE.md` §1.1 には「C driver 部分は Bosch SensorAPI、BSD-3-Clause、アルゴリズム本体は未改変」とある。しかし実際のファイル構成（独自実装＋設定データ）とは合わない。
- `sf_hal_bmi270/docs/bmi270_bosh_doc-34-40.pdf`（データシートの抜粋とみられる）を再配布してよいかは未確認。
- 旧ファームウェアは `lib/bmi270` に Bosch のヘッダと LICENSE を残しており、この点では問題がない。
- **対応**
  - 設定データのファイルに Bosch の著作権表示と BSD-3-Clause の本文を付ける。
  - `NOTICE.md` の記述を実際に合わせる。
  - PDF は削除するか、配布してよいか確認する。

### A4. 研究用・未完成の機能が既定のビルドに入っている

- **SNN 制御器と SNN 推定器**
  - `main/CMakeLists.txt` で、条件なしにビルドに入っている。
  - どちらを使うかはパラメータで決まり、既定値は PID と ESKF。
  - SNN 制御器のヘッダ（`snn_controller.hpp:32-33`）には、「未学習の固定乱数の仮置き。SILS では墜落するのが想定どおりの結果」とある。
  - CLI で `param set controller.type 1` → `param save` とすると、再起動後もこの制御器で動く。
- **自動調整（autotune）**
  - UDP API の `autotune` コマンドで、飛行中に開始できる（`api_task.cpp:1209-1212`）。
  - 得られたゲインはその場で反映される（調査エージェントの読解）。
- **フリップ**：既定で無効（`flip.enable = 0`）。操作説明書には、2026-10-02 の実機での初回フリップが失敗し、原因を解析中と書かれている。
- **対応**：出荷用のビルド構成を別に用意し、研究用の機能はコンパイル時に外す。そうすれば、利用者が誤ってこれらを選ぶことはなくなる。

### A5. 通信が途切れた直後の 3 秒間の動作が、要件と実装で食い違っている

- **要件**
  - `state_manager.cpp:528-530` のコメントは、「通信途絶 → ホバーを維持し、3 秒後に自動着陸（要件 §9）」としている。
  - ログにも `Comm lost → hover 3000 ms then LANDING` と出る。
- **実装**
  - 状態管理は、3 秒のタイマを起動するだけである（`state_manager.cpp:552-557`、`:716-721`）。
  - 制御の目標値（`command_setpoint`）は、パケットを受信したときにしか更新されない（`sf_command/command.cpp:66-68`、`:132`）。
  - 制御タスクは、毎周期その最新値を読む（`control_task.cpp:277`）。
  - 古い目標値を水平に戻す処理は、着陸中（LANDING）にしかない（`pid_controller.cpp:574-590`）。
- **結果**
  - 3 秒間は、途切れる直前のスティック入力（傾き、スロットル）がそのまま使われる。
  - 例：前に倒していたなら、最大 30 度傾いたまま 3 秒進む。
  - 例：ALT_HOLD でスロットルを上げていたなら、上昇を続ける。
- **対応**：通信断の検出時に、目標値を中立に置き換える（姿勢は水平、高度は維持）。それで要件どおりになる。

### A6. 出荷前に確認が必要な点

- **StampFly V1.1**
  - どちらのファームウェアも、機体のリビジョンを判別していない。
  - 両者のピン割り当ては同じ。
  - V1.1 のハードウェアに違いがあるか、vehicle を V1.1 で試したかは未確認。M5Stack に確認が必要。
- **文書の食い違い**：vehicle の `README.md:125` は「Phase 2 進行中、ToF／Baro／Mag／Power／NVS／Logger 残」としている。一方 `docs/feature_status.md` ではこれらは完了扱い。製品向けの文書としてそろえる必要がある。
- **SoftAP（機体が自分で Wi-Fi のアクセスポイントになるモード）**
  - 既定では無効（`wifi.mode = 0`）。
  - 有効にしたときの既定パスワードは `stampfly` で、全機共通（`comm.hpp:261`）。
  - UDP の 8889 番ポートでは、接続した誰からでも Tello 互換の `takeoff` などのコマンドを受け付ける。ARM 前の確認を通れば離陸する。
  - 教室で多数の機体を使うなら、機体ごとに違うパスワードにすることを検討したい。

## 3. 置き換えで今の利用者が失うもの・操作が変わるもの

- **開発環境**
  - Arduino／PlatformIO から ESP-IDF 5.5.2 ＋ `sf` コマンドに変わる。
  - インストーラは macOS／Linux／Windows で、CI による導入試験がある。
  - Arduino IDE で開いて少し書き換える、という使い方はできなくなる。
  - コード量が一桁以上増えるので、改造の入口は高くなる（文書と例題は多い）。
- **フリップ**：旧はコントローラのボタンで使えるが、vehicle では既定で無効。
- **コントローラへのテレメトリ**：ESP-NOW では送られなくなる（A1）。旧コントローラがこれを表示に使っているかは未確認。
- **操作と表示**
  - ボタンの意味、LED の色、ブザーの音がすべて変わる。
    - 例：本体ボタンのクリックは、旧では「再起動」、vehicle では地上での ARM／DISARM。3 秒の長押しは vehicle では再ペアリング。
  - M5Stack の取扱説明は書き直しになる。
- **ペアリング**：旧は毎回の起動時。vehicle は最初の 1 回だけで、以後は保存した相手につながる。改善だが、手順は変わる。
- **Wi-Fi チャンネル**：既定が 3 から 1 に変わる。教室での電波干渉の状況が変わる。
- **通信が途切れたときの動作**
  - 旧
    - 100 ms で自動着陸に入る。
    - ただし着陸中も、最後に受けたスティックの値で姿勢を指令し続ける（`flight_control.cpp:816-832`）。
    - 壊れたパケットを受けても接続中と扱う（`rc.cpp:57`）。
  - vehicle
    - 途切れてから 3 秒間は、最後に受けたスティック入力のまま飛ぶ（A5）。
    - その後、FLYING 状態なら着陸に入る。
    - 着陸中は、指令が 500 ms より古くなると角速度の目標値を 0 にする。
  - 途切れた瞬間にスティックを倒していた場合、旧ではすぐ着陸に入るが、vehicle では最大 3 秒傾いたまま進む。

## 4. vehicle で良くなる点

- **処理の構成**
  - センサの読み出しは core 0、制御は core 1 のタスクに分かれている。そのため、I2C の読み出しが止まっても制御ループは止まらない。
  - IMU からの通知が 10 ms 途絶えると、モータを止める（`control_task.cpp:253-255`）。
  - 旧は 1 本のループで、LED の更新（FastLED）と電池電圧の読み出しを毎周期行っている。
- **I2C の扱い**：PR #6 が Arduino-ESP32 3.x 向けに回避しようとしている問題は、vehicle では構造上起きない。
  - 旧の問題：Wire1 と旧ドライバの併用、戻り値を確かめずに無視、ToF の初期化待ちにタイムアウトがない。
  - vehicle：新しいドライバに統一し、エラーを VL53LX のエラーコードに変換して返している。
- **推定と制御**
  - 15 状態の ESKF で、オプティカルフローと ToF を融合する。
  - それにより位置保持（POS_HOLD）と自動離着陸ができる。
  - 位置保持の精度は文書上で ±6〜7 cm、RMS 16 mm とされている（文書上の主張で、今回は検証していない）。
- **安全機能**
  - ARM 前の確認がある（電池電圧、校正の完了、姿勢の一致）。
  - 衝撃と異常回転を検出する。
  - 電池電圧が 3.0 V まで下がると着陸に入る。
  - 受信パケットは長さ・チェックサム・全ゼロ・送信元 MAC の 4 段階で確かめる。旧は長さを確かめず、25 バイト分を読む。
- **設定の保存と調整**：パラメータは NVS に保存でき、CLI から変えられる。学習したトリムとホバリング推力は着陸時に保存される。
- **外部との接続**：USB／TCP の CLI、Tello 互換の UDP API、高速のデータ転送（Data Stream）がある。授業や研究での計測に使える。
- **保守性**
  - コンポーネントに分かれた構成になっている。
  - PC 上の単体試験と、CI での SILS 再確認試験がある。
    - SILS：ソフトウェアだけで機体モデルと組み合わせて飛行を模擬する試験。
    - 再確認試験：変更で既存の動作が壊れていないかを自動で確かめる試験。
  - タグを付けたときに、配布用のバイナリをビルドするワークフローがある。
  - 旧は src/ が 1 層の構成で、試験はない。README は 15 行。CI の clang-format と Arduino Lint のワークフローは設定の不備で機能していない。

### vehicle に残っている課題（旧より悪化しているわけではない）

- センサの健全性（最後のデータの古さ）は 1 Hz で集計されているが、ARM 前の確認にも異常時の処理にも使われていない。ARM 前の確認では「保留」とコメントされている（`state_manager.cpp:179`）。
- 推定器の発散を知らせる通知（`ESKF_DIVERGED`）は受け手の処理はあるが、それを発する箇所が見つからない。
- 着陸を始める電池電圧 3.0 V は定数で、パラメータでは変えられない。
- 起動時の校正値は保存しない。NVS への書き込みで 400 Hz のループが 10 ms 以上止まるため、意図的に保留している。
- ボタンの 5 秒長押しとダブルクリックは検出されるが、対応する処理がない。

## 5. 未確認の事項

- 実機での飛行性能と処理時間。今回はどちらのファームウェアもビルド・書き込みをしていない。
- M5Stack が現在配布しているコントローラのファームウェアが、AtomJoy2024June と同じ 25 バイト形式か。旧の機体ファームウェアはこの形式しか受け付けないので、同じ形式である可能性は高い。
- StampFly V1.1 のハードウェアの違い。
- vehicle の単体試験の結果（今回は実行していない）と、SILS 再確認試験の最近の失敗の原因。
- 文書にある主張（前世代のファームウェアでの実機 87 回の飛行、位置保持の精度など）。

## 6. 次に進めるなら

1. コントローラの扱いを決める：両方のファームウェアを同時に配布するか、移行期間だけ vehicle に旧形式の受信処理を入れるか。
2. BMI270 のライセンス表記と `NOTICE.md` を直す。
3. 出荷用のビルド構成を用意する：SNN と autotune を外し、フリップの扱いを決める。
4. 飛行中の DISARM の仕様（STABILIZE で即停止）を製品としてどうするか決める。
5. 通信断の直後に目標値を中立に置き換え、要件どおり「3 秒ホバー → 着陸」にする（A5）。
6. M5Stack に V1.1 のハードウェアの違いを確認する。

---

<a id="english"></a>

## 1. Overview

### About This Document

Date written: 2026-10-28. File locations are given as paths relative to `firmware/vehicle` (`components/` and `tasks/` are sometimes omitted).

- **Viewpoint**: Can the vehicle firmware of stampfly_ecosystem replace the vehicle firmware of the StampFly sold by M5Stack?
- **Order**: The document covers the points that block replacement, then what current users lose or what changes in operation, then where vehicle is better. Problems are placed first so that the document does not lean toward promoting vehicle.
- **Subjects compared**
  - Old: main of `m5stack/M5StampFly` (commit fa7b6a5). PR #6 (Arduino-ESP32 3.x support) has not been merged yet, so the state before that merge was examined.
  - vehicle: `firmware/vehicle` of `M5Fly-kanazawa/stampfly_ecosystem` (commit 884f2af, the same as main as of 2026-10-24).
- **Method**: The source code was read and compared. Nothing was built and nothing was tested on a real vehicle.
  - The following were checked by directly opening the relevant source lines.
    - A1 to A3, A5
    - In A4, how the SNN and autotune can be activated
  - Everything else is from what the investigation agents read, with file names and line numbers attached.
- **Glossary**
  - NVS: An area in the ESP32 flash memory that keeps setting values saved when the power is turned off.
  - Task: In FreeRTOS (a small OS that runs on the ESP32), a unit of processing that runs concurrently.
  - CI: A mechanism that automatically runs builds and checks on every change.
  - CLI: A text interface where commands are typed to operate the system.
  - API: An entry point for operating the vehicle from an external program (here, text commands received over UDP).
  - Tello: A small drone from DJI and others. vehicle has an API compatible with its command set.
  - SNN: Spiking neural network. A neural network that imitates the firing (pulses) of nerve cells.
  - AHRS: Attitude and heading estimation.
  - Data Stream: High-speed measurement data that vehicle sends to a PC over Wi-Fi (400 Hz samples are bundled and sent at 50 Hz).

### Conclusions

1. **As vehicle firmware, vehicle is far ahead in functions, safety features and maintainability.**
   - The pin assignments are the same in both.
   - The partition layout fits the StampS3 (8 MB flash, no PSRAM).
2. **Replacement cannot be completed on the vehicle alone.**
   - vehicle accepts nothing other than the 14-byte control packet.
   - It can neither pair with nor be flown from the old-generation controller (AtomJoy2024June family, 25 bytes, float format).
   - The controller firmware therefore has to be replaced at the same time.
3. **There are 5 points to fix, or to decide a policy on, before shipping.**
   - Inconsistent license notices for the BMI270
   - Research features (such as the untrained SNN controller) being included in the default build
   - The meaning of the DISARM operation in flight changes
   - The behavior during the 3 seconds right after a link loss differs between the requirement (hold hover) and the implementation (hold the last stick input)
   - Operation check on StampFly V1.1

### Comparison Table

| Item | Old (M5StampFly) | vehicle |
|---|---|---|
| Development environment | Arduino (PlatformIO, no Arduino-ESP32 version specified) | ESP-IDF 5.5.2 (pinned by `dependencies.lock`), installed with the `sf` command |
| Size of own code | About 3,500 lines in src/ | About 7,200 lines in tasks plus each component (tens of thousands of lines, excluding the ST ToF driver) |
| Processing structure | A single Arduino loop. 400 Hz by an empty loop waiting for a flag set by a timer interrupt | 16 FreeRTOS tasks. Control on core 1, sensors on core 0. 400 Hz driven by the IMU timer |
| Sensors used | IMU (BMI270), downward ToF, power monitor (INA3221) | The above plus optical flow (PMW3901), barometer (BMP280), magnetometer (BMM150). Barometer and magnetometer are not fused by default |
| State estimation | Madgwick (attitude) plus a 3-state Kalman filter for altitude | 15-state error-state Kalman filter (ESKF) |
| Flight modes | Angle / angular rate x manual throttle / altitude hold, flip | ACRO / STABILIZE / ALT_HOLD / POS_HOLD (position hold by optical flow). Flip is disabled by default |
| I2C driver | Arduino Wire1 and the old ESP-IDF driver used together on the same port. Return values left unchecked | Unified on the new ESP-IDF `i2c_master` driver. Errors are returned to the upper layer |
| Pre-arm checks | None | Battery voltage, completion of the startup calibration, agreement between attitude and gravity direction, not in pairing |
| Link loss | Goes to automatic landing after 100 ms (a corrupted packet still counts as "connected") | Detected after 500 ms, keeps flying on the last stick input for 3 seconds, then goes to landing (A5) |
| Low battery voltage | LED indication only | Warning at 3.4 V, landing at 3.0 V |
| Impact / abnormal rotation | Motors stop above 2 g | Motors stop above 3 g or above 800 deg/s angular rate |
| Saving settings | None (all gains are constants) | Parameters, pairing partner and magnetometer calibration values are saved in NVS |
| External connections | ESP-NOW only (sends telemetry to the controller at 40 Hz) | ESP-NOW (control reception only), USB / TCP CLI, Tello-compatible UDP API, Data Stream, UDP telemetry |
| Tests | None | About 3,100 lines of unit tests on a PC (no execution from CI found), regression tests with SILS in CI |
| License notices | Keeps the Bosch and ST headers | Inconsistent BMI270 notice (A3) |

## 2. Points That Block Replacement and Points to Decide Before Shipping

### A1. No compatibility with the controller (most important)

The control packets of the two have completely different formats.

| | Old (vehicle `src/rc.cpp:56-143`, controller AtomJoy2024June `senddata[25]`) | vehicle (`common/protocol/include/espnow_protocol.hpp:45-56`) |
|---|---|---|
| Length | 25 bytes (no length check) | Discarded unless exactly 14 bytes (`sf_comm/comm.cpp:323`) |
| Destination check | First 3 bytes = lower 3 bytes of the vehicle MAC | Same |
| Sticks | 4 floats (4 bytes each) | 4 uint16 (12-bit AD values, 2048 is the center) |
| Buttons | 1 byte each (ARM, FLIP, mode, altitude mode, AHRS reset) | 1 byte of bit flags |
| Checksum | Sum of the first 24 bytes (byte 24) | Sum of the first 13 bytes (byte 13) |

- **The pairing procedure and storage differ.**
  - The pairing packet that the vehicle broadcasts (11 bytes: channel, MAC, `AA 55 16 88`) is identical byte for byte in both.
  - Old
    - The vehicle sends this packet for 2.5 seconds at every startup.
    - It remembers the sender of the first packet that arrives as its communication partner.
    - The remembered partner is not saved.
    - The channel is fixed at 3.
  - vehicle
    - If no partner is registered, it enters the pairing state automatically and sends the packet every 500 ms.
    - The partner MAC is saved in NVS.
    - The channel can be changed by a parameter, and the default is 1.
  - Controller of stampfly_ecosystem
    - It enters the pairing screen only when BtnA is held at startup.
    - The vehicle is chosen from candidates sorted by received signal strength.
- **Telemetry stops arriving.** The old vehicle sends 120-byte telemetry to the controller over ESP-NOW at 40 Hz (`rc.cpp:219`). The only thing vehicle sends over ESP-NOW is the pairing packet (`comm.cpp:501`).
- **Impact**
  - Replacement becomes a job of replacing two firmwares, the vehicle and the controller, at the same time.
  - A user who updates only one side ends up with a combination that cannot pair.
  - The distribution method through M5Burner (M5Stack's firmware writing tool) and the procedure documents have to be rewritten.
- **Options**
  - One option is to add reception of the old format (25 bytes) to vehicle for a transition period.
  - Since the pairing packet and the destination check method are common, it appears feasible in principle.
  - However, whether the old controller works on channel 1 and similar points are not verified.

### A2. The meaning of DISARM (button operation) in flight changes

| Mode | Old | vehicle |
|---|---|---|
| All modes | Button operation in flight causes automatic landing (`flight_control.cpp:339`) | (Split into the next two rows) |
| ALT_HOLD / POS_HOLD | — | Automatic landing (descends at 0.3 m/s) |
| STABILIZE / ACRO | — | **Motors stop on the spot** (`sf_state/state_manager.cpp:253-296`) |

The behavior of vehicle is written in the operation manual (`operation_manual.md:134`) and is an intended specification. Even so, if a user accustomed to the old firmware performs the same operation in STABILIZE, the vehicle falls from a high position. It is necessary to decide whether this specification is acceptable for a product and to make it known through the procedure documents.

### A3. Inconsistent license notice (BMI270)

- The content of `sf_hal_bmi270/src/bmi270_config_file.c` is the BMI270 configuration data (8,192 bytes) distributed by Bosch, as is (stated in the file's comment, lines 29-31).
  - However, the notice at the top of the file reads `SPDX: MIT, Copyright (c) 2025 Kouhei Ito`.
  - Bosch's BMI270_SensorAPI is BSD-3-Clause, and the copyright notice must be kept when redistributing.
  - The component's `LICENSE` is also MIT only, and no Bosch notice is found.
- `NOTICE.md` §1.1 says "the C driver part is from the Bosch SensorAPI, BSD-3-Clause, and the algorithm itself is unmodified". This does not match the actual file structure (own implementation plus configuration data).
- Whether `sf_hal_bmi270/docs/bmi270_bosh_doc-34-40.pdf` (which appears to be an excerpt of the datasheet) may be redistributed is not verified.
- The old firmware keeps the Bosch headers and LICENSE in `lib/bmi270`, so it has no problem on this point.
- **Action**
  - Add Bosch's copyright notice and the BSD-3-Clause text to the configuration data file.
  - Make the description in `NOTICE.md` match the actual state.
  - Delete the PDF or confirm that it may be distributed.

### A4. Research and unfinished features are in the default build

- **SNN controller and SNN estimator**
  - They are included in the build unconditionally in `main/CMakeLists.txt`.
  - Which one is used is decided by a parameter, and the default is PID and ESKF.
  - The header of the SNN controller (`snn_controller.hpp:32-33`) says "a placeholder of untrained fixed random numbers; crashing in SILS is the expected result".
  - If `param set controller.type 1` followed by `param save` is run from the CLI, the vehicle runs with this controller even after a restart.
- **Automatic tuning (autotune)**
  - It can be started in flight by the `autotune` command of the UDP API (`api_task.cpp:1209-1212`).
  - The resulting gains are applied immediately (from an investigation agent's reading).
- **Flip**: Disabled by default (`flip.enable = 0`). The operation manual states that the first flip on a real vehicle on 2026-10-02 failed and the cause is under analysis.
- **Action**: Prepare a separate factory build configuration and exclude the research features at compile time. Then users can no longer select them by mistake.

### A5. The behavior during the 3 seconds right after a link loss differs between the requirement and the implementation

- **Requirement**
  - The comment at `state_manager.cpp:528-530` says "link loss -> keep hovering, automatic landing after 3 seconds (requirement §9)".
  - The log also prints `Comm lost → hover 3000 ms then LANDING`.
- **Implementation**
  - The state management only starts a 3-second timer (`state_manager.cpp:552-557`, `:716-721`).
  - The control setpoint (`command_setpoint`) is updated only when a packet is received (`sf_command/command.cpp:66-68`, `:132`).
  - The control task reads that latest value every cycle (`control_task.cpp:277`).
  - The processing that returns a stale setpoint to level exists only during landing (LANDING) (`pid_controller.cpp:574-590`).
- **Result**
  - For 3 seconds, the stick input (tilt, throttle) from just before the link loss is used as it is.
  - Example: if the stick was pushed forward, the vehicle advances for 3 seconds while tilted by up to 30 degrees.
  - Example: if the throttle was raised in ALT_HOLD, the vehicle keeps climbing.
- **Action**: When link loss is detected, replace the setpoint with neutral (level attitude, altitude held). That makes the behavior match the requirement.

### A6. Points to confirm before shipping

- **StampFly V1.1**
  - Neither firmware identifies the vehicle revision.
  - The pin assignments of the two are the same.
  - Whether the V1.1 hardware differs, and whether vehicle has been tried on V1.1, are not verified. Confirmation from M5Stack is needed.
- **Inconsistent documents**: `README.md:125` of vehicle says "Phase 2 in progress, ToF / Baro / Mag / Power / NVS / Logger remaining". On the other hand, `docs/feature_status.md` treats these as complete. They need to be aligned as documents for a product.
- **SoftAP (a mode in which the vehicle itself becomes a Wi-Fi access point)**
  - Disabled by default (`wifi.mode = 0`).
  - When enabled, the default password is `stampfly`, common to all vehicles (`comm.hpp:261`).
  - On UDP port 8889, Tello-compatible commands such as `takeoff` are accepted from anyone who connects. If the pre-arm checks pass, the vehicle takes off.
  - If many vehicles are used in a classroom, using a different password for each vehicle should be considered.

## 3. What Current Users Lose or What Changes in Operation by Replacement

- **Development environment**
  - It changes from Arduino / PlatformIO to ESP-IDF 5.5.2 plus the `sf` command.
  - The installer covers macOS / Linux / Windows, and installation is tested in CI.
  - The use of opening the code in the Arduino IDE and rewriting it a little is no longer possible.
  - The amount of code grows by more than an order of magnitude, so the entry barrier to modification becomes higher (there are many documents and examples).
- **Flip**: Usable by a controller button in the old firmware, but disabled by default in vehicle.
- **Telemetry to the controller**: No longer sent over ESP-NOW (A1). Whether the old controller uses it for display is not verified.
- **Operation and indication**
  - The meanings of the buttons, the LED colors and the buzzer sounds all change.
    - Example: a click of the body button means "restart" in the old firmware and ARM / DISARM on the ground in vehicle. A 3-second long press means re-pairing in vehicle.
  - M5Stack's instruction manual has to be rewritten.
- **Pairing**: In the old firmware, at every startup. In vehicle, only the first time, and afterward it connects to the saved partner. This is an improvement, but the procedure changes.
- **Wi-Fi channel**: The default changes from 3 to 1. The radio interference situation in a classroom changes.
- **Behavior on link loss**
  - Old
    - It enters automatic landing after 100 ms.
    - However, even during landing it keeps commanding attitude from the last received stick values (`flight_control.cpp:816-832`).
    - It treats a corrupted packet as connected as well (`rc.cpp:57`).
  - vehicle
    - For 3 seconds after the link is lost, it flies on the last received stick input (A5).
    - After that, if in the FLYING state, it enters landing.
    - During landing, it sets the angular rate target to 0 when the command is older than 500 ms.
  - If the stick was pushed at the moment of the loss, the old firmware enters landing immediately, whereas vehicle advances while tilted for up to 3 seconds.

## 4. Where vehicle Is Better

- **Processing structure**
  - Sensor reading and control are split into tasks on core 0 and core 1. Therefore the control loop does not stop even if I2C reading stalls.
  - If the notification from the IMU is absent for 10 ms, the motors are stopped (`control_task.cpp:253-255`).
  - The old firmware updates the LEDs (FastLED) and reads the battery voltage in every cycle of a single loop.
- **I2C handling**: The problems that PR #6 tries to work around for Arduino-ESP32 3.x do not occur in vehicle by construction.
  - Old problems: using Wire1 and the old driver together, ignoring return values without checking them, no timeout in waiting for ToF initialization.
  - vehicle: Unified on the new driver, with errors converted to VL53LX error codes and returned.
- **Estimation and control**
  - The 15-state ESKF fuses optical flow and ToF.
  - This enables position hold (POS_HOLD) and automatic takeoff and landing.
  - The position hold accuracy is stated in the documents as ±6–7 cm, RMS 16 mm (a documented claim, not verified here).
- **Safety features**
  - There are pre-arm checks (battery voltage, completion of calibration, attitude agreement).
  - Impact and abnormal rotation are detected.
  - If the battery voltage falls to 3.0 V, landing starts.
  - Received packets are checked in 4 stages: length, checksum, all zeros and source MAC. The old firmware does not check the length and reads 25 bytes.
- **Saving and adjusting settings**: Parameters can be saved in NVS and changed from the CLI. The learned trim and hover thrust are saved at landing.
- **External connections**: There are the USB / TCP CLI, the Tello-compatible UDP API and high-speed data transfer (Data Stream). They can be used for measurement in classes and research.
- **Maintainability**
  - The code is organized into components.
  - There are unit tests on a PC and regression tests with SILS in CI.
    - SILS: A test that simulates flight by combining the software with a vehicle model, using software only.
    - Regression test: A test that automatically checks that a change has not broken existing behavior.
  - There is a workflow that builds distribution binaries when a tag is pushed.
  - The old firmware has a single-level src/ structure and no tests. The README is 15 lines. The CI workflows for clang-format and Arduino Lint do not work because of configuration defects.

### Remaining Issues in vehicle (not worse than the old firmware)

- Sensor health (age of the last data) is tallied at 1 Hz, but it is used neither in the pre-arm checks nor in abnormal-condition handling. It is commented as "deferred" in the pre-arm checks (`state_manager.cpp:179`).
- The notification that reports estimator divergence (`ESKF_DIVERGED`) has a handler on the receiving side, but no place that raises it was found.
- The battery voltage of 3.0 V at which landing starts is a constant and cannot be changed by a parameter.
- The startup calibration values are not saved. Writing to NVS stalls the 400 Hz loop for 10 ms or more, so this is deliberately deferred.
- The 5-second long press and the double click of the button are detected, but there is no corresponding processing.

## 5. Items Not Verified

- Flight performance and processing time on a real vehicle. Neither firmware was built or written to a device this time.
- Whether the controller firmware currently distributed by M5Stack uses the same 25-byte format as AtomJoy2024June. The old vehicle firmware accepts only this format, so it is likely to be the same format.
- Differences in the StampFly V1.1 hardware.
- The results of the vehicle unit tests (not run this time), and the cause of the recent failures of the SILS regression tests.
- Claims in the documents (87 flights on real vehicles with the previous-generation firmware, position hold accuracy, and so on).

## 6. Next Steps

1. Decide how to handle the controller: distribute both firmwares at the same time, or add reception of the old format to vehicle for a transition period only.
2. Fix the BMI270 license notice and `NOTICE.md`.
3. Prepare a factory build configuration: remove the SNN and autotune, and decide how to handle flip.
4. Decide what to do, as a product, about the DISARM specification in flight (immediate stop in STABILIZE).
5. Replace the setpoint with neutral right after a link loss, so that the behavior is "hover for 3 seconds, then land" as required (A5).
6. Ask M5Stack about the differences in the V1.1 hardware.
