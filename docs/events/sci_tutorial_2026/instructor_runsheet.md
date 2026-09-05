# 実演・実習ランシート（講師用）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

2026年9月10日（木）開催の SCI/SICE チュートリアル講座 2026（全122ページのスライド `docs/events/sci_tutorial_2026/slides/sci_tutorial.pdf`）から、講師が当日**実際に実演する（実演）**か**参加者にやってもらう（実習）**フレーム、および**期待結果を提示するだけ**のフレームだけを抜き出し、進行順に並べたもの。122ページ全体をめくらなくても、この1枚でリハーサルと本番進行ができることを目的とする。

### 出典と手法

- スライド本文: `docs/events/sci_tutorial_2026/slides/sci_tutorial.tex` および `chapters/sci_intro.tex`、`sci_s1_overview.tex`〜`sci_s5_sim_analysis.tex`、`sci_appendix.tex`
- 参加者向け・復習向け資料: `README.md`（タイムテーブル・事前準備）、`handson_guide.md`（帰宅後の復習手順）、`cheatsheet.md`（コマンド・API早見表）、`verification_checklist.md`（講師のリハーサル手順）、`fallback/README.md`（代替素材の索引）
- ページ番号は `pdftotext -f N -l N sci_tutorial.pdf -` で1ページずつ本文を抽出し、各チャプターの `\begin{frame}` 数（intro 5 + S1 22 + S2 17 + S3 16 + S4 24 + S5 23 + 付録 8 = 115、セッション区切り6枚（Session 1〜5 と付録）・表紙1枚を加えて122）が実ページ数122と一致することを確認したうえで、代表ページを個別に照合した

### 使い方

- 各セッションの表は「実演」「実習」「期待結果の提示」のいずれかに分類されるフレームだけを載せている（地図・要点3つ・理論とコード対応表・復習パス・チェックポイントなど純粋な講義フレームは含まない）
- 「使うコマンド」はスライドに印字されている文字列をそのまま転記している。スライドが手順を散文で説明するだけでコマンド文字列を明記していない箇所は、その旨を注記したうえで他ページに印字されている同じ操作のコマンドを補って示す
- 「所要目安」はスライドに時間配分の明記がないため、セッション全体の時間とフレーム数から講師が按分した目安（分）である
- 「代替」列は実演できない場合の代替として `docs/events/sci_tutorial_2026/fallback/` にある素材を指す。同ディレクトリには S1・S4・S5 の素材しかなく、S2・S3 用の代替素材は存在しない（§10にも記載）

## 2. 開始前チェック

### 数日前まで（リハーサル）

`verification_checklist.md` の §1〜§4（ベンチ確認・飛行確認・システム同定確認・本番ファームでのデモ確認）と §5（動画・ログの代替取得）を完了しておく。これは当日の朝に行うものではなく、事前に完了させておくべき作業である。

### 当日、進行を始める直前

| 項目 | 確認内容 | 出典 |
|------|---------|------|
| 実機・コントローラ | 講師用デモ機と貸出用予備のバッテリーを全数充電。プロペラガードを装着。対面参加者は基本的に実機を持参する（README §「持ち物」）ため、予備の必要台数はその日の参加者構成による — 資料に具体的な予備台数の指定はない | README 開催情報, verification_checklist §5 |
| Web Flasher | `https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/` をブラウザで開いておく | README §4 |
| 講師 PC 環境 | `source setup_env.sh` を実行し、`sf doctor` がエラーなく通ることを確認 | README §4, sci_s2 p.35 |
| 代替素材フォルダ | `docs/events/sci_tutorial_2026/fallback/` を開いておく（実演が失敗した場合に即座に画像・動画を提示できる状態にする） | verification_checklist §5, fallback/README.md |
| 投影・配信 | プロジェクタ接続を確認。Zoom 画面共有をテストする（オンライン参加者向け、当日はオンデマンド配信もあり） | README 開催情報 |
| ペアリング | 貸出用コントローラは事前ペアリング済みであることを確認（初回のみ手動: コントローラ LCD パネルボタンを押しながら電源投入 → StampFly 本体ボタンを2秒長押し → 双方ビープで完了） | sci_s2 p.34, sci_s3 p.57 |

## 3. Session 1: StampFly Ecosystem の全体像と設計思想（10:05–11:00）

全22フレーム中、実演・期待結果の提示は4件。

| 種別 | ページ | 内容 | 使うコマンド | 期待する結果 | 所要目安 | 事前準備・注意 | 代替 |
|------|--------|------|-------------|-------------|---------|---------------|------|
| 実演（見るだけ） | p.24 | デモ①: シミュレータ操縦。`sf sim run vpython` でVPythonシミュレータを起動し、実機と同じ制御アルゴリズムをHIDジョイスティックで操縦する | `sf sim run vpython` | ブラウザに3D表示が立ち上がり、スティック操作に応じて機体が動く。実機がなくても制御コードの挙動を確認できる | 3〜4分（目安） | HIDジョイスティック（AtomS3 + Atom JoyStick）をUSB接続しておく。会場ネットワークに依存しないため失敗しにくい実演 | 特になし（fallback/にS1のシミュレータ操縦専用の素材はない。うまくいかない場合はスライドの図解のみで説明を続ける） |
| 実演（見るだけ） | p.25 | デモ②: 実機POS_HOLD飛行。ホバー中の位置保持精度と、外乱を与えたときの復帰動作を見せる | （このページにコマンド文字列の明記なし。実機操作: ARM → POS_HOLDへモード切替 → 手で軽く押すなどの外乱を与える） | 水平ドリフトが小さく抑えられ、外乱後に元の位置へ戻る（実測 ±6–7cm、p.19「vehicleファーム構造」フレームの数値） | 3〜4分（目安） | verification_checklist §4「本番ファームでのデモ確認」でPOS_HOLDの安定動作を事前リハーサル必須。プロペラガード装着、飛行エリアをネットで区画。不安定な場合はALT_HOLDまたはSTABILIZEへ切替（checklist §4の代替方針） | `S1_pos_hold_flight.mp4`（POS_HOLD飛行動画）。または次の2枚の期待結果スライド（p.26, p.27）をそのまま見せる |
| 期待結果の提示 | p.26 | デモ②の期待結果: 位置保持（SILSでのシミュレーション結果を参考として提示） | なし（静止画） | `pos_roll.scn`（vehicle）: 離陸→ロール外乱→POS_HOLD係合→保持。係合後の水平ドリフト最大0.39m | 1〜2分（目安） | 画像は`fallback/S1_pos_hold_xy.png`（スライドに埋め込み済み） | 該当なし（本フレーム自体が代替素材） |
| 期待結果の提示 | p.27 | デモ②の期待結果: 高度と姿勢 | なし（静止画） | 同じ飛行の高度・姿勢角の時系列。外乱直後の傾きが位置制御で戻る | 1〜2分（目安） | 画像は`fallback/S1_altitude_attitude.png` | 該当なし |

## 4. Session 2: 開発環境のセットアップとセンサデータの取得（11:00–12:00）

全17フレーム中、実演・実習は2件。

| 種別 | ページ | 内容 | 使うコマンド | 期待する結果 | 所要目安 | 事前準備・注意 | 代替 |
|------|--------|------|-------------|-------------|---------|---------------|------|
| 実習（全員参加） | p.35 | 事前準備の確認。全員で`sf doctor`を実行し、ESP-IDF・Python・USBシリアルドライバ・sf CLI自体のバージョンを診断する | `source setup_env.sh`<br>`sf doctor` | エラーなく完了する（README §4の合格の目安と同じ） | 3〜5分（目安。個別に通らない参加者への対応を含めるとさらに延びる） | 事前準備（README §4）で前日までに済ませておくよう案内済み。ここで通らなくても心配不要 — 以降のデモは「見るだけ」で参加してもらい、復習パスで後日対応（p.35 alertblock） | fallback/に専用素材はなし。個別に通らない参加者は「見るだけ」に切替え、`docs/guides/troubleshooting.md`を案内 |
| 実演（一緒に打つ） | p.43 | デモ: 傾けて確認。StampFlyを手に持ち前後左右に傾け、`gyro_x`/`gyro_y`が傾ける速さに応じて振れ、静止すると`accel_z`が−9.81付近に戻ることを確認する | `sf lesson switch sci2026:2`<br>`sf lesson build && sf lesson flash`<br>`sf monitor workshop`（VSCode拡張 `alexnesnes.teleplot` 導入が前提。`sf lesson flash`直後は自動でこの接続状態のまま開く） | Teleplotで`gyro_x`/`gyro_y`/`accel_z`の波形が動く。静止時ジャイロ≈0、`accel_z`≈−9.81 m/s² | 5〜7分（目安。ビルド・書き込み時間を含む） | VSCode拡張`alexnesnes.teleplot`を事前に導入しておく。事前にp.40のコード（下記参照）を`user_code.cpp`に書く。参加者ごとにビルド時間が発生するため、講師機で先に流れを見せてから各自試すと時間短縮できる | fallback/に専用素材はなし。うまくいかない場合は講師機のTeleplot画面を共有し「見るだけ」に切替え |

参考コード（p.40、実演前に書くコード）:

```cpp
static uint32_t tick = 0;
void loop_400Hz(float dt) {
    if (tick++ % 4 == 0) {   // 100Hz decimation
        ws::print(">gyro_x:%.3f", ws::gyro_x());
        ws::print(">gyro_y:%.3f", ws::gyro_y());
    }
}
```

## 5. Session 3: モータ制御とコントローラ入力の実装（13:00–14:00）

全16フレーム中、実演・実習は2件。**安全（p.51）**: プロペラは外した状態で実施。Duty は0.15以下に固定（ファーム側では強制されないため必ず目視確認）。ARMは機体ボタンの単クリックまたはコントローラのみで行い、コード内で自動ARMしない。

| 種別 | ページ | 内容 | 使うコマンド | 期待する結果 | 所要目安 | 事前準備・注意 | 代替 |
|------|--------|------|-------------|-------------|---------|---------------|------|
| 実習 | p.54 | 実習: Dutyをハードコードして回す。`user_code.cpp`に`motor_set_duty`を直接指定し、プロペラなしでモータの回転を確認する | `sf lesson switch sci2026:3`<br>（このページにビルド・書き込みコマンドの明記なし。p.37の標準ワークフローに従い`sf lesson build`→`sf lesson flash`を実行） | 機体ボタンでARMするとFR（M1）のみが低速回転する（duty 0.10、他モータは0.00）。DISARMで停止 | 5〜8分（目安） | 【安全 p.51】プロペラは外した状態、Duty 0.15以下、ARMは機体ボタン単クリックまたはコントローラのみ。**注:** 模範解答（`--solution`）は起動時に自動ARMするため、プロペラを外した状態でのみ使う（p.54の注意書き） | fallback/にS3専用素材はなし。ベンチ確認（verification_checklist §1）でNGの場合は`sf flash vehicle`で標準ファームに戻し、このデモを「見るだけ」に切替える |
| 実演（一緒に打つ） | p.60 | デモ: スティックでモータを回す。`ws::motor_mixer`を書き、スティック操作で4モータの回転差を確認する（コントローラの指令はESP-NOWで約50Hzの`ControlPacket`として届く）。プロペラなしでも外乱に弱いことを見せる | `sf lesson switch sci2026:4`<br>`sf lesson build && sf lesson flash` | スティックを倒すと4モータの回転数に差が出る。風で軽く煽ると指令通りの回転が保てない（オープンループ制御の限界） | 5〜7分（目安） | ペアリング未実施なら先にp.57の手順（コントローラLCDボタンを押しながら電源投入→本体ボタン2秒長押し→双方ビープ）を済ませる。プロペラは外したまま | fallback/にS3専用素材はなし。checklist §1と同様、NG時は`sf flash vehicle`に戻す |

参考コード（p.54、実習で書くコード）:

```cpp
void setup() {
    ws::print("Motor duty test");
    // Do NOT auto-arm in code / コードでは自動ARMしない
}
void loop_400Hz(float dt) {
    ws::motor_set_duty(1, 0.10f);   // FR
    ws::motor_set_duty(2, 0.00f);   // RR
    ws::motor_set_duty(3, 0.00f);   // RL
    ws::motor_set_duty(4, 0.00f);   // FL
}
```

## 6. Session 4: フィードバック制御の基礎 — PID による姿勢安定化（14:00–15:00）

全24フレーム中、実演・期待結果の提示は2件のみ。**本セッションの実習5〜9そのもの（コードを書いて飛ばす作業）は当日は「見るだけ」で、実際に手を動かすのは帰宅後**（p.84のデモの見方: 「帰宅後に再現: 本日は見るだけにして，後日実習5〜9を通しで動かす」）。当日その場で実際に飛行させるのは講師のみである。

| 種別 | ページ | 内容 | 使うコマンド | 期待する結果 | 所要目安 | 事前準備・注意 | 代替 |
|------|--------|------|-------------|-------------|---------|---------------|------|
| 実演（見るだけ） | p.84 | デモ: 完成コードで飛行。PID化した学習者コード（実習8）で飛行し、ロール角速度のステップ応答をP制御（実習5）時と比較する | `sf log wifi -d 30 -o flight.csv`<br>`sf log viz flight.csv`（p.84本文は「`sf log wifi`でテレメトリ取得」「`sf log viz`でステップ応答を表示」と散文で説明するのみで具体的な引数は明記されていない。上記はp.42に印字されている構文を援用） | 目標15.1 deg/sに対し実習5（P）はピーク17.9 deg/sと−2.4 deg/sのアンダーシュート、実習8（PID）はピーク13.2 deg/sで振動なし（p.85キャプション） | 5〜8分（目安） | 実際に飛行させるのは講師のみ。verification_checklist §2「飛行確認」で実習5・実習8の安定動作を事前リハーサル必須。プロペラガード装着・保護メガネ着用 | `fallback/S4_lesson5_p_flight.mp4`、`fallback/S4_lesson8_pid_flight.mp4`（checklist §2の方針どおり、不安定なレッスンだけ動画に切替も可） |
| 期待結果の提示 | p.85 | デモの期待結果: PとPIDのステップ応答（スライドタイトルからは「制御」「比較グラフ」の語が落ち、簡潔化されている） | なし（静止画） | 上記に同じ数値（p.85キャプション） | 2〜3分（目安） | 画像は`fallback/S4_roll_step_p_vs_pid.png` | 該当なし（本フレーム自体が代替素材） |

**本表から除外したフレーム（参考として口頭で触れるにとどめる、または講師の裁量で任意実演）:**

- p.75「実習5の振り返り: レートP制御」（旧題「実習5を振り返る: レートP制御」から簡潔化）: 実習5のコードを再掲する解説フレームで、その場での再実行はない
- p.82「システム同定: sf sysid fit」、p.83「自動チューニング: sf sysid rate-fit / rate-tune」: いずれもコマンドの出力例を提示するのみで、本番中に講師が実際に実行するとはスライド上に明記されていない

## 7. Session 5: シミュレータ・解析ツールの活用と発展的テーマ（15:30–16:30）

全23フレーム中、実演・実習・期待結果の提示は3件。

| 種別 | ページ | 内容 | 使うコマンド | 期待する結果 | 所要目安 | 事前準備・注意 | 代替 |
|------|--------|------|-------------|-------------|---------|---------------|------|
| 実習 | p.100 | 実習: 自分のPIDをSILSで飛ばす。実習8（PID）の`setup()`/`loop_400Hz()`を、実機で飛ばす前にSILSで検証する | `sf sils build --target workshop`<br>`sf lesson switch sci2026:8 --solution`（または自分のコード）<br>`touch firmware/workshop/main/user_code.cpp`<br>`sf sils build --target workshop`（再ビルドして反映）<br>`sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop` | 合格基準（`.expect`）は離陸（真値高度 > 0.1m）と傾き15°未満（発散しない）の2点。高度ループがないため着陸はDISARM降下のみ。上記どおりに実行すると`alt_max`≈0.64m、`tilt_max`=0.0でPASS | 4〜6分（目安。`sf sils build`の初回コンパイル時間を含めると延びる） | `sf lesson switch`は`user_code.cpp`の更新日時を保持したままコピーするため，切替直後に`sf sils build --target workshop`を実行しても新しいコードとして認識されない。`touch`で更新日時を進めてから**もう一度**`sf sils build --target workshop`を実行して反映させる、という2段階が毎回必要な手順（省略できる例外ではない）。`sf sils gui`はworkshopターゲットを選べないため、この一連のCLIコマンドを使う（既定で`sf sils regression`から除外、`--include-workshop`で含む） | fallback/に本実習専用の動画はない。CLI実行が失敗した場合は口頭説明に切替え、SILS自体の信頼性は`fallback/S5_regression_summary.txt`（28 PASS + 5 既知の追跡中課題 + 1 workshop対象スキップ、計34本）で補強できる |
| 実演（`sf sim run vpython`部分は一緒に打つ） | p.101 | デモ: シミュレータを動かす。VPythonシミュレータの3D操縦と、`sf sils gui`でのシナリオ実行・合否判定確認、パラメータ変更による挙動変化の確認 | `sf sim run vpython`<br>`sf sils build`<br>`sf sils gui` | ブラウザに3D操縦画面が出る。`sf sils gui`でシナリオを実行するとPASS/FAILの判定結果が表示され、パラメータを変えると挙動が変わる | 5〜8分（目安） | `sf sils build`は初回のみでよい（p.99）。HIDジョイスティック（AtomS3 + Atom JoyStick）があれば接続、なければキーボード操作にフォールバック | `fallback/S5_stab_flight.mp4`（STABILIZE飛行動画）、`fallback/S5_attitude_rate.png`、`fallback/S5_gate_result.txt`（12項目の合否判定結果） |
| 期待結果の提示 | p.102 | デモの期待結果: SILSシナリオ実行。姿勢角・角速度の時系列グラフ | なし（静止画） | `stab_flight.scn`（vehicle）の姿勢角と角速度。12項目の合否判定をすべて満たす（`att_rmse=2.82<3.0`, `tilt_max=13.77<18.0`, `duty_max=1.00<1.001` — 数値は`fallback/README.md`の数値サマリより） | 2〜3分（目安） | 画像は`fallback/S5_attitude_rate.png` | 該当なし（本フレーム自体が代替素材） |

## 8. コマンド全一覧（実行順）

セッションごとに、当日実際にタイプする順でコマンドを列挙する（同一セッション内の重複は除去）。以前はスライド内に`sf sim run`（引数なし）と`sf sim run vpython`の表記ゆれがあったが、再構成後の現行スライドは全箇所`sf sim run vpython`に統一されている（§10参照）。

**Session 1**

```bash
sf sim run vpython
```

**Session 2**

```bash
source setup_env.sh
sf doctor
sf lesson switch sci2026:2
sf lesson build && sf lesson flash
```

**Session 3**

```bash
sf lesson switch sci2026:3
sf lesson build
sf lesson flash
sf lesson switch sci2026:4
sf lesson build && sf lesson flash
```

**Session 4**

```bash
sf log wifi -d 30 -o flight.csv
sf log viz flight.csv
```

**Session 5**

```bash
sf sils build --target workshop
sf lesson switch sci2026:8 --solution
touch firmware/workshop/main/user_code.cpp
sf sils build --target workshop
sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop
sf sim run vpython
sf sils build
sf sils gui
```

## 9. 実習の対応表

「実習 N」は本チュートリアル専用のコース `sci2026`（`firmware/workshop/lessons/lesson_manifest.yaml`）が指す実体のWorkshopレッスン番号に対応する。`sf lesson switch sci2026:N`で切り替える。

| 実習 | セッション | 対応する Workshop レッスン | 参加者が書くもの（関数名など） |
|------|-----------|---------------------------|-------------------------------|
| 実習1 | S2 | environment_setup（Workshop Lesson 0） | なし（環境確認のみ。`sf doctor`の実行が中心で、コード記述は発生しない） |
| 実習2 | S2 | imu_sensor（Workshop Lesson 4） | `loop_400Hz`内で`ws::print(">gyro_x:%.3f", ws::gyro_x())`等、IMU値をTeleplot形式で出力 |
| 実習3 | S3 | motor_control（Workshop Lesson 1） | `loop_400Hz`内で`ws::motor_set_duty(id, duty)`を各モータへ直接指定 |
| 実習4 | S3 | controller_input（Workshop Lesson 2） | `ws::rc_throttle/roll/pitch/yaw()`を読み、`ws::motor_mixer(T,R,P,Y)`で4モータへ配分 |
| 実習5 | S4 | rate_p_control（Workshop Lesson 5） | 目標角速度との誤差（`re`/`pe`/`ye`）を計算し、`Kp_rp*re`等を`ws::motor_mixer`に渡す比例制御 |
| 実習6 | S4 | system_modeling（Workshop Lesson 6） | なし（座学のみ、実機操作なし。実測パラメータ$K$, $\tau_m$から$\zeta=0.7$設計の$K_p$を計算する） |
| 実習7 | S4 | system_identification（Workshop Lesson 7） | 角速度目標を計算した直後に`ws::set_rate_target(roll,pitch,yaw)`を呼び、Data Streamの`rate_ref_*`に記録 |
| 実習8 | S4 | pid_control（Workshop Lesson 8） | 理想微分から不完全微分フィルタへの置換（`alpha`, `a`, `b`, `d_filt`の計算） |
| 実習9 | S4 | attitude_estimation（Workshop Lesson 9） | 相補フィルタ $\hat\theta_k=\alpha(\hat\theta_{k-1}+\omega\Delta t)+(1-\alpha)\theta_{accel}$ を自作（変数`cf_roll`等）し、`ws::estimated_roll()`と比較 |

## 10. スライドとMarkdown資料の間で見つかった不整合（参考）

修正はせず、事実として記録する。

| 箇所 | 内容 |
|------|------|
| `sf sim run` の表記ゆれ（**解消済み**） | 従来、p.24・p.28・p.17（S1深掘り）は引数なし`sf sim run`と表記し、p.98・p.102（S5）、付録p.116、README、handson_guide、cheatsheetは`sf sim run vpython`（引数あり）と表記していた。批判的レビューを反映した再構成後の現行スライドは全ページが`sf sim run vpython`に統一されており、この表記ゆれは解消済み |
| `verification_checklist.md`のレッスン番号と`実習N`の不一致（**解消済み**） | 従来、`verification_checklist.md` §1は`sf lesson switch 0/1/2/4`という**Workshopの生のレッスン番号**でベンチ確認しており、`lesson_manifest.yaml`の`sci2026`コースにおける実習1〜4の順序とは対応がずれていた（正確な対応関係は§9の表を参照）。`verification_checklist.md`側を本ランシートと同じ`sci2026:N`表記（`sf lesson switch sci2026:N`）に統一し、この不一致は解消済み |
| S4「本日は見るだけ」の実演範囲 | p.84のデモの見方には「本日は見るだけにして，後日実習5〜9を通しで動かす」とあるが、`verification_checklist.md` §2は講師自身が実習5・実習8を当日リハーサルで飛行確認する前提で書かれている。両者は矛盾しないが（参加者は見るだけ・講師は飛ばす、という役割分担）、スライド単体からは「講師が実際に飛ばすのか、代替動画のみを見せるのか」は判別できない。verification_checklist §2のNG時対応（動画切替）を踏まえ、本ランシートのp.84行では代替動画を明記した |
| S2/S3用の代替素材が存在しない | `docs/events/sci_tutorial_2026/fallback/README.md`が提供する代替素材はS1・S4・S5のみで、S2（IMU確認）・S3（モータ・コントローラ）専用の動画・画像は用意されていない。これらのセッションでNGが出た場合の代替手段は「標準ファーム（`sf flash vehicle`）に戻す」「見るだけに切替える」のみで、verification_checklist §1にその旨明記されている |

---

<a id="english"></a>

## 1. Overview

### About this document

Extracted from the 122-page slide deck for the SCI/SICE Tutorial 2026 (`docs/events/sci_tutorial_2026/slides/sci_tutorial.pdf`, held 2026-09-10), this document lists — in running order — only the frames the instructor will actually **demonstrate**, have participants **do hands-on**, or that **present an expected result**. The goal is a single sheet the instructor can rehearse from and run the day off, without paging through all 122 slides.

### Sources and method

- Slide body: `docs/events/sci_tutorial_2026/slides/sci_tutorial.tex` and `chapters/sci_intro.tex`, `sci_s1_overview.tex` through `sci_s5_sim_analysis.tex`, `sci_appendix.tex`
- Participant- and review-facing material: `README.md` (timetable, pre-workshop prep), `handson_guide.md` (post-event reproduction steps), `cheatsheet.md` (command/API reference), `verification_checklist.md` (instructor rehearsal procedure), `fallback/README.md` (index of fallback material)
- Page numbers were derived by extracting each page's text with `pdftotext -f N -l N sci_tutorial.pdf -`, cross-checked against the frame count per chapter file (intro 5 + S1 22 + S2 17 + S3 16 + S4 24 + S5 23 + appendix 8 = 115, plus 6 session dividers (Sessions 1-5 and the appendix) and 1 title page = 122), matching the deck's actual 122 pages, then spot-verified on representative pages

### How to use this document

- Each session's table includes only frames classified as demonstration, hands-on, or expected-result presentation (pure lecture frames such as the map, three-takeaways, theory-to-code map, review path, and checkpoint are excluded)
- The "Command" column transcribes exactly what is printed on the slide. Where a slide only describes a step in prose without printing a literal command string, that is noted, and the same operation's command as printed on another page is supplied instead
- "Rough time" has no basis in the slides (which give no time breakdown); it is the instructor's own estimate, apportioned from the session's total length and frame count
- The "Fallback" column points to material under `docs/events/sci_tutorial_2026/fallback/` to show if the live run fails. That directory only holds material for S1, S4, and S5 — there is no fallback material for S2 or S3 (also noted in §10)

## 2. Pre-Start Checklist

### Days before (rehearsal)

Complete `verification_checklist.md` §1-§4 (bench check, flight check, system-identification check, production-firmware demo check) and §5 (capturing backup video/logs). This is rehearsal work to finish in advance, not something to do the morning of.

### Right before starting on the day

| Item | What to confirm | Source |
|------|------------------|--------|
| Vehicle & controller | Fully charge the instructor's demo unit and any loaner spares. Prop guards on. On-site participants generally bring their own hardware (README "Bring"), so the number of spares needed depends on the day's attendee mix — the materials give no specific spare count | README event info, verification_checklist §5 |
| Web Flasher | Open `https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/` in the browser | README §4 |
| Instructor PC environment | Run `source setup_env.sh` and confirm `sf doctor` completes with no errors | README §4, sci_s2 p.35 |
| Fallback material folder | Open `docs/events/sci_tutorial_2026/fallback/` (so images/videos can be shown immediately if a live demo fails) | verification_checklist §5, fallback/README.md |
| Projector / Zoom | Confirm the projector connection. Test Zoom screen sharing (for online attendees; the day is also recorded for on-demand viewing) | README event info |
| Pairing | Confirm loaner controllers are already paired (first time only, manual: power on the controller while holding its LCD panel button, then hold the StampFly body button for 2 seconds; both beep when done) | sci_s2 p.34, sci_s3 p.57 |

## 3. Session 1: Overview and Design Philosophy of the StampFly Ecosystem (10:05-11:00)

Of 22 frames, 4 are demonstration or expected-result frames.

| Type | Page | Content | Command | Expected result | Rough time | Prep & pitfalls | Fallback |
|------|------|---------|---------|------------------|-----------|------------------|----------|
| Demo (watch only) | p.24 | Demo 1: Simulator Piloting. `sf sim run vpython` launches the VPython simulator, running the same control algorithm as the vehicle, flown with an HID joystick | `sf sim run vpython` | A 3D view opens in the browser and the vehicle responds to stick input. Confirms the control code's behavior without hardware | 3-4 min (rough) | Have the HID joystick (AtomS3 + Atom JoyStick) connected over USB beforehand. This demo does not depend on venue networking, so it is low-risk | None specific (fallback/ has no material for S1's simulator-piloting demo; if it fails, continue with the slide diagram alone) |
| Demo (watch only) | p.25 | Demo 2: Real POS_HOLD Flight. Shows position-hold accuracy while hovering, and the recovery behavior after a disturbance | (No command string printed on this page. Hardware actions: ARM -> switch to POS_HOLD -> apply a light push as a disturbance) | Horizontal drift stays small and the vehicle returns to position after the disturbance (measured +/-6-7cm, the figure already shown on p.19 "vehicle Firmware Structure") | 3-4 min (rough) | Rehearse stable POS_HOLD beforehand per verification_checklist §4 ("Production-Firmware Demo Check"). Prop guards on, flight area netted off. If unstable, switch to ALT_HOLD or STABILIZE (checklist §4's fallback) | `S1_pos_hold_flight.mp4` (POS_HOLD flight video), or simply show the next two expected-result slides (pp.26-27) |
| Expected-result presentation | p.26 | Demo 2 Expected Result: Position Hold (a SILS simulation result, shown as a reference for how the live demo should go) | None (static image) | `pos_roll.scn` (vehicle): takeoff -> roll disturbance -> POS_HOLD engages -> holds. Max horizontal drift after engage: 0.39 m | 1-2 min (rough) | Image is `fallback/S1_pos_hold_xy.png` (already embedded in the slide) | Not applicable (this frame is itself the fallback material) |
| Expected-result presentation | p.27 | Demo 2 Expected Result: Altitude & Attitude | None (static image) | Altitude/attitude time series for the same flight. Tilt right after the disturbance returns to level under position control | 1-2 min (rough) | Image is `fallback/S1_altitude_attitude.png` | Not applicable |

## 4. Session 2: Environment Setup and Sensor Data Acquisition (11:00-12:00)

Of 17 frames, 2 are demonstration/hands-on frames.

| Type | Page | Content | Command | Expected result | Rough time | Prep & pitfalls | Fallback |
|------|------|---------|---------|------------------|-----------|------------------|----------|
| Hands-on (everyone) | p.35 | Environment Check. Everyone runs `sf doctor` together, diagnosing ESP-IDF, Python, the USB serial driver, and the sf CLI's own version | `source setup_env.sh`<br>`sf doctor` | Completes with no errors (same pass criterion as README §4) | 3-5 min (rough; longer if individual attendees need troubleshooting) | This should already be done before the day per README §4's pre-workshop prep. Not passing here is not a problem — those attendees continue "watch only" and catch up later via the review path (p.35 alertblock) | No dedicated fallback/ material. Attendees who fail individually switch to "watch only"; point them to `docs/guides/troubleshooting.md` |
| Demo (follow along) | p.43 | Demo: Tilt Test. Hold the StampFly and tilt it front/back/left/right; `gyro_x`/`gyro_y` swing with the tilt speed, and `accel_z` returns to about -9.81 at rest | `sf lesson switch sci2026:2`<br>`sf lesson build && sf lesson flash`<br>`sf monitor workshop` (requires the VSCode extension `alexnesnes.teleplot`; `sf lesson flash` opens this same connection right after flashing) | The Teleplot waveforms for `gyro_x`/`gyro_y`/`accel_z` move. At rest, gyro is approximately 0 and `accel_z` is approximately -9.81 m/s^2 | 5-7 min (rough; includes build/flash time) | Install the VSCode extension `alexnesnes.teleplot` beforehand. Write the code from p.40 (below) into `user_code.cpp` first. Since each attendee's build takes time, showing the flow once on the instructor's machine before everyone tries it saves time | No dedicated fallback/ material. If it does not work, share the instructor's own Teleplot view and switch to "watch only" |

Reference code (p.40, written before this demo):

```cpp
static uint32_t tick = 0;
void loop_400Hz(float dt) {
    if (tick++ % 4 == 0) {   // 100Hz decimation
        ws::print(">gyro_x:%.3f", ws::gyro_x());
        ws::print(">gyro_y:%.3f", ws::gyro_y());
    }
}
```

## 5. Session 3: Motor Control and Controller Input Implementation (13:00-14:00)

Of 16 frames, 2 are demonstration/hands-on frames. **Safety (p.51):** propellers removed throughout. Duty capped at 0.15 (not enforced by the firmware, so verify visually). ARM only via a single click of the body button or the controller, never auto-armed in code.

| Type | Page | Content | Command | Expected result | Rough time | Prep & pitfalls | Fallback |
|------|------|---------|---------|------------------|-----------|------------------|----------|
| Hands-on | p.54 | Hands-on: Hardcoded Duty. Hardcode `motor_set_duty` calls in `user_code.cpp` and confirm motor spin with no propellers | `sf lesson switch sci2026:3`<br>(No build/flash command printed on this page; per the standard workflow on p.37, run `sf lesson build` then `sf lesson flash`) | Arming from the body button spins only FR (M1) at low speed (duty 0.10; the others are 0.00). DISARM stops it | 5-8 min (rough) | **Safety (p.51):** propellers off, duty <=0.15, ARM only via body button or controller. **Note:** the distributed solution (`--solution`) auto-arms on boot, so use it only with propellers off (per the warning now printed on p.54) | No dedicated S3 fallback/ material. If the bench check (verification_checklist §1) fails, revert to production firmware (`sf flash vehicle`) and switch this demo to watch-only |
| Demo (follow along) | p.60 | Demo: Stick to Motors. Write `ws::motor_mixer` and see the four motors' speed differ with stick input (the controller's commands arrive over ESP-NOW as a `ControlPacket` at roughly 50 Hz); shows that even prop-off it cannot hold commanded speed under a light disturbance | `sf lesson switch sci2026:4`<br>`sf lesson build && sf lesson flash` | Motor speeds diverge with stick input. A light gust defeats the commanded speed (the limit of open-loop control) | 5-7 min (rough) | If pairing has not been done, do it first per p.57 (hold the controller's LCD panel button while powering on -> hold the body button 2s -> both beep). Keep propellers off | No dedicated S3 fallback/ material. As in checklist §1, revert to `sf flash vehicle` if this fails |

Reference code (p.54, written for this exercise):

```cpp
void setup() {
    ws::print("Motor duty test");
    // Do NOT auto-arm in code / コードでは自動ARMしない
}
void loop_400Hz(float dt) {
    ws::motor_set_duty(1, 0.10f);   // FR
    ws::motor_set_duty(2, 0.00f);   // RR
    ws::motor_set_duty(3, 0.00f);   // RL
    ws::motor_set_duty(4, 0.00f);   // FL
}
```

## 6. Session 4: Feedback Control Basics -- PID Attitude Stabilization (14:00-15:00)

Of 24 frames, only 2 are demonstration/expected-result frames. **Exercises 5-9 themselves (writing code and flying it) are watch-only on the day** (p.84's demo-viewing note: "reproduce at home: today is watch-only, run Exercises 5-9 end to end afterward"). Only the instructor actually flies live that day.

| Type | Page | Content | Command | Expected result | Rough time | Prep & pitfalls | Fallback |
|------|------|---------|---------|------------------|-----------|------------------|----------|
| Demo (watch only) | p.84 | Demo: Final Code. Fly the PID-ified learner code (Exercise 8) and compare the roll-rate step response against P-control (Exercise 5) | `sf log wifi -d 30 -o flight.csv`<br>`sf log viz flight.csv` (p.84 only describes this in prose -- "capture telemetry with `sf log wifi`", "show the step response with `sf log viz`" -- without printing the exact arguments; the above borrows the syntax printed on p.42) | Against a 15.1 deg/s target, Exercise 5 (P) peaks at 17.9 deg/s with a -2.4 deg/s undershoot; Exercise 8 (PID) peaks at 13.2 deg/s with no ringing (p.85 caption) | 5-8 min (rough) | Only the instructor flies live. Rehearse stable flight for Exercises 5 and 8 per verification_checklist §2 ("Flight Check"). Prop guards on, eye protection | `fallback/S4_lesson5_p_flight.mp4`, `fallback/S4_lesson8_pid_flight.mp4` (per checklist §2, switch only the unstable lesson to video if needed) |
| Expected-result presentation | p.85 | Demo Expected Result: P vs. PID step response (the on-slide title dropped "control" and "comparison graph" for brevity) | None (static image) | Same figures as above (p.85 caption) | 2-3 min (rough) | Image is `fallback/S4_roll_step_p_vs_pid.png` | Not applicable (this frame is itself the fallback material) |

**Frames excluded from this table (mention verbally only, or demo at the instructor's discretion):**

- p.75 "Recap" (retitled from "Rate P-Control Recap"; JA title "実習5の振り返り: レートP制御"): re-displays Exercise 5's code for explanation; nothing is re-run live here
- p.82 "System ID: sf sysid fit", p.83 "Autotuning: sf sysid rate-fit / rate-tune": both present example command output only; the slides do not state that the instructor runs these live on the day

## 7. Session 5: Simulator and Analysis Tools, Advanced Topics (15:30-16:30)

Of 23 frames, 3 are demonstration/hands-on/expected-result frames.

| Type | Page | Content | Command | Expected result | Rough time | Prep & pitfalls | Fallback |
|------|------|---------|---------|------------------|-----------|------------------|----------|
| Hands-on | p.100 | Exercise: Fly Your PID in SILS. Verify Exercise 8's (PID) `setup()`/`loop_400Hz()` in SILS before flying it for real | `sf sils build --target workshop`<br>`sf lesson switch sci2026:8 --solution` (or your own code)<br>`touch firmware/workshop/main/user_code.cpp`<br>`sf sils build --target workshop` (rebuild to pick it up)<br>`sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop` | Pass criteria (`.expect`): lift-off (true altitude > 0.1 m) and tilt under 15 degrees (no tumble). No altitude loop, so landing is DISARM descent only. Following the sequence as written yields `alt_max` ~= 0.64 m, `tilt_max` = 0.0, i.e. PASS | 4-6 min (rough; longer including `sf sils build`'s first compile) | `sf lesson switch` copies the file while preserving `user_code.cpp`'s old mtime, so building right after the switch does not detect the new code. Touching the file and then rebuilding a **second** time is a required two-step sequence every time -- not an occasional workaround. `sf sils gui` cannot select the workshop target, hence this CLI sequence (excluded from `sf sils regression` by default; include it with `--include-workshop`) | No dedicated video for this exercise in fallback/. If the CLI run fails, explain verbally; SILS's general reliability can be backed by `fallback/S5_regression_summary.txt` (28 PASS + 5 known, tracked issues + 1 workshop-target skip, 34 total) |
| Demo (the `sf sim run vpython` part is follow-along) | p.101 | Demo: The Simulator. 3D piloting in VPython, plus running a scenario and checking pass/fail in `sf sils gui`, then varying a parameter to see the effect | `sf sim run vpython`<br>`sf sils build`<br>`sf sils gui` | A 3D piloting view opens in the browser. Running a scenario in `sf sils gui` shows a PASS/FAIL verdict, and changing a parameter changes the behavior | 5-8 min (rough) | `sf sils build` only needs to run once (p.99). Connect the HID joystick (AtomS3 + Atom JoyStick) if available; otherwise it falls back to keyboard control | `fallback/S5_stab_flight.mp4` (STABILIZE flight video), `fallback/S5_attitude_rate.png`, `fallback/S5_gate_result.txt` (12-item pass verdict) |
| Expected-result presentation | p.102 | Demo Expected Result: SILS Scenario Run. Attitude/angular-rate time series | None (static image) | `stab_flight.scn` (vehicle) attitude and angular rate. All 12 pass criteria satisfied (`att_rmse=2.82<3.0`, `tilt_max=13.77<18.0`, `duty_max=1.00<1.001` -- figures from `fallback/README.md`'s numeric summary) | 2-3 min (rough) | Image is `fallback/S5_attitude_rate.png` | Not applicable (this frame is itself the fallback material) |

## 8. All Commands, in Execution Order

Listed per session in the order typed on the day (duplicates within a session removed). The slides previously showed both bare `sf sim run` and `sf sim run vpython`; the reworked deck now prints `sf sim run vpython` consistently everywhere -- see §10.

**Session 1**

```bash
sf sim run vpython
```

**Session 2**

```bash
source setup_env.sh
sf doctor
sf lesson switch sci2026:2
sf lesson build && sf lesson flash
```

**Session 3**

```bash
sf lesson switch sci2026:3
sf lesson build
sf lesson flash
sf lesson switch sci2026:4
sf lesson build && sf lesson flash
```

**Session 4**

```bash
sf log wifi -d 30 -o flight.csv
sf log viz flight.csv
```

**Session 5**

```bash
sf sils build --target workshop
sf lesson switch sci2026:8 --solution
touch firmware/workshop/main/user_code.cpp
sf sils build --target workshop
sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop
sf sim run vpython
sf sils build
sf sils gui
```

## 9. Exercise-to-Lesson Map

"Exercise N" refers to the tutorial-specific course `sci2026` (`firmware/workshop/lessons/lesson_manifest.yaml`), which maps onto the underlying Workshop lesson numbers below. Switch with `sf lesson switch sci2026:N`.

| Exercise | Session | Underlying Workshop Lesson | What participants write (function names, etc.) |
|----------|---------|------------------------------|---------------------------------------------------|
| Exercise 1 | S2 | environment_setup (Workshop Lesson 0) | None (environment check only; centers on running `sf doctor`, no code written) |
| Exercise 2 | S2 | imu_sensor (Workshop Lesson 4) | In `loop_400Hz`, print IMU values Teleplot-style with `ws::print(">gyro_x:%.3f", ws::gyro_x())` etc. |
| Exercise 3 | S3 | motor_control (Workshop Lesson 1) | In `loop_400Hz`, drive each motor directly with `ws::motor_set_duty(id, duty)` |
| Exercise 4 | S3 | controller_input (Workshop Lesson 2) | Read `ws::rc_throttle/roll/pitch/yaw()` and distribute to the four motors with `ws::motor_mixer(T,R,P,Y)` |
| Exercise 5 | S4 | rate_p_control (Workshop Lesson 5) | Compute the rate error (`re`/`pe`/`ye`) against target and feed `Kp_rp*re` etc. into `ws::motor_mixer` as proportional control |
| Exercise 6 | S4 | system_modeling (Workshop Lesson 6) | None (lecture only, no hardware; compute the $K_p$ for a $\zeta=0.7$ design from the measured $K$, $\tau_m$) |
| Exercise 7 | S4 | system_identification (Workshop Lesson 7) | Call `ws::set_rate_target(roll,pitch,yaw)` right after computing the rate target, logging it into the Data Stream's `rate_ref_*` |
| Exercise 8 | S4 | pid_control (Workshop Lesson 8) | Replace the ideal derivative with an incomplete-derivative filter (computing `alpha`, `a`, `b`, `d_filt`) |
| Exercise 9 | S4 | attitude_estimation (Workshop Lesson 9) | Hand-write a complementary filter $\hat\theta_k=\alpha(\hat\theta_{k-1}+\omega\Delta t)+(1-\alpha)\theta_{accel}$ (e.g. a `cf_roll` variable) and compare it against `ws::estimated_roll()` |

## 10. Inconsistencies Found Between the Slides and the Markdown Companions (For Reference)

Recorded as found, not fixed.

| Where | Detail |
|-------|--------|
| `sf sim run` wording differs (**resolved**) | pp.24/28/17 (S1 deep-dive) used to print bare `sf sim run` (no argument), while pp.98/102 (S5), appendix p.116, README, handson_guide, and cheatsheet printed `sf sim run vpython` (with the argument). After the post-review rebuild, every page in the current deck prints `sf sim run vpython` consistently, so this wording gap is resolved |
| `verification_checklist.md`'s lesson numbers vs. "Exercise N" (**resolved**) | `verification_checklist.md` §1 used to bench-check using the **raw Workshop lesson numbers** (`sf lesson switch 0/1/2/4`), which did not match the `sci2026` course's Exercise 1-4 sequence in `lesson_manifest.yaml` (see §9's table for the exact correspondence). The checklist has since been normalized to the same `sci2026:N` notation used in this runsheet (`sf lesson switch sci2026:N`), resolving the mismatch |
| Scope of "watch only" in S4 | p.84's demo-viewing note says "today is watch-only; run Exercises 5-9 end to end afterward," while `verification_checklist.md` §2 is written assuming the instructor personally rehearses and flies Exercises 5 and 8 live that day. The two are not actually contradictory (participants watch; the instructor flies), but the slide alone does not make clear whether the instructor really flies live or only shows the fallback video. This runsheet's p.84 row spells out the fallback video option, informed by checklist §2's failure-mode guidance |
| No fallback material for S2/S3 | `docs/events/sci_tutorial_2026/fallback/README.md`'s material covers only S1, S4, and S5 -- there is no dedicated video/image for S2 (IMU check) or S3 (motor/controller). If either fails on the day, the only documented fallbacks are reverting to production firmware (`sf flash vehicle`) or switching to watch-only, per verification_checklist §1 |
