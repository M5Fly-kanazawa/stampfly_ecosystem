# 実演・実習ランシート（講師用）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

2026年9月10日（木）開催の SCI/SICE チュートリアル講座 2026（全160ページのスライド `docs/events/sci_tutorial_2026/slides/sci_tutorial.pdf`）から、講師が当日**実際に実演する（実演）**か**参加者にやってもらう（実習）**フレーム、および**期待結果を提示するだけ**のフレームだけを抜き出し、進行順に並べたもの。157ページ全体をめくらなくても、この1枚でリハーサルと本番進行ができることを目的とする。

### 出典と手法

- スライド本文: `docs/events/sci_tutorial_2026/slides/sci_tutorial.tex` および `chapters/sci_intro.tex`、`sci_s1_overview.tex`〜`sci_s5_sim_analysis.tex`、`sci_appendix.tex`
- 参加者向け・復習向け資料: `README.md`（タイムテーブル・事前準備）、`handson_guide.md`（帰宅後の復習手順）、`cheatsheet.md`（コマンド・API早見表）、`verification_checklist.md`（講師のリハーサル手順）、`fallback/README.md`（代替素材の索引）
- ページ番号は `pdftotext -f N -l N sci_tutorial.pdf -` で1ページずつ本文を抽出し、各チャプターの `\begin{frame}` 数（intro 6 + S1 23 + S2 21 + S3 17 + S4 50 + S5 26 + 付録 10 = 153、セッション区切り6枚（Session 1〜5と付録）・表紙1枚を加えて160）が実ページ数160と一致することを確認したうえで、代表ページを個別に照合した

### 使い方

- 各セッションの表は「実演」「実習」「期待結果の提示」のいずれかに分類されるフレームだけを載せている（地図・セッション冒頭の主張・理論とコード対応表・復習パス・チェックポイントなど純粋な講義フレームは含まない）
- 「使うコマンド」はスライドに印字されている文字列をそのまま転記している。スライドが手順を散文で説明するだけでコマンド文字列を明記していない箇所は、その旨を注記したうえで他ページに印字されている同じ操作のコマンドを補って示す
- 「所要目安」はスライドに時間配分の明記がないため、セッション全体の時間とフレーム数から講師が按分した目安（分）である
- 「代替」列は実演できない場合の代替として `docs/events/sci_tutorial_2026/fallback/` にある素材を指す。同ディレクトリには S1・S4・S5 の素材しかなく、S2・S3 用の代替素材は存在しない（§10にも記載）

## 2. 開始前チェック

### 数日前まで（リハーサル）

`verification_checklist.md` の §1〜§4（ベンチ確認・飛行確認・システム同定確認・本番ファームでのデモ確認）と §5（動画・ログの代替取得）を完了しておく。これは当日の朝に行うものではなく、事前に完了させておくべき作業である。

### 当日、進行を始める直前

| 項目 | 確認内容 | 出典 |
|------|---------|------|
| 実機・コントローラ | 講師用デモ機と貸出用予備のバッテリーを全数充電。対面参加者は基本的に実機を持参する（README §「持ち物」）ため、予備の必要台数はその日の参加者構成による — 資料に具体的な予備台数の指定はない | README 開催情報, verification_checklist §5 |
| 書き込み | 講師機・貸出機とも `sf flash vehicle` / `sf flash controller` で当日朝までに書き込み済みにする（Web Flasher は p.36 で紹介するだけで使わない） | README §4 |
| 講師 PC 環境 | `source setup_env.sh` を実行し、`sf doctor` がエラーなく通ることを確認 | README §4, sci_s2 p.37 |
| 代替素材フォルダ | `docs/events/sci_tutorial_2026/fallback/` を開いておく（実演が失敗した場合に即座に画像・動画を提示できる状態にする） | verification_checklist §5, fallback/README.md |
| 投影・配信 | プロジェクタ接続を確認。Zoom 画面共有をテストする（オンライン参加者向け、当日はオンデマンド配信もあり） | README 開催情報 |
| ペアリング | 貸出用コントローラは事前ペアリング済みであることを確認（初回のみ手動: コントローラ LCD パネルボタンを押しながら電源投入 → StampFly 本体ボタンを 3 秒以上押し続けビープで離す → コントローラ画面の一覧から機体（MAC下4桁）を選んでボタンで確定。5 秒以上押し続けるとシステムリセット。機体には MAC 下4桁のラベルを貼っておく〈`mac` コマンドで確認〉） | sci_s2 p.36, sci_s3 p.64 |

## 3. Session 1: StampFly Ecosystem の全体像と設計思想（10:05–11:00）

全23フレーム中、実演・期待結果の提示は4件。セッションは「目的（p.11〜14: 制御がなければ飛べない → 教材にする 4 つの障壁 → 既存プラットフォームとの比較 → Ecosystem が目指すもの）」「思想（p.15〜17: 5 つの設計の考え方 → 4 階層アクセス → 実機とシミュレータをつなぐ 3 原則）」「提供（p.18〜25: エコシステムの地図 → 機体 → ファームウェア → 400 Hz ループ → 3 種類のシミュレーション → sf sils gui の画面 → 教材の 8 分野）」の三段で進め、そのあとデモ①②（p.26〜29）、復習パス（p.30）、チェックポイント（p.31）で締める。各フレーム題名の「目的:／思想:／提供:」がどの段かを示す。sf sils gui（p.24）は画面を見せるだけで当日デモは不要（使い方と実習は S5）。

| 種別 | ページ | 内容 | 使うコマンド | 期待する結果 | 所要目安 | 事前準備・注意 | 代替 |
|------|--------|------|-------------|-------------|---------|---------------|------|
| 実演（見るだけ） | p.26 | デモ①: シミュレータ操縦。`sf sim run vpython` でVPythonシミュレータを起動し、実機と同じ制御アルゴリズムをHIDジョイスティックで操縦する | `sf sim run vpython` | ブラウザに3D表示が立ち上がり、スティック操作に応じて機体が動く。実機がなくても制御コードの挙動を確認できる | 3〜4分（目安） | HIDジョイスティック（AtomS3 + Atom JoyStick）をUSB接続しておく。会場ネットワークに依存しないため失敗しにくい実演 | 特になし（fallback/にS1のシミュレータ操縦専用の素材はない。うまくいかない場合はスライドの図解のみで説明を続ける） |
| 実演（見るだけ） | p.27 | デモ②: 実機POS_HOLD飛行。ホバー中の位置保持精度と、外乱を与えたときの復帰動作を見せる | （このページにコマンド文字列の明記なし。実機操作: ARM → POS_HOLDへモード切替 → 手で軽く押すなどの外乱を与える） | 水平ドリフトが小さく抑えられ、外乱後に元の位置へ戻る（実測 ±6–7cm、p.20「vehicleファーム構造」フレームの数値） | 3〜4分（目安） | verification_checklist §4「本番ファームでのデモ確認」でPOS_HOLDの安定動作を事前リハーサル必須。飛行エリアをネットで区画し、指定エリアの中だけで飛ばし、操作は必ず立って行う。異常時は即 DISARM。不安定な場合はALT_HOLDまたはSTABILIZEへ切替（checklist §4の代替方針） | `S1_pos_hold_flight.mp4`（POS_HOLD飛行動画）。または次の2枚の期待結果スライド（p.28, p.29）をそのまま見せる |
| 期待結果の提示 | p.28 | デモ②の期待結果: 位置保持（SILSでのシミュレーション結果を参考として提示） | なし（静止画） | `pos_roll.scn`（vehicle）: 離陸→ロール外乱→POS_HOLD係合→保持。係合後の水平ドリフト最大0.39m | 1〜2分（目安） | 画像は`fallback/S1_pos_hold_xy.png`（スライドに埋め込み済み） | 該当なし（本フレーム自体が代替素材） |
| 期待結果の提示 | p.29 | デモ②の期待結果: 高度と姿勢 | なし（静止画） | 同じ飛行の高度・姿勢角の時系列。外乱直後の傾きが位置制御で戻る | 1〜2分（目安） | 画像は`fallback/S1_altitude_attitude.png` | 該当なし |

## 4. Session 2: 開発環境のセットアップとセンサデータの取得（11:00–12:00）

全21フレーム中、実演・実習は5件（再構成で17→19→18→20→21フレームに推移。実習1・実習2それぞれに手順を明記した専用フレームが立った後、「開発環境の導入 (1/2): PC側」がオープニング（先出し）へ移動し1枚減り、さらにWiFi接続の説明フレームが2枚新設され、実習 1 (2/2)「機体の初期設定を一度に済ませる」が加わった）。

未導入の参加者への案内は、このセッション冒頭ではなくオープニングの「開発環境の導入（先出し）: PC側」で行う（CLI 導入の 3 段階: 前提ツール → clone と install.bat → setup_env.bat と sf doctor。GUI 版インストーラは安定性未確認のため使わない。参加者はWindowsが多い想定なので `install.bat` / `setup_env.bat` を主に書き、macOS/Linuxは括弧内）。当セッションは導入済み前提でOS差異の参照とペアリング・実習1の確認から始まる。

新設の p.35「OSによる違いはここだけ」は macOS/Linux と Windows のコマンド差異（`./install.sh` 対 `install.bat`、`source setup_env.sh` 対 `setup_env.bat`、シリアルポート名）をまとめた参照フレームで、当日はデモしない。Windows参加者から質問が出た際の説明に使う。

**新設（p.46–47）:** 実習2（2/2・傾けて確認）の直後、「データ取得経路② sf telemetry」の直前に、フレーム「WiFi でつなぐ: 有線と無線の使い分け」「接続手順: 機体を WiFi 網に入れる」を2枚追加した。②③（`sf telemetry`／`sf log wifi`）はいずれも WiFi 接続が前提のため、その手前で接続方法を説明する。以下2点が追加分の要点：

- 出荷時既定は `wifi.mode`=0（STA・資格情報未設定）でテレメトリ無効。SoftAP（機体自身が出す WiFi）への切替は実習 1 (2/2)「機体の初期設定を一度に済ませる」で `param reset` → `param set wifi.mode 1` → `param set wifi.channel N` → `param save` → `reboot` として済ませる（以後は電源投入だけで自動的に SoftAP になる）。この枠では設定済みであることを確認するだけでよい
- **当日の運用は SoftAP モード**（各 StampFly が既定 SSID `StampFly-XXYY`〈MAC末尾から自動生成〉・既定パスワード `stampfly`・既定 IP `192.168.10.1` で自分の AP を出す）とし、会場 WiFi には繋がない・繋げない前提で進める（`verification_checklist.md` も「会場 WiFi 不通に備え」の前提で運用計画を立てている）。参加者ごとに機体の SSID が異なる点を口頭で明示すること

| 種別 | ページ | 内容 | 使うコマンド | 期待する結果 | 所要目安 | 事前準備・注意 | 代替 |
|------|--------|------|-------------|-------------|---------|---------------|------|
| 実習（全員参加） | p.37 | 実習1: 環境確認とビルド。全員で`sf doctor`を実行してESP-IDF・Python・USBシリアルドライバ・sf CLI自体を診断したうえで、機体ファーム（vehicle）をビルド・書き込みし、モニタを開く。sf CLIの基本4操作（診断・ビルド・書き込み・モニタ）だけに絞り、`sf lesson` はここでは出さない（p.40以降で扱う） | `setup_env.bat`（macOS/Linuxは`source setup_env.sh`）<br>`sf doctor`<br>`sf build vehicle`<br>`sf flash vehicle -m` | 書き込み直後に標準起動音（C5→E5→G5）が鳴り、機体のLEDが白（初期化中）→緑常灯（準備完了）になる。モニタに起動ログが流れる（終了は Ctrl+]）。`sf doctor`が全てOKにならなくても心配不要（p.37 alertblock） | 4〜6分（目安。個別に通らない参加者への対応を含めるとさらに延びる） | 事前準備（README §4）で前日までに済ませておくよう案内済み。通らなかった参加者は以降「見るだけ」で参加し、復習パス（本セッション末尾）で後日対応 | fallback/に専用素材はなし。個別に通らない参加者は「見るだけ」に切替え、`docs/guides/troubleshooting.md`を案内 |
| 実習（全員参加） | p.38 | 実習1 (2/2): 機体の初期設定を一度に済ませる。書き込み直後のモニタ CLI で既定値リセット・SoftAP 化・参加者ごとのチャンネル設定を保存して再起動し、続けてコントローラとペアリングする | `param reset`<br>`param set wifi.mode 1`<br>`param set wifi.channel N`（N は講師が参加者ごとに 1/6/11 を指定）<br>`param save`<br>`reboot`<br>ペアリング: コントローラ LCD ボタンを押しながら電源投入 → 機体ボタンを 3 秒以上押し続けビープで離す → コントローラ画面の一覧から自分の機体（MAC下4桁）を選んでボタンで確定 | 再起動後に `param get wifi.channel` が N を返し、確定後に機体からの応答があればペアリング完了（機体LEDが緑常灯になり、コントローラ操作に機体が反応する） | 5〜8分（目安。全員分のチャンネル指定を含む） | チャンネル表（誰が 1/6/11 か）を事前に用意して掲示する。ボタンは 5 秒以上押し続けるとシステムリセットになるのでビープで離す。実習コードの模範解答はチャンネルを変更しない（`ws::set_channel` は削除済み）。**全員同時にペアリングしてよい**（機体は自分宛の電文だけを相手候補にするため取り違えない）が、一覧には他の参加者の機体も表示されるので、必ず自分の機体のラベル（MAC下4桁。事前に `mac` コマンドで確認しシールを貼っておく）と一致する行を選ぶよう周知する | 失敗した機体は `sf flash vehicle -m` からやり直す |
| 実演（一緒に打つ） | p.43 | 実習2（1/2）: IMUの値を見る。`user_code.cpp`にp.41のコードを書き、ビルド・書き込み・`sf lesson monitor`でTeleplot接続まで行う | `sf lesson switch sci2026:2`<br>`sf lesson edit`（`user_code.cpp`を開いてp.41のコードを書く）<br>`sf lesson build`<br>`sf lesson flash`<br>`sf lesson monitor`（VSCode拡張 `alexnesnes.teleplot` 導入が前提。`sf lesson flash`直後は自動でこの接続状態のまま開く） | Teleplotで`gyro_x`/`gyro_y`の波形表示画面が開く（値はまだ静止状態） | 4〜6分（目安。ビルド・書き込み時間を含む） | VSCode拡張`alexnesnes.teleplot`を事前に導入しておく。参加者ごとにビルド時間が発生するため、講師機で先に流れを見せてから各自試すと時間短縮できる | fallback/に専用素材はなし。うまくいかない場合は講師機のTeleplot画面を共有し「見るだけ」に切替え |
| 実演（一緒に打つ） | p.45 | 実習2（2/2）: 傾けて確認。StampFlyを手に持ち前後左右に傾け、`gyro_x`/`gyro_y`が傾ける速さに応じて振れ、静止すると`accel_z`が−9.81付近に戻ることを確認する | （このページにコマンド文字列の明記なし。前ページ〈p.43〉までの`sf lesson build`/`sf lesson flash`完了後、Teleplot画面を見ながら機体を傾ける） | 静止時ジャイロ≈0、`accel_z`≈−9.81 m/s²。傾ける速さに応じて`gyro_x`/`gyro_y`の波形が振れる | 1〜2分（目安） | p.43の実習2（1/2）が完了していることが前提 | fallback/に専用素材はなし。うまくいかない場合は講師機のTeleplot画面を共有し「見るだけ」に切替え |
| 実演（新設。`sf telemetry`の前提説明） | p.46–47 | WiFiでつなぐ／接続手順（2枚）。次ページ以降の`sf telemetry`/`sf log wifi`はWiFi接続が前提であるため、機体をSoftAPモードへ切り替えPCを接続する手順を実演する | USB接続のまま`sf monitor`でCLIに入る<br>`param set wifi.mode 1`（`0`=STA既定／`1`=SoftAP）<br>`param save`<br>`reboot`<br>PC側WiFi設定で`StampFly-XXYY`（既定パスワード`stampfly`）に接続 | 機体のAPがPCのWiFi一覧に見え、接続後`sf telemetry`（IP指定不要、既定`192.168.10.1`）で50Hzテレメトリが受信できる | 3〜5分（目安。初回のみ。以後は電源投入だけでSoftAPに戻るため2回目以降は不要） | 事前に全参加機体の`wifi.mode`を講師が一括設定しておくと当日の時間短縮になる（検討事項）。参加者ごとに機体のSSIDが異なる点を明示する。会場WiFiには繋がない | 接続できない参加者は講師機の画面共有に切替え、`docs/guides/troubleshooting.md`の「WiFi接続」表を案内 |

参考コード（p.44「データ取得経路① Teleplot」、実習2で`user_code.cpp`に書くコード）:

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

全17フレーム中、実演・実習は2件（新設のTDMA解説フレームが1枚増え16→17）。**安全（p.57）**: 机上で実施し、モータ回転中は手を近づけない。Duty は0.15以下に固定（ファーム側では強制されないため必ず目視確認）。ARMは機体ボタンの単クリックまたはコントローラのみで行い、コード内で自動ARMしない。

新設の p.63「TDMAで30機を同時に飛ばす」はTDMA（時分割多重アクセス、Time Division Multiple Access）の原理と、非重複3チャンネル（1/6/11ch）で理論上30機まで運用できることを説明する参照フレームで、当日は実演しない。教室で複数機体が同時に飛ぶ場合の混信対策として質疑応答に使える。

| 種別 | ページ | 内容 | 使うコマンド | 期待する結果 | 所要目安 | 事前準備・注意 | 代替 |
|------|--------|------|-------------|-------------|---------|---------------|------|
| 実習 | p.60 | 実習3: Dutyをハードコードして回す。`sf lesson edit` で`loop_400Hz`を右のコードにして保存し、机上でモータの回転を確認する | `sf lesson switch sci2026:3`<br>`sf lesson edit`（右のコードにして保存）<br>`sf lesson build`<br>`sf lesson flash`<br>機体ボタンを1回クリックしてARM | 機体ボタンでARMするとFR（M1）のみが低速回転する（duty 0.10、他モータは0.00）。DISARMで停止 | 5〜7分（目安） | 【安全 p.57】机上で実施しモータ回転中は手を近づけない、Duty 0.15以下、ARMは機体ボタン単クリックまたはコントローラのみ。**注意（p.60）:** 配布される模範解答（`--solution`）は起動時に自動ARMするため、机上に固定し回転中は手を近づけない状態でのみ使う | fallback/にS3専用素材はなし。ベンチ確認（verification_checklist §1）でNGの場合は`sf flash vehicle`で標準ファームに戻し、このデモを「見るだけ」に切替える |
| 実演（一緒に打つ） | p.67 | 実習4: スティックでモータを回す。`ws::motor_mixer`を書き、スティック操作で4モータの回転差を確認する（コントローラの指令はESP-NOWで約50Hzの`ControlPacket`として届く）。外乱に弱いことを見せる | `sf lesson switch sci2026:4`<br>コントローラとペアリング<br>`sf lesson edit`（右のコードにして保存。`ws::motor_mixer(T,R,P,Y)`）<br>`sf lesson build`<br>`sf lesson flash` | スティックを倒すと4モータの回転数に差が出る。風で軽く煽ると指令通りの回転が保てない（オープンループ制御の限界） | 5〜7分（目安） | ペアリング未実施なら先にp.64の手順（コントローラLCDボタンを押しながら電源投入→本体ボタンを 3 秒以上押し続けビープで離す→コントローラ画面の一覧から自分の機体を選んで確定）を済ませる。機体は机上に置いたまま、モータ回転中は手を近づけない | fallback/にS3専用素材はなし。checklist §1と同様、NG時は`sf flash vehicle`に戻す |

参考コード（p.60「実習3: Dutyをハードコードして回す」、`user_code.cpp`の`loop_400Hz`）:

```cpp
void loop_400Hz(float dt) {
    ws::motor_set_duty(1, 0.10f);   // FR
    ws::motor_set_duty(2, 0.00f);   // RR
    ws::motor_set_duty(3, 0.00f);   // RL
    ws::motor_set_duty(4, 0.00f);   // FL
}
```

（`setup()`側のコードはこのフレームには印字されていない。ARMは機体ボタンの単クリックで行い、コードの中で自動ARMしないことをp.57本文が明記している。）

## 6. Session 4: フィードバック制御の基礎 — PID による姿勢安定化（14:00–15:00）

全50フレーム中、実演・実習・期待結果の提示は7件（29→46フレームへの再構成で、実習5〜9・デモ本編・デモの期待結果・期待結果の再現手順という実演系フレームの位置・内容・使うコマンドは変更していない。増えた17フレームはすべて座学の理論解説フレームで、多くのタイトル末尾に「あとで読む」の印が付き、当日は読み飛ばして帰宅後に参照する設計。ページ番号は再構成後の PDF（154ページ版）で振り直し済み）。

**参加者にとっては実習5〜9そのもの（コードを書いて実機で試す作業）は当日「見るだけ」で、実際に手を動かすのは帰宅後**（デモの見方フレームの記載: 「帰宅後に再現: 本日は見るだけにして，後日実習5〜9を通しで動かす」）。ただし、各実習フレーム自体には**講師が実演する内容**（初飛行・ζ比較・システム同定・PID比較・姿勢推定の一致確認）が numbered steps として明記されているため、下表では実習5〜9も実演フレームとして扱う。

**実習9の位置が変わった点に注意:** 新構成では実習9（姿勢推定）は「ジャイロドリフト問題」「加速度センサからの傾き角の導出」「相補フィルタ」という理論フレーム群の**後**に置かれる（再構成前は理論フレームより先に実習9があった）。実演の内容・手順・使うコマンドは変わらないが、当日の説明順序としては「まず相補フィルタの理論を説明してから実習9に入る」流れになる点を意識してリハーサルする。

**リハーサル状況に注意:** `verification_checklist.md` §2がリハーサル必須と定めているのは実習5・実習8・実習9の3件のみ。実習6（ζを3種類変えた飛行比較）と実習7（システム同定用の飛行）には専用のリハーサル項目がなく、当日の時間と機体状態次第で口頭説明のみに切り替えるか講師が判断する（下表の該当行に明記）。全実演を律儀に行うと5〜8分×5件+デモ本編5〜8分で合計30分近くを要し、60分のセッションを圧迫する点に留意する。理論フレームが大幅に増えたため、当日はライブでの解説を最小限にとどめ「あとで読む」フレームは口頭で存在だけ触れて先へ進む判断が今まで以上に重要になる。

| 種別 | ページ | 内容 | 使うコマンド | 期待する結果 | 所要目安 | 事前準備・注意 | 代替 |
|------|--------|------|-------------|-------------|---------|---------------|------|
| コードは一緒に打つ・飛行は見るだけ | p.77 | 実習5: レートP制御で初飛行。誤差＝目標角速度−実測角速度、出力＝Kp×誤差をRoll/Pitch/Yawで計算し`ws::motor_mixer`に渡すコードを各自が書いてビルドし、飛行は講師がKp=0.5の完成コードで行う | 各自: `sf lesson switch sci2026:5`<br>`sf lesson edit`（TODOを埋めて保存: `ws::gyro_x/y/z()`, `ws::rc_roll/pitch/yaw()`）<br>`sf lesson build`<br>講師のみ: `sf lesson flash` → 飛行 | スティック追従はできるが、わずかな振動が見える（P制御のみの限界）。角速度目標は`ws::set_rate_target()`でData Streamに記録され、実習7で使う | 4〜6分（目安） | verification_checklist §2でリハーサル必須（Kp=0.5でホバリング安定を確認）。低スロットルから開始、異常時は即DISARM | `fallback/S4_ex5_p_flight.mp4` |
| 実演（見るだけ。時間・リハーサル状況次第） | p.90 | 実習6: システムモデリング。設計式 $K_p=1/(4\zeta^2 K \tau_m)$ をRoll/Pitch/Yawで実装し、ζ=0.7→0.5→1.0の順に変えて3回飛行し違いを見せる | `sf lesson switch sci2026:6`<br>`sf lesson edit`（設計式でKp_roll/pitch/yawを計算するコードを書いて保存）<br>`sf lesson build`<br>`sf lesson flash`（ζを変えるたびに再ビルド・再書き込み） | ζ=0.5でやや振動的、ζ=0.7で滑らか、ζ=1.0で遅い応答という質的な違いが見える | 3〜5分（目安。時間が厳しい場合は1〜2種類のζに絞るか口頭説明のみに短縮） | verification_checklist §2に本実習専用のリハーサル項目はない（実習5のリハーサルで機体の基本安定性は確認済みという前提での実演）。3回の飛行を通しで行うと時間を圧迫する | 専用動画はfallback/にない。口頭説明への切替で対応 |
| 実演（見るだけ。時間・リハーサル状況次第） | p.94 | 実習7: システム同定（sf sysid fit）。実習5のP制御コードに`ws::set_rate_target(roll,pitch,yaw)`を追加し、飛行しながらログ取得後、プラント（$K, \tau_m$）を同定する | `sf lesson switch sci2026:7`<br>`sf lesson edit`（Kpを設定し`ws::set_rate_target`を呼ぶコードにして保存）<br>`sf lesson build`<br>`sf lesson flash`<br>飛行しながら`sf log wifi -o flight.csv`<br>`sf sysid fit flight.csv --plot` | 同定結果の$K$が実習6の理論値（$K_{roll}=102$, $K_{pitch}=70$）と数%の誤差で一致する（$\tau_m$は励振の強さに強く依存し理論値0.02sの数倍にズレることがあるが、それ単独では失敗ではない） | 4〜6分（目安） | verification_checklist §3で同定パイプライン自体は確認済みだが、§2に本実習専用の飛行リハーサル項目はない。収束しない場合はchecklist §3の代替方針どおり`analysis/reports/rate_sysid_reference/`の参照ログに切替え、実演区分を「一緒に打つ」ではなく「見るだけ」にする | 参照ログでのデモに切替（checklist §3） |
| コードは一緒に打つ・飛行は見るだけ | p.100 | 実習8 (1/2): PID制御。実習5のP制御にI項（`integral += error*dt`を±0.5でクランプ）とD項（`Kd*(error-prev_error)/dt`）を各自が追加してビルドし、飛行は講師が行って実習5（Pのみ）と比較する。書いたコードは実習8 (2/2)（p.132）でSILSで飛ばすので、それまで別の実習へ切り替えさせない（切り替えると`user_code.cpp`が上書きされる。上書き前に`firmware/workshop/my_code/`へ自動退避） | 各自: `sf lesson switch sci2026:8`<br>`sf lesson edit`（軸ごとにI項・D項・アンチワインドアップを追加して保存）<br>`sf lesson build`<br>講師のみ: `sf lesson flash` → 飛行 | 実習5と比べて定常偏差とオーバーシュートの変化が見える | 4〜6分（目安） | verification_checklist §2でリハーサル必須（PID化した状態で離陸しPのみと比べ定常偏差が減ることを確認）。disarm時はintegral/prev_errorを0にリセット | `fallback/S4_ex8_pid_flight.mp4` |
| 実演（見るだけ。手で傾ける、飛行はしない） | p.108 | 実習9: 姿勢推定（相補フィルタ）。`atan2f`で加速度から角度を計算し、相補フィルタ $\hat\theta_k=\alpha(\hat\theta_{k-1}+\omega\Delta t)+(1-\alpha)\theta_{accel}$（α=0.98）を実装、`ws::estimated_roll()`（機体既定のESKF）とTeleplotで比較する。**新構成ではジャイロドリフト・加速度からの傾き角導出・相補フィルタの理論フレームの後に位置する** | `sf lesson switch sci2026:9`<br>`sf lesson edit`（相補フィルタを実装して保存）<br>`sf lesson build`<br>`sf lesson flash`<br>`sf monitor`＋Teleplotで`cf_roll`と`eskf_roll`を比較 | 機体を手で傾けると、`cf_roll`と`eskf_roll`が近い挙動を示す | 3〜5分（目安） | verification_checklist §2でリハーサル必須（Teleplotで`cf_roll`と`eskf_roll`がおおむね一致することを確認）。飛行ではなくS2の実習2と同様の手持ち確認でよい | 専用動画なし。うまくいかない場合は講師機のTeleplot画面を共有 |
| 実演（`sf sysid fit`部分は一緒に打つ） | p.117 | デモ: 完成コードで飛行。PID化した実習コード（実習8）でホバーし、ロールのスティックを短く切って角速度目標（rate_ref）への追従の速さ・行き過ぎを波形で定性的に見る（手動では正確なステップ入力は入れられないため、設計値との定量比較は次フレームのSILSで行う）。取得した`flight.csv`は参加者にも共有し、各自のPCで`sf sysid fit`を一緒に実行する | `sf log wifi -d 30 -o flight.csv`<br>`sf log viz flight.csv`<br>（該当フレームの本文は「`sf log wifi`でテレメトリ取得」「`sf log viz`で角速度目標と実測角速度を重ねる」と散文で説明するのみで具体的な引数は明記されていない。上記はS2に印字されている構文を援用）<br>`sf sysid fit flight.csv --plot`（参加者が一緒に打つ部分） | 実機のスティック応答で、P制御（実習5）よりPID（実習8）の方が行き過ぎ・振動が小さいことが視覚的に確認できる（定量的なステップ応答の比較は次の期待結果フレームのSILS参考値、下記参照） | 5〜8分（目安） | 実際に飛行させるのは講師のみ。verification_checklist §2で実習8のリハーサル必須（実習5は別途リハーサル済み）。低スロットルから開始、異常時は即 DISARM | `fallback/S4_ex5_p_flight.mp4`、`fallback/S4_ex8_pid_flight.mp4`（checklist §2の方針どおり、不安定なレッスンだけ動画に切替も可） |
| 期待結果の提示 | p.118 | デモの期待結果: PとPIDのステップ応答。**SILS（MuJoCo）でロールステップ試験（ロール+0.3を0.5秒）を実行した結果であり、実機フライトそのものの数値ではなく設計の目安として提示される**（当該フレームのキャプション明記） | なし（静止画） | 目標15.1 deg/sに対し実習5（P）はピーク17.9 deg/sと−2.4 deg/sのアンダーシュート、実習8（PID）はピーク13.2 deg/sで振動なし（数値はSILSシミュレーション結果） | 2〜3分（目安） | 画像は`fallback/S4_roll_step_p_vs_pid.png`。詳しいSILS再現手順は次の「期待結果の再現手順」フレーム参照 | 該当なし（本フレーム自体が代替素材） |

**本表から除外したフレーム（参考として口頭で触れるにとどめる、または講師の裁量で任意実演）:**

- 「実習5の振り返り: 実習コード」: 実習5のコードを再掲する解説フレームで、その場での再実行はない
- 「同定結果と理論値の比較」（`sf sysid fit`の出力例と理論値比較表をまとめたフレーム）、「sf sysid rate-fit / rate-tune とは」「rate-excite → rate-fit → rate-tune の手順」「機体内で完結する自動チューン: sf_autotune」（いずれも vehicle ファーム専用・あとで読む）: いずれもコマンドの出力例を提示するのみで、本番中に講師が実際に実行するとはスライド上に明記されていない
- 「期待結果の再現手順」（あとで読む）: デモの期待結果フレームの数値をSILSで自分で再現するCLI手順（`sf lesson switch sci2026:5 --solution`→`sf lesson sils --scenario step`）。当日のライブ実演ではなく、事前準備または帰宅後の参照用
- 上記に加え、新設された多数の理論解説フレーム（フィードバック制御の基礎・P制御の限界・プラント物理モデルの深掘り・2次系設計とループ整形・システム同定の考え方・PID理論の深掘り（不完全微分/アンチワインドアップ/離散化/ARM時リセット）・加速度センサからの傾き角の導出・周波数掃引同定の原理とETFE・仕様ベースのゲイン設計）はすべて座学（実演なし）で、多くに「あとで読む」の印が付く。例外は「システム同定の考え方」（p.92）で、実習 7 の前提になるため当日講義する（「あとで読む」の印は外した）。当日は要点のみ口頭で触れ、詳細は帰宅後に資料で参照してもらう設計

## 7. Session 5: シミュレータ・解析ツールの活用と発展的テーマ（15:30–16:30）

全28フレーム中、実演・実習・期待結果の提示は3件（「研究利用の入口」は4枚 p.137〜140。(1/4)「自分のコードはどこに書くか」は `sf app new/edit/build/flash` の手順で当日1分だけ触れ、(2/4)〜(4/4) は「あとで読む」の参照フレームで実演しない）。

| 種別 | ページ | 内容 | 使うコマンド | 期待する結果 | 所要目安 | 事前準備・注意 | 代替 |
|------|--------|------|-------------|-------------|---------|---------------|------|
| 実習（全員参加） | p.132 | 実習8 (2/2): 自分のPIDをSILSで飛ばす。実習8 (1/2) で各自が書いた`setup()`/`loop_400Hz()`を、実機で飛ばす前にSILSで検証する | `sf lesson sils`（S4 で書いたコードのまま。書き切れなかった人は先に `sf lesson switch sci2026:8 --solution`） | 合格基準（`.expect`）は離陸（真値高度 > 0.1m）と傾き15°未満（発散しない）の2点。高度ループがないため着陸はDISARM降下のみ。上記どおりに実行すると`alt_max`≈0.64m、`tilt_max`=0.0でPASS | 4〜6分（目安。初回コンパイル時間を含めると延びる） | `sf lesson sils`が内部で再ビルドとシナリオ実行までまとめて行うため、切替後にこの1行を打つだけでよい。実習コードのSILSは`sf lesson sils`を使う（`sf sils gui`はvehicle本体向け。既定で`sf sils regression`から除外、`--include-workshop`で含む） | fallback/に本実習専用の動画はない。CLI実行が失敗した場合は口頭説明に切替え、SILS自体の信頼性は`fallback/S5_regression_summary.txt`（28 PASS + 5 既知の追跡中課題 + 1 実習コード対象スキップ、計34本）で補強できる |
| 実演（`sf sim run vpython`部分は一緒に打つ） | p.133 | デモ: シミュレータを動かす。VPythonシミュレータの3D操縦と、`sf sils gui`でのシナリオ実行・合否判定確認、パラメータ変更による挙動変化の確認 | `sf sim run vpython`<br>`sf sils gui`（`sf sils build`はp.127に印字。demoページ自体は「シナリオを1本実行し判定結果を確認する」と散文で説明するのみ） | ブラウザに3D操縦画面が出る。`sf sils gui`でシナリオを実行するとPASS/FAILの判定結果が表示され、パラメータを変えると挙動が変わる | 5〜8分（目安） | HIDジョイスティック（AtomS3 + Atom JoyStick）があれば接続、なければキーボード操作にフォールバック（フォールバック自体はスライドに明記なし、VPythonシミュレータの一般的な操作性による） | `fallback/S5_stab_flight.mp4`（STABILIZE飛行動画）、`fallback/S5_attitude_rate.png`、`fallback/S5_gate_result.txt`（12項目の合否判定結果） |
| 期待結果の提示 | p.134 | デモの期待結果: SILSシナリオ実行。姿勢角・角速度の時系列グラフ | なし（静止画） | `stab_flight.scn`（vehicle）の姿勢角と角速度。12項目の合否判定をすべて満たす（`att_rmse=2.82<3.0`, `tilt_max=13.77<18.0`, `duty_max=1.00<1.001` — 数値は`fallback/README.md`の数値サマリより） | 2〜3分（目安） | 画像は`fallback/S5_attitude_rate.png` | 該当なし（本フレーム自体が代替素材） |

## 8. コマンド全一覧（実行順）

セッションごとに、当日実際にタイプする順でコマンドを列挙する（同一セッション内の重複は除去）。以前はスライド内に`sf sim run`（引数なし）と`sf sim run vpython`の表記ゆれがあったが、再構成後の現行スライドは全箇所`sf sim run vpython`に統一されている（§10参照）。

**Session 1**

```bash
sf sim run vpython
```

**Session 2**

```bash
setup_env.bat            # macOS/Linux: source setup_env.sh
sf doctor
sf build vehicle
sf flash vehicle -m
sf lesson switch sci2026:2
sf lesson edit
sf lesson build
sf lesson flash
sf lesson monitor
```

**Session 3**

```bash
sf lesson switch sci2026:3
sf lesson build
sf lesson flash
sf lesson switch sci2026:4
sf lesson build
sf lesson flash
```

**Session 4**

見るだけ扱いの実習5〜9まで含めた「理想の通し」の全順序。実際にタイプするのは講師のみで、実習6・実習7は時間次第で省略しうる（§6参照）。

```bash
sf lesson switch sci2026:5
sf lesson build
sf lesson flash
sf lesson switch sci2026:6
sf lesson build
sf lesson flash
sf lesson switch sci2026:7
sf lesson build
sf lesson flash
sf log wifi -o flight.csv
sf sysid fit flight.csv --plot
sf lesson switch sci2026:8
sf lesson build
sf lesson flash
sf lesson switch sci2026:9
sf lesson build
sf lesson flash
sf log wifi -d 30 -o flight.csv
sf log viz flight.csv
sf sysid fit flight.csv --plot
```

**Session 5**

```bash
sf lesson switch sci2026:8 --solution
sf lesson sils
sf sim run vpython
sf sils build
sf sils gui
```

## 9. 実習の対応表

「実習 N」は本チュートリアル専用のコース `sci2026`（`firmware/workshop/lessons/lesson_manifest.yaml` で定義）のN番目の実習を指す。`sf lesson switch sci2026:N`で切り替える。

| 実習 | セッション | `lesson_manifest.yaml` 上の内部名 | 参加者が書くもの（関数名など） |
|------|-----------|---------------------------|-------------------------------|
| 実習1 | セッション2 | environment_setup | なし（環境確認とビルド確認のみ。`sf doctor`の実行とビルド・書き込みが中心で、コード記述は発生しない） |
| 実習2 | セッション2 | imu_sensor | `loop_400Hz`内で`ws::print(">gyro_x:%.3f", ws::gyro_x())`等、IMU値をTeleplot形式で出力 |
| 実習3 | セッション3 | motor_control | `loop_400Hz`内で`ws::motor_set_duty(id, duty)`を各モータへ直接指定 |
| 実習4 | セッション3 | controller_input | `ws::rc_throttle/roll/pitch/yaw()`を読み、`ws::motor_mixer(T,R,P,Y)`で4モータへ配分 |
| 実習5 | セッション4 | rate_p_control | 目標角速度との誤差（`re`/`pe`/`ye`）を計算し、`Kp_rp*re`等を`ws::motor_mixer`に渡す比例制御 |
| 実習6 | セッション4 | system_modeling | なし（座学のみ、実機操作なし。実測パラメータ$K$, $\tau_m$から$\zeta=0.7$設計の$K_p$を計算する） |
| 実習7 | セッション4 | system_identification | 角速度目標を計算した直後に`ws::set_rate_target(roll,pitch,yaw)`を呼び、Data Streamの`rate_ref_*`に記録 |
| 実習8 | セッション4 | pid_control | 理想微分から不完全微分フィルタへの置換（`alpha`, `a`, `b`, `d_filt`の計算） |
| 実習9 | セッション4 | attitude_estimation | 相補フィルタ $\hat\theta_k=\alpha(\hat\theta_{k-1}+\omega\Delta t)+(1-\alpha)\theta_{accel}$ を自作（変数`cf_roll`等）し、`ws::estimated_roll()`と比較 |

## 10. スライドとMarkdown資料の間で見つかった不整合（参考）

修正はせず、事実として記録する。

| 箇所 | 内容 |
|------|------|
| `sf sim run` の表記ゆれ（**解消済み**） | 従来、pp.24/28/17（S1深掘り）は引数なし`sf sim run`と表記し、S5（現行では p.128「VPythonとGenesis」・p.133「デモ: シミュレータを動かす」）・付録（現行では p.152「sf CLIチートシート(2/2)」）、README、handson_guide、cheatsheetは`sf sim run vpython`（引数あり）と表記していた。批判的レビューを反映した再構成後の現行スライドは全ページが`sf sim run vpython`に統一されており、この表記ゆれは解消済み |
| `verification_checklist.md`のレッスン番号と`実習N`の不一致（**解消済み**） | 従来、`verification_checklist.md` §1は`sf lesson switch 0/1/2/4`という**`lesson_manifest.yaml`上の生の内部レッスン番号**でベンチ確認しており、同ファイルの`sci2026`コースにおける実習1〜4の順序とは対応がずれていた（正確な対応関係は§9の表を参照）。`verification_checklist.md`側を本ランシートと同じ`sci2026:N`表記（`sf lesson switch sci2026:N`）に統一し、この不一致は解消済み |
| S4「本日は見るだけ」の実演範囲 | p.117のデモの見方には「本日は見るだけにして，後日実習5〜9を通しで動かす」とあるが、`verification_checklist.md` §2は講師自身が実習5・実習8・実習9を当日リハーサルで確認する前提で書かれている（実習6・実習7には専用のリハーサル項目がない）。両者は矛盾しないが（参加者は見るだけ・講師は実演する、という役割分担）、実習6・実習7を含めた5件すべての実演を60分のセッション内で律儀にこなすと時間を圧迫する。**未解決:** 実習6（ζ3種類の飛行比較）・実習7（システム同定用の飛行）を当日実際に飛ばすのか、口頭説明のみに留めるのかはスライド・checklistのどちらにも明記がなく、講師の当日の判断に委ねられている |
| S4「デモの期待結果」の数値の出どころ（**解消済み**） | p.118キャプションは「SILS（MuJoCo）でロールステップ試験を実行」した結果であることを明記しており、実機フライトの実測値ではない。p.117「デモ: 完成コードで飛行」も、実機では手動のスティック操作で追従を定性的に見るだけで、設計値との定量比較はp.118のSILSで行うと明記した。ライブ実演の波形がp.118の参考値（ピーク17.9/13.2 deg/s等）と一致しないのは前提どおり |
| S2/S3用の代替素材が存在しない | `docs/events/sci_tutorial_2026/fallback/README.md`が提供する代替素材はS1・S4・S5のみで、S2（IMU確認）・S3（モータ・コントローラ）専用の動画・画像は用意されていない。これらのセッションでNGが出た場合の代替手段は「標準ファーム（`sf flash vehicle`）に戻す」「見るだけに切替える」のみで、verification_checklist §1にその旨明記されている |
| `sf sils build`が「初回のみでよい」という旧注記 | 旧版のランシートはp.119（S5）に「`sf sils build`は初回のみでよい」という注記の出典を求めていたが、現行スライド（p.123「地図: 今ここ」）にはこの文言は見当たらない。`sf sils build`という文字列自体はp.24・p.130・p.152・p.153に印字されているが、いずれも「初回のみでよい」という趣旨の記述はない。SILSベンチはソースを変更したときだけ再ビルドが必要という一般的なCLIの使用感自体は妥当だが、スライド上の特定ページに印字された記述ではないため、本版では出典なしの運用知識として扱い、旧来の「（p.124）」という個別ページ引用は削除した |

---

<a id="english"></a>

## 1. Overview

### About this document

Extracted from the 160-page slide deck for the SCI/SICE Tutorial 2026 (`docs/events/sci_tutorial_2026/slides/sci_tutorial.pdf`, held 2026-09-10), this document lists — in running order — only the frames the instructor will actually **demonstrate**, have participants **do hands-on**, or that **present an expected result**. The goal is a single sheet the instructor can rehearse from and run the day off, without paging through all 154 slides.

### Sources and method

- Slide body: `docs/events/sci_tutorial_2026/slides/sci_tutorial.tex` and `chapters/sci_intro.tex`, `sci_s1_overview.tex` through `sci_s5_sim_analysis.tex`, `sci_appendix.tex`
- Participant- and review-facing material: `README.md` (timetable, pre-tutorial prep), `handson_guide.md` (post-event reproduction steps), `cheatsheet.md` (command/API reference), `verification_checklist.md` (instructor rehearsal procedure), `fallback/README.md` (index of fallback material)
- Page numbers were derived by extracting each page's text with `pdftotext -f N -l N sci_tutorial.pdf -`, cross-checked against the frame count per chapter file (intro 6 + S1 23 + S2 21 + S3 17 + S4 50 + S5 26 + appendix 10 = 153, plus 6 session dividers (Sessions 1-5 and the appendix) and 1 title page = 160), matching the deck's actual 160 pages, then spot-verified on representative pages

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
| Vehicle & controller | Fully charge the instructor's demo unit and any loaner spares. On-site participants generally bring their own hardware (README "Bring"), so the number of spares needed depends on the day's attendee mix — the materials give no specific spare count | README event info, verification_checklist §5 |
| Flashing | Instructor and loaner vehicles/controllers flashed with `sf flash vehicle` / `sf flash controller` before the morning (the Web Flasher is only mentioned on p.36, not used) | README §4 |
| Instructor PC environment | Run `source setup_env.sh` and confirm `sf doctor` completes with no errors | README §4, sci_s2 p.37 |
| Fallback material folder | Open `docs/events/sci_tutorial_2026/fallback/` (so images/videos can be shown immediately if a live demo fails) | verification_checklist §5, fallback/README.md |
| Projector / Zoom | Confirm the projector connection. Test Zoom screen sharing (for online attendees; the day is also recorded for on-demand viewing) | README event info |
| Pairing | Confirm loaner controllers are already paired (first time only, manual: power on the controller while holding its LCD panel button, then hold the StampFly body button for 3 seconds or more and release at the beep, then pick the vehicle (last 4 hex digits of its MAC) from the controller's on-screen list and confirm; holding 5 seconds or more triggers a system reset. Put a sticker with the vehicle's last-4-hex-digit MAC label on it — check it with the `mac` command) | sci_s2 p.36, sci_s3 p.64 |

## 3. Session 1: Overview and Design Philosophy of the StampFly Ecosystem (10:05-11:00)

Of 23 frames, 4 are demonstration or expected-result frames. The session runs in three parts: Purpose (p.11-14: control is indispensable, the four barriers to using drones as teaching material, comparison with existing platforms, what the Ecosystem aims at), Philosophy (p.15-17: the five design ideas, 4-tier access, the three principles that tie the real vehicle and the simulators together), and What is provided (p.18-25: ecosystem map, vehicle, firmware, the 400 Hz loop, the three kinds of simulation, the sf sils gui screen, the eight fields of the teaching material), followed by Demos 1 and 2 (p.26-29), the review path (p.30) and the checkpoint (p.31). Each frame title carries a 目的:/思想:/提供: prefix marking its part. sf sils gui (p.24) is shown as a screen only; no live demo is needed (usage and the hands-on are in S5).

| Type | Page | Content | Command | Expected result | Rough time | Prep & pitfalls | Fallback |
|------|------|---------|---------|------------------|-----------|------------------|----------|
| Demo (watch only) | p.26 | Demo 1: Simulator Piloting. `sf sim run vpython` launches the VPython simulator, running the same control algorithm as the vehicle, flown with an HID joystick | `sf sim run vpython` | A 3D view opens in the browser and the vehicle responds to stick input. Confirms the control code's behavior without hardware | 3-4 min (rough) | Have the HID joystick (AtomS3 + Atom JoyStick) connected over USB beforehand. This demo does not depend on venue networking, so it is low-risk | None specific (fallback/ has no material for S1's simulator-piloting demo; if it fails, continue with the slide diagram alone) |
| Demo (watch only) | p.27 | Demo 2: Real POS_HOLD Flight. Shows position-hold accuracy while hovering, and the recovery behavior after a disturbance | (No command string printed on this page. Hardware actions: ARM -> switch to POS_HOLD -> apply a light push as a disturbance) | Horizontal drift stays small and the vehicle returns to position after the disturbance (measured +/-6-7cm, the figure already shown on p.20 "vehicle Firmware Structure") | 3-4 min (rough) | Rehearse stable POS_HOLD beforehand per verification_checklist §4 ("Production-Firmware Demo Check"). Flight area netted off; fly only inside the designated area and always operate standing up; DISARM immediately if anything looks wrong. If unstable, switch to ALT_HOLD or STABILIZE (checklist §4's fallback) | `S1_pos_hold_flight.mp4` (POS_HOLD flight video), or simply show the next two expected-result slides (p.28, p.29) |
| Expected-result presentation | p.28 | Demo 2 Expected Result: Position Hold (a SILS simulation result, shown as a reference for how the live demo should go) | None (static image) | `pos_roll.scn` (vehicle): takeoff -> roll disturbance -> POS_HOLD engages -> holds. Max horizontal drift after engage: 0.39 m | 1-2 min (rough) | Image is `fallback/S1_pos_hold_xy.png` (already embedded in the slide) | Not applicable (this frame is itself the fallback material) |
| Expected-result presentation | p.29 | Demo 2 Expected Result: Altitude & Attitude | None (static image) | Altitude/attitude time series for the same flight. Tilt right after the disturbance returns to level under position control | 1-2 min (rough) | Image is `fallback/S1_altitude_attitude.png` | Not applicable |

## 4. Session 2: Environment Setup and Sensor Data Acquisition (11:00-12:00)

Of 21 frames, 5 are demonstration/hands-on frames (this session went 17 -> 19 -> 18 -> 20 -> 21: Exercise 1 and Exercise 2 first got their own frames with explicit steps, then "Installing the Dev Environment (1/2): PC Side" moved to the opening, dropping the count by one, and two WiFi-connection frames were later added, then Exercise 1 (2/2) "One-time vehicle setup").

Guidance for attendees who haven't installed yet now lives in the opening frame "Installing the Dev Environment (moved earlier): PC Side" (the three CLI stages: prerequisites, clone + install.bat, setup_env.bat + sf doctor; the GUI installer is not used because its stability is unconfirmed; written Windows-first with `install.bat` / `setup_env.bat`, macOS/Linux in parentheses, since most attendees are expected on Windows), not at the top of this session. This session now opens assuming the install is underway, starting from the OS-differences reference and pairing/Exercise-1 verification.

The new p.35 "OS-Specific Differences" is a reference frame summarizing macOS/Linux vs. Windows command differences (`./install.sh` vs. `install.bat`, `source setup_env.sh` vs. `setup_env.bat`, serial port names). It is not demoed on the day; use it if a Windows attendee has trouble.

**New (p.46–47):** Two frames, "Connecting over WiFi: What Wired Can and Can't Do" and "Connection Steps: Joining the Vehicle's WiFi", were inserted right after Exercise 2 (2/2, Tilt Test) and right before "Data Path 2: sf telemetry". Both "Data Path 2" and "3" (`sf telemetry` / `sf log wifi`) assume a WiFi connection, so the new frames explain how to set one up first. Two key points from the new material:

- Out of the box, `wifi.mode` defaults to `0` (STA, no credentials configured), which leaves telemetry inert. The switch to SoftAP (the vehicle's own WiFi) is done once in Exercise 1 (2/2) "One-time vehicle setup": `param reset` -> `param set wifi.mode 1` -> `param set wifi.channel N` -> `param save` -> `reboot`. In this slot only confirm it has been done. After that, every subsequent boot comes up in SoftAP mode automatically
- **Run the day in SoftAP mode**: each StampFly serves its own AP (default SSID `StampFly-XXYY`, generated from the MAC tail; default password `stampfly`; default IP `192.168.10.1`). Do not rely on venue WiFi (`verification_checklist.md` likewise plans around "venue WiFi being down"). Call out explicitly that each attendee's vehicle has a different SSID

| Type | Page | Content | Command | Expected result | Rough time | Prep & pitfalls | Fallback |
|------|------|---------|---------|------------------|-----------|------------------|----------|
| Hands-on (everyone) | p.37 | Exercise 1: Environment Check and Build. Everyone runs `sf doctor` together, diagnosing ESP-IDF, Python, the USB serial driver, and the sf CLI's own version, then builds and flashes the vehicle firmware and opens the monitor. Deliberately limited to the four basic sf CLI operations (diagnose, build, flash, monitor); `sf lesson` is not introduced here (it comes from p.40 on) | `setup_env.bat` (macOS/Linux: `source setup_env.sh`)<br>`sf doctor`<br>`sf build vehicle`<br>`sf flash vehicle -m` | The standard boot chime (C5-E5-G5) plays right after flashing and the onboard LED goes white (initializing) then steady green (ready). The boot log streams in the monitor (exit with Ctrl+]). `sf doctor` not passing cleanly is not a problem (p.37 alertblock) | 4-6 min (rough; longer if individual attendees need troubleshooting) | This should already be done before the day per README §4's pre-tutorial prep. Attendees who fail continue "watch only" and catch up later via the review path (end of this session) | No dedicated fallback/ material. Attendees who fail individually switch to "watch only"; point them to `docs/guides/troubleshooting.md` |
| Hands-on (everyone) | p.38 | Exercise 1 (2/2): One-time vehicle setup. In the monitor CLI right after flashing: reset to defaults, switch to SoftAP, save the per-attendee channel, reboot, then pair the controller | `param reset`<br>`param set wifi.mode 1`<br>`param set wifi.channel N` (the instructor assigns N = 1/6/11 per attendee)<br>`param save`<br>`reboot`<br>Pairing: power on the controller while holding its LCD button, then hold the vehicle button for 3 s or more and release at the beep, then pick your own vehicle (last 4 hex digits of its MAC) from the controller's on-screen list and confirm | After the reboot `param get wifi.channel` returns N; once confirmed and the vehicle replies, pairing is complete (the vehicle's LED turns solid green and it responds to the controller) | 5-8 min (rough; includes assigning everyone's channel) | Prepare and post a channel table (who is on 1/6/11). Holding the button 5 s or more triggers a system reset, so release at the beep. Solution code no longer changes the channel (`ws::set_channel` calls removed). **Everyone may pair at the same time** (the vehicle only treats packets addressed to itself as candidates, so this cannot cross-pair), but the on-screen list also shows other attendees' vehicles — remind everyone to pick only the row matching their own vehicle's label (last 4 hex digits of its MAC; check it beforehand with the `mac` command and stick a label on the vehicle) | Redo from `sf flash vehicle -m` for any vehicle that fails |
| Demo (follow along) | p.43 | Exercise 2 (1/2): Viewing IMU Values. Write the code from p.44 into `user_code.cpp`, then build, flash, and connect Teleplot via `sf lesson monitor` | `sf lesson switch sci2026:2`<br>`sf lesson edit` (open `user_code.cpp` and write the code from p.44)<br>`sf lesson build`<br>`sf lesson flash`<br>`sf lesson monitor` (requires the VSCode extension `alexnesnes.teleplot`; `sf lesson flash` opens this same connection right after flashing) | The Teleplot view for `gyro_x`/`gyro_y` opens (still flat while the vehicle is stationary) | 4-6 min (rough; includes build/flash time) | Install the VSCode extension `alexnesnes.teleplot` beforehand. Since each attendee's build takes time, showing the flow once on the instructor's machine before everyone tries it saves time | No dedicated fallback/ material. If it does not work, share the instructor's own Teleplot view and switch to "watch only" |
| Demo (follow along) | p.45 | Exercise 2 (2/2): Tilt Test. Hold the StampFly and tilt it front/back/left/right; `gyro_x`/`gyro_y` swing with the tilt speed, and `accel_z` returns to about -9.81 at rest | (No command string printed on this page. After the build/flash from the previous page, p.43, watch the Teleplot view while tilting the vehicle) | At rest, gyro is approximately 0 and `accel_z` is approximately -9.81 m/s^2. The `gyro_x`/`gyro_y` waveforms swing with tilt speed | 1-2 min (rough) | Assumes Exercise 2 (1/2) on p.43 is already complete | No dedicated fallback/ material. If it does not work, share the instructor's own Teleplot view and switch to "watch only" |
| Demo (new; sets up the `sf telemetry` assumption) | p.46–47 | Connecting over WiFi / Connection Steps (2 frames). Since the upcoming `sf telemetry`/`sf log wifi` frames assume a WiFi link, demonstrate switching the vehicle to SoftAP mode and joining it from the PC | Enter the CLI over the still-live USB link with `sf monitor`<br>`param set wifi.mode 1` (`0`=STA default / `1`=SoftAP)<br>`param save`<br>`reboot`<br>On the PC, join `StampFly-XXYY` in WiFi settings (default password `stampfly`) | The vehicle's AP appears in the PC's WiFi list; once joined, `sf telemetry` (no `--ip` needed, default `192.168.10.1`) receives the 50Hz stream | 3-5 min (rough; one-time only -- later boots come up in SoftAP automatically, so this is skipped after the first time) | Pre-setting `wifi.mode` on every attendee vehicle ahead of time would save time on the day (worth considering). Call out that each attendee's vehicle has a different SSID. Do not join venue WiFi | Attendees who cannot connect switch to watching the instructor's screen; point them to the WiFi row in `docs/guides/troubleshooting.md` |

Reference code (p.44 "Data Path 1: Teleplot", written into `user_code.cpp` for Exercise 2):

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

Of 17 frames, 2 are demonstration/hands-on frames (a new TDMA explainer frame grew this session from 16 to 17). **Safety (p.57):** do this on a table, keeping hands clear of the spinning motors throughout. Duty capped at 0.15 (not enforced by the firmware, so verify visually). ARM only via a single click of the body button or the controller, never auto-armed in code.

The new p.63 "Flying 30 Vehicles at Once with TDMA" is a reference frame explaining TDMA (Time Division Multiple Access) and how three non-overlapping channels (1/6/11) support up to 30 vehicles in theory. It is not demoed on the day; it can be used if asked about running many vehicles in one classroom.

| Type | Page | Content | Command | Expected result | Rough time | Prep & pitfalls | Fallback |
|------|------|---------|---------|------------------|-----------|------------------|----------|
| Hands-on | p.60 | Exercise 3: Hardcoded Duty. Rewrite `loop_400Hz` in `user_code.cpp` to the code shown on the slide and confirm motor spin on the table | `sf lesson switch sci2026:3`<br>`sf lesson edit` (replace with the code on the right and save)<br>`sf lesson build`<br>`sf lesson flash`<br>Click the body button once to ARM | Arming from the body button spins only FR (M1) at low speed (duty 0.10; the others are 0.00). DISARM stops it | 5-7 min (rough) | **Safety (p.57):** do this on a table, keep hands clear of the spinning motors, duty <=0.15, ARM only via body button or controller. **Note (p.60):** the distributed solution (`--solution`) auto-arms on boot, so use it only on the table with hands clear of the motors | No dedicated S3 fallback/ material. If the bench check (verification_checklist §1) fails, revert to production firmware (`sf flash vehicle`) and switch this demo to watch-only |
| Demo (follow along) | p.67 | Exercise 4: Stick to Motors. Write `ws::motor_mixer` and see the four motors' speed differ with stick input (the controller's commands arrive over ESP-NOW as a `ControlPacket` at roughly 50 Hz); shows that it cannot hold commanded speed under a light disturbance | `sf lesson switch sci2026:4`<br>Pair with the controller<br>`sf lesson edit` (replace with the code on the right and save; `ws::motor_mixer(T,R,P,Y)`)<br>`sf lesson build`<br>`sf lesson flash` | Motor speeds diverge with stick input. A light gust defeats the commanded speed (the limit of open-loop control) | 5-7 min (rough) | If pairing has not been done, do it first per p.64 (hold the controller's LCD panel button while powering on -> hold the body button 3s and release at the beep -> pick your own vehicle from the controller's on-screen list and confirm). Keep the vehicle on the table, hands clear of the motors | No dedicated S3 fallback/ material. As in checklist §1, revert to `sf flash vehicle` if this fails |

Reference code (p.60 "Exercise 3: Hardcoded Duty", `loop_400Hz` in `user_code.cpp`):

```cpp
void loop_400Hz(float dt) {
    ws::motor_set_duty(1, 0.10f);   // FR
    ws::motor_set_duty(2, 0.00f);   // RR
    ws::motor_set_duty(3, 0.00f);   // RL
    ws::motor_set_duty(4, 0.00f);   // FL
}
```

(This frame does not print a `setup()` body. The p.60 body text states ARM is a single click of the body button and that code must not auto-arm.)

## 6. Session 4: Feedback Control Basics -- PID Attitude Stabilization (14:00-15:00)

Of 50 frames, 7 are demonstration/hands-on/expected-result frames (the 29-to-46 rebuild kept Exercises 5-9, the final demo, its expected result, and the reproduction-steps frame at the same content, commands, and role -- only their frame position shifted. The 17 added frames are all lecture-only theory frames, many tagged "read later" in their title, meant to be skimmed live and read at home afterward. Page numbers below are from the rebuilt 154-page PDF).

**For participants, Exercises 5-9 themselves (writing code and trying it on real hardware) are watch-only on the day** (the demo-viewing note reads: "reproduce at home: today is watch-only, run Exercises 5-9 end to end afterward"). However, each exercise frame itself spells out numbered steps for a **live instructor demonstration** (first flight, comparing damping ratios, system identification, PID comparison, attitude-estimation agreement), so this table treats Exercises 5-9 as demo frames too.

**Note the reordering of Exercise 9:** in the rebuilt deck, Exercise 9 (attitude estimation) now comes *after* the "Gyro Drift Problem," "Deriving Tilt Angle from the Accelerometer," and "Complementary Filter" theory frames (previously Exercise 9 preceded those theory frames). The demo content, steps, and commands are unchanged, but the day-of narration now explains the complementary filter theory before running Exercise 9 -- keep this in mind while rehearsing.

**Rehearsal caveat:** `verification_checklist.md` §2 only requires rehearsing Exercises 5, 8, and 9. Exercise 6 (three flights at different damping ratios) and Exercise 7 (a flight for system identification) have no dedicated rehearsal item; the instructor decides on the day, based on time and hardware condition, whether to fly them live or describe them verbally only (noted in the relevant rows below). Running all five live demos back-to-back plus the 5-8 min final comparison demo can approach 30 minutes, which is tight inside a 60-minute session. With so much more theory now in the deck, it matters even more on the day to keep live explanation of "read later" frames to a minimum and move on.

| Type | Page | Content | Command | Expected result | Rough time | Prep & pitfalls | Fallback |
|------|------|---------|---------|------------------|-----------|------------------|----------|
| Code: follow-along, flight: watch only | p.77 | Exercise 5: First Flight with Rate P-Control. Compute error = target rate - measured rate, output = Kp * error for Roll/Pitch/Yaw and feed it to `ws::motor_mixer`; the instructor flies the finished code with Kp = 0.5 | Everyone: `sf lesson switch sci2026:5`<br>`sf lesson edit` (fill in the TODOs and save: `ws::gyro_x/y/z()`, `ws::rc_roll/pitch/yaw()`)<br>`sf lesson build`<br>Instructor only: `sf lesson flash` -> flight | Stick input is followed, but with a slight oscillation (the limit of P-only control). The rate target is logged to the Data Stream via `ws::set_rate_target()` for use in Exercise 7 | 4-6 min (rough) | Rehearse per verification_checklist §2 (confirm stable hover at Kp=0.5). Start at low throttle; DISARM immediately if anything looks wrong | `fallback/S4_ex5_p_flight.mp4` |
| Demo (watch only; time/rehearsal permitting) | p.90 | Exercise 6: System Modeling. Implement the design formula $K_p=1/(4\zeta^2 K \tau_m)$ for Roll/Pitch/Yaw and fly three times with $\zeta=0.7 \to 0.5 \to 1.0$ to show the difference | `sf lesson switch sci2026:6`<br>`sf lesson edit` (write code computing Kp_roll/pitch/yaw from the design formula, save)<br>`sf lesson build`<br>`sf lesson flash` (rebuild/reflash for each $\zeta$) | $\zeta=0.5$ looks somewhat oscillatory, $\zeta=0.7$ smooth, $\zeta=1.0$ visibly slower -- a qualitative difference | 3-5 min (rough; if time is tight, limit to 1-2 values of $\zeta$ or describe verbally only) | verification_checklist §2 has no dedicated rehearsal item for this exercise (it assumes basic stability was already confirmed via Exercise 5's rehearsal). Three back-to-back flights eat into the schedule | No dedicated video in fallback/. Fall back to a verbal explanation |
| Demo (watch only; time/rehearsal permitting) | p.94 | Exercise 7: System Identification (sf sysid fit). Add `ws::set_rate_target(roll,pitch,yaw)` to Exercise 5's P-control code, capture a log while flying, then identify the plant ($K$, $\tau_m$) | `sf lesson switch sci2026:7`<br>`sf lesson edit` (set Kp and call `ws::set_rate_target`, save)<br>`sf lesson build`<br>`sf lesson flash`<br>while flying: `sf log wifi -o flight.csv`<br>`sf sysid fit flight.csv --plot` | The identified $K$ matches Exercise 6's theoretical values ($K_{roll}=102$, $K_{pitch}=70$) to within a few percent ($\tau_m$ is far more excitation-sensitive and can land several times the 0.02s theoretical value -- that alone isn't a failure) | 4-6 min (rough) | verification_checklist §3 confirms the identification pipeline itself, but §2 has no dedicated flight-rehearsal item for this exercise. If the fit does not converge, switch to the reference logs under `analysis/reports/rate_sysid_reference/` per checklist §3's fallback, and downgrade this from "follow along" to "watch only" | Switch to a demo using the reference logs (checklist §3) |
| Code: follow-along, flight: watch only | p.100 | Exercise 8 (1/2): PID Control. Add an I term (`integral += error*dt`, clamped to +/-0.5) and a D term (`Kd*(error-prev_error)/dt`) to Exercise 5's P-control, fly it, and compare against Exercise 5 (P only) Attendees write and build the code; the instructor flies. Keep their code in place until Exercise 8 (2/2) (p.132) flies it in SILS: do not have them switch to another exercise (switching overwrites `user_code.cpp`; an edited file is auto-saved to `firmware/workshop/my_code/` first) | Everyone: `sf lesson switch sci2026:8`<br>`sf lesson edit` (add I term, D term, and anti-windup per axis, save)<br>`sf lesson build`<br>Instructor only: `sf lesson flash` -> flight | Steady-state error and overshoot visibly change compared to Exercise 5 | 4-6 min (rough) | Rehearse per verification_checklist §2 (confirm taking off with PID reduces steady-state error vs. P-only). Reset integral/prev_error to 0 on disarm | `fallback/S4_ex8_pid_flight.mp4` |
| Demo (watch only; hand-tilted, no flight) | p.108 | Exercise 9: Attitude Estimation (Complementary Filter). Compute angle from acceleration with `atan2f`, implement the complementary filter $\hat\theta_k=\alpha(\hat\theta_{k-1}+\omega\Delta t)+(1-\alpha)\theta_{accel}$ ($\alpha=0.98$), and compare it against `ws::estimated_roll()` (the vehicle's default ESKF) on Teleplot. **In the rebuilt deck this now follows the gyro-drift, accelerometer-tilt-derivation, and complementary-filter theory frames** | `sf lesson switch sci2026:9`<br>`sf lesson edit` (implement the complementary filter, save)<br>`sf lesson build`<br>`sf lesson flash`<br>`sf monitor` + Teleplot to compare `cf_roll` and `eskf_roll` | Tilting the vehicle by hand shows `cf_roll` and `eskf_roll` tracking each other closely | 3-5 min (rough) | Rehearse per verification_checklist §2 (confirm `cf_roll` and `eskf_roll` roughly agree on Teleplot). This is a hand-held check like Exercise 2 in S2, not a flight | No dedicated video. If it does not work, share the instructor's own Teleplot view |
| Demo (the `sf sysid fit` part is follow-along) | p.117 | Demo: Final Code. Hover with the PID-ified exercise code (Exercise 8), flick the roll stick briefly, and look qualitatively at how fast the measured rate follows the rate target (rate_ref) and how much it overshoots (a clean step cannot be applied by hand, so the quantitative comparison against the design value is done in SILS on the next frame). Share the captured `flight.csv` so attendees can run `sf sysid fit` on their own machines together with the instructor | `sf log wifi -d 30 -o flight.csv`<br>`sf log viz flight.csv` (this frame only describes the steps in prose -- "capture telemetry with `sf log wifi`", "overlay the rate target and the measured rate with `sf log viz`" -- without printing the exact arguments; the above borrows the syntax printed elsewhere in S2)<br>`sf sysid fit flight.csv --plot` (the follow-along part) | The live stick response visibly shows less overshoot/ringing for PID (Exercise 8) than for P (Exercise 5) (the quantitative step-response comparison is the SILS reference on the next frame, see below) | 5-8 min (rough) | Only the instructor flies live. Rehearse per verification_checklist §2 for Exercise 8 (Exercise 5 is rehearsed separately). Start at low throttle; DISARM immediately if anything looks wrong | `fallback/S4_ex5_p_flight.mp4`, `fallback/S4_ex8_pid_flight.mp4` (per checklist §2, switch only the unstable lesson to video if needed) |
| Expected-result presentation | p.118 | Demo Expected Result: P vs. PID step response. **The caption explicitly states these numbers come from running a roll-step test in SILS (MuJoCo), not from measuring the live flight itself; they are a design-target reference** | None (static image) | Against a 15.1 deg/s target, Exercise 5 (P) peaks at 17.9 deg/s with a -2.4 deg/s undershoot; Exercise 8 (PID) peaks at 13.2 deg/s with no ringing (figures are from the SILS simulation) | 2-3 min (rough) | Image is `fallback/S4_roll_step_p_vs_pid.png`. See the following "Reproducing the Expected Result" frame for the detailed SILS reproduction steps | Not applicable (this frame is itself the fallback material) |

**Frames excluded from this table (mention verbally only, or demo at the instructor's discretion):**

- "Recap: Exercise 5 Exercise Code": re-displays Exercise 5's code for explanation; nothing is re-run live here
- "Identification Results vs. Theory" (the frame combining `sf sysid fit`'s example output with a theory-vs-identified comparison table), "What sf sysid rate-fit / rate-tune Are" and "rate-excite -> rate-fit -> rate-tune Procedure", "Onboard Autotune: sf_autotune" (all vehicle-firmware only, read-later): all present example command output only; the slides do not state that the instructor runs these live on the day
- "Reproducing the Expected Result" (read later): the CLI steps to reproduce the expected-result frame's numbers yourself via SILS (`sf lesson switch sci2026:5 --solution` -> `sf lesson sils --scenario step`). Not a live in-session demo; it is a prep/at-home reference
- In addition, the many newly added theory frames (feedback control basics, the limits of P control, a deep dive into the plant's physical model, second-order design and loop shaping, the idea behind system identification, a deep dive into PID theory (incomplete derivative / anti-windup / discretization / ARM-transition reset), deriving tilt angle from the accelerometer, the principle of swept-frequency identification and ETFE, spec-based gain design) are all lecture-only (no live demo), and most are tagged "read later." The exception is "The Idea Behind System Identification" (p.92): it is the lead-in to Exercise 7, so it is taught live and its read-later tag was removed. The plan is to touch on them only briefly out loud on the day and let attendees read the details afterward

## 7. Session 5: Simulator and Analysis Tools, Advanced Topics (15:30-16:30)

Of 28 frames, 3 are demonstration/hands-on/expected-result frames ("Where Research Begins" is now 4 frames, p.137-140: (1/4) "Where to write your own code" gets one minute live for the `sf app new/edit/build/flash` flow; (2/4)-(4/4) are "read later" reference material and are not demoed on the day).

| Type | Page | Content | Command | Expected result | Rough time | Prep & pitfalls | Fallback |
|------|------|---------|---------|------------------|-----------|------------------|----------|
| Hands-on (everyone) | p.132 | Exercise 8 (2/2): Fly Your PID in SILS. Verify Exercise 8's (PID) `setup()`/`loop_400Hz()` in SILS before flying it for real | `sf lesson sils` (with the code written in S4 still in place; anyone who did not finish runs `sf lesson switch sci2026:8 --solution` first) | Pass criteria (`.expect`): lift-off (true altitude > 0.1 m) and tilt under 15 degrees (no tumble). No altitude loop, so landing is DISARM descent only. Following the sequence as written yields `alt_max` ~= 0.64 m, `tilt_max` = 0.0, i.e. PASS | 4-6 min (rough; longer including the first compile) | `sf lesson sils` handles the rebuild and scenario run internally, so typing this one line right after switching is enough. Use `sf lesson sils` for exercise-code SILS runs (`sf sils gui` targets the vehicle firmware; excluded from `sf sils regression` by default, include it with `--include-workshop`) | No dedicated video for this exercise in fallback/. If the CLI run fails, explain verbally; SILS's general reliability can be backed by `fallback/S5_regression_summary.txt` (28 PASS + 5 known, tracked issues + 1 exercise-code-target skip, 34 total) |
| Demo (the `sf sim run vpython` part is follow-along) | p.133 | Demo: The Simulator. 3D piloting in VPython, plus running a scenario and checking pass/fail in `sf sils gui`, then varying a parameter to see the effect | `sf sim run vpython`<br>`sf sils gui` (`sf sils build` is printed on p.130; the demo page itself only describes "run one scenario and check the result" in prose) | A 3D piloting view opens in the browser. Running a scenario in `sf sils gui` shows a PASS/FAIL verdict, and changing a parameter changes the behavior | 5-8 min (rough) | Connect the HID joystick (AtomS3 + Atom JoyStick) if available; otherwise it falls back to keyboard control (the keyboard fallback itself is not printed on the slide -- it follows from VPython's general controls) | `fallback/S5_stab_flight.mp4` (STABILIZE flight video), `fallback/S5_attitude_rate.png`, `fallback/S5_gate_result.txt` (12-item pass verdict) |
| Expected-result presentation | p.134 | Demo Expected Result: SILS Scenario Run. Attitude/angular-rate time series | None (static image) | `stab_flight.scn` (vehicle) attitude and angular rate. All 12 pass criteria satisfied (`att_rmse=2.82<3.0`, `tilt_max=13.77<18.0`, `duty_max=1.00<1.001` -- figures from `fallback/README.md`'s numeric summary) | 2-3 min (rough) | Image is `fallback/S5_attitude_rate.png` | Not applicable (this frame is itself the fallback material) |

## 8. All Commands, in Execution Order

Listed per session in the order typed on the day (duplicates within a session removed). The slides previously showed both bare `sf sim run` and `sf sim run vpython`; the reworked deck now prints `sf sim run vpython` consistently everywhere -- see §10.

**Session 1**

```bash
sf sim run vpython
```

**Session 2**

```bash
setup_env.bat            # macOS/Linux: source setup_env.sh
sf doctor
sf build vehicle
sf flash vehicle -m
sf lesson switch sci2026:2
sf lesson edit
sf lesson build
sf lesson flash
sf lesson monitor
```

**Session 3**

```bash
sf lesson switch sci2026:3
sf lesson build
sf lesson flash
sf lesson switch sci2026:4
sf lesson build
sf lesson flash
```

**Session 4**

The full "ideal walkthrough" order, including the watch-only Exercises 5-9. Only the instructor types these; Exercises 6 and 7 may be skipped depending on time (see §6).

```bash
sf lesson switch sci2026:5
sf lesson build
sf lesson flash
sf lesson switch sci2026:6
sf lesson build
sf lesson flash
sf lesson switch sci2026:7
sf lesson build
sf lesson flash
sf log wifi -o flight.csv
sf sysid fit flight.csv --plot
sf lesson switch sci2026:8
sf lesson build
sf lesson flash
sf lesson switch sci2026:9
sf lesson build
sf lesson flash
sf log wifi -d 30 -o flight.csv
sf log viz flight.csv
sf sysid fit flight.csv --plot
```

**Session 5**

```bash
sf lesson switch sci2026:8 --solution
sf lesson sils
sf sim run vpython
sf sils build
sf sils gui
```

## 9. Exercise Reference Table

"Exercise N" refers to the Nth exercise in the tutorial-specific course `sci2026` (defined in `firmware/workshop/lessons/lesson_manifest.yaml`). Switch with `sf lesson switch sci2026:N`.

| Exercise | Session | Internal name in `lesson_manifest.yaml` | What participants write (function names, etc.) |
|----------|---------|------------------------------|---------------------------------------------------|
| Exercise 1 | Session 2 | environment_setup | None (environment and build check only; centers on running `sf doctor` and building/flashing, no code written) |
| Exercise 2 | Session 2 | imu_sensor | In `loop_400Hz`, print IMU values Teleplot-style with `ws::print(">gyro_x:%.3f", ws::gyro_x())` etc. |
| Exercise 3 | Session 3 | motor_control | In `loop_400Hz`, drive each motor directly with `ws::motor_set_duty(id, duty)` |
| Exercise 4 | Session 3 | controller_input | Read `ws::rc_throttle/roll/pitch/yaw()` and distribute to the four motors with `ws::motor_mixer(T,R,P,Y)` |
| Exercise 5 | Session 4 | rate_p_control | Compute the rate error (`re`/`pe`/`ye`) against target and feed `Kp_rp*re` etc. into `ws::motor_mixer` as proportional control |
| Exercise 6 | Session 4 | system_modeling | None (lecture only, no hardware; compute the $K_p$ for a $\zeta=0.7$ design from the measured $K$, $\tau_m$) |
| Exercise 7 | Session 4 | system_identification | Call `ws::set_rate_target(roll,pitch,yaw)` right after computing the rate target, logging it into the Data Stream's `rate_ref_*` |
| Exercise 8 | Session 4 | pid_control | Replace the ideal derivative with an incomplete-derivative filter (computing `alpha`, `a`, `b`, `d_filt`) |
| Exercise 9 | Session 4 | attitude_estimation | Hand-write a complementary filter $\hat\theta_k=\alpha(\hat\theta_{k-1}+\omega\Delta t)+(1-\alpha)\theta_{accel}$ (e.g. a `cf_roll` variable) and compare it against `ws::estimated_roll()` |

## 10. Inconsistencies Found Between the Slides and the Markdown Companions (For Reference)

Recorded as found, not fixed.

| Where | Detail |
|-------|--------|
| `sf sim run` wording differs (**resolved**) | pp.24/28/17 (S1 deep-dive) used to print bare `sf sim run` (no argument), while S5 (now p.128 "VPython and Genesis" / p.133 "Demo: The Simulator") and the appendix (now p.152 "sf CLI Cheat Sheet (2/2)"), README, handson_guide, and cheatsheet printed `sf sim run vpython` (with the argument). After the post-review rebuild, every page in the current deck prints `sf sim run vpython` consistently, so this wording gap is resolved |
| `verification_checklist.md`'s lesson numbers vs. "Exercise N" (**resolved**) | `verification_checklist.md` §1 used to bench-check using the **raw internal lesson numbers from `lesson_manifest.yaml`** (`sf lesson switch 0/1/2/4`), which did not match the `sci2026` course's Exercise 1-4 sequence in that same file (see §9's table for the exact correspondence). The checklist has since been normalized to the same `sci2026:N` notation used in this runsheet (`sf lesson switch sci2026:N`), resolving the mismatch |
| Scope of "watch only" in S4 | p.117's demo-viewing note says "today is watch-only; run Exercises 5-9 end to end afterward," while `verification_checklist.md` §2 is written assuming the instructor personally confirms Exercises 5, 8, and 9 on the day (Exercises 6 and 7 have no dedicated rehearsal item). The two are not actually contradictory (participants watch; the instructor demonstrates), but running all five live demos inside a 60-minute session is tight. **Unresolved:** whether Exercise 6 (three flights at different damping ratios) and Exercise 7 (a flight for system identification) are actually flown live on the day, or only described verbally, is not settled by either the slides or the checklist -- it is left to the instructor's on-the-day judgment |
| Source of the S4 "expected result" numbers (**resolved**) | p.118's caption explicitly states the figures come from running a roll-step test in SILS (MuJoCo), not from a measured real flight. p.117 "Demo: Final Code" now also states that the live flight only shows the response to a manual stick flick qualitatively and that the quantitative comparison against the design value is done in SILS on p.118. The live waveform not matching the p.118 reference figures (peak 17.9/13.2 deg/s etc.) is therefore by design |
| No fallback material for S2/S3 | `docs/events/sci_tutorial_2026/fallback/README.md`'s material covers only S1, S4, and S5 -- there is no dedicated video/image for S2 (IMU check) or S3 (motor/controller). If either fails on the day, the only documented fallbacks are reverting to production firmware (`sf flash vehicle`) or switching to watch-only, per verification_checklist §1 |
| Old note that `sf sils build` "only needs to run once" | The previous version of this runsheet cited p.123 (S5) as the source for "`sf sils build` only needs to run once." That wording no longer appears anywhere in the current deck (p.124 is now "Map: You Are Here"). The string `sf sils build` itself is printed on pp.24, 130, 152, and 153, but none of them say anything like "only needs to run once." The underlying operational claim (the SILS bench only needs rebuilding when its source changes) is still reasonable CLI know-how, but it is no longer text printed on a specific slide, so this version treats it as unsourced operational knowledge and drops the old page-119 citation |
