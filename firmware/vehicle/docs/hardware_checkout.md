# 実機確認手順 / Hardware Checkout Procedure

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

実機の vehicle ファームウェアが、飛行安全に関わる機能（400 Hz ループの周期、有限でない数の監視、離着陸の判定、電池・通信途絶の処置、回頭、気圧高度の観測）を設計どおりに実行するかを、人が手で確かめる手順書である。推力の無い机上の確認から低空の飛行へ、危険の小さい順に進み、最後に 1 回の総合飛行で締める。

**責任者**とは、このリポジトリの責任者（制御・ハードウェアの専門家）で、この手順の合否と、不合格のまま進むかの最終判断をする人である。章番号と段階の対応: 段階 N は **N + 4 章**（段階 0 = 4 章 … 段階 4 = 8 章）。

## 1. 進行の一覧

**1 ページで全体を見る表。** 時間は見積もりで、実測していない。電池 2 本以上（段階 2 だけで電池交換を数回含む）を前提とする。**段階 1 と段階 2 は、それぞれほぼ 1 日がかりになる見込みなので、別の日に分けてよい**（分けるときは段階 1 の終わりで結果表を埋める）。

| 段階 | 目的 | 目安の時間 | プロペラ |
|------|------|-----------|---------|
| 0 準備 | プロペラを外し、試験するファームの確定、ビルド・書き込み、設定値・送信機・電池・モータの位置の確認 | 1〜1.5 時間 | **外す** |
| 1 机上 | 推力の無い状態で、ループ周期・有限性・床の ToF・ログ・WiFi の CLI・離陸判定・通信途絶・電池・ToF 再初期化の有無を確かめる（1a〜1i） | 4〜6 時間 | **外す** |
| 2 低空の自由飛行 | 危険の小さい順に 10 種の飛行（手順 2-1〜2-10。2-4 は表面ごとに 3 回以上）。1 回の飛行は 1 つの目的 | 4〜6 時間（電池交換を含む） | 付ける |
| 3 総合飛行 | 離陸・1 回転・着陸を 1 回で通す（C16）。段階 2 が全て合格してから | 30〜45 分 | 付ける |
| 4 取りまとめ | 記録の回収、パラメータの復元、結果表 | 30 分 | — |

### 人員と端末

実機確認は、**開発者 1 人と Claude Code（PC 側の操作）**で行う（2026-11-22 の決定。2 人目はいない）。**モータを止める手段は人だけが持つ**: Claude Code の応答は数秒遅れうるので、緊急停止は Claude Code に任せない。

| 区分 | 人（開発者） | Claude Code |
|------|-------------|-------------|
| 機体・送信機・電池・プロペラ | 触れる操作は全て人（送信機の ARM・DISARM、電池の接続、プロペラの取付） | 触れない |
| モータを止める | 送信機のスロットル側スティックの押し込み、機体のボタン、電池を外す。異常のときの停止は全てこれ | 任せない。`emergency`（CLI の命令、または `sf emergency`）は頼まれたときに送るが、数秒遅れうるので止める手段に数えない |
| USB コンソールへの入力 | — | 送ってよい（モータが回る `motor test` を含む）。送る前に、何が起きるかを音声（macOS の `say`）で伝える |
| CLI の `arm`・`disarm`・`mode`（USB・WiFi） | — | 送ってよい（開発者の承認、2026-11-22）。送る前に音声で伝える |
| WiFi の取得（`sf log wifi`）・WiFi の CLI（`nc 192.168.10.1 23`） | — | 実行してよい |
| 飛行の命令（`sf takeoff`・`sf cw` など） | 送信機を持ち、DISARM の用意をする | 取得の開始と、命令の送信まで。飛行の前に目的・操作の順・中止の段を全部伝え、人の合図を受けて送る |
| 保存された記録の読み取りと合格線との照合 | — | 行う |

手順ごとの分担と、1 人（開発者 1 人 + Claude Code）での実施の可否:

| 手順 | 人が行うこと | Claude Code が行うこと | 1 人での実施 |
|------|-------------|----------------------|-------------|
| 0、1a〜1f、1h-1、1i | 機体・送信機・電池の操作、目で見る確認 | USB・WiFi の入力、取得、照合（0-13 の CLI の命令を含む） | 可 |
| 1g | 片手で機体を持ち上げ、片手で送信機を持つ（スティックの押し込みですぐ止められる）。**プロペラを外して行うことを推奨**する。付けたまま行うかは責任者の判断 | 取得の開始、`status` の読み取り | 可 |
| 1h-2、1h-3 | 電源装置の電圧の操作、機体を持つ操作 | 取得の開始 | **任意のまま**。電源装置が要る。機体を持つ操作と電圧の操作が重なる場面があるので、1 人で行うかは責任者が判断する。行わなければ「未実施」。後日、緊急電圧のしきい値のパラメータ化が入る予定（電源装置なしで行えるようにするため） |
| 2-1、2-2、2-4 | 操縦者として送信機を持つ | 飛行の前に取得を始める | 可 |
| 2-3、2-6、2-9、2-10、3 | 操縦者として送信機を持ち、DISARM の用意をする。**止めるのは人** | 取得の開始、飛行の命令の送信（`sf takeoff` など）。緊急停止は任せない | 可 |
| 2-5 | 操縦者 + 高さを `sf tof` で読んで伝える人 | — | **不可**（高さの読み取りは数秒の遅れが許されないので Claude Code に任せない。2 人目がいるときだけ行う） |
| 2-7、2-8 | 送信機の電源を切る操縦者 + `sf emergency` を Enter だけで送れる状態で待つ人 | — | **1 人では行わない**（送信機の電源を切ると、操縦者の DISARM が届かず、人の止める手段が無くなる。Claude Code の `emergency` は数秒遅れうるので代わりにならない。通信途絶の模擬が入るまで、この 2 手順は 2 人目がいるときだけ行う。後日追加の予定） |

PC は、有線 LAN や iPhone の USB テザリングなどでインターネットにつないだまま、WiFi を機体につなぐ同時接続で進められる（2026-11-22 に実機で確認。3 章の「記録の取り方」に macOS での注意がある）。

PC 側の端末（飛行の段階。全て同じ PC、機体の WiFi につないだ状態。Claude Code が使う。「PC 担当者」は、2 人目がいるときにこの役を引き受ける人）:

| 端末 | 持たせるもの |
|------|-------------|
| T1 | 取得（`sf log wifi -d <秒> -o ...`）。飛行の前から動かす |
| T2 | 飛行の命令（`sf takeoff` など）。次の命令を打つ前に前の応答を待つ |
| T3 | `sf emergency`（モータを即停止する最後の手段。`emergency` は `command` を送っていなくても受け付ける: `operation_manual.md` 210 行）。各 `sf` の呼び出しは 1 回ごとに別のソケットを開く（`lib/sfcli/utils/vehicle_api.py` の `send`・`send_urgent`）ので、T2 の命令の待機中でも T3 から送れる。**2 人目がいるときは、その人が Enter だけで送れる状態で待つ。開発者 1 人のときは、手が送信機にふさがるので T3 では待たない**（止める手段は人の操作） |

机上の段階（段階 1）: T1 = USB コンソール（`script -q ... sf monitor`）、T2 = 取得または WiFi の CLI（`nc 192.168.10.1 23`）。

### 操作の対応

**送信機（コントローラ）**: 電源を入れると前回の機体へ自動で再接続する（`docs/guides/controller.md` 3 章）。

| 操作 | 送信機 | 確認 |
|------|--------|------|
| ARM / DISARM | **スロットル側のスティックを押し込む。押すたびに ARM と DISARM が切り替わる**（モーメンタリボタン。スイッチではない。スロットル側は左スティック（スティック Mode 2）または右スティック（Mode 3）。`controller.md` 9 章、`state_task.cpp` 791-803 行）。機体の ARM 済みの間に押すと DISARM、ARM 前に押すと ARM | `status` の `armed`、StampS3 LED（ARMED_GROUND = 緑の低速点滅） |
| STABILIZE（姿勢安定。既定） | 左ボタンを押して高度モードを OFF に、右ボタンの制御モードを STABILIZE にする | 本体 LED が黄緑の常灯 |
| ACRO（角速度制御） | 右ボタンを押して STABILIZE ⇔ ACRO を切り替える（高度モードが OFF のとき） | 本体 LED が青の常灯 |
| ALT_HOLD（高度保持） | 左ボタンを押して高度モードを OFF → ALT HOLD にする | 本体 LED がオレンジの常灯 |
| POS_HOLD（位置保持） | 左ボタンをもう 1 回押す（OFF → ALT → POS → OFF の順） | 本体 LED がマゼンタの常灯 |

優先順位は POS_HOLD > ALT_HOLD > ACRO > STABILIZE（`operation_manual.md` 151-156 行）。地上でも ARM 中でもモードは変えられる（同 71 行）。**`sf takeoff` は、機体を自分で ARM し、POS_HOLD に切り替えて離陸する**（`api_task.cpp` 543-551 行、`state_task.cpp` 910 行）。API の飛行では送信機の ARM 操作もモード操作もしない（送信機は電源を入れて、スティックを中立にして持つ）。API が切り替えたモードと送信機のモード指定が食い違ったときの優先は**未確認**なので、API の飛行の間は送信機のボタンに触れない。

### 最初に知っておく用語

| 用語 | 意味 |
|------|------|
| 操縦者 | 送信機を持ち、DISARM を押せる状態でいる人。全段階で 1 人決める |
| ARM / DISARM | ARM は、モータを回せる状態にすること。DISARM はモータを止めて回せない状態に戻すこと |
| 状態（`state`） | IDLE_GROUND（地上・待機）、ARMED_GROUND（ARM 済み・地上）、TAKEOFF（離陸シーケンス）、FLYING（飛行中）、LANDING（着陸シーケンス） |
| スプール | ARM 後にモータが回り始め、回転が上がっていく短い時間（約 0.3 秒） |
| 一式（`.sflog.zip`） | `sf log wifi` で取得した飛行ログの 1 ファイル。信号ごとの CSV を 1 つの zip にまとめたもの。機体のログの文字（`text_log`）と診断値（`loop_timing`・`nonfinite`・`link_health`・`tof_diagnostics`）を含む |
| ESKF | 誤差状態カルマンフィルタ。機体の姿勢・位置・速度を推定する推定器。`posvel` の位置は ESKF の推定値 |
| 400 Hz ループ | ImuTask（センサ読み取りと姿勢推定）と ControlTask（制御計算）が 2.5 ms ごとに繰り返す処理 |
| `late`、`period max`、`busy max`、`over budget` | `late` = 前の起床から 1.5 周期（3.75 ms）を超えて起きた回数。`period max` = 連続する起床の間隔の最大。`busy max` = 1 周期の処理時間の最大。`over budget` = 処理が 2.5 ms を超えた回数。`status` は ms、一式の診断値は µs で表示する（10 ms = 10000 µs、2.5 ms = 2500 µs） |
| finite / `nonfinite` | 制御計算の値が NaN（非数）・無限大でないこと。`status` の `finite` 行と一式の `nonfinite` が、ARM 中の周期のうち有限でない値があった数を数える |
| ログシンク | ログの文字を一旦リングバッファ（円環状に使い回す記憶領域）に入れ、別のタスクだけがコンソールや WiFi へ書き出す仕組み。400 Hz のタスクが出力で待たされないようにする |
| CLI | 文字で命令を入力する窓口。USB（`sf monitor`）と WiFi（TCP ポート 23）の 2 経路 |
| API | 機体が UDP 8889 で受ける文字命令（Tello 風）。`sf takeoff` などが送る |
| Mcps、sigma | ToF の診断値。`signal`・`ambient` の単位 Mcps は 1 秒あたり百万カウント（受光量）、`sigma` は距離のばらつき [mm] |
| SILS | Software-In-the-Loop Simulation。制御ソフトを、実機の代わりに計算機上の機体モデルへつないで動かす試験 |

**ToF の状態**（下向き ToF = 機体底面から床までの距離を測るレーザ距離センサ）:

| 呼び方 | 意味 | どこに現れるか |
|--------|------|----------------|
| 無効（invalid） | 標本は来ているが、距離を測れていない（`status=254`、距離 0）。約 3.1 cm より近い不感帯で起きる。床の上では普通 | `sensor tof` の `valid=no`、`tof_bottom.csv` の `status` |
| 不感帯 | 約 3.1 cm より近くて測れない範囲。「無効」の主な原因 | 同上 |
| 出力停止（silent、新サンプル無し） | 新しい標本が 0.3 秒以上来ない、または空中で無効な標本だけが続く | `status` の `tof     : SILENT`、黄の高速点滅、`ARM rejected: downward ToF is silent` |
| 範囲外 | 遠すぎて測れない（上限の値は本書では未確認） | `status` の値 |

**LED と音**（`operation_manual.md` 53-103 行。StampS3 LED = 基板上の LED、本体 LED = 機体の 2 系統。優先は上から）:

| 状態 | 色とパターン | 音 |
|------|--------------|----|
| 低電圧（`safety.battery.low_v` 以下） | シアンの低速点滅（本体 LED は高速点滅） | 低電圧警告（繰り返し） |
| ARM 中の下向き ToF 出力停止 | **黄の高速点滅**（両方の LED） | 警告音 |
| ペアリング中 | 青の高速点滅 | — |
| ARM 拒否の理由（地上） | 赤の低速点滅 = 傾き不一致（置き直す）、マゼンタの低速点滅 = 校正・待機（動かさない）、黄の高速点滅 = ToF 出力停止 | エラー音（長く低い）を 1 回 |
| IDLE_GROUND / ARMED_GROUND / TAKEOFF / FLYING / LANDING | 緑の常灯 / 緑の低速点滅 / 白の高速点滅 / 緑の常灯 / オレンジの低速点滅（StampS3 LED） | ARM: 上昇 2 音、DISARM: 下降 2 音、校正完了: C5 の 3 連 |
| 致命的なハードウェア故障 | 赤の高速点滅 | エラー音 |

## 2. 安全上の規則

**この章を最初に読む。** 以降の全段階に適用する。

### 共通の規則

| # | 規則 | 理由 |
|---|------|------|
| 1 | モータが回りうる間は、全員が保護めがね（目を守る眼鏡）を着ける | プロペラや破片が目に当たるのを防ぐ |
| 2 | 機体の真上に顔や手を出さない。プロペラの回転面に指を入れない | 手指の怪我を防ぐ |
| 3 | 操縦者を 1 人決める。**操縦者は、全段階で、送信機のスロットル側スティックを押せる（DISARM できる）状態でいる。** `sf` コマンドで飛ばすときも同じ。PC の操作と記録は Claude Code が行う（1 章の人員の表） | 異常時に最も速く確実にモータを止める手段が、送信機の DISARM だから |
| 4 | **飛行中の中止の手順（下の「中止の段」）を、飛ばす前に全員が読み、声に出して確認する** | 異常時に迷わないため |
| 5 | 離陸したら、まず高度と下向き ToF を確かめる。どちらかが異常なら、すぐ着陸させる。<br>・高度の異常: 壁や柱に付けた高さの目印（1.0 m と 1.2 m）を目で見て、1.0 m の目印を超えた、上昇が止まらない、または下がり続ける。<br>・ToF の異常: 本体 LED と StampS3 LED が黄の高速点滅になり、警告音が鳴る（出力停止）。<br>・ACRO・STABILIZE ではファームが強制着陸しないので、操縦者が自分で着陸させる | 高度センサが当てにならない状態で飛び続けるのを避ける。高度の誤りは天井や床への衝突に直結する |
| 6 | 天井までの高さが 2.0 m 以上ある場所で飛ばす（高度上限 1.2 m に余裕 0.8 m 以上を足した値） | 上昇が行き過ぎたときの天井への衝突を避ける |
| 7 | 飛行区域は 2 m × 2 m 以上で障害物が無く、段階 2・3 はネットで囲み、区域内に人を入れない（`docs/guides/safety.md`）。機体は区域の中央に置く。**壁や柱に 1.0 m と 1.2 m の高さの目印を付ける**（目で判定するため） | 機体が逸れたときに人と機体を守る。判定を目で行えるようにする |
| 8 | 電池は専用充電器で充電し、充電中は離れない。高温・直射日光を避ける。膨らみ・傷のある電池は使わない。事故時は DISARM（または `sf emergency`）の後に電池を外す（`safety.md`） | リチウム電池の発熱・火災を避ける |
| 9 | `reboot`・`param save`・`wifi ssid`・`wifi pass` は ARM 中は機体が拒否する（`reboot` は再起動でモータが止まるため、他の 3 つはフラッシュ消去が 400 Hz ループを止めるため）。ただし拒否に頼らず、ARM 中はこれらを入力しない | 飛行中の再起動や制御の停止は墜落になる |
| 10 | **1 回の飛行は 1 つの目的だけにする。** 飛行ごとに記録を取り、結果表に書いてから次へ進む | 異常が出たとき、原因の飛行を特定できるようにする |
| 11 | 合格しなかった項目があるまま次の段階へ進むときは、責任者がその判断と理由を結果表の備考に書く | 未解決の項目を気づかずに抱えたまま飛ばさない |
| 12 | **送信機の電源を入れ直すときは、スロットル側スティックに触れない。** 機体が着陸して DISARM（またはモータ停止）したことを確かめてから入れる | ARM / DISARM は押すたびに切り替わるので、再接続の直後に押すと ARM または DISARM が反転する。再接続の所要時間と、再接続の直後のモード指定の挙動は**未確認**（`controller.md` 3 章は自動で再接続すると述べるだけ） |

### 中止の段（飛行中に異常が出たとき。段階 2・3 の全ての飛行手順がここを参照する）

| 段 | 状況 | 操作 | 効き方 |
|----|------|------|--------|
| 1 | ACRO・STABILIZE の飛行中・地上、およびどのモードでも TAKEOFF 中 | 送信機のスロットル側スティックを押す（DISARM） | **即モータ停止**（`operation_manual.md` 136 行）。空中で押すと落下する |
| 2 | ALT_HOLD・POS_HOLD の FLYING（`sf takeoff` の後を含む） | スティックを 1 回押す（DISARM） | **自動着陸**（約 0.3 m/s の緩降下）。モータは切れない（同 137 行） |
| 3 | 段 2 の自動着陸が始まらない・降りない（暫定: 押してから 3 秒たっても高さが下がらない）、または傾きが増える・揺れが増える | 2 回目の押し込み。効かなければ電池を外す（2 人目がいるときは、T3 で `sf emergency` の Enter。Claude Code への `sf emergency` の依頼は数秒遅れうるので当てにしない） | 2 回目の DISARM は即モータ停止（同 137 行）。`sf emergency` は送信機に関係なく即停止（同 210 行）。高度 0.5 m 前後からの落下になる |
| 4 | 送信機の電源を切る試験（1g-2、2-7、2-8）の間 | **送信機を切る前に T3 に `sf emergency` を打っておき、別の人が Enter を押せる状態にする**（2-7・2-8 は 2 人目がいるときだけ行う）。1g-2 は機体を手に持つので、プロペラを外して行い（1 章の推奨）、異常なら電池を外す | 通信途絶の間、操縦者の DISARM は届かない。`sf emergency` が唯一の停止手段 |

**中止の目で見る基準**（暫定。壁や柱の目印と目視で判定する）:

| 異常 | 基準 | 根拠 |
|------|------|------|
| 高さ | 目標 0.5 m の飛行で、1.0 m の目印を超えた。どの飛行でも 1.2 m の目印を超えた | 高度上限 1.2 m（`flight_limits.hpp` 55 行）。離陸の行き過ぎは SILS で約 0.84 m（`operation_manual.md` 216 行付近の記述）で、1.0 m はそれより上の線 |
| 傾き | 機体の面が水平から約 15 度（目視で、プロペラ面が床に対して明らかに傾いて見える）を超えた | SILS の合否線（`api_rotate_direction.expect` の傾き上限 0.2618 rad = 15 度）。実機の基準は設計文書に無い |
| 揺れ | 高さまたは水平位置が、保持の目標から約 0.15 m を超えて揺れる、または揺れの幅が 3 周期続けて増える | POS_HOLD の保持精度 ±6〜7 cm（`CLAUDE.md`）の約 2 倍。実機の基準は設計文書に無い |

### 段階ごとの中止条件

次のどれかが起きたら、その場で中止する（モータを止め、電池を外し、記録を保存して、原因を調べる）。各手順の表の「中止」の行は、ここに足す条件である。

| 段階 | 中止条件 |
|------|----------|
| 全段階 | `status` の `finite` 行、または取得した一式の `nonfinite` が、有限でない値の検出を示した。ARM 中の `period max` が 10 ms 以上になった。予定外の再起動（起動の表示が出る、LED が白に戻る）。スタックあふれの表示。LED が赤の高速点滅 |
| 段階 0 | `sf doctor` がエラーを出す。ビルド・書き込みが失敗する。`param get` の値が 0-7 の表と異なり、理由が分からない。`motor test` で位置が合わないモータがある |
| 段階 1 | 床の ToF が 5 cm を超える有効値を出す表面（C3 不合格）。ARM が拒否され理由が分からない。想定外の状態遷移。ToF が自然に止まった（1i の記録をとって責任者へ報告し、段階 2 へ進まない） |
| 段階 2・3 | 上の「中止の目で見る基準」。黄の高速点滅と警告音。想定外の状態遷移。電池の警告音。通信が切れて想定外の挙動。回頭（2-9・2-10）では、ToF の高さが 0.30 m を下回る、または区域の中央から 0.5 m を超えて水平位置がずれる。段階 3 は、手順のどこかで異常が出たら着陸して終える |

## 3. このドキュメントの使い方

Claude Code を進行役にして進めるときは、リポジトリで Claude Code を起動して `/hardware-checkout` と入力する（`.claude/skills/hardware-checkout/SKILL.md`）。手順を 1 つずつ示し、保存した記録を読んで合格線と照合する。Claude Code は、USB コンソールへの入力（モータが回る `motor test` を含む）、WiFi の取得（`sf log wifi`）、WiFi の CLI への入力、CLI の `arm`・`disarm`・`mode` を送ってよい（2026-11-22 の決定）。送る前に、何が起きるかを音声（macOS の `say`）で伝える。Claude Code の応答は数秒遅れうるので、異常のときにモータを止めるのは人である（送信機のスロットル側スティックの押し込み、機体のボタン、電池を外す）。人は、その間いつでもそれらを操作できる位置にいる。

### 確認項目

各項目に ID（C1〜C16）を付ける。結果表（8 章）は実施 1 回ごとの行を持ち、各行にこの ID を書く。

| ID | 確認すること | 何が壊れていると分かるか | 手順 |
|----|-------------|------------------------|------|
| C1 | 400 Hz ループが周期を守っている。WiFi の取得の有無で変わらない | ログ出力や処理の重さ、通信の取得で周期が乱れると、制御が数 ms 止まる | 1a・1d・1e、段階 2・3 の各飛行後 |
| C2 | 制御経路に NaN・無限大が現れない（`finite` 行・`nonfinite` が 0 件）。**後段のファームがこれを検出してモータを止める前提の確認**なので、毎回の件数を記録する | 計算の異常値がモータ出力に届くと、機体が制御できなくなる | 1a・1d、段階 2・3 の各飛行後 |
| C3 | ARM した状態で実際に使う床に置いたとき、下向き ToF の有効な読みが 5 cm 以下、または無効である。床ごとの診断値も記録する | 床の読みが 5 cm（`kTakeoffGroundBandM`）を超えると、地上にいるのに「床の上」と判定されない。余白が小さい（これまでの床の読みは 3.1〜5.0 cm） | 1b |
| C4 | ARM 中のログ出力が 400 Hz ループを乱さない。ホストが読まない USB でも、WiFi で取得中でも乱れない。ログの行が失われない | ログの書き出しが滞ると、ループが最大 50 ms 止まる。行が失われると原因の記録が欠ける | 1d・1e |
| C5 | WiFi の CLI で、警告が接続先に表示される。`reboot` が ARM 中は拒否され、DISARM 中は動く | WiFi 側から操作したとき、警告が見えない。ARM 中の再起動でモータが止まる | 1f |
| C6 | 各タスクのスタック（関数呼び出しの作業領域）に余裕がある | スタックあふれは予定外の再起動や誤動作になる | 1i |
| C7 | 電池の警告と緊急の通知が出て、再発報され、状態ごとに設計どおりに動く | 電池が尽きる前に着陸できない、または床の上で誤って着陸動作に入る | 1h |
| C8 | 手動（STABILIZE）で低いスロットルのまま浮いたとき、ToF が 5 cm を超える値を途切れず 1 秒示したら、ARMED_GROUND から空中の扱い（TAKEOFF）に移る。その後に通信が途絶えたら、モータを切らず着陸する | 空中にいるのに「地上」の扱いのままだと、通信途絶や電池の着陸が働かず、ToF の出力停止でモータが切れる | 1g・2-1・2-8 |
| C9 | `sf takeoff` が 0.5 m を保持する（SILS では約 0.84 m まで行き過ぎ、戻りに約 20 s）。`sf land`、`sf stop` が働く | 離陸後の高度が目標と違う、止めたいときに止まらない | 2-3・2-6 |
| C10 | 高度モード（ALT_HOLD・POS_HOLD）の離陸中に送信機で DISARM すると、すぐモータが止まる（設計どおり） | 離陸中の DISARM を着陸にすると、床の上で着陸動作が機体を浮かせることがある | 1g・2-2 |
| C11 | 着陸で、接地の判定が床の上でだけモータを止める。ホバーからの着陸で、3 つの判定経路（確実な接地・降下停滞・ToF 不感帯）のどれが働いたかと、着陸開始からモータ停止までの時間を記録する。上昇中に始まる着陸でも、5 cm を超える高さでは止まらない | 空中でモータが止まる、または接地しても止まらない | 1g・1h・2-3・2-4・2-5 |
| C12 | 飛行中に通信が途絶えたら着陸する | 送信機が切れたとき機体が飛び続ける | 1g・2-7・2-8 |
| C13 | `sf cw`・`sf ccw` が、指令した向きに指令した角度だけ回る（1〜360 度）。回頭中に高度が下がらず、水平位置がずれない。回頭の途中の `sf stop` で向きを保つ | 大きい角度で逆回りする、回らずに応答する、回頭中に高度や位置が崩れる | 2-9・2-10 |
| C14 | 気圧計から求めた床基準の高さ（観測専用）が、床の上で 0 付近・有効・オフセット未学習で、ホバーでオフセットを学習し、ToF の高さと比べてどれだけずれるか。**制御は気圧高度を使わない** | 観測値の大きさを知らないまま、将来の利用を判断してしまう | 1c・2-3 |
| C15 | 下向き ToF の再初期化（地上で標本を出さなくなったときだけ働く）が、正常な運用中は 1 度も働かない。自然に止まったときは、その経過を記録する | 健全なセンサが無用にリセットされる。止まったまま戻らない・ARM できてしまう | 1i、段階 2・3 の各飛行後 |
| C16 | 総合飛行: 離陸、高度と ToF の確認、1 回転、着陸を 1 回の飛行で通し、取得した一式に警告・異常が残らない | 部品ごとには合格しても、通すと現れる問題 | 3 |

拘束またはプロペラ付きの手持ちで低空の制御を確かめる段階は設けない（そのための既存の手順が無い。`development_roadmap.md` 3.0.2 の拘束試験は推力測定であり、`operation_manual.md` の `motor` コマンドはモータ単体の確認である）。代わりに、段階 1 の手持ち（プロペラなし）と、段階 2 の最低高度の飛行で確かめる。

### コマンドと出力の形について

本書のコマンドと出力の形は、ファームウェアと sf CLI のソースから取った（9 章に確認元を示す）。実機での出力の確認は、この手順の初回の実施で行う。ソースで確かめられなかったものは、本文中に「未確認」と書いた。「暫定」と書いた合格線・中止線は、設計文書に根拠が無く、本書が置いた仮の値で、根拠を併記した。測った値はそのまま記録し、超えたら責任者へ報告する。**5 cm の線は、5 cm 以下を「床」、5 cm を超える値を「床を離れた」とする**（ファームの `kTakeoffGroundBandM` の判定と同じ向き。`sf log stats tof` の `valid above 5 cm` は 5 cm を超える個数）。

### 変更するパラメータ

| パラメータ | 変更する手順 | 戻す手順 | 再確認 |
|-----------|-------------|---------|--------|
| `safety.battery.low_v`（既定 3.4） | 1h-1 | 1h-1 の最後 | 段階 2 の入場、段階 4 |
| `wifi mode`（`ap` が正） | 0-8（`sta` だったときだけ） | — | 0-8 |
| 全パラメータ（`param reset`） | 0-7（責任者が必要と判断したときだけ） | — | 0-7 |
| `flip.enable`（既定 0） | 変更しない | — | 0-7、段階 4 |

### 記録の取り方

飛行中は USB ケーブルを外す。USB が無くても、状態遷移・接地判定の経路・拒否・400 Hz ループの数え・有限でない数の数えは、WiFi の取得（`sf log wifi`）の一式から読める。ただし、機体のログの文字は取得を始めた後のものだけが載るので、飛行の前に取得を始める。取得の名前は `s<段階>_<手順>_<内容>_<回>`（回は同じ手順の繰り返しの通し番号 1, 2, ...。例 `s1_1b_tof_floor_1`、`s2_2-3_takeoff_1`）。**取得の長さ `-d`** は、最悪の応答待ち（`sf takeoff` 15 s、`sf land` 35 s、`sf up 50` 14 s、回頭は 角度 ÷ 57.3 + 6 + 3 s。`lib/sfcli/utils/vehicle_api.py` 59-76 行）の合計に余裕を足して決めた（下の表）。機体の WiFi は PC が 1 つ。

| 手順 | `-d`（秒） | 内訳 |
|------|-----------|------|
| 1b | 60 | ARM まで 15 + 保持 40 + 余裕 |
| 1e | 120 | 手順の時刻表（95 s）+ 余裕 |
| 1g | 90 | 4 つの操作 + 余裕 |
| 1h（各場面） | 120 | |
| 2-1 | 60 | |
| 2-2 | 30 | |
| 2-3 | 120 | 離陸 15 + ホバー 30 + 着陸 35 + 余裕 |
| 2-4、2-7、2-8 | 90 | 離陸 15 + 保持 + 着陸 + 余裕 |
| 2-5 | 120 | |
| 2-6 | 120 | 離陸 15 + 整定 10 + 上昇 14 + 着陸 35 + 余裕 |
| 2-9 | 180 | 離陸 15 + 整定 5 + 90 度 × 4（各 11 + 待ち 3）+ 360 度（15 + 3）+ 270 度（14 + 3）+ 着陸 35 + 余裕 |
| 2-10 | 90 | |
| 3 | 150 | |

**PC の接続（macOS）**: PC は、インターネットにつないだまま WiFi を機体につなぐ同時接続で進められる（2026-11-22 に実機で確認）。ただし、この PC（macOS）では、端末アプリからの Python のローカルネットワーク通信が拒否されることがあり、そのとき `sf log wifi` は `No data received` になる。**見分け方**: `nc -z 192.168.10.1 23` は通るのに、`sf log wifi -d 5 --no-save` はパケットが届かない。**対処**: システム設定の「ローカルネットワーク」で、使っている端末アプリを許可する。別の対処として、UDP の中継を使った。

| 読みたいもの | コマンド |
|--------------|----------|
| 取得の開始（保存直後に構造検査が自動実行される） | `sf log wifi -d <秒> -o logs/checkout/<名前>.sflog.zip` |
| 取得の要約（起動以降の累計の最後の値） | `sf log info <一式>` の末尾の `Diagnostics (status v4 and text log):`。`imu`・`control` の 2 行（`loop: cycles=... late=... period_max=...us busy_max=...us over_budget=... \| armed: late=... period_max=...us busy_max=...us`）、`nonfinite: detected N of M armed cycles ...`、`ToF bottom: ...`、`log paths: ...` |
| 機体のログの文字 | `sf log info <一式> --text`。絞り込みは `--grep <文字列>`（`-I` で大小無視）、`--since <秒>`、`--until <秒>`（機体の起動からの秒）。各行は `[<機体時刻> s] <本文>`。行が失われた位置に `*** N lines dropped here ***` |
| 実機確認用の要約（表） | `sf log stats <モード> <一式> [--since <秒> --until <秒>]`。モードと読む行は次の表 |
| グラフ | `sf log viz <一式> --mode position`（`attitude`・`sensors` も可）、`--save <ファイル>.png` |
| 解析 | `sf log analyze <一式>` |
| USB コンソール（段階 1。**全て記録する**） | `script -q logs/checkout/console_<手順>_<回>.txt sf monitor`（`script` は端末の入出力をファイルへ写す macOS 標準のコマンド。`sf monitor` を終了すると記録も終わる）。1d の条件 B だけは `sf monitor` を直接動かし、端末の表示を手で写してファイル名 `console_1d_B_<回>.txt` に貼る |
| `status` の出力 | USB の `stampfly>` または WiFi の CLI（`nc 192.168.10.1 23`）に入力し、表示を結果表に写す |
| 電源の入れ直し（power cycle） | ① USB ケーブルを抜く。② 電池を外す（前方 ToF の基板ごと主基板から抜く）。③ 5 秒待つ。④ **電池を先に接続する**（電池は前方 ToF の基板に取り付けてから、基板ごと主基板に挿す。基板を主基板に付けた後では電池を取り付けられない）。⑤ 緑の常灯（校正完了、ピッ×3）まで機体を動かさない。⑥ USB ケーブルを接続する。④⑥の順は実機で確かめた（2026-11-21、コミット 53553678、各順 2 回、電源スイッチ付き USB ハブ経由）: 電池を先につなぐと全センサが動き（`sensors : present=0x7F`）、後から USB を挿しても機体はリセットされず（LED は緑の常灯のまま、起動からの周期の累計が続く）、PC は機体を認識し CLI も使える。USB を先につなぐと、機体は USB の給電だけで起動し、前方 ToF が読めず無効になる（`present=0x3F`）。後から電池をつないでも機体は再起動しないので、前方 ToF は次のリセットまで無効のまま残り、その状態でも `prearm  : ready` になる。**PC がシリアルポートを開くと（`sf monitor`、`sf flash`）機体はリセットされる**（`rst:0x15 (USB_UART_CHIP_RESET)`）。このリセットの後は、どちらの順でつないでも電池があれば `present=0x7F` になる。段階 2・3（USB なし）は ②③④⑤だけ |

`sf log stats` のモードと、結果表に写す行（区間の `<秒>` は機体時刻）:

| モード | 写す行 | 区間 |
|--------|--------|------|
| `tof` | `valid minimum` / `valid maximum`（有効な読みの最小・最大 [cm]）、`valid above 5 cm`（5 cm を超えた個数。続く行がその時刻と値）、`ToF samples` の `valid` と `invalid` | 省略 |
| `spread` | 各行の `spread`（1 秒の窓ごとの最大 − 最小 [cm]）、`largest 1 s spread` | `--since` と `--until` が必須 |
| `loop` | `imu` / `control` の `armed:` の行、増えた行の時刻、`non-finite`、`log paths`、`text log`（欠落の数）、`ToF output stopped` | 省略 |
| `events` | `peak ToF height`、`last time outside 0.50 +- 0.07 m`（保持高度の帯に最後に収まらなかった時刻。保持高度と帯は `--target <m>`・`--band <m>` で変えられる）、`LANDING` の行、`last valid ToF before stop` | 省略 |
| `turn` | `yaw change`（向きの変化 [度]。時計回りが正）、`ToF height` の最小・最大、`horizontal drift`（推定器による水平位置のずれ [m]） | 必須 |
| `baro` | `baro height - ToF`（rms と最大 [m]）、`learned offset` の最小・最大と `age max` | 必須 |
| `boot` | `reset reasons`、`battery voltage` | 省略 |

`sf log stats` は事実だけを出し、合否は判定しない。必要なストリームが一式に無いと、そのストリーム名を示して終了コード 1 で終わる。

**飛行後の共通の確認**（段階 2・3 の飛行の後、毎回。結果表の C1・C2・C15 の行に写す）:

| 確認 | コマンド | 合格 |
|------|----------|------|
| C1 | `sf log stats loop <一式>` の `armed:` | `period max` 10 ms 未満、`busy max` 2.5 ms 未満、`over budget` 0 |
| C2 | 同上の `non-finite` | `detected 0 of N` |
| C15 | `sf log info <一式> --text --grep "re-init"` | 出力が空 |
| 電池 | 飛行の前と後の `battery` の電圧 | 記録（「電池」の規則に従う） |
| 構造 | `sf log check <一式>` | エラー 0 |

## 4. 段階 0: 準備

**入場条件**: なし。**目的**: プロペラを外し、試験するファームを確定し、記録を取れる状態にする。**中止**: 2 章の表（全段階・段階 0）。

| 手順 | 操作 | 記録 | 合格 |
|------|------|------|------|
| 0-1 | **最初に、プロペラ全 4 枚を外す**（以後、段階 2 の入場まで付けない）。写真を撮る | 写真 | 4 枚とも外れている |
| 0-2 | 試験するコミットを確定する。`git status --short`（`firmware/` と `lib/sfcli/` 配下に差分が無いこと）と `git log -1 --format=%H`。以後、機体に書き込むファームは必ずこの状態からビルドする | コミットの ID | 差分なし |
| 0-3 | `source setup_env.sh`、`sf doctor` | 結果 | エラーなし |
| 0-4 | `sf build vehicle`（成功で `Build successful`） | 結果 | 成功 |
| 0-5 | 機体を USB で接続し、`sf flash vehicle` で書き込む。続けて `mkdir -p logs/checkout` の後 `script -q logs/checkout/console_0-5_1.txt sf monitor` で起動を見る | 起動時の表示 | 起動する |
| 0-6 | USB コンソールで `version`（`StampFly vehicle firmware` と `build : <日付> <時刻>`） | ビルド日時 | コミットの時刻と矛盾しない |
| 0-7 | 設定値: `param get safety.battery.low_v`（`= 3.4 (float)`）、`safety.battery.usb_v`（`= 3.3 (float)`）、`safety.comm.timeout_ms`（`= 500 (float)`）、`flip.enable`（`= 0 (int)`）。**表の値と違うときは、保存済みの古い値が既定値を覆っている。** `param reset` で既定値に戻せるが、調整済みのゲインを含む全パラメータが戻るので、責任者が必要と判断したときだけ行い、その後 `param save`（DISARM 中）と `reboot` が要る | 4 つの値 | 表と一致、または違いを記録して責任者が承認 |
| 0-8 | `wifi show` が `mode : ap`（SoftAP: 機体が WiFi 基地局になる）であること。`mode : sta` なら `wifi mode ap` → `param save` → `reboot` | 出力 | `mode : ap` |
| 0-9 | PC を機体の WiFi（SSID `StampFly-XXYY`、XXYY は機体 MAC の下 4 桁。既定パスワード `stampfly`。`docs/guides/block_programming.md`）につなぎ、`sf log wifi -d 5 --no-save` | 統計 | パケットが届く |
| 0-10 | 送信機のペアリング（`operation_manual.md` 5 章、`controller.md`）。`status` で `pairing : Paired`、`prearm  : ready` | `status` | 両方そのとおり |
| 0-11 | 電池: 満充電（4.2 V 付近）から始める。`sensor power`（`power: 4.12 V  100 mA  400 mW` の形） | 電圧 | `safety.md` の表で「飛行可能」 |
| 0-12 | **モータの位置の確認**（プロペラなし、DISARM、USB コンソール）: `motor test 1 20 2`、`motor test 2 20 2`、`motor test 3 20 2`、`motor test 4 20 2`（M1 = 右前（FR）、M2 = 右後（RR）、M3 = 左後（RL）、M4 = 左前（FL）、デューティ 20 %・2 秒。`operation_manual.md` 322 行）。1 つずつ、その位置のモータだけが回ることを目で見る。回転の向きと、プロペラ（時計回り用・反時計回り用）の取付位置の正しい対応は、リポジトリの文書に見つからない（**未確認**）。モータの軸やプロペラの刻印・機体付属の説明で責任者が確認し、段階 2 の入場の表で再確認する | 各モータの位置と回転の向き | 4 つとも位置が一致 |
| 0-13 | **CLI の飛行命令の確認**（重りで押さえた机上。CLI の `arm`・`disarm`・`mode` の実機での確認は、この手順で最初に行う。SILS では確かめたが、実機では未確認）。USB コンソールで行い、⑥だけ WiFi の CLI（`nc 192.168.10.1 23`）で行う。送信機は電源を入れ、ペアリング済みにする。ARM でモータがアイドル回転で回るので、保護めがねを着け、プロペラの周りに手を出さない。Claude Code は、各命令を送る前に何が起きるかを音声で伝える。<br>① `arm` → `status` → `mode acro` → `mode stabilize` → `disarm`<br>② `arm` → 送信機のスロットル側スティックを押し込む（人の操作。CLI で ARM した機体を送信機で止める）<br>③ 送信機の押し込みで ARM（人の操作）→ `disarm`<br>④ `arm` → `emergency`<br>⑤ 送信機の左ボタンを 1 回押して ALT_HOLD にし（本体 LED がオレンジ）、`arm`（拒否される）。確かめたら左ボタンをもう 2 回押して OFF に戻す<br>⑥ WiFi の CLI で `arm` → `status` → `disarm`、続けて `arm` → `emergency` | 各命令の表示、`status` の `state`・`mode`・`armed` | 表示が期待どおりで、`status` の状態が一致（期待は下の表） |

**0-13 の期待される表示**（`cli_task.cpp` の `cmd_arm`・`requestMotorStop`・`printModeResult`・`printArmRefusal` から取った文言）:

| 命令・操作 | 期待される表示 | その後の `status` |
|-----------|---------------|-------------------|
| `arm`（DISARM 中、STABILIZE） | `ok: armed (state ARMED_GROUND, mode STABILIZE)` | `state   : ARMED_GROUND`、`armed   : yes` |
| `arm`（既に ARM 済み） | `no change: already armed (state ARMED_GROUND)` | 変わらない |
| `mode acro`（ARM 済み） | `ok: mode ACRO (state ARMED_GROUND)` | `mode    : ACRO` |
| `mode stabilize`（ARM 済み） | `ok: mode STABILIZE (state ARMED_GROUND)` | `mode    : STABILIZE` |
| `disarm`（ARM 済み） | `ok: motors stopped (state ARMED_GROUND -> IDLE_GROUND)` | `state   : IDLE_GROUND`、`armed   : no` |
| `emergency`（ARM 済み） | `ok: motors stopped (state ARMED_GROUND -> IDLE_GROUND)` | 同上 |
| `disarm`（DISARM 中） | `no change: not armed (state IDLE_GROUND)` | 変わらない |
| CLI で ARM した後の、送信機の押し込み | CLI には何も表示されない。モータが止まる | `state   : IDLE_GROUND`、`armed   : no` |
| `arm`（ALT_HOLD） | `` refused: the CLI arms only in STABILIZE or ACRO (mode is ALT_HOLD: an ARM there starts an auto-takeoff) - use `mode stabilize` or `mode acro` `` | `state   : IDLE_GROUND`、`armed   : no` |

地上での `mode` の注意: 地上（IDLE_GROUND）で ARM する前の `mode` は、送信機の電文が届いていると送信機のスイッチ位置に戻されることがある（`changed, then put back to ... by the transmitter's switch position`）。0-13 では ARM の後に `mode` を送るので、この表示にはならない。

**次へ進む条件**: 0-1〜0-13 が全て合格。

## 5. 段階 1: プロペラを外した机上の確認

**入場条件**: 段階 0 が完了し、プロペラが 4 枚とも外れている（**1g と 1h の前にも、もう一度 4 枚とも外れていることを目で確かめる**）。電池で駆動し、USB のコンソールを接続する（USB 給電だけでは ARM が拒否される）。機体は、段階 2 で使う床の表面に置く。**目的**: 推力が出ないので、失敗しても機体は動かない。飛ばす前に、机上でできる部分を全部確かめる。**中止**: 2 章の表（全段階・段階 1）。

### 1a 起動直後の基準値（C1・C2 の基準）

| 項目 | 内容 |
|------|------|
| 入場 | 電源を入れ直し（3 章の power cycle）、緑の常灯まで機体を動かさない |
| 操作 | USB で `status` |
| 期待される出力 | `state   : IDLE_GROUND`、`armed   : no`、`prearm  : ready`、`battery : ...`、`loop    : 400Hz period 2500 us; late = ...`、`imu     all  : cycles N, period max X ms, late N, busy max X ms, over budget N`（`imu     armed:`・`control all  :`・`control armed:` も同形）、`finite  : no armed cycle checked yet` |
| 記録 | `imu all`・`control all` の `late`、`period max` |
| 合格 | `all` の行の `late` が 0（ARM 前の地上は 0 が健全。`operation_manual.md` 338 行） |

### 1b 床の下向き ToF と診断値（C3）

床の読みが 5 cm を超えると、地上の機体が「床の上」と判定されない。段階 2 で使う**表面ごと**（床・マット等）に行う。

| 項目 | 内容 |
|------|------|
| 入場 | 表面に機体を水平に置く。プロペラは外れている |
| 操作 | 1. `sensor tof` を 3 回（`tof  : 0.034 m  status=0 valid=yes` の形）。<br>2. `sf log wifi -d 60 -o logs/checkout/s1_1b_tof_<表面名>_1.sflog.zip` を始める。<br>3. 15 秒以内に送信機で ARM する（STABILIZE にしてスロットル側スティックを押す）。`status` で `state   : ARMED_GROUND`。機体を動かさず 40 秒待つ。<br>4. 同じスティックを押して DISARM。取得の終了後に次を読む |
| 読む | `sf log stats tof logs/checkout/s1_1b_tof_<表面名>_1.sflog.zip`、`sf log info <一式>` の `ToF bottom:` の行。`status` に `tof     : SILENT` が無いこと |
| 記録 | 表面名、`valid minimum` / `valid maximum` [cm]、`valid above 5 cm` の個数と時刻・値、`ToF samples` の `valid` と `invalid`（**有効な標本がその表面にあるか、無いか**を明記する。1h-2 は有効な標本のある表面を要する）、`signal`・`ambient`（Mcps）・`sigma`（mm） |
| 合格 | `valid above 5 cm` が 0。無効はあってよい（床の不感帯）。診断値は**記録のみ**（基準にする範囲は無い）。不合格ならその表面は段階 2 に使わない。しきい値を変える判断は責任者が行う |
| 中止 | 2 章の段階 1 |

### 1c 床の上の気圧高度（C14）

| 項目 | 内容 |
|------|------|
| 入場 | 1b の取得を終えている |
| 操作 | 1. DISARM のまま床に置いて 10 秒以上動かさず、`sensor baro`（`baro : 101300.0 Pa  alt 12.34 m (absolute)` と続く行 `       height 0.01 m  offset 0.00 m  valid, offset not learned`）。<br>2. 1b の一式の ARM 後の値を `sf log viz <一式> --mode sensors` で見る（baro_height の `height`・`valid`・`learned`） |
| 記録 | DISARM の床の上の `height`、`valid`、`learned`。ARM 後の `height` の動き |
| 合格 | 観測のみで値の範囲の基準は無い。`height` が 0 付近、`valid`、`learned` が 0（`offset not learned`）であること |

### 1d ループ周期・有限性・USB の状態（C1・C2・C4）

ARM の瞬間にログを数行出すと、USB の書き出しが滞ったとき（ホストが読まず、機体側の送信が詰まる）、400 Hz のタスクが 5〜50 ms 止まることがあった。ログシンクがそれを防ぐ設計なので、USB の 3 つの状態で確かめる。カウンタは起動以降の累計なので、**条件ごとに電源を入れ直す**（数えを 0 に戻すのが目的なので、USB につないでいる条件 A・B は、`reboot`（DISARM 中）または USB 経由のリセットで代えてよい。2026-11-22 の実施でそうした。条件 C（USB なし）は電源の入れ直しのまま）。

**ARM・DISARM・モード切替は、CLI の命令で行うのを標準とする**（Claude Code が USB コンソールまたは WiFi の CLI から `arm`・`disarm`・`mode` を送る。送る前に音声で伝える）。下の表の「ARM を押す」は `arm`、「DISARM を押す」は `disarm`、「右ボタンで STABILIZE ⇔ ACRO を切り替える」は `mode acro` と `mode stabilize` を交互に送ることに読み替える。送信機で行う表のとおりの形は、CLI の命令が使えないときの代わりとして残す。CLI の `mode` は、ARM の前の地上では送信機のスイッチ位置に戻されることがあるので、ARM 中に送る。

| 項目 | 内容 |
|------|------|
| 入場 | 1a が合格。条件ごとに power cycle |
| 操作 A（USB を読んでいる） | `sf monitor` を動かしたまま（コンソールは `console_1d_A_1.txt` に記録）、ARM を 1 回押し、**5 秒後に DISARM**し、すぐ `status`（ARM の瞬間の停止を見るため）。続けて、もう一度 ARM し 60 秒保って DISARM、`status` |
| 操作 B（ホストが読まない + ログ大量） | `script` を使わず `sf monitor` を直接動かす。別の端末で `ps` により `sf monitor` のプロセス番号（PID）を調べ、`kill -STOP <PID>` で一時停止する（USB のホストが読むのをやめる。ポートを開いたまま読まない方法でもよい。どちらの方法でも、機体側の USB 送信が実際に滞るかは実機で**未確認**）。ARM → DISARM を 20 回（各 2 秒、ARM のたびにログが出る）。さらに ARM 中に右ボタンで STABILIZE ⇔ ACRO を 10 回切り替える（切替のたびに `Mode change: ...`）。WiFi の CLI で `status`。最後に `kill -CONT <PID>` で再開し、端末の出力を `console_1d_B_1.txt` に貼る |
| 操作 C（USB ケーブルを抜く） | 電池だけで駆動。ARM して 5 秒で DISARM → WiFi の CLI で `status`。もう一度 ARM して 60 秒保ち、DISARM、`status` |
| 記録 | 各条件で、5 秒の ARM の後と 60 秒の ARM の後の `imu armed`・`control armed`（`cycles`・`period max`・`late`・`busy max`・`over budget`）と `finite` 行。A・B では `W (<時刻 ms>) LogTask: log sink full: <N> lines dropped` が出たか。`status` は ms、一式は µs（10 ms = 10000 µs） |
| 合格（C1） | 3 条件とも `armed` の `period max` が 10 ms 未満、`busy max` が 2.5 ms 未満、`over budget` が 0、5 秒の ARM の後の `late` が 0 に近い（ARM の瞬間に増えない。`operation_manual.md` 338-342 行）。**暫定**: B・C の `late` ÷ `cycles` が A より明らかに増えない（根拠: 設計文書に条件間の比較の基準は無く、A を基準にした本書の仮の線）。ARM 中に 13〜22% の周期で続く 5 ms 級の遅れは別件（未解決。`detailed_design.md`）である。`late` が 0 にならなくても、これだけでは不合格にしない。記録して責任者が判断する |
| 合格（C2） | 3 条件とも `finite  : all control values finite (N armed cycles checked)` で N > 0 |
| 合格（C4） | A で `log sink full` が出ない（暫定: 基準は設計文書に無い）。B で出ても、その間の `armed` の `period max` が 10 ms 未満 |

### 1e WiFi の取得の有無によるループ周期の比較（C1・C4）

| 項目 | 内容 |
|------|------|
| 入場 | 1d が合格。機体は床の上。ARM・DISARM・モード切替は、1d と同じく CLI の命令（`arm`・`disarm`・`mode`）で時刻表どおりに送るのを標準とし、表の「押す」はその読み替えとする（送信機で行う形は、CLI の命令が使えないときの代わり）。ARM の前の地上の `mode` は送信機のスイッチ位置に戻されることがあるので、ARM 中に送る。USB コンソールは接続し、`console_1e_<条件>_<回>.txt` に記録する（6 回とも同じ） |
| 操作（取得なし 3 回・取得あり 3 回で**同じ時刻表**。時刻は緑の常灯の時点を t = 0 とする） | 毎回、power cycle し、緑の常灯を待つ（数えを 0 に戻すのが目的なので、1d の条件 A・B と同じく、DISARM 中の `reboot` または USB 経由のリセットで代えてよい。6 回とも同じ方法にそろえる）。<br>t = 0: 取得ありの回は `sf log wifi -d 120 -o logs/checkout/s1_1e_cap_on_<回>.sflog.zip` を始める（取得なしの回は何もしない）。<br>t = 5 s: ARM を押す（STABILIZE）。<br>t = 10, 15, 20, ..., 55 s: 右ボタンを押して STABILIZE ⇔ ACRO を切り替える（10 回）。<br>t = 65 s: DISARM を押す。<br>t = 70 s から: ARM を押して 2 秒待ち、DISARM を押して 2 秒待つ、を 5 回（t = 90 s まで）。<br>t = 95 s: 終了。USB で `status`。取得ありの回は取得の終了後に `sf log info <一式>` と `sf log stats loop <一式>` |
| 記録 | 各回の `imu armed`・`control armed` の `cycles`・`late`・`period max`・`busy max`・`over budget`。取得ありは加えて `text_dropped_lines` と、`loop` の `text log` の欠落の数 |
| 合格（C1、**暫定**） | 取得ありの `late`・`over budget` が、取得なしの 3 回と同じ（3 回が互いに違うときはその幅の中）。`period max`・`busy max` が取得なしの 3 回の最小〜最大の幅の中（根拠: 設計文書に比較の基準は無く、取得なしの実測を基準にした本書の仮の線） |
| 合格（C4） | 取得あり 3 回とも `text_dropped_lines` が 0、`--text` に `*** N lines dropped here ***` が無い |
| 幅を超えたとき | `sf log stats loop <一式>` が、ARM 中のカウンタが増えた行の時刻を出す。その前後を `sf log info <一式> --text --since <時刻-2> --until <時刻+1>` で読み、どのログの行が出ていたかを突き合わせて記録する |

### 1f WiFi の CLI（C5）

CLITask（CLI を担当するタスク）が出す警告は、ログシンクを通らず、その接続へ直接出る設計である。

| 項目 | 内容 |
|------|------|
| 入場 | 機体は床の上、DISARM。`nc 192.168.10.1 23` で接続（`StampFly vehicle TCP CLI — type 'help' for commands` と `stampfly> `）。USB コンソールも見る（`console_1f_1.txt` に記録） |
| 操作と期待 | 1. `param set safety.battery.low_v 9`（範囲外。何も変わらない）→ その接続に `W (<時刻 ms>) Params: set_float: 'safety.battery.low_v' = 9.000000 out of range [3.000000, 4.200000]` と `set failed (out of range?): safety.battery.low_v`。**USB コンソールには同じ警告が出ない**。<br>2. `param get safety.battery.low_v` → `= 3.4 (float)`。<br>3. ARM（ARMED_GROUND）して `param save` → `refused: param save stalls the 400Hz loop (flash erase) — land/disarm first`。DISARM 後は `params saved to NVS`。<br>4. ARM して `reboot` → `refused: reboot stops the motors — land/disarm first`。再起動せず `rebooting...` も出ない（確かめたら DISARM する）。<br>5. DISARM 中に `reboot` → その接続に `rebooting...`、接続が切れる。**USB コンソールには、再起動の直前までのログが出し切られてから**起動の表示が出る。機体が戻り（緑の常灯）、再接続できる |
| 記録 | 警告の表示場所、`reboot` の拒否の文、再接続までの時間 |
| 合格 | 上の期待どおり |

### 1g 手持ちでの離陸判定・通信途絶・離陸中の DISARM（C8・C10・C11・C12）

**1 人（開発者）で行う。片手で機体を持ち、片手で送信機を持つ。** 手動モードでは、スロットルが低いまま浮いた機体を、ToF が 1 秒示すまで「地上」のままにすると、通信途絶や電池の着陸が働かない。機体を手で持ち上げて、判断の部分を実機の ToF で確かめる。

| 項目 | 内容 |
|------|------|
| 入場 | プロペラが外れていることを目で確かめた。機体に強い衝撃を与えない（3 G の衝撃検出で DISARM になる。`safety.impact.accel_g` の既定 3.0）。1 人で行う: 片手で機体を持ち、片手で送信機を持つ（スティックの押し込みですぐ止められる）。**プロペラを外して行うことを推奨**する（付けたまま行うかは責任者の判断）。`sf log wifi -d 90 -o logs/checkout/s1_1g_handlift_1.sflog.zip` を Claude Code が始める |
| 操作 1g-1（C8） | STABILIZE で ARM（スロットルは最小）。機体を床から約 20 cm まで水平に持ち上げ、3 秒保つ |
| 期待 | ToF の 5 cm を超える有効な読みが途切れず 1 秒続いたら `Transition: ARMED_GROUND → TAKEOFF`。さらに 15 cm を超える有効な読みが来たら FLYING（`Takeoff complete → FLYING/STABILIZE`）。スロットルは最小のまま |
| 操作 1g-2（C8・C12） | 20 cm で持ったまま、**送信機の電源を切る**（通信途絶。送信機を持つ手で切る。異常なら電池を外す） |
| 期待 | `Comm lost → hover 3000 ms then LANDING`。約 3.5 秒後（`safety.comm.timeout_ms` の 0.5 秒 + 猶予 3 秒）に `Comm lost → LANDING (hover grace elapsed)`、LANDING。**モータは切れない**（`motor.csv` の duty が LANDING の間 0 より大きい） |
| 操作 1g-3（C11） | 機体をゆっくり床に置く。期待: `Landing detected (...)`、`Landing complete`、`Transition: LANDING → IDLE_GROUND`、モータ停止 |
| 操作 1g-4（C10） | 送信機の電源を入れ直す（規則 12。スティックに触れない）。ALT_HOLD にして（左ボタンを 1 回）ARM を押す（機体は床の上）。約 0.3 秒のスプールの後に TAKEOFF になる。**ARM の約 1 秒以内に** DISARM を押す（2 秒以上たつと、床の ToF が無効のとき ToF 無効の規則が TAKEOFF を LANDING に変える） |
| 期待 | `DISARM accepted (TAKEOFF → IDLE_GROUND)`（`auto-land` ではない）。LANDING を経由せず IDLE_GROUND。全モータの duty が 0 |
| 読む | `sf log info <一式> --text --grep Transition`、`--grep "Comm lost"`、`--grep Landing`、`--grep DISARM`。`sf log stats events <一式>` |
| 記録 | 状態遷移の時刻、途絶から LANDING までの時間、`Landing detected` の括弧内の文言、1g-4 の DISARM から duty 0 までの遅れ（基準値は無い。記録のみ） |
| 合格 | 上の期待どおり。**1g-4（C10）が合格するまで 2-2 を行わない** |

### 1h 電池（C7）と、手持ちで上昇中に始める着陸（C11）

プロペラが外れていることを目で確かめてから始める。電池の通知が出て再発報されること、状態ごとの動きが設計どおりかを確かめる。

**1h-1 警告（WARNING）: パラメータで確かめる。**

| 項目 | 内容 |
|------|------|
| 入場 | DISARM、プロペラなし。`status` の `battery` の値 V を読む。`low_v` の許容範囲は 3.0〜4.2 V なので、**`battery` が 4.15 V 未満**であること（満充電に近いときは、段階 1 の作業で電圧が下がってから行う） |
| 操作 | `param set safety.battery.low_v <battery の値 + 0.05>`（小数 2 桁。例: 4.08 V なら 4.13）→ `param save` → `reboot`。起動後、USB コンソール（または取得した一式の `--text`）を見る。ARM する（地上）。**最後に復元する**: `param set safety.battery.low_v 3.4` → `param save` → `reboot` → `param get safety.battery.low_v` |
| 期待 | `W (<時刻 ms>) failsafe: Low battery: 4.0xV` が約 5 秒ごとに繰り返される（10 Hz の更新 50 回。`battery_alert_policy.hpp` 67 行）。`W (<時刻 ms>) StateManager: Low battery warning`。警告音。StampS3 LED がシアンの低速点滅。ARM はできる（状態は変わらない）。復元後は `= 3.4 (float)` |
| 記録・合格 | 通知の表示、再発報の周期、復元の確認（**復元を 1h-2 の前に必ず終える**） |

**1h-2 緊急（EMERGENCY、3.0 V 未満）。任意の項目（電源装置の導線を責任者が用意しているときだけ行う。用意が無ければ「未実施」）。** 危険電圧の閾値（3.0 V）はパラメータにできないので、電池の代わりに電源装置を電池の接続部につなぎ、電圧を下げて確かめる。電源装置の接続方法（コネクタの形・極性・電流制限の値）は、リポジトリの文書にもファームのソースにも無い（**未確認**）。最低限の注意: 極性を取り違えない（責任者が確認した導線だけを使う）、電流制限を `status` の `battery` の電流に余裕を足した値に設定する、ARM 前の電圧は 3.6 V より上、機体を持つ操作と電圧の操作が重なる場面があるので、1 人で行うかは責任者が判断する（1 章の人員の表）、プロペラなし。ARM は 3.3 V 以下（`safety.battery.usb_v`）で拒否されるので、約 3.6 V から始めて ARM し、その後で 3.0 V 未満に下げる。USB コンソールを記録し、`sf log wifi -d 120 -o logs/checkout/s1_1h_battery_<場面>_1.sflog.zip` も取る。**電圧を下げた一式は、1i の `boot` の確認と `re-init` の確認から外し、別に判定する**（低電圧そのものが再起動や ToF の停止を起こしうるため）。

| 場面 | 操作 | 期待される出力・動き |
|------|------|----------------------|
| 床・ARMED_GROUND | 3.6 V で STABILIZE の ARM（スロットルは最小）。電圧を 2.9 V へ下げる | `E (<時刻 ms>) failsafe: Critical battery: 2.9xV`、`StateManager: Battery emergency: no transition in ARMED_GROUND`。**状態は ARMED_GROUND のまま、モータはこれでは切れない**。同じ行が約 5 秒ごとに繰り返される |
| TAKEOFF・床を離れた証拠なし | 3.6 V に戻して DISARM。**床の ToF が有効な読みを返す表面（1b で有効な標本があった表面）で**、3.6 V で ARM し、スロットルを 0.5 より上へ（`TAKEOFF_THROTTLE_THRESH` は 0.5。`config.hpp` 278 行）。状態が TAKEOFF になったら、電圧を 3.0 V 未満へ。床の ToF が無効のままの表面では、約 2 秒後に ToF 出力停止になり FLYING の表示になるので、この場面は成立しない | `Battery emergency in TAKEOFF without evidence of the air (the healthy ToF has not shown the craft off the floor) → DISARM`。モータ停止、IDLE_GROUND |
| TAKEOFF・床を離れた証拠あり | 3.6 V で ARM（STABILIZE、スロットル最小）。機体を床から約 10 cm（5〜15 cm）に持ち上げ、状態 TAKEOFF を確かめる。電圧を 3.0 V 未満へ | `Battery emergency → LANDING`。**モータは切れない**。機体をゆっくり床へ置くと `Landing complete` |
| FLYING | 約 20 cm（15 cm 超）に持ち上げて状態 FLYING を確かめ、電圧を下げる | `Battery emergency → LANDING`。モータは切れない |
| 床・IDLE_GROUND（DISARM 後） | 電圧が 3.0 V 未満のまま DISARM し、約 5 秒待つ | `Battery emergency: no transition in IDLE_GROUND`（ログのみ）。ARM を試すと拒否: `ARM rejected: battery X.XXV (USB/low — unsafe to fly)`、`prearm  : BLOCKED (battery low / USB power)`、エラー音 |

**1h-3 手持ちで上昇中に始まる着陸（C11）。任意の項目（1h-2 と同じ条件）。** 1h-2 の「TAKEOFF・床を離れた証拠あり」と同じ構成で、LANDING が始まる瞬間に機体を手で動かしておく。機体を床から約 0.2〜0.4 m/s でゆっくり持ち上げ始め、5〜15 cm を通過している途中で電圧を 3.0 V 未満へ下げる。その後、2 秒ほど 5〜15 cm の範囲で機体を上下にゆっくり動かし、最後に床へ置く。`sf log stats events <一式>` を読む。合格: モータが止まるのは機体を床に置いた後だけ（`last valid ToF before stop` が 5 cm 以下）。5 cm を超える高さの間は止まらない。

### 1i 全記録の総点検（C6・C15）

| 項目 | 内容 |
|------|------|
| 入場 | 1a〜1h を終え、`logs/checkout/` に全ての一式とコンソールの記録（1d の B を含む）が残っている |
| 操作 C6 | 1. `grep -n -i "panic\|stack overflow\|Guru Meditation" logs/checkout/console_*.txt`（ESP-IDF の標準の表示。この機体での実際の文面は未確認）。<br>2. 1h を除く全ての一式で `sf log stats boot <一式>`。 |
| 合格 C6 | panic の行が無い。`reset reasons` が `1 POWERON` と `3 SW` だけ（4 PANIC・5 INT_WDT・6 TASK_WDT・9 BROWNOUT は不合格）。スタックの残りは DISARM 中に `stack` で読める（各タスクの `min free`、単位は B。ARM 中は拒否される）。合否の目安の数値は無いので**記録のみ**: 段階 1 の終わりと、段階 2・3 の最後の飛行の後（DISARM 後）に `stack` の全文を写す。内蔵メモリの空きは `status` の `heap` 行（free・minimum・largest block）で、同じ時点と、WiFi 取得と WiFi の CLI をつないだ状態で写す（床の上・USB のみの値は `docs/hardware/mcu-memory.md` 6・8 章）。設計上の大きさは `detailed_design.md` の「メモリ配置」（ImuTask 16 KB、ControlTask 8 KB、LogTask 6 KB 等） |
| 操作 C15 | 1h を除く段階 1 の全ての一式で: `for f in logs/checkout/s1_*.sflog.zip; do case "$f" in *s1_1h_*) continue;; esac; echo "$f"; sf log info "$f" --text --grep "re-init"; done`。コンソールの記録は `grep -n "re-init" logs/checkout/console_*.txt`。補助として `sf log stats loop <一式>` の `ToF output stopped` が 0 行であること、記録した `status` に `tof     : SILENT` が無いこと |
| 合格 C15（正常な運用） | `re-init` を含む行が 1 行も無い。この機能は、標本が 2 秒以上途絶えた（`kTofReinitSilenceUs`）ときだけ働く。正常に動くセンサに試行が出たなら、それ自体が不具合である（記録して責任者へ報告） |
| センサが自然に止まったとき | 下向き ToF を止める操作は、この手順に含めない（この機体の下向き ToF が、はんだ付けなしで安全に外せる位置にあるかを記した文書が無く、センサの抜き差しや遮断を操作として示せない）。センサが自然に止まった（`tof     : SILENT`、`sensor tof` が無効、`re-init` の行）ときは、**モータが回らない状態のまま**、次を記録して責任者へ報告し、段階 2 へ進まない。<br>・最初の試行の行 `VL53L3CX bottom re-init: attempt 1 (no sample for N ms)` の時刻と N（設計では 2000 以上）。<br>・以後の試行の間隔（設計では約 10 秒。`kTofReinitRetryIntervalUs`）。<br>・各試行の結果の行: 成功 `... succeeded (attempt N, X ms)`、失敗 `... failed (attempt N, X ms): <エラー名>`（失敗の行は 1 回目と、以後 6 回目ごと = 毎分 1 行）。失敗した試行ごとにドライバが `I2C read failed at 0x00E5`・`WaitDeviceBooted failed` の 2 行を足す。<br>・成功後に `sensor tof` が `valid=yes` に戻るまでの時間。<br>・その間の `prearm  : BLOCKED (downward ToF silent / not re-acquired - ...)`、ARM を試した場合は `ARM rejected: downward ToF is silent (...)` |
| 合格 C15（止まったとき、**暫定**） | 最初の試行が出力停止の約 2 秒後、以後約 10 秒おき、成功すれば `valid=yes` に戻る。根拠は設計値（`flight_limits.hpp` 196-199 行の 2 s・10 s・1 s・6）で、実機の所要時間は設計文書でも未確認 |

**次へ進む条件**: C1〜C5 が合格。C6〜C8・C10〜C12・C14・C15 が合格、または責任者が承認。**段階 2 に進む日が別でも、下の「段階 2 の入場」を毎回行う。**

## 6. 段階 2: プロペラを付けた低空の自由飛行

### 段階 2 の入場（毎回の飛行日の最初に全部確かめる）

| # | 確認 | 内容 |
|---|------|------|
| 1 | 段階 1 の結果 | C1〜C5 が合格。1g-1（C8）と 1g-4（C10）が合格。C3 が合格した表面の上で飛ばす。1h-1 が合格。**1h-2 が「未実施」のとき**: 責任者が承認し、段階 2 の全ての飛行を高さ 0.5 m 以下で行い、電池の通知（低電圧のシアン点滅と警告音）が出たら直ちに着陸する（危険電圧の動作は実機で未確認のため） |
| 2 | プロペラ | 損傷・変形の無い 4 枚を、モータの位置（0-12 で確認）と回転の向きに合わせて付け、保護ガードの損傷を確かめる（`safety.md` の飛行前チェックリスト 1・2）。時計回り用と反時計回り用の取り違えが無いこと（正しい対応は**未確認**: 責任者が確認する）。**付けた後は、モータを回すコマンド（`motor`）を使わない** |
| 3 | パラメータ | `param get safety.battery.low_v` が `= 3.4 (float)`、`flip.enable` が `= 0 (int)`（3 章の「変更するパラメータ」の表） |
| 4 | 電池 | 満充電に近く、**各飛行の開始が 4.0 V 以上**（暫定。根拠: `safety.md` の表で通常の 3.7〜4.0 V が「飛行可能」の下限 3.7 V に、飛行ごとの降下の余裕を足した本書の仮の線）。飛行の終わりに 3.7 V 未満、または警告音が出たら、その飛行で終え、電池を替える（`safety.md`: 3.5〜3.7 V は着陸推奨、3.5 V 以下は即時着陸。`safety.battery.low_v` 3.4 V は警告） |
| 5 | 人員・端末・区域 | 1 章の表の人員。T1〜T3。天井 2.0 m 以上、ネットの 2 m × 2 m 以上、壁や柱の 1.0 m と 1.2 m の目印。中止の段を全員が読んだ |
| 6 | 送信機 | 電源が入り、ペアリング済み（`pairing : Paired`）。モードは各手順の指定どおり（LED の色で確認） |

**目的**: 危険の小さい順（1 章の表）に、1 回の飛行 1 つの目的で飛ぶ。前の飛行が不合格なら、次へ進まない。**中止**: 2 章の「中止の段」と「段階ごとの中止条件」。

### 共通の手順（毎回）

| 順 | 操作 |
|----|------|
| 前 | 電池を入れ直し、校正の完了（緑の常灯、ピッ×3）を待つ。WiFi の CLI で `status`: `prearm  : ready`、`pairing : Paired`、`battery` の電圧を記録（4.0 V 以上） |
| 前 | T1 で `sf log wifi -d <秒> -o logs/checkout/s2_<手順>_<内容>_<回>.sflog.zip`（秒は 3 章の表）。2 人目がいるときは、T3 に `sf emergency` を打っておく |
| 後 | 着陸・DISARM の後、取得の終了を待つ。3 章の「飛行後の共通の確認」を行い、`sf log info <一式> --text --grep Transition` で状態遷移を読む。`--grep "E ("` と `--grep "W ("` でエラー・警告の行を読む（ESP-IDF の行の先頭が `E (`・`W (`。実機の形は未確認） |

`sf` で飛ばす手順（2-3・2-6・2-9・2-10・3）は、次のコマンドで操作する（`--ip` で機体の IP を変えられる。既定は 192.168.10.1）。**`sf takeoff` は機体を自分で ARM し、POS_HOLD で離陸する**（送信機の ARM 操作は不要。1 章の「操作の対応」）:

```bash
sf takeoff     # 自動離陸（ARM から）。固定の 0.5 m まで上昇し、FLYING になるまで待って応答を表示（高度の引数は無い。待つ時間は既定で 15 秒）
sf up 50       # 50 cm 上昇（10〜300 cm）。到達するまで待つ（14 秒）。上限 1.2 m を超える指令は PC が拒否する
sf cw 90       # 時計回りに 90 度（1〜360 度）。回り終えるまで待つ。sf ccw は反時計回り
sf stop        # 現在位置（高度・向き）を保持。別の端末から送ると、待っているコマンドも打ち切る
sf land        # 着陸。接地するまで待つ（既定で 35 秒）
sf emergency   # 全モータを即座に止める（待たされずに送られる）
sf height      # 推定の高度 [cm]（sf tof は下向き ToF [cm]、sf baro は気圧の高度 [m]）
```

終了コードは、0 が成功、1 が機体の `error ...` または PC による引数の拒否、3 が応答なし、130 が Ctrl-C。待っている `sf takeoff`・`sf up`・`sf cw` などを Ctrl-C で中断すると `stop` が自動で送られる（離陸の上昇中は拒否され、離陸は続く）。`sf land` を中断しても何も送られない。**`emergency` は自動では送られない。** 所要時間は `time sf cw 90` で測る。操縦者は送信機を中立にして持ち、DISARM の用意をする（スティックを動かすと API の誘導は解除される）。

**タイミングのある操作は 1 行で用意しておく**（飛行中に 2 つの端末を操作しない）。例（`sf up 50` を始めて 1 秒後に `sf stop`）: `sf up 50 & sleep 1 && sf stop; wait`。1 行目の `sf up` と `sf stop` は、別のソケットで機体に届く。**区間の取り方**（`sf log stats` の `--since`・`--until`）: `sf log info <一式> --text --grep "cmd: "` が、PC から届いた命令（`cmd: takeoff`・`cmd: cw 90` など）の機体時刻を出す。区間は、その回の命令の時刻から次の命令の時刻まで。ホバーの区間は `Transition: TAKEOFF → FLYING` の時刻から `cmd: land` の時刻まで。**「整定した」の判定**（共通、暫定）: `sf height` が 2 回続けて 0.5 ± 0.07 m（保持の帯。`sf log stats events` の既定）に入り、目で見て揺れが 0.15 m 未満。

### 2-1 手動の低スロットル離陸の検出（C8）

| 項目 | 内容 |
|------|------|
| 目的 | スロットルが低いまま浮いたとき、ToF の証拠で ARMED_GROUND から TAKEOFF に移るか。1g-1 で判断は確かめたので、実機の浮き方で確かめる |
| 入場 | Claude Code が取得を始めている。STABILIZE（本体 LED が黄緑）。人員: 操縦者（開発者）1 人 |
| 操作 | ARM を押し、スロットルをゆっくり上げ、浮いたら約 10〜20 cm で 2〜3 秒保ち、スロットルを下げて着陸。高さは 20 cm 以内。**浮かなければ 1 回だけ繰り返し**、それでも浮かなければ記録して終える |
| 記録 | 浮いたときのスロットル値（一式の `pilot.csv` の `throttle` 列、0〜1。これが 0.5 の閾値（機体内の推力指令）と同じ尺度かは**未確認**）。`--grep Transition` の時刻 |
| 合格 | 浮いたスロットルが 0.5 以下のとき、ToF の 5 cm 超が 1 秒続いた後に TAKEOFF へ遷移する。0.5 を超えて浮いたときは、スロットルの閾値で TAKEOFF へ遷移するので、**この確認は成立しなかった**と記録する（C8 は 1g の結果で判断する） |
| 中止 | 高さ 20 cm 超、傾きの増大（中止の段 1） |

### 2-2 高度モードの離陸中の DISARM（C10）

| 項目 | 内容 |
|------|------|
| 目的 | 高度モードの離陸中に DISARM すると、設計どおり即モータ停止か（着陸にしない理由: 床の近くで始めた着陸動作が機体を浮かせることがあるため）。**1g-4 で机上で確かめてから行う（入場）** |
| 入場 | 1g-4 が合格。C3 が合格した柔らかいマットの上。ALT_HOLD（本体 LED がオレンジ）。人員: 操縦者（開発者）1 人 |
| 操作 | ARM を押す。約 0.3 秒のスプールの後に上昇が始まる。**ARM の約 1 秒後**（高さ 0.3 m 未満）に DISARM を押す。機体は DISARM の後、その高さから落ちる |
| 記録 | `--grep Transition`: TAKEOFF → IDLE_GROUND（LANDING を経由しない）の時刻。`sf log stats events <一式>` |
| 合格 | 1g-4 と同じ（LANDING を経由せず、全 duty が 0） |
| 中止 | 1 秒たっても DISARM が効かない（電池を外す。2 人目がいるときは T3 の `sf emergency` を Enter） |

### 2-3 離陸の高度保持と通常の着陸、気圧高度の観測（C9・C11・C14）

| 項目 | 内容 |
|------|------|
| 目的 | `sf takeoff` が 0.5 m を保持するか。SILS では約 0.84 m まで行き過ぎ、0.5 m に戻るまで約 20 s かかる。実機の値を記録する。続けて `sf land` で接地判定を初めて実機で見る。同じ取得で気圧高度を観測する（観測だけ） |
| 入場 | 2-2 が合格。人員: 操縦者（開発者）+ Claude Code（取得と命令の送信。止めるのは操縦者）。POS_HOLD は `sf takeoff` が切り替える |
| 操作 | `time sf takeoff`。応答が返ったら 30 秒ホバーを見る（操縦者は DISARM に備える。中止の段 2）。その後 `sf land` |
| 読む | `sf log stats events <一式>`、`--grep "Landing detected"`、ホバーの区間で `sf log stats baro <一式> --since <t0> --until <t1>`、WiFi の CLI の `sensor baro`（`offset age`） |
| 記録 | `peak ToF height`、`last time outside 0.50 +- 0.07 m`（FLYING 開始との差）、`LANDING` の行の時間、`last valid ToF before stop`、`time sf takeoff` の所要時間、接地の経路、`baro height - ToF` の rms と最大、`learned offset` |
| 合格 | 最高到達が **1.0 m 以下**（暫定。根拠: 1.0 m の目印の中止線。高度上限 1.2 m、SILS の行き過ぎ 0.84 m）。保持と整定の合格線は設計文書に無い（実測を SILS と比べ、責任者が判断）。`sf land` が成功で終わり、モータが接地の後に止まる。**気圧高度は観測のみで合否の基準は無い**: ホバーで学習済み（`learned`）。過去の記録でオフセットは約 +0.2〜+0.9 m、`baro height - ToF` は rms 0.22 m・最大約 1.2 m（参考値。基準ではない） |
| 中止 | 1.0 m の目印を超えた、黄の高速点滅、`sf takeoff` が `error ...` を返した（`status` で状態を確かめる）。中止の段 2・3 |

### 2-4 ホバーからの着陸と接地判定の経路（C11）

| 項目 | 内容 |
|------|------|
| 目的 | 接地の 3 つの判定経路（確実な接地: ToF が 5 cm 以下＋静止、降下停滞: 地面効果で降下が止まる、ToF 不感帯: 床近くの有効な読みの後に 0.2 秒無効が続く）が、床の上でだけモータを止めるか |
| 入場 | 2-3 が合格。**表面ごとに 3 回以上**（1 回ごとに取得、回の番号を付ける）。ALT_HOLD。人員: 操縦者（開発者）1 人 |
| 操作 | ARM を押し（自動離陸）、0.5 m で保持。DISARM を押す（自動着陸）。**接地してもモータが止まらないときは、押してから 12 秒待って 2 回目の DISARM**（根拠: `sf land` が着陸の完了を待つ時間の前半 12 s。`vehicle_api.py` 64 行。通常の着陸は 0.5 m を約 0.3 m/s で約 2 秒なので、12 秒は 5 倍以上の余裕） |
| 読む | `--grep "Landing detected"`: `(ground)` / `(stalled descent / ground effect)` / `(ToF dead zone: only invalid samples for N ms after a valid X m)` のどれか。`sf log stats events <一式>` |
| 記録 | 経路の文言、着陸開始からモータ停止までの時間、`last valid ToF before stop`、直前 0.5 s の無効標本の数。モータが止まったとき機体が床の上にあったか（見ている人・ビデオ） |
| 合格 | モータ停止の瞬間に機体が床の上。最後の有効な ToF が 5 cm 以下。5 cm を超え 15 cm 以下のときは降下停滞の経路で、責任者が判断する（記録する） |
| 中止 | 接地の後もモータが止まらない（上の 12 秒で 2 回目の DISARM）、空中でモータが止まった |

### 2-5 低いホバーからの着陸と ToF のばらつき（C11）

実機で、離陸の開始から約 1 秒以内に LANDING が始まる状況（上昇中の着陸）を安全に起こす手段は無い。TAKEOFF から LANDING に入る道は、通信途絶・電池の緊急（床を離れた証拠あり）・ToF の規則・解けない床の証拠の 4 つだけである（`detailed_design.md` §3.1）。送信機の DISARM は即停止、`sf land` は TAKEOFF の間は保留、通信途絶の猶予は 3 秒、解けない床の証拠は 5 秒を要する。電池の緊急は 1h-3（机上）で行う。そこで、実機では低いホバーからの着陸で代える。

| 項目 | 内容 |
|------|------|
| 目的 | (a) 低いホバーから始めた着陸でも、5 cm を超える高さでモータが止まらない。(b) 接地の静止の判定は、床の上の ToF の読みが 1 秒間 0.02 m の幅に収まること（`kLandingRestBandM`、`kLandingRestHoldMs` 1000 ms）を静止の証拠にする。実機の静止した機体がこの幅に収まるか（過去の実機の記録では 0.022 m で、帯をわずかに超えていた） |
| 入場 | 2-4 が合格。ALT_HOLD。人員: 操縦者 + 高さを `sf tof` で読んで伝える人。**1 人では行わない**（Claude Code の読み取りは数秒遅れうるので任せない。1 章の人員の表） |
| 操作 | ARM を押し（自動離陸）、操縦者がスロットルスティックを下げて、高さ **0.10〜0.12 m** まで降ろして 2 秒以上保つ（`sf tof` で確かめる）。その 2 秒の区間の始まりと終わりの機体時刻を控える。DISARM を押す（自動着陸）。スティックでそこまで降りられるかは**未確認**（API の下限は 0.2 m。スティック操作の下限はコードに見つからなかった）。降りられなければ、その旨を記録して 2-5 を行わない |
| 読む | `--grep "Landing detected"`、`sf log stats events <一式>`、**その 2 秒の区間**（`<t0>`〜`<t1>`）で `sf log stats spread <一式> --since <t0> --until <t1>`（区間は 2 秒以上取る） |
| 記録 | 経路、`LANDING` の行の時間、`last valid ToF before stop`、`largest 1 s spread` と各窓の `spread` [cm] |
| 合格 | (a) モータ停止の最後の有効な ToF が 5 cm 以下（経路と時間を記録）。(b) 基準は無い（記録のみ）。`largest 1 s spread` が 2 cm を超えるなら責任者に報告する |
| 中止 | 2-4 と同じ |

### 2-6 離陸後の `sf stop` による高度保持（C9）

| 項目 | 内容 |
|------|------|
| 目的 | `sf stop` が、そのときの高度を保持するか。`takeoff` の待機中の `stop` は拒否される（`error not flying or guidance released - takeoff/go to re-engage`）ので、離陸が済んだ後の上昇で確かめる |
| 入場 | 2-3 が合格。人員: 操縦者（開発者）+ Claude Code。T2 に次の 1 行を用意する |
| 操作 | `sf takeoff`。「整定した」（共通の判定）後に、`sf up 50 & sleep 1 && sf stop; wait`（上昇の約 1 秒後に停止）。続けて `sf land`。`sf up 50` の目標は 1.0 m で、停止が効かなければ 1.0 m に向かう（上限 1.2 m で機体が止める） |
| 記録 | `sf stop` の応答（`ok`）、`sf up 50` の応答（打ち切られると `error ...`）、`sf log viz <一式> --mode position` で停止した高度と、その後の保持。停止高度の予想は約 0.8 m（0.5 m + 1 秒分の上昇。上昇の速さは設計値 0.3 m/s 前後で実機は未確認） |
| 合格（**暫定**） | 停止した高度で、停止後 5 秒間の高度の変化が 0.07 m 以内（根拠: 保持の帯 0.5 ± 0.07 m と同じ幅を流用）。高度が 1.2 m を超えない |
| 中止 | `sf stop` が拒否される、保持せず上昇を続けて 1.2 m の目印に近づく（中止の段 2・3） |

### 2-7 高度モードの飛行中の通信途絶（C12）

**通信途絶の間は、操縦者の DISARM が届かない。止める手段は T3 の `sf emergency` だけである。**

| 項目 | 内容 |
|------|------|
| 目的 | 高度モードの飛行中に送信機が切れたとき、着陸するか |
| 入場 | 2-3 が合格。ALT_HOLD。人員 2 人（操縦者 + `sf emergency` を打って待つ PC 担当者）。**開発者 1 人 + Claude Code では行わない**（送信機の電源を切ると人の止める手段が無くなり、Claude Code の `emergency` は数秒遅れうる。通信途絶の模擬が入るまで、2 人目がいるときだけ行う。後日追加の予定） |
| 操作 | ARM を押し（自動離陸）、0.5 m で保持。PC 担当者が「切る」の声で合図し、操縦者が送信機の電源を切る。**着陸して DISARM（モータ停止）を確かめるまで、送信機の電源を入れ直さない**（規則 12。ARM は押すたびに切り替わるので、再接続の直後にスティックに触れない）。異常なら T3 で Enter |
| 読む | `--grep "Comm lost"`、`--grep Transition`、`sf log stats events <一式>`、`sf log viz <一式> --mode attitude` |
| 記録 | 途絶から LANDING までの時間、LANDING の長さ、接地、モータ停止 |
| 合格 | 約 3.5 秒で LANDING になり、水平に降り、接地で止まる。空中でモータが止まらない |
| 中止 | 着陸しない、傾きが増える（T3 の `sf emergency`）。中止の段 3・4 |

### 2-8 手動で浮いた後の通信途絶（C8・C12）

| 項目 | 内容 |
|------|------|
| 目的 | 手動で浮いて TAKEOFF/FLYING になった後に通信が途絶えたとき、モータを切らずに着陸するか（切ると落下する） |
| 入場 | 2-1 と 2-7 が合格。STABILIZE。床はマット。人員 2 人（操縦者 + `sf emergency` を待つ PC 担当者）。**開発者 1 人 + Claude Code では行わない**（2-7 と同じ理由）。操縦者の手は、片手でスティックを持ったまま、もう片方の手で送信機の電源スイッチを切る（**スティックを持ったまま電源を切る手順は両手が要る**。片手で保てなければ、PC 担当者に高さの保持を頼まず、この手順を行わない） |
| 操作 | 高さ約 0.3 m で保ち、合図の後に送信機の電源を切る。以後は 2-7 と同じ（規則 12） |
| 読む | `--grep "Comm lost"`、`--grep Transition`、`sf log stats events <一式>` |
| 記録 | 途絶から LANDING までの時間（約 3.5 秒が設計値）、LANDING の間の duty、接地、モータ停止 |
| 合格 | LANDING になる。モータは接地まで切れない。着陸の姿勢が水平 |
| 中止 | 傾きが大きい、接地しない（T3 の `sf emergency`）。中止の段 4 |

### 2-9 回頭: 90 度 × 4、360 度、反対向き 270 度（C13）

`cw`・`ccw` は 1〜360 度のどの角度も、指令した向きに指令した角度だけ回る。完了の待ち時間は「角度 ÷ 1.0 rad/s（約 57.3 度/s）+ 6 秒」。SILS は回頭と高度・位置の結合を持たないので、回頭中の高度の低下と水平位置のずれの実機の値は**未知**であり、この飛行が初めて測る。2-9 と 2-10 は別の飛行にする。

| 項目 | 内容 |
|------|------|
| 目的 | 回頭が指令どおりの向きと角度か（大きい角度で逆回りしない、回らずに応答しない）。回頭中に高度が下がらず、水平位置がずれないか |
| 入場 | 2-3・2-6・2-7 が合格。人員: 操縦者（開発者）+ Claude Code。POS_HOLD |
| 操作 | `sf takeoff`。「整定した」の後、各回の前に 3 秒待って、次の順: `time sf cw 90` を 4 回（元の向きに戻る）、`time sf cw 360`、`time sf ccw 270`。続けて `sf land` |
| 読む | 回頭ごとに `ok` までの時間（設計値は 角度 ÷ 57.3 度/s 付近）。区間（その回の `cmd: cw ...` から次の命令まで）で `sf log stats turn <一式> --since <t0> --until <t1>` |
| 記録 | `yaw change`（指令との差が向きの誤差）、`ToF height` の最小（0.5 m との差が高度の低下）、`horizontal drift`（推定器による水平位置のずれ。実際のずれは床の目印やビデオでも確かめる）、`ok` までの時間 |
| 合格（**暫定**） | `yaw change` が指令の向きと角度（時計回りが正）に一致する。仮の線: **角度の誤差 10 度以内**、**最低の ToF の高さ 0.30 m 以上**（保持から 0.2 m 以内の低下）、**水平位置のずれ 0.5 m 以内**（根拠: 本書の安全側の仮の線。ずれ 0.5 m は 2 m の区域の中央から壁まで 1 m の半分）。超えたら責任者へ報告する。測った値はそのまま記録する |
| 中止 | ToF の高さが 0.30 m を下回る、または区域の中央から 0.5 m を超えてずれる、または傾きが大きくなる。直ちに `sf stop`、止まらなければ中止の段 2・3 |

### 2-10 回頭の途中の `sf stop`（C13）

| 項目 | 内容 |
|------|------|
| 目的 | 回頭の途中で `sf stop` を送ると、その向きで止まり、元の向きへ戻ったり回り続けたりしないか |
| 入場 | 2-9 が合格。人員: 操縦者（開発者）+ Claude Code。POS_HOLD。T2 に次の 1 行を用意する |
| 操作 | `sf takeoff`。「整定した」の後、`sf cw 360 & sleep 3 && sf stop; wait`（約 3 秒後 = 360 度の約半分で停止）。続けて `sf land` |
| 読む | `sf log info <一式> --text --grep "cmd: stop"` の時刻 `<ts>`。`sf log stats turn <一式> --since <ts> --until <ts + 5>` |
| 記録 | `sf stop` の応答（`ok`）と `sf cw 360` の応答（`error interrupted by stop`）。停止後 5 秒間の `yaw change` |
| 合格（**暫定**） | 停止後 5 秒間の `yaw change` が 10 度以内（根拠: 2-9 と同じ仮の線）。応答が上のとおり |
| 中止 | 2-9 と同じ |

**次へ進む条件**: 2-1〜2-10 の全てが合格（または責任者の承認つき）。

## 7. 段階 3: 総合飛行

**入場条件**: **段階 2 の「次へ進む条件」を満たし**（全手順が合格、または責任者の承認つき。承認のあるものは結果表の備考に書いてある）、結果表が埋まっている。段階 2 の入場の表（6 項目）を全部確かめ直す。同じ区域・同じ人員（開発者 1 人 + Claude Code）・満充電の電池。この飛行は**1 回だけ**行い、目的は、離陸から着陸までを通したときに、部品ごとの確認では現れない異常が無いことを確かめること（C16）。**中止**: 2 章の「中止の段」「段階ごとの中止条件」。

| 順 | 操作 | 確認 |
|----|------|------|
| 前 | 段階 2 の「共通の手順（前）」。`sf log wifi -d 150 -o logs/checkout/s3_final_1.sflog.zip`。2 人目がいるときは、T3 に `sf emergency` | — |
| 1 | `sf takeoff` | 応答が返る |
| 2 | **離陸直後に高度と下向き ToF を確かめる**（規則 5）。操縦者は 1.0 m の目印と LED を見る。Claude Code が `sf height`・`sf tof` を 1 回ずつ読む（応答は数秒遅れうるので、判断は操縦者の目で行い、値は記録とする） | 高度が 0.5 m 付近、ToF が正常。**どちらかが異常なら、ここで `sf land`（または DISARM）して終える** |
| 3 | 1 回転: `sf cw 90` を 4 回（2-9 が合格していれば `sf cw 360` 1 回でもよい）。各回の前に高度を確かめる | 各 `sf cw` が成功で返る |
| 4 | `sf land` | 応答が返り、接地でモータが止まる |
| 後 | DISARM を確かめ、取得の終了を待つ。電源を切る前に WiFi の CLI で `status` を取る | — |

**取得した一式から読むもの**（`<一式>` は `logs/checkout/s3_final_1.sflog.zip`）。3 章の「飛行後の共通の確認」（C1・C2・C15）に加えて:

| 読むもの | コマンド | 合格 |
|----------|----------|------|
| ログの文字: エラー・警告 | `sf log info <一式> --text --grep "E ("` と `--grep "W ("` | エラー（`E (`）が無い。警告（`W (`）は、`Downward ToF`・`Comm lost`・`Battery`・`Impact`・`ESKF diverged`・`ARM rejected`・`log sink full`・`set_float` の種類が無く、残る行は 1 行ずつ原因を説明できる |
| 状態遷移 | `--text --grep Transition` | `IDLE_GROUND → ARMED_GROUND → TAKEOFF → FLYING → LANDING → IDLE_GROUND` の順だけ |
| 接地・モータ停止 | `--text --grep "Landing detected"`、`sf log stats events <一式>` | 接地の行が 1 行。文言と、モータ停止までの時間を記録 |
| ループ・ログ経路・ToF 出力停止 | `sf log stats loop <一式>` | `late` は 1d・1e の取得ありの値と同程度（暫定）。`log paths` が 4 つとも 0。`text log` の欠落が 0。`ToF output stopped` が `tof_silent=1 in 0 of ...` |
| ToF 診断値 | `Diagnostics` の `ToF bottom:` | 記録のみ |
| 高度と向きの軌跡 | `sf log viz <一式> --mode position --save logs/checkout/s3_position.png`、`--mode attitude`、`sf log analyze <一式>`、回頭の区間で `sf log stats turn` | 高度が 1.0 m の目印を超えない（ビデオ・グラフで確認）。保持高度が 0.5 m 付近。向きの変化の合計が時計回り 360 度（誤差 10 度以内。暫定）。着陸の後に再び浮かない |
| 再起動 | `sf log stats boot <一式>` | `reset reasons` が `1 POWERON` か `3 SW` だけ |

## 8. 結果表と取りまとめ（段階 4）

### 取りまとめの手順

| 順 | 操作 |
|----|------|
| 4-1 | 変更したパラメータが戻っているか: `param get safety.battery.low_v`（`= 3.4 (float)`）、`param get flip.enable`（`= 0 (int)`） |
| 4-2 | `logs/checkout/` の一式とコンソールの記録を保存する。全ての一式で `sf log check <一式>` がエラー 0。コミットするかは責任者が決める（`logs/` は Git の対象外） |
| 4-3 | 結果表を埋める。測れなかった項目は「未実施」「未確認」と書く |
| 4-4 | 不合格・保留の項目は、原因と次の対応を備考に書く |
| 4-5 | 電池を外し、充電状態を記録する |

### 結果表

試験したコミット（ID）: ____________　ビルド日時（`version`）: ____________　実施者: ____________

実施 1 回が 1 行。同じ手順を繰り返したら「回」を増やして行を足す。「開始電圧」は飛行・ARM の前の `battery` [V]。「合否」は「合」「否」「未実施」「未確認」のどれか。括弧は確認項目の ID。

| 手順（項目） | 回 | 開始電圧 | 基準 | 測定値 | 合否 | 一式／記録のファイル名 | 備考 |
|--------------|----|---------|------|--------|------|------------------------|------|
| 0-1 | 1 | — | プロペラ 4 枚が外れている | | | | |
| 0-2 | 1 | — | 差分なし | コミット ID: | | | |
| 0-3〜0-4 | 1 | — | `sf doctor` エラーなし、ビルド成功 | | | | |
| 0-5〜0-6 | 1 | — | 起動する | `version`: | | | |
| 0-7 | 1 | — | 4 つの設定値が表と一致 | | | | |
| 0-8〜0-9 | 1 | — | `mode : ap`、パケットが届く | | | | |
| 0-10〜0-11 | 1 | | ペアリング済み、「飛行可能」の電圧 | | | | |
| 0-12 | 1 | — | 4 つのモータの位置が一致 | 回転の向き: | | | |
| 1a（C1・C2） | 1 | | `all` の `late` が 0 | | | | |
| 1b 表面 1（C3） | | | `valid above 5 cm` が 0 | min / max / valid 数 / invalid 数: | | | |
| 1b 表面 2（C3） | | | 同上 | | | | |
| 1b 診断値（C3） | | | 記録のみ | signal / ambient / sigma: | — | | |
| 1c（C14） | 1 | | 0 付近・`valid`・オフセット未学習 | | | | |
| 1d 条件 A（C1・C2・C4） | 1 | | `period max` < 10 ms、`busy max` < 2.5 ms、`over budget` 0、N > 0、`log sink full` なし | | | | |
| 1d 条件 B（C1・C2・C4） | 1 | | 同上。`late`/`cycles` が A より増えない（暫定） | | | | |
| 1d 条件 C（C1・C2） | 1 | | 同上 | | | | |
| 1e 取得なし（C1・C4） | 1〜3 | | 幅（最小・最大）を記録 | | — | | |
| 1e 取得あり（C1・C4） | 1〜3 | | 取得なしの幅の中（暫定）。`text_dropped_lines` 0 | | | | |
| 1f（C5） | 1 | | 警告は TCP 側だけ。ARM 中の `reboot` は拒否 | | | | |
| 1g-1（C8） | 1 | | 5 cm 超 1 秒で TAKEOFF | | | | |
| 1g-2（C8・C12） | 1 | | 約 3.5 秒で LANDING、モータは切れない | | | | |
| 1g-3（C11） | 1 | | `Landing detected`、モータ停止 | | | | |
| 1g-4（C10） | 1 | | LANDING を経由せず停止 | | | | |
| 1h-1（C7） | 1 | | 通知と約 5 秒の再発報。復元した | | | | |
| 1h-2 床・ARMED_GROUND（C7） | 1 | | ログのみ、モータは切れない | | | | |
| 1h-2 TAKEOFF・証拠なし（C7） | 1 | | モータ停止 | | | | |
| 1h-2 TAKEOFF・証拠あり（C7） | 1 | | LANDING、モータは切れない | | | | |
| 1h-2 FLYING（C7） | 1 | | LANDING、モータは切れない | | | | |
| 1h-2 IDLE_GROUND（C7） | 1 | | ログのみ。ARM 拒否 | | | | |
| 1h-3（C11） | 1 | | 5 cm を超える間は止まらない | | | | |
| 1i（C6） | 1 | | panic なし、`reset reasons` が 1・3 のみ | | | | |
| 1i（C15） | 1 | | `re-init` の行が 0 | | | | |
| 2-1（C8） | 1〜2 | | 0.5 以下で浮いて、5 cm 超 1 秒の後に TAKEOFF | | | | |
| 2-2（C10） | 1 | | LANDING を経由せず停止 | | | | |
| 2-3（C9・C11・C14） | 1 | | 最高到達 ≤ 1.0 m（暫定）。`sf land` 成功 | 最高 / 整定 / 経路 / 時間: | | | |
| 2-4 表面 1（C11） | 1〜3 | | 最後の有効 ToF ≤ 5 cm | 経路 / 時間: | | | |
| 2-4 表面 2（C11） | 1〜3 | | 同上 | | | | |
| 2-5（C11） | 1 | | 最後の有効 ToF ≤ 5 cm。ばらつきは記録のみ | 経路 / 時間 / 最大ばらつき: | | | |
| 2-6（C9） | 1 | | 停止後 5 秒の高度変化 ≤ 0.07 m（暫定） | 停止高度: | | | |
| 2-7（C12） | 1 | | 約 3.5 秒で LANDING、空中で止まらない | | | | |
| 2-8（C8・C12） | 1 | | LANDING、モータは切れない | | | | |
| 2-9 90 度 × 4（C13） | 1 | | 向きが戻る。誤差・低下・ずれを記録（暫定 10 度・0.30 m・0.5 m） | 誤差 / 最低 ToF / ずれ: | | | |
| 2-9 360 度（C13） | 1 | | 時計回りに 360 度 | 所要 / 誤差 / 最低 ToF / ずれ: | | | |
| 2-9 反時計回り 270 度（C13） | 1 | | 反時計回りに 270 度 | 所要 / 誤差 / 最低 ToF / ずれ: | | | |
| 2-10（C13） | 1 | | 停止後 5 秒の `yaw change` ≤ 10 度（暫定） | | | | |
| 段階 2・3 の各飛行後（C1・C2・C15） | 飛行ごと | | 3 章の共通の確認 | | | | |
| 3（C16） | 1 | | 7 章の表のとおり | | | | |
| 4-1〜4-5 | 1 | — | パラメータが戻っている、`sf log check` エラー 0 | | | | |

## 9. 付録: コマンドと出力の形の確認元

| コマンド・出力 | 形 | 確認元（リポジトリ内） |
|---------------|----|------------------------|
| `status` の各行 | `state   :`、`armed   :`、`pairing :`、`battery : %.2f V, %.0f mA`、`prearm  :`、`tof     : SILENT - ...`、`loop    :`、`imu/control all/armed`、`finite  :` | `firmware/vehicle/tasks/cli_task.cpp` 283-402 行 |
| `sensor tof`・`sensor baro`・`sensor power`、`param get/set/save`、`version`、`wifi show`、`reboot`、TCP CLI のバナー | 各行の形 | 同上 80-105、136、192、472-514、1168、1268-1272、1321 行。範囲外の警告は `components/sf_core/params.cpp` 1130 行 |
| `log sink full` | `W (<ms>) LogTask: log sink full: <N> lines dropped` | `components/sf_core/log_sink.cpp` 143 行 |
| 電池の通知 | `Low battery: %.2fV`、`Critical battery: %.2fV`、`Low battery warning`、`Battery emergency ...`、`ARM rejected: battery ...`。警告の再発報は約 5 秒 | `components/sf_failsafe/failsafe.cpp` 283-287 行、`battery_alert_policy.hpp` 67 行、`components/sf_state/state_manager.cpp` 203・775-797 行 |
| 状態遷移・DISARM・通信途絶 | `Transition: A → B`、`DISARM accepted (...)`、`Comm lost → hover 3000 ms then LANDING`、`Comm lost → LANDING (hover grace elapsed)`、`Takeoff complete → FLYING/...` | `components/sf_state/state_manager.cpp` 331・367・750・1054・1167 行 |
| 接地判定 | `Landing detected (...)` | `components/sf_takeoff_landing/takeoff_landing.cpp` 327・331 行 |
| ToF の再初期化 | `VL53L3CX bottom re-init: attempt %u (no sample for %u ms)`、`... succeeded ...`、`... failed ...: <エラー名>`。定数 2 s・10 s・1 s・6 | `tasks/tof_task.cpp` 330-347 行、`components/sf_state/include/tof_silence_policy.hpp` 309 行、`components/sf_core/include/flight_limits.hpp` 196-199 行 |
| ARM の操作・モード | ARM はスティックの押し込みのトグル、`sf takeoff` は自分で ARM して POS_HOLD | `tasks/state_task.cpp` 791-803・910 行、`tasks/api_task.cpp` 543-551 行、`docs/guides/controller.md` 9 章 |
| DISARM の効き方・LED・音 | 段 1〜3 の中止の段、LED と音の表 | `docs/operation_manual.md` 53-103・136-140・210 行 |
| モータ単体試験 | `motor test <1-4> <0-100> [秒]`、M1 = FR、M2 = RR、M3 = RL、M4 = FL | `docs/operation_manual.md` 322 行 |
| 閾値 | `kTakeoffGroundBandM` 0.05 m、`kTakeoffAirborneBandM` 0.15 m、`kAltitudeCeilingM` 1.2 m、`kAutoTakeoffTargetAltitudeM` 0.5 m、`kLandingRestBandM` 0.02 m、`kLandingRestHoldMs` 1000 ms、`kAutoTakeoffNoLiftAbortUs` 5 s、通信途絶の猶予 3 s、`critical_battery_v` 3.0 V、`low_battery_v` 3.4 V、`TAKEOFF_THROTTLE_THRESH` 0.5 | `flight_limits.hpp` 55・72・291-296・326-377 行、`state_manager.cpp` 49 行、`failsafe.hpp` 38-39 行、`main/config.hpp` 278 行 |
| 回頭・移動の待ち時間 | 回頭 = 角度 ÷ 57.3 + 6 s（+ 3 s の通信余裕）、移動 = 距離 ÷ 10 cm/s + 6 s（+ 3 s）、離陸 15 s、着陸 35 s | `components/sf_command/include/rotate_guidance.hpp` 98 行、`lib/sfcli/utils/vehicle_api.py` 59-122 行 |
| `sf` の飛行・問い合わせ・ログのコマンド | `--ip`、`--timeout`、終了コード 0/1/3/130、`emergency`・`stop` は手順なしで送る（呼び出しごとに別のソケット） | `lib/sfcli/commands/flight.py`、`query.py`、`log.py`、`lib/sfcli/utils/vehicle_api.py` 160-200 行、`docs/commands/sf-log.md` |
| API の応答 | `error not flying or guidance released ...`、`error interrupted by stop`、`cmd: <行>` | `tasks/api_task.cpp` 453・795・1930 行 |
| 飛行ログの列 | `flight_phase`、`motor`、`attitude`、`posvel`、`tof_bottom`、`baro_height`、`status`、`loop_timing`、`nonfinite`、`link_health`、`tof_silence`、`text_log`、`pilot` | `protocol/spec/flight_log.yaml` |
| 電池の目安 | 満充電 4.2 V、通常 3.7〜4.0 V 飛行可能、3.5〜3.7 V 着陸推奨 | `docs/guides/safety.md` 4 章 |

**確認できなかったもの**: panic 表示の実際の文面、電池の接続部へ電源装置をつなぐ方法、`sf takeoff` 系のコマンドと `kill -STOP` による USB の滞りの実機での動作、`E (`・`W (` による絞り込みの実機での形、ALT_HOLD のスティックで 0.10 m まで降りられるか、下向き ToF が自然に止まったときの実機での再初期化の所要時間、送信機の再接続の所要時間と再接続直後のモード指定の挙動、API が切り替えたモードと送信機のモード指定の優先、時計回り用・反時計回り用プロペラの取付位置の対応、USB を先に接続する順序の要否、`pilot.csv` の `throttle` と 0.5 の閾値の尺度の対応、ToF の範囲外の上限値、`sf log stats` の出力の実機の一式での形（SILS と過去の実機ログでは確認済み）。

---

<a id="english"></a>

This is a manual procedure for verifying, on the real vehicle, that the vehicle firmware performs its flight-safety functions as designed: the 400 Hz loop timing, the monitor for non-finite numbers, the takeoff and landing decisions, the battery and communication-loss actions, turning, and the observation of barometric height. It moves from a thrust-free bench check to low-altitude flight in order of increasing risk, and closes with one integrated flight.

The **owner** is the repository's owner (the control and hardware engineer), who makes the final decision on the pass or fail of this procedure and on whether to go on with an item that failed. Stage N is **chapter N + 4** (stage 0 = chapter 4 ... stage 4 = chapter 8).

## 1. Sequence at a Glance

**One page that shows the whole procedure.** The times are estimates (not measured). Two or more batteries are assumed (stage 2 alone includes several battery changes). **Stage 1 and stage 2 will each take most of a day, so they may be split over different days** (when splitting, fill in the results sheet at the end of stage 1).

| Stage | Purpose | Estimated time | Propellers |
|-------|---------|----------------|-----------|
| 0 Preparation | Remove the propellers, fix the firmware under test, build and flash, check settings, transmitter, battery and motor positions | 1-1.5 h | **Removed** |
| 1 Bench | Without thrust, check loop timing, finiteness, floor ToF, logging, WiFi CLI, takeoff decision, communication loss, battery, and whether the ToF re-initialisation ever acts (1a-1i) | 4-6 h | **Removed** |
| 2 Low free flight | Ten kinds of flight in order of increasing risk (steps 2-1 to 2-10; 2-4 is repeated three or more times per surface). One purpose per flight | 4-6 h (battery changes included) | Fitted |
| 3 Integrated flight | Takeoff, one full turn and landing in one flight (C16), after everything in stage 2 passed | 30-45 min | Fitted |
| 4 Wrap-up | Collect records, restore parameters, results sheet | 30 min | — |

### People and terminals

The hardware checkout is done by **one developer plus Claude Code (the PC-side operations)** (decided 2026-11-22; there is no second person). **Only the person has a way to stop the motors**: Claude Code's reply can be several seconds late, so the emergency stop is not left to Claude Code.

| Kind | Person (developer) | Claude Code |
|------|--------------------|-------------|
| Vehicle, transmitter, battery, propellers | Every operation that touches them (ARM and DISARM on the transmitter, connecting the battery, fitting the propellers) | Does not touch them |
| Stopping the motors | Pressing the throttle-side stick of the transmitter, the button on the vehicle, removing the battery. Every stop in an abnormal situation is one of these | Not left to it. It sends `emergency` (the CLI command or `sf emergency`) when asked, but it can be several seconds late, so it is not counted as a way to stop |
| Input to the USB console | — | May send it (`motor test`, which turns the motors, included). Before sending, it says by voice (macOS `say`) what will happen |
| `arm`, `disarm`, `mode` of the CLI (USB and WiFi) | — | May send them (the developer's approval, 2026-11-22). Before sending, it says by voice what will happen |
| WiFi capture (`sf log wifi`) and the WiFi CLI (`nc 192.168.10.1 23`) | — | May run them |
| Flight commands (`sf takeoff`, `sf cw` etc.) | Holds the transmitter and is ready to DISARM | Up to starting the capture and sending the commands. Before the flight it tells the purpose, the order of operations and the abort rungs in full, and sends on the person's signal |
| Reading the saved records and comparing them with the pass lines | — | Does it |

The division per step, and whether one person (the developer alone plus Claude Code) may do it:

| Steps | What the person does | What Claude Code does | Done by one person |
|-------|----------------------|-----------------------|--------------------|
| 0, 1a-1f, 1h-1, 1i | Operations on the vehicle, transmitter and battery; checks by eye | USB and WiFi input, the capture, the comparison (the CLI commands of 0-13 included) | Yes |
| 1g | Lifts the vehicle with one hand and holds the transmitter with the other (a press of the stick stops it at once). **Removing the propellers is recommended.** Whether to keep them fitted is the owner's decision | Starts the capture, reads `status` | Yes |
| 1h-2, 1h-3 | Operates the voltage of the power supply and holds the vehicle | Starts the capture | **Optional, as before.** A power supply is needed. Some scenes need holding the vehicle and operating the voltage at the same time, so the owner decides whether one person does them. If not done: "not done". Later, parameterising the critical-voltage threshold is planned (so that it can be done without a power supply) |
| 2-1, 2-2, 2-4 | Holds the transmitter as the pilot | Starts the capture before the flight | Yes |
| 2-3, 2-6, 2-9, 2-10, 3 | Holds the transmitter as the pilot and is ready to DISARM. **The person stops it** | Starts the capture and sends the flight commands (`sf takeoff` etc.). The emergency stop is not left to it | Yes |
| 2-5 | The pilot + a person who reads the height with `sf tof` and tells it | — | **No** (reading the height cannot tolerate a delay of several seconds, so it is not left to Claude Code. Done only when a second person is present) |
| 2-7, 2-8 | The pilot who switches the transmitter off + a person waiting with `sf emergency` ready to send with only Enter | — | **Do not do alone** (once the transmitter is off, the pilot's DISARM does not arrive and the person has no way to stop; Claude Code's `emergency` can be several seconds late and is no substitute. Until a simulation of communication loss is added, these two steps are done only when a second person is present. Planned for later) |

The PC can be used while connected to the Internet (wired LAN, the USB tethering of an iPhone, etc.) with its WiFi connected to the vehicle at the same time (confirmed on the real vehicle on 2026-11-22; chapter 3, "How to Record", has a macOS note).

The PC-side terminals (flight stages; all on the same PC connected to the vehicle's WiFi; used by Claude Code. The "PC operator" is the person who takes this role when a second person is present):

| Terminal | What it holds |
|----------|---------------|
| T1 | The capture (`sf log wifi -d <seconds> -o ...`), running before the flight |
| T2 | The flight commands (`sf takeoff` etc.). Wait for the reply before typing the next |
| T3 | `sf emergency` (the last resort that stops the motors at once; `emergency` is accepted even without a `command`: `operation_manual.md` line 210). Every `sf` call opens its own socket (`send` and `send_urgent` in `lib/sfcli/utils/vehicle_api.py`), so T3 can send while a command in T2 is waiting. **When a second person is present, that person waits with only Enter left to press. With the developer alone, T3 is not waited on, since the hands are busy with the transmitter** (the way to stop is the person's own action) |

Bench stage (stage 1): T1 = USB console (`script -q ... sf monitor`), T2 = capture or WiFi CLI (`nc 192.168.10.1 23`).

### Operation map

**Transmitter (controller)**: when powered on it reconnects to the last vehicle automatically (`docs/guides/controller.md` chapter 3).

| Operation | Transmitter | Check |
|-----------|-------------|-------|
| ARM / DISARM | **Press the throttle-side stick in. Each press toggles between ARM and DISARM** (a momentary button, not a switch. The throttle side is the left stick (stick Mode 2) or the right stick (Mode 3); `controller.md` chapter 9, `state_task.cpp` lines 791-803). Pressing while the vehicle is ARMed means DISARM; pressing before ARM means ARM | `armed` in `status`, the StampS3 LED (ARMED_GROUND = slow green blink) |
| STABILIZE (attitude stabilization; the default) | Press the left button to set the altitude mode OFF, and keep the right button's control mode at STABILIZE | The body LED is steady yellow-green |
| ACRO (angular-rate control) | Press the right button to toggle STABILIZE <-> ACRO (while the altitude mode is OFF) | The body LED is steady blue |
| ALT_HOLD (altitude hold) | Press the left button to change the altitude mode from OFF to ALT HOLD | The body LED is steady orange |
| POS_HOLD (position hold) | Press the left button one more time (the order is OFF -> ALT -> POS -> OFF) | The body LED is steady magenta |

The priority is POS_HOLD > ALT_HOLD > ACRO > STABILIZE (`operation_manual.md` lines 151-156). The mode can be changed on the ground and while ARMed (same, line 71). **`sf takeoff` ARMs the vehicle by itself, switches to POS_HOLD and takes off** (`api_task.cpp` lines 543-551, `state_task.cpp` line 910). In an API flight nobody presses ARM or the mode buttons on the transmitter (the transmitter is powered on and held with the sticks neutral). Which one wins when the mode the API switched to and the mode given by the transmitter disagree is **unconfirmed**, so do not touch the transmitter's buttons during an API flight.

### Terms to know first

| Term | Meaning |
|------|---------|
| Pilot | The person who holds the transmitter and is ready to press DISARM. One is named in every stage |
| ARM / DISARM | ARM puts the vehicle in a state where the motors can spin. DISARM stops the motors and returns to the state where they cannot spin |
| State (`state`) | IDLE_GROUND (on the ground, waiting), ARMED_GROUND (ARMed, on the ground), TAKEOFF (takeoff sequence), FLYING (in flight), LANDING (landing sequence) |
| Spool-up | The short time (about 0.3 s) after ARM in which the motors start and their speed rises |
| Bundle (`.sflog.zip`) | One file of a flight log captured with `sf log wifi`: one CSV per signal collected in one zip. It holds the vehicle's log text (`text_log`) and diagnostics (`loop_timing`, `nonfinite`, `link_health`, `tof_diagnostics`) |
| ESKF | Error-state Kalman filter: the estimator of the vehicle's attitude, position and velocity. The position in `posvel` is the ESKF's estimate |
| 400 Hz loop | The work that ImuTask (sensor reading and attitude estimation) and ControlTask (control computation) repeat every 2.5 ms |
| `late`, `period max`, `busy max`, `over budget` | `late` = the number of wakes later than 1.5 periods (3.75 ms) after the previous one. `period max` = the longest interval between consecutive wakes. `busy max` = the longest processing time of one cycle. `over budget` = the number of cycles whose work exceeded 2.5 ms. `status` shows ms and the bundle's diagnostics show µs (10 ms = 10000 µs, 2.5 ms = 2500 µs) |
| finite / `nonfinite` | Values of the control computation are not NaN (not a number) or infinity. The `finite` line of `status` and the `nonfinite` of the bundle count the ARMed cycles that held a non-finite value |
| Log sink | A mechanism that puts log text into a ring buffer (a memory area reused in a circle) and lets only another task write it to the console or to WiFi, so that the 400 Hz tasks are not made to wait by the output |
| CLI | An entry point for typing commands. Two routes: USB (`sf monitor`) and WiFi (TCP port 23) |
| API | Text commands (Tello style) that the vehicle accepts on UDP 8889. `sf takeoff` and the like send them |
| Mcps, sigma | ToF diagnostic values. Mcps, the unit of `signal` and `ambient`, is million counts per second (received light); `sigma` is the distance scatter [mm] |
| SILS | Software-In-the-Loop Simulation: running the control software against a vehicle model on a computer instead of the real vehicle |

**ToF states** (the downward ToF = Time of Flight, a laser distance sensor under the vehicle that measures the height above the floor):

| Name | Meaning | Where it shows |
|------|---------|----------------|
| Invalid | A sample arrives but no distance could be measured (`status=254`, distance 0). It happens in the dead zone closer than about 3.1 cm. Normal on a floor | `valid=no` of `sensor tof`, `status` of `tof_bottom.csv` |
| Dead zone | The range closer than about 3.1 cm where it cannot measure. The main cause of "invalid" | Same |
| Silent (no new sample) | No new sample for 0.3 s or more, or only invalid samples in the air | `tof     : SILENT` in `status`, yellow fast blinking, `ARM rejected: downward ToF is silent` |
| Out of range | Too far to measure (the upper limit value is unconfirmed in this document) | Value in `status` |

**LEDs and sounds** (`operation_manual.md` lines 53-103. StampS3 LED = the LED on the board, body LED = the vehicle's two LED channels. Priority from the top):

| State | Color and pattern | Sound |
|-------|-------------------|-------|
| Low battery (`safety.battery.low_v` or below) | Slow cyan blink (the body LED blinks fast) | Low-battery warning (repeated) |
| Downward ToF silent while ARMed | **Fast yellow blink** (both LEDs) | Warning sound |
| Pairing | Fast blue blink | — |
| Reason ARM is refused (on the ground) | Slow red blink = tilt mismatch (set it down again), slow magenta blink = calibrating or waiting (keep still), fast yellow blink = ToF silent | The error sound (long, low) once |
| IDLE_GROUND / ARMED_GROUND / TAKEOFF / FLYING / LANDING | Steady green / slow green blink / fast white blink / steady green / slow orange blink (StampS3 LED) | ARM: two rising tones, DISARM: two falling tones, calibration done: three C5 beeps |
| Critical hardware fault | Fast red blink | Error sound |

## 2. Safety Rules

**Read this chapter first.** It applies to every stage that follows.

### Rules for All Stages

| # | Rule | Reason |
|---|------|--------|
| 1 | Everyone wears eye protection whenever a motor can spin | Keeps propeller or debris away from the eyes |
| 2 | Never put your face or hands above the vehicle or into the propeller plane | Prevents finger injuries |
| 3 | Name one pilot. **In every stage the pilot stays able to press the throttle-side stick of the transmitter (to DISARM).** The same applies when flying with `sf` commands. Claude Code operates the PC and records (the crew table in chapter 1) | The transmitter DISARM is the fastest and most reliable way to stop the motors in an abnormal situation |
| 4 | **Before flying, everyone reads the abort ladder below (the "Abort ladder") and says it aloud** | So nobody hesitates in an abnormal situation |
| 5 | After takeoff, first check the altitude and the downward ToF. If either is abnormal, land immediately.<br>- Abnormal altitude: judged by eye against the height marks (1.0 m and 1.2 m) on a wall or pole: above the 1.0 m mark, still climbing, or sinking steadily.<br>- Abnormal ToF: the body LED and the StampS3 LED blink yellow fast and a warning sound plays (silent).<br>- In ACRO and STABILIZE the firmware does not force a landing, so the pilot lands the vehicle | Avoids flying on when the height sensor cannot be trusted. A wrong altitude leads straight to a ceiling or floor collision |
| 6 | Fly only where the ceiling is at least 2.0 m high (the 1.2 m altitude limit plus a margin of at least 0.8 m) | Avoids hitting the ceiling if the climb overshoots |
| 7 | The flight area is at least 2 m x 2 m and free of obstacles; stages 2 and 3 are netted off and nobody enters the area (`docs/guides/safety.md`). Place the vehicle at the center. **Put height marks at 1.0 m and 1.2 m on a wall or pole** (to judge by eye) | Protects people and the vehicle if the vehicle strays. Lets the judgement be made by eye |
| 8 | Charge the battery with the dedicated charger and never leave it unattended while charging. Avoid heat and direct sunlight. Do not use a swollen or damaged battery. After an accident, DISARM (or `sf emergency`) first, then remove the battery (`safety.md`) | Avoids heating and fire of the lithium battery |
| 9 | The vehicle refuses `reboot`, `param save`, `wifi ssid` and `wifi pass` while ARMed (`reboot` because a restart stops the motors, the other three because a flash erase stalls the 400 Hz loop). Still, do not rely on the refusal: do not type them while ARMed | A restart or a control stall in flight is a crash |
| 10 | **One flight has one purpose.** Record every flight and write the result in the sheet before the next one | Lets you identify the flight that caused an abnormality |
| 11 | When moving to the next stage with an item that did not pass, the owner writes the decision and its reason in the notes column of the results sheet | Prevents flying with an unresolved item that nobody noticed |
| 12 | **When powering the transmitter on again, do not touch the throttle-side stick.** Power it on only after confirming that the vehicle has landed and DISARMed (or the motors have stopped) | ARM / DISARM toggles with every press, so pressing right after the reconnection reverses the state. The time to reconnect and the mode behavior right after the reconnection are **unconfirmed** (`controller.md` chapter 3 only says it reconnects automatically) |

### Abort ladder (when something abnormal appears in flight; every flight step of stages 2 and 3 refers here)

| Rung | Situation | Action | How it acts |
|------|-----------|--------|-------------|
| 1 | ACRO or STABILIZE, in flight or on the ground, and TAKEOFF in any mode | Press the throttle-side stick of the transmitter (DISARM) | **The motors stop at once** (`operation_manual.md` line 136). Pressed in the air, the vehicle falls |
| 2 | FLYING in ALT_HOLD or POS_HOLD (after `sf takeoff` too) | Press the stick once (DISARM) | **Automatic landing** (a slow descent of about 0.3 m/s). The motors are not cut (same, line 137) |
| 3 | The automatic landing of rung 2 does not start or does not descend (provisional: the height has not decreased 3 s after the press), or the tilt or the oscillation grows | A second press. If it does not act, remove the battery (when a second person is present, Enter on `sf emergency` in T3; a request to Claude Code for `sf emergency` can be several seconds late, so do not count on it) | The second DISARM stops the motors at once (same, line 137). `sf emergency` stops them regardless of the transmitter (same, line 210). The vehicle falls from about 0.5 m |
| 4 | During a test that switches the transmitter off (1g-2, 2-7, 2-8) | **Type `sf emergency` into T3 before switching the transmitter off, and have another person ready to press Enter** (2-7 and 2-8 are done only when a second person is present). 1g-2 is done with the vehicle in the hand and with the propellers removed (the recommendation of chapter 1); on an abnormality remove the battery | During the communication loss the pilot's DISARM does not arrive. `sf emergency` is the only way to stop |

**Visual abort criteria** (provisional; judged by eye against the marks on the wall or pole):

| Abnormality | Criterion | Basis |
|-------------|-----------|-------|
| Height | In a flight with a 0.5 m target: above the 1.0 m mark. In any flight: above the 1.2 m mark | The altitude limit of 1.2 m (`flight_limits.hpp` line 55). The takeoff overshoot is about 0.84 m in SILS (around `operation_manual.md` line 216), and 1.0 m is a line above it |
| Tilt | The body is tilted more than about 15 degrees from level (to the eye, the propeller plane clearly leans against the floor) | The pass line of SILS (tilt limit 0.2618 rad = 15 degrees in `api_rotate_direction.expect`). A real-vehicle criterion is not in a design document |
| Oscillation | The height or the horizontal position swings more than about 0.15 m from the hold target, or the swing width grows for 3 cycles in a row | About twice the POS_HOLD hold accuracy of +-6-7 cm (`CLAUDE.md`). A real-vehicle criterion is not in a design document |

### Abort Criteria per Stage

If any of these occurs, stop on the spot (stop the motors, remove the battery, save the records, and investigate). The "Abort" row of each step's table adds to these.

| Stage | Abort criterion |
|-------|-----------------|
| All stages | The `finite` line of `status`, or the `nonfinite` of the captured bundle, shows a detection of non-finite values. The `period max` while ARMed reaches 10 ms or more. An unplanned reboot (a boot message appears, the LED returns to white). A stack-overflow report. A red fast-blinking LED |
| Stage 0 | `sf doctor` reports an error. Build or flash fails. A `param get` value differs from the table in 0-7 and the reason is unknown. A `motor test` shows a motor in the wrong position |
| Stage 1 | A surface whose floor ToF gives a valid reading above 5 cm (C3 fails). ARM is refused for an unknown reason. An unexpected state transition. The ToF stopped on its own (take the records of 1i, report to the owner, and do not go on to stage 2) |
| Stages 2 and 3 | The visual abort criteria above. Yellow fast blinking and the warning sound. An unexpected state transition. The battery warning sound. Unexpected behavior after a communication loss. In the turns (2-9, 2-10): the ToF height falls below 0.30 m, or the horizontal position drifts more than 0.5 m from the center of the area. Stage 3 ends with a landing as soon as anything abnormal appears in the sequence |

## 3. How to Use This Document

To have Claude Code lead the procedure, start Claude Code in the repository and enter `/hardware-checkout` (`.claude/skills/hardware-checkout/SKILL.md`). It presents one step at a time, reads the saved records and compares them with the pass lines. Claude Code may send input to the USB console (`motor test`, which turns the motors, included), run the WiFi capture (`sf log wifi`), type into the WiFi CLI, and send `arm`, `disarm` and `mode` of the CLI (decided 2026-11-22). Before sending, it says by voice (macOS `say`) what will happen. Claude Code's reply can be several seconds late, so the one who stops the motors in an abnormal situation is the person (pressing the throttle-side stick of the transmitter, the button on the vehicle, removing the battery). Meanwhile the person stays where those can be done at any time.

### Check Items

Each item has an ID (C1 to C16). The results sheet (chapter 8) has one row per execution and writes these IDs on it.

| ID | What is checked | What failure it would reveal | Steps |
|----|-----------------|------------------------------|-------|
| C1 | The 400 Hz loop keeps its period, with or without the WiFi capture | If logging, load or the capture disturbs the period, control stalls for several ms | 1a, 1d, 1e, after every flight of stages 2 and 3 |
| C2 | No NaN or infinity appears on the control path (the `finite` line and `nonfinite` show 0 detections). **This is the precondition of a later firmware stage that will stop the motors on detection**, so record the count every time | An abnormal value reaching the motor output makes the vehicle uncontrollable | 1a, 1d, after every flight of stages 2 and 3 |
| C3 | When ARMed on the floor actually used, the downward ToF's valid readings are 5 cm or less, or invalid. Also record the diagnostic values on each floor | If the floor reading exceeds 5 cm (`kTakeoffGroundBandM`), a vehicle on the ground is not recognized as on the floor. The margin is small (floors have read 3.1 to 5.0 cm) | 1b |
| C4 | Log output while ARMed does not disturb the 400 Hz loop, whether the USB host does not read or a WiFi capture runs. No log line is lost | If log writing stalls, the loop stops for up to 50 ms. Lost lines leave holes in the record of the cause | 1d, 1e |
| C5 | On the WiFi CLI, warnings appear on that connection. `reboot` is refused while ARMed and works while DISARMed | From the WiFi side, warnings would be invisible. A restart while ARMed stops the motors | 1f |
| C6 | Each task's stack (working memory for function calls) has margin | A stack overflow causes an unplanned reboot or a malfunction | 1i |
| C7 | The battery warning and emergency notices appear, are re-raised, and act per state as designed | The vehicle cannot land before the battery runs out, or wrongly starts a landing on the floor | 1h |
| C8 | In manual (STABILIZE) mode, when the vehicle rises at low throttle, an unbroken 1 s of ToF readings above 5 cm moves ARMED_GROUND to the airborne handling (TAKEOFF). A communication loss after that leads to a landing without cutting the motors | If the vehicle stays treated as "on the ground" in the air, the communication-loss and battery landings do not act, and a silent ToF cuts the motors | 1g, 2-1, 2-8 |
| C9 | `sf takeoff` holds 0.5 m (SILS overshoots to about 0.84 m and takes about 20 s to return). `sf land` and `sf stop` work | The altitude after takeoff differs from the target, or the vehicle does not stop when asked | 2-3, 2-6 |
| C10 | A transmitter DISARM during the takeoff of an altitude mode (ALT_HOLD, POS_HOLD) stops the motors at once (by design) | Turning a DISARM during takeoff into a landing can let the landing action lift the vehicle from the floor | 1g, 2-2 |
| C11 | In a landing, the touchdown decision stops the motors only on the floor. For a landing from hover, record which of the three decision paths (firm ground, stalled descent, ToF dead zone) acted and the time from landing start to motor stop. Even in a landing that begins during a climb, the motors do not stop while the vehicle is higher than 5 cm | Motors stop in the air, or do not stop after touchdown | 1g, 1h, 2-3, 2-4, 2-5 |
| C12 | When communication is lost in flight, the vehicle lands | The vehicle keeps flying after the transmitter is lost | 1g, 2-7, 2-8 |
| C13 | `sf cw` and `sf ccw` turn the commanded angle (1 to 360 degrees) in the commanded direction. The altitude does not drop and the horizontal position does not drift during the turn. A `sf stop` in the middle of a turn holds the heading | Turning the wrong way at large angles, answering without turning, or a loss of altitude or position during a turn | 2-9, 2-10 |
| C14 | The floor-relative height from the barometer (observation only) is near 0, valid and with no learned offset on the floor, learns its offset in hover, and differs from the ToF height by some amount. **Control does not use the barometric height** | Judging a future use without knowing how large the values are | 1c, 2-3 |
| C15 | The re-initialisation of the downward ToF (it acts only when the sensor stops delivering samples on the ground) never acts during normal operation. If the sensor does stop on its own, record the course | A healthy sensor is reset for no reason. A stopped sensor never comes back, or ARM is possible with it | 1i, after every flight of stages 2 and 3 |
| C16 | Integrated flight: takeoff, a check of altitude and ToF, one full turn and landing in one flight, with no warning or abnormality left in the captured bundle | Problems that appear only when everything is run together, although each part passed | 3 |

There is no stage that checks low-altitude control with a restrained vehicle or a hand-held vehicle with propellers (no existing procedure covers it: the restrained test in `development_roadmap.md` 3.0.2 measures thrust, and the `motor` command of `operation_manual.md` checks a single motor). The hand-held check without propellers (stage 1) and the lowest flights of stage 2 take its place.

### About Command Forms and Outputs

The commands and output forms in this document were taken from the sources of the firmware and the sf CLI (chapter 9 lists where each was confirmed). Confirming the output on the real vehicle is done in the first run of this procedure. Items that could not be confirmed from the sources are written in the text as "unconfirmed". A pass or abort line marked "provisional" has no basis in a design document; it is a temporary value set by this document, with its basis given. Record the measured value as it is, and report to the owner when it is exceeded. **The 5 cm line treats 5 cm or less as "floor" and a value above 5 cm as "left the floor"** (the same direction as the firmware's `kTakeoffGroundBandM` decision; `valid above 5 cm` of `sf log stats tof` counts the values above 5 cm).

### Parameters that are changed

| Parameter | Changed in step | Restored in step | Re-checked at |
|-----------|-----------------|------------------|---------------|
| `safety.battery.low_v` (default 3.4) | 1h-1 | The end of 1h-1 | Entry of stage 2, stage 4 |
| `wifi mode` (`ap` is correct) | 0-8 (only if it was `sta`) | — | 0-8 |
| All parameters (`param reset`) | 0-7 (only when the owner judges it necessary) | — | 0-7 |
| `flip.enable` (default 0) | Not changed | — | 0-7, stage 4 |

### How to Record

Unplug the USB cable for flights. Even without USB, the state transitions, the touchdown path, refusals, the 400 Hz loop counters and the non-finite counters can be read from the bundle of a WiFi capture (`sf log wifi`). The vehicle's log text in the bundle, however, holds only what came after the capture started, so start the capture before the flight. Name a capture `s<stage>_<step>_<content>_<run>` (the run is the running number 1, 2, ... of a repeated step; for example `s1_1b_tof_floor_1`, `s2_2-3_takeoff_1`). **The capture length `-d`** was decided from the worst-case reply waits (`sf takeoff` 15 s, `sf land` 35 s, `sf up 50` 14 s, a turn angle / 57.3 + 6 + 3 s; `lib/sfcli/utils/vehicle_api.py` lines 59-76) added up, plus a margin (table below). The vehicle's WiFi serves one PC.

| Step | `-d` (s) | Breakdown |
|------|----------|-----------|
| 1b | 60 | 15 until ARM + 40 hold + margin |
| 1e | 120 | the time table of the step (95 s) + margin |
| 1g | 90 | four actions + margin |
| 1h (each scene) | 120 | |
| 2-1 | 60 | |
| 2-2 | 30 | |
| 2-3 | 120 | takeoff 15 + hover 30 + landing 35 + margin |
| 2-4, 2-7, 2-8 | 90 | takeoff 15 + hold + landing + margin |
| 2-5 | 120 | |
| 2-6 | 120 | takeoff 15 + settling 10 + climb 14 + landing 35 + margin |
| 2-9 | 180 | takeoff 15 + settling 5 + 90 degrees x 4 (11 each + 3 wait) + 360 degrees (15 + 3) + 270 degrees (14 + 3) + landing 35 + margin |
| 2-10 | 90 | |
| 3 | 150 | |

**PC connection (macOS)**: the PC can be used while connected to the Internet with its WiFi connected to the vehicle at the same time (confirmed on the real vehicle on 2026-11-22). On this PC (macOS), however, the Python network communication from a terminal application can be refused on the local network, and then `sf log wifi` ends with `No data received`. **How to tell**: `nc -z 192.168.10.1 23` succeeds while `sf log wifi -d 5 --no-save` receives no packet. **Remedy**: allow the terminal application in "Local Network" in System Settings. As another remedy, a UDP relay was used.

| What to read | Command |
|--------------|---------|
| Start a capture (a structure check runs automatically right after saving) | `sf log wifi -d <seconds> -o logs/checkout/<name>.sflog.zip` |
| Summary of the capture (the last values of the totals since boot) | The `Diagnostics (status v4 and text log):` section at the end of `sf log info <bundle>`: the two lines `imu` and `control` (`loop: cycles=... late=... period_max=...us busy_max=...us over_budget=... \| armed: late=... period_max=...us busy_max=...us`), `nonfinite: detected N of M armed cycles ...`, `ToF bottom: ...`, `log paths: ...` |
| The vehicle's log text | `sf log info <bundle> --text`. Filters: `--grep <string>` (`-I` ignores case), `--since <seconds>`, `--until <seconds>` (seconds since vehicle boot). Each line is `[<vehicle time> s] <text>`. Where log lines were lost, `*** N lines dropped here ***` appears |
| Checkout summaries (tables) | `sf log stats <mode> <bundle> [--since <seconds> --until <seconds>]`. The modes and the lines to read are in the next table |
| Plots | `sf log viz <bundle> --mode position` (`attitude` and `sensors` also work), `--save <file>.png` |
| Analysis | `sf log analyze <bundle>` |
| USB console (stage 1; **record all of them**) | `script -q logs/checkout/console_<step>_<run>.txt sf monitor` (`script` is a standard macOS command that copies the terminal input and output to a file; the record ends when `sf monitor` exits). Only condition B of 1d runs `sf monitor` directly: copy the terminal output by hand into the file `console_1d_B_<run>.txt` |
| `status` output | Type it at the USB `stampfly>` prompt or on the WiFi CLI (`nc 192.168.10.1 23`) and copy the display into the results sheet |
| Power cycle | (1) Unplug the USB cable. (2) Remove the battery (pull the front ToF board with the battery off the main board). (3) Wait 5 seconds. (4) **Connect the battery first** (attach the battery to the front ToF board, then plug the board with the battery into the main board; a battery cannot be attached once the board is on the main board). (5) Do not move the vehicle until solid green (calibration complete, three beeps). (6) Connect the USB cable. The order of (4) and (6) was checked on hardware (2026-11-21, commit 53553678, each order twice, through a USB hub with a power switch): with the battery first all sensors run (`sensors : present=0x7F`), plugging USB in afterwards does not reset the vehicle (the LED stays solid green and the cycle count since boot continues), and the PC sees the vehicle and the CLI works. With USB first the vehicle boots on USB power alone, the front ToF cannot be read and is disabled (`present=0x3F`); connecting the battery afterwards does not restart the vehicle, so the front ToF stays disabled until the next reset, and `prearm  : ready` is shown in that state too. **When the PC opens the serial port (`sf monitor`, `sf flash`) the vehicle is reset** (`rst:0x15 (USB_UART_CHIP_RESET)`); after that reset `present=0x7F` is shown for either order as long as the battery is connected. Stages 2 and 3 (no USB) use (2)(3)(4)(5) only |

The modes of `sf log stats` and the lines to copy into the results sheet (`<seconds>` of a window are vehicle times):

| Mode | Lines to copy | Window |
|------|---------------|--------|
| `tof` | `valid minimum` / `valid maximum` (minimum and maximum of the valid readings [cm]), `valid above 5 cm` (the number above 5 cm; the following lines give their times and values), `valid` and `invalid` of `ToF samples` | omitted |
| `spread` | the `spread` of each row (maximum - minimum per 1 s window [cm]), `largest 1 s spread` | `--since` and `--until` required |
| `loop` | the `armed:` lines of `imu` / `control`, the times of the rows where a counter grew, `non-finite`, `log paths`, `text log` (the number of losses), `ToF output stopped` | omitted |
| `events` | `peak ToF height`, `last time outside 0.50 +- 0.07 m` (the last time outside the hold-height band; the hold height and the band can be changed with `--target <m>` and `--band <m>`), the `LANDING` line, `last valid ToF before stop` | omitted |
| `turn` | `yaw change` (heading change [deg]; clockwise is positive), the minimum and maximum of `ToF height`, `horizontal drift` (the estimator's horizontal position drift [m]) | required |
| `baro` | `baro height - ToF` (rms and maximum [m]), the minimum and maximum of `learned offset` and `age max` | required |
| `boot` | `reset reasons`, `battery voltage` | omitted |

`sf log stats` prints facts only and does not judge pass or fail. When a needed stream is missing from the bundle, it names the stream and ends with exit code 1.

**Common check after each flight** (after every flight of stages 2 and 3; copy into the C1, C2 and C15 rows of the results sheet):

| Check | Command | Pass |
|-------|---------|------|
| C1 | The `armed:` lines of `sf log stats loop <bundle>` | `period max` below 10 ms, `busy max` below 2.5 ms, `over budget` 0 |
| C2 | The `non-finite` line of the same | `detected 0 of N` |
| C15 | `sf log info <bundle> --text --grep "re-init"` | Empty output |
| Battery | The `battery` voltage before and after the flight | Record (follow the "battery" rule) |
| Structure | `sf log check <bundle>` | 0 errors |

## 4. Stage 0: Preparation

**Entry condition**: none. **Purpose**: remove the propellers, fix the firmware under test and get into a state where records can be taken. **Abort**: the table in chapter 2 (all stages, stage 0).

| Step | Action | Record | Pass |
|------|--------|--------|------|
| 0-1 | **First, remove all four propellers** (do not fit them again until the entry of stage 2). Take a photo | Photo | All four are removed |
| 0-2 | Fix the commit under test. `git status --short` (no differences under `firmware/` and `lib/sfcli/`) and `git log -1 --format=%H`. From now on, always build the firmware that goes on the vehicle from this state | Commit ID | No differences |
| 0-3 | `source setup_env.sh`, `sf doctor` | Result | No errors |
| 0-4 | `sf build vehicle` (success prints `Build successful`) | Result | Success |
| 0-5 | Connect the vehicle by USB and flash with `sf flash vehicle`. Then, after `mkdir -p logs/checkout`, watch the boot with `script -q logs/checkout/console_0-5_1.txt sf monitor` | Boot output | It boots |
| 0-6 | `version` on the USB console (`StampFly vehicle firmware` and `build : <date> <time>`) | Build date and time | Consistent with the commit's time |
| 0-7 | Settings: `param get safety.battery.low_v` (`= 3.4 (float)`), `safety.battery.usb_v` (`= 3.3 (float)`), `safety.comm.timeout_ms` (`= 500 (float)`), `flip.enable` (`= 0 (int)`). **When a value differs from the table, an old saved value is overriding the default.** `param reset` returns to the defaults, but every parameter (the tuned gains included) goes back, so do it only when the owner judges it necessary; afterwards `param save` (while DISARMed) and `reboot` are needed | The four values | Equal to the table, or the difference is recorded and approved by the owner |
| 0-8 | `wifi show` must say `mode : ap` (SoftAP: the vehicle is the WiFi base station). If `mode : sta`: `wifi mode ap` -> `param save` -> `reboot` | Output | `mode : ap` |
| 0-9 | Connect the PC to the vehicle's WiFi (SSID `StampFly-XXYY`, XXYY = the last four digits of the vehicle's MAC; the default password is `stampfly`; `docs/guides/block_programming.md`) and run `sf log wifi -d 5 --no-save` | Statistics | Packets arrive |
| 0-10 | Pair the transmitter (`operation_manual.md` chapter 5, `controller.md`). `status` shows `pairing : Paired` and `prearm  : ready` | `status` | Both as stated |
| 0-11 | Battery: start from fully charged (about 4.2 V). `sensor power` (form: `power: 4.12 V  100 mA  400 mW`) | Voltage | "Ready to fly" in the table of `safety.md` |
| 0-12 | **Check the motor positions** (no propellers, DISARMed, USB console): `motor test 1 20 2`, `motor test 2 20 2`, `motor test 3 20 2`, `motor test 4 20 2` (M1 = front right (FR), M2 = rear right (RR), M3 = rear left (RL), M4 = front left (FL), duty 20 %, 2 s; `operation_manual.md` line 322). One at a time, see by eye that only the motor at that position spins. The correct correspondence of the rotation direction and the mounting position of the propellers (clockwise type and counterclockwise type) was not found in the repository's documents (**unconfirmed**). The owner checks it from the motor shaft, the propeller marking or the vehicle's own instructions, and it is checked again in the entry table of stage 2 | Position and rotation direction of each motor | All four positions match |
| 0-13 | **Check the CLI flight commands** (on a bench, held down by weights. The check of `arm`, `disarm` and `mode` of the CLI on the real vehicle is done first in this step. They were checked in SILS, not yet on the real vehicle). Use the USB console, and only ⑥ on the WiFi CLI (`nc 192.168.10.1 23`). The transmitter is on and paired. ARM spins the motors at idle, so wear eye protection and keep hands away from the propellers. Before sending each command Claude Code says by voice what will happen.<br>① `arm` -> `status` -> `mode acro` -> `mode stabilize` -> `disarm`<br>② `arm` -> press the throttle-side stick of the transmitter in (the person's action: stopping a vehicle ARMed by the CLI with the transmitter)<br>③ ARM with a press of the transmitter stick (the person's action) -> `disarm`<br>④ `arm` -> `emergency`<br>⑤ Press the transmitter's left button once to select ALT_HOLD (the body LED is orange) and `arm` (it is refused). After checking, press the left button two more times to return to OFF<br>⑥ On the WiFi CLI: `arm` -> `status` -> `disarm`, then `arm` -> `emergency` | The display of each command; `state`, `mode` and `armed` of `status` | The displays are as expected and the state of `status` agrees (the expectations are in the table below) |

**Expected displays of 0-13** (the wording taken from `cmd_arm`, `requestMotorStop`, `printModeResult` and `printArmRefusal` of `cli_task.cpp`):

| Command or operation | Expected display | `status` afterwards |
|----------------------|------------------|---------------------|
| `arm` (DISARMed, STABILIZE) | `ok: armed (state ARMED_GROUND, mode STABILIZE)` | `state   : ARMED_GROUND`, `armed   : yes` |
| `arm` (already ARMed) | `no change: already armed (state ARMED_GROUND)` | Unchanged |
| `mode acro` (ARMed) | `ok: mode ACRO (state ARMED_GROUND)` | `mode    : ACRO` |
| `mode stabilize` (ARMed) | `ok: mode STABILIZE (state ARMED_GROUND)` | `mode    : STABILIZE` |
| `disarm` (ARMed) | `ok: motors stopped (state ARMED_GROUND -> IDLE_GROUND)` | `state   : IDLE_GROUND`, `armed   : no` |
| `emergency` (ARMed) | `ok: motors stopped (state ARMED_GROUND -> IDLE_GROUND)` | Same as above |
| `disarm` (DISARMed) | `no change: not armed (state IDLE_GROUND)` | Unchanged |
| A press of the transmitter stick after an ARM by the CLI | Nothing is displayed on the CLI. The motors stop | `state   : IDLE_GROUND`, `armed   : no` |
| `arm` (ALT_HOLD) | `` refused: the CLI arms only in STABILIZE or ACRO (mode is ALT_HOLD: an ARM there starts an auto-takeoff) - use `mode stabilize` or `mode acro` `` | `state   : IDLE_GROUND`, `armed   : no` |

Note on `mode` on the ground: a `mode` sent on the ground (IDLE_GROUND) before ARM can be put back to the transmitter's switch position once transmitter packets are arriving (`changed, then put back to ... by the transmitter's switch position`). 0-13 sends `mode` after ARM, so this display does not appear.

**Condition to proceed**: 0-1 to 0-13 all pass.

## 5. Stage 1: Bench Check with the Propellers Removed

**Entry condition**: stage 0 is complete and all four propellers are removed (**check by eye once more, before 1g and before 1h, that all four are removed**). Run on the battery with the USB console connected (USB power alone makes ARM refused). Place the vehicle on the floor surface that stage 2 will use. **Purpose**: no thrust is produced, so a failure does not move the vehicle. Before flying, check everything that the bench can show. **Abort**: the table in chapter 2 (all stages, stage 1).

### 1a Baseline right after boot (baseline for C1, C2)

| Item | Content |
|------|---------|
| Entry | Power cycle (chapter 3) and do not move the vehicle until the LED is steady green |
| Action | `status` on USB |
| Expected output | `state   : IDLE_GROUND`, `armed   : no`, `prearm  : ready`, `battery : ...`, `loop    : 400Hz period 2500 us; late = ...`, `imu     all  : cycles N, period max X ms, late N, busy max X ms, over budget N` (`imu     armed:`, `control all  :`, `control armed:` have the same form), `finite  : no armed cycle checked yet` |
| Record | `late` and `period max` of `imu all` and `control all` |
| Pass | `late` of the `all` lines is 0 (0 is healthy on the ground before ARM; `operation_manual.md` line 338) |

### 1b Floor downward ToF and diagnostic values (C3)

If the floor reading exceeds 5 cm, a vehicle on the ground is not recognized as on the floor. Do this for **each surface** (floor, mat, ...) that stage 2 will use.

| Item | Content |
|------|---------|
| Entry | Place the vehicle level on the surface. The propellers are removed |
| Action | 1. `sensor tof` three times (form: `tof  : 0.034 m  status=0 valid=yes`).<br>2. Start `sf log wifi -d 60 -o logs/checkout/s1_1b_tof_<surface>_1.sflog.zip`.<br>3. Within 15 s ARM with the transmitter (set STABILIZE and press the throttle-side stick). `status` shows `state   : ARMED_GROUND`. Do not move the vehicle for 40 seconds.<br>4. Press the same stick to DISARM. When the capture has ended, read the following |
| Read | `sf log stats tof logs/checkout/s1_1b_tof_<surface>_1.sflog.zip`, the `ToF bottom:` line of `sf log info <bundle>`. `status` must have no `tof     : SILENT` |
| Record | Surface name, `valid minimum` / `valid maximum` [cm], the number of `valid above 5 cm` with times and values, `valid` and `invalid` of `ToF samples` (**state whether valid samples exist on that surface**; 1h-2 needs a surface with valid samples), `signal`, `ambient` (Mcps), `sigma` (mm) |
| Pass | `valid above 5 cm` is 0. Invalid readings are allowed (the floor is in the dead zone). The diagnostic values are **recorded only** (there is no range to judge them by). On a failure the surface is not used in stage 2. Changing the threshold is the owner's decision |
| Abort | Stage 1 of chapter 2 |

### 1c Barometric height on the floor (C14)

| Item | Content |
|------|---------|
| Entry | The capture of 1b is done |
| Action | 1. Leave the vehicle DISARMed on the floor without moving it for 10 s or more and run `sensor baro` (form: `baro : 101300.0 Pa  alt 12.34 m (absolute)` and the next line `       height 0.01 m  offset 0.00 m  valid, offset not learned`).<br>2. Look at the values after ARM in the bundle of 1b with `sf log viz <bundle> --mode sensors` (`height`, `valid`, `learned` of baro_height) |
| Record | `height`, `valid`, `learned` while DISARMed on the floor. How `height` moves after ARM |
| Pass | Observation only; there is no value range. `height` near 0, `valid`, `learned` is 0 (`offset not learned`) |

### 1d Loop timing, finiteness and USB states (C1, C2, C4)

Printing a few log lines at the instant of ARM used to stall the 400 Hz task for 5 to 50 ms when the USB writing stalled (the host does not read and the vehicle's sending is blocked). The log sink is designed to prevent that, so check it in three USB states. The counters are totals since boot, so **power-cycle for each condition** (the aim is to reset the counters to 0, so for conditions A and B, where USB is connected, `reboot` (while DISARMed) or a reset through USB may stand in; this was done in the 2026-11-22 run. Condition C (no USB) stays a power cycle).

**ARM, DISARM and the mode switching are done with CLI commands as the standard** (Claude Code sends `arm`, `disarm` and `mode` from the USB console or the WiFi CLI; it says by voice beforehand). In the table below, "press ARM" reads `arm`, "press DISARM" reads `disarm`, and "press the right button to toggle STABILIZE <-> ACRO" reads sending `mode acro` and `mode stabilize` alternately. The form with the transmitter, as written in the table, remains as the substitute for when the CLI commands cannot be used. The CLI `mode`, before ARM on the ground, may be put back to the transmitter's switch position, so send it while ARMed.

| Item | Content |
|------|---------|
| Entry | 1a passed. Power cycle for each condition |
| Action A (USB being read) | With `sf monitor` running (the console is recorded in `console_1d_A_1.txt`), press ARM once, **DISARM after 5 s**, and run `status` at once (to see the stall at the ARM instant). Then ARM again, hold 60 s, DISARM, `status` |
| Action B (the host does not read + a lot of logging) | Run `sf monitor` directly, without `script`. In another terminal find the process number (PID) of `sf monitor` with `ps` and pause it with `kill -STOP <PID>` (the USB host stops reading; keeping the port open without reading is also acceptable. By either method, whether the vehicle's USB sending actually blocks is **unconfirmed** on the real vehicle). ARM -> DISARM 20 times (2 s each; every ARM prints log lines). Also, while ARMed, press the right button to toggle STABILIZE <-> ACRO 10 times (every switch prints `Mode change: ...`). `status` on the WiFi CLI. Finally resume with `kill -CONT <PID>` and paste the terminal output into `console_1d_B_1.txt` |
| Action C (USB cable unplugged) | Run on the battery alone. ARM, DISARM after 5 s, `status` on the WiFi CLI. Then ARM again, hold 60 s, DISARM, `status` |
| Record | For each condition, after the 5 s ARM and after the 60 s ARM: the `imu armed` and `control armed` lines (`cycles`, `period max`, `late`, `busy max`, `over budget`) and the `finite` line. In A and B, whether `W (<time ms>) LogTask: log sink full: <N> lines dropped` appeared. `status` shows ms and the bundle µs (10 ms = 10000 µs) |
| Pass (C1) | In all three conditions `period max` of `armed` is below 10 ms, `busy max` below 2.5 ms, `over budget` 0, and `late` after the 5 s ARM is near 0 (it does not grow at the instant of ARM; `operation_manual.md` lines 338-342). **Provisional**: `late` / `cycles` of B and C is not clearly larger than A (basis: no design document gives a comparison between conditions; this is a temporary line of this document taking A as the reference). A 5 ms-class delay that continues in 13-22% of the cycles while ARMed is a separate, unresolved issue (`detailed_design.md`). `late` not reaching 0 alone is not a failure. Record it and the owner decides |
| Pass (C2) | In all three conditions `finite  : all control values finite (N armed cycles checked)` with N > 0 |
| Pass (C4) | No `log sink full` in A (provisional: no criterion in a design document). In B it may appear, provided `period max` of `armed` stays below 10 ms meanwhile |

### 1e Loop timing with and without the WiFi capture (C1, C4)

| Item | Content |
|------|---------|
| Entry | 1d passed. The vehicle is on the floor. ARM, DISARM and the mode switching are sent as CLI commands (`arm`, `disarm`, `mode`) at the times of the table as the standard, as in 1d, and the "press" of the table reads as that (the form with the transmitter is the substitute for when the CLI commands cannot be used). A `mode` before ARM on the ground may be put back to the transmitter's switch position, so send it while ARMed. The USB console is connected and recorded in `console_1e_<condition>_<run>.txt` (the same for all six runs) |
| Action (the **same time table** for three runs without capture and three with; t = 0 is the moment of the steady green) | Every run: power-cycle and wait for the steady green (the purpose is to return the counters to 0, so, as in conditions A and B of 1d, a `reboot` while DISARMed or a reset over USB may replace it; use the same method for all six runs).<br>t = 0: for the runs with capture start `sf log wifi -d 120 -o logs/checkout/s1_1e_cap_on_<run>.sflog.zip` (nothing for the runs without capture).<br>t = 5 s: press ARM (STABILIZE).<br>t = 10, 15, 20, ..., 55 s: press the right button to toggle STABILIZE <-> ACRO (10 times).<br>t = 65 s: press DISARM.<br>From t = 70 s: press ARM, wait 2 s, press DISARM, wait 2 s, five times (until t = 90 s).<br>t = 95 s: end. `status` on USB. For the runs with capture, after the capture ends run `sf log info <bundle>` and `sf log stats loop <bundle>` |
| Record | For every run `cycles`, `late`, `period max`, `busy max`, `over budget` of `imu armed` and `control armed`. With capture also `text_dropped_lines` and the number of losses in the `text log` line of `loop` |
| Pass (C1, **provisional**) | `late` and `over budget` of the runs with capture are the same as the three without (within their spread when the three differ). `period max` and `busy max` are within the minimum-to-maximum spread of the three without capture (basis: no design document gives a comparison criterion; this is a temporary line of this document taking the runs without capture as the reference) |
| Pass (C4) | In all three runs with capture `text_dropped_lines` is 0 and `--text` shows no `*** N lines dropped here ***` |
| When the spread is exceeded | `sf log stats loop <bundle>` prints the times of the rows where an armed counter grew. Read around that time with `sf log info <bundle> --text --since <time-2> --until <time+1>`, match which log lines were printed then and record it |

### 1f WiFi CLI (C5)

By design the warnings that CLITask (the task in charge of the CLI) raises do not pass the log sink and go straight to that connection.

| Item | Content |
|------|---------|
| Entry | The vehicle is on the floor, DISARMed. Connect with `nc 192.168.10.1 23` (`StampFly vehicle TCP CLI — type 'help' for commands` and `stampfly> `). Watch the USB console too (recorded in `console_1f_1.txt`) |
| Action and expectation | 1. `param set safety.battery.low_v 9` (out of range; nothing changes) -> on that connection `W (<time ms>) Params: set_float: 'safety.battery.low_v' = 9.000000 out of range [3.000000, 4.200000]` and `set failed (out of range?): safety.battery.low_v`. **The USB console does not show the same warning**.<br>2. `param get safety.battery.low_v` -> `= 3.4 (float)`.<br>3. ARM (ARMED_GROUND) and `param save` -> `refused: param save stalls the 400Hz loop (flash erase) — land/disarm first`. After DISARM: `params saved to NVS`.<br>4. ARM and `reboot` -> `refused: reboot stops the motors — land/disarm first`. No restart and no `rebooting...` (DISARM after checking).<br>5. `reboot` while DISARMed -> `rebooting...` on that connection and the connection drops. **On the USB console the boot output appears only after the log up to just before the restart has been drained.** The vehicle returns (steady green) and can be reconnected |
| Record | Where the warning appears, the text of the refusal of `reboot`, the time until reconnection |
| Pass | As expected above |

### 1g Hand-held takeoff decision, communication loss and DISARM during takeoff (C8, C10, C11, C12)

**One person (the developer) does it, lifting the vehicle with one hand and holding the transmitter with the other.** In manual mode, if a vehicle that rises at low throttle is treated as "on the ground" until the ToF has shown 1 s, the communication-loss and battery landings do not act. Lift the vehicle by hand and check that decision with the real ToF.

| Item | Content |
|------|---------|
| Entry | Confirmed by eye that the propellers are removed. Do not give the vehicle a strong shock (a 3 G shock detection causes DISARM; the default of `safety.impact.accel_g` is 3.0). One person does it: lift the vehicle with one hand and hold the transmitter with the other (a press of the stick stops it at once). **Removing the propellers is recommended** (whether to keep them fitted is the owner's decision). Claude Code starts `sf log wifi -d 90 -o logs/checkout/s1_1g_handlift_1.sflog.zip` |
| Action 1g-1 (C8) | ARM in STABILIZE (throttle at minimum). Lift the vehicle level to about 20 cm above the floor and hold it for 3 s |
| Expected | When ToF readings above 5 cm have lasted unbroken for 1 s: `Transition: ARMED_GROUND → TAKEOFF`. When a valid reading above 15 cm arrives: FLYING (`Takeoff complete → FLYING/STABILIZE`). The throttle stays at minimum |
| Action 1g-2 (C8, C12) | Keep holding it at 20 cm and **switch the transmitter off** (communication loss; switched off with the hand that holds the transmitter; on an abnormality remove the battery) |
| Expected | `Comm lost → hover 3000 ms then LANDING`. After about 3.5 s (0.5 s of `safety.comm.timeout_ms` + 3 s of grace) `Comm lost → LANDING (hover grace elapsed)` and LANDING. **The motors are not cut** (the duty in `motor.csv` is above 0 during LANDING) |
| Action 1g-3 (C11) | Put the vehicle slowly on the floor. Expected: `Landing detected (...)`, `Landing complete`, `Transition: LANDING → IDLE_GROUND`, motors stopped |
| Action 1g-4 (C10) | Switch the transmitter on again (rule 12; do not touch the sticks). Set ALT_HOLD (left button once) and press ARM (the vehicle is on the floor). After about 0.3 s of spool-up it becomes TAKEOFF. Press DISARM **within about 1 s of ARM** (after 2 s or more, with an invalid floor ToF the ToF-invalid rule turns TAKEOFF into LANDING) |
| Expected | `DISARM accepted (TAKEOFF → IDLE_GROUND)` (not `auto-land`). IDLE_GROUND without passing LANDING. All motor duties 0 |
| Read | `sf log info <bundle> --text --grep Transition`, `--grep "Comm lost"`, `--grep Landing`, `--grep DISARM`. `sf log stats events <bundle>` |
| Record | Times of the state transitions, the time from the loss to LANDING, the wording in the parentheses of `Landing detected`, the delay from DISARM to duty 0 in 1g-4 (no reference value; record only) |
| Pass | As expected above. **Do not do 2-2 until 1g-4 (C10) passes** |

### 1h Battery (C7) and a landing that begins during a hand-held climb (C11)

Confirm by eye that the propellers are removed before starting. Check that the battery notices appear and are re-raised, and that the action per state is as designed.

**1h-1 Warning (WARNING): check with a parameter.**

| Item | Content |
|------|---------|
| Entry | DISARMed, no propellers. Read the `battery` value in V of `status`. The allowed range of `low_v` is 3.0-4.2 V, so **`battery` must be below 4.15 V** (when it is nearly full, do this after the work of stage 1 has lowered it) |
| Action | `param set safety.battery.low_v <battery value + 0.05>` (two decimals; e.g. for 4.08 V use 4.13) -> `param save` -> `reboot`. After boot, watch the USB console (or `--text` of a captured bundle). ARM (on the ground). **Restore at the end**: `param set safety.battery.low_v 3.4` -> `param save` -> `reboot` -> `param get safety.battery.low_v` |
| Expected | `W (<time ms>) failsafe: Low battery: 4.0xV` repeats about every 5 s (50 updates at 10 Hz; `battery_alert_policy.hpp` line 67). `W (<time ms>) StateManager: Low battery warning`. The warning sound. The StampS3 LED blinks cyan slowly. ARM is possible (the state does not change). After the restore: `= 3.4 (float)` |
| Record, pass | The notices shown, the re-raise period, the confirmation of the restore (**finish the restore before 1h-2**) |

**1h-2 Emergency (EMERGENCY, below 3.0 V). Optional (do it only when the owner has prepared a power-supply lead; otherwise "not done").** The critical-voltage threshold (3.0 V) cannot be a parameter, so connect a power supply to the battery connector instead of the battery and lower the voltage. How to connect the supply (connector shape, polarity, current-limit value) is in neither the repository's documents nor the firmware source (**unconfirmed**). Minimum cautions: do not reverse the polarity (use only a lead the owner has checked), set the current limit to the current shown as `battery` in `status` plus a margin, keep the voltage above 3.6 V before ARM, some scenes need holding the vehicle and controlling the voltage at the same time, so the owner decides whether one person does it (the crew table in chapter 1), no propellers. ARM is refused at 3.3 V or less (`safety.battery.usb_v`), so start at about 3.6 V, ARM, and lower it below 3.0 V afterwards. Record the USB console and also take `sf log wifi -d 120 -o logs/checkout/s1_1h_battery_<scene>_1.sflog.zip`. **Judge the bundles with the lowered voltage separately, and leave them out of the `boot` check and the `re-init` check of 1i** (the low voltage itself can cause a restart or a ToF stop).

| Scene | Action | Expected output and behavior |
|-------|--------|------------------------------|
| Floor, ARMED_GROUND | ARM in STABILIZE at 3.6 V (throttle at minimum). Lower the voltage to 2.9 V | `E (<time ms>) failsafe: Critical battery: 2.9xV`, `StateManager: Battery emergency: no transition in ARMED_GROUND`. **The state stays ARMED_GROUND and the motors are not cut by this.** The same line repeats about every 5 s |
| TAKEOFF, no evidence of leaving the floor | Return to 3.6 V and DISARM. **On a surface where the floor ToF returns valid readings (a surface that had valid samples in 1b)**, ARM at 3.6 V and raise the throttle above 0.5 (`TAKEOFF_THROTTLE_THRESH` is 0.5; `config.hpp` line 278). When the state is TAKEOFF, lower the voltage below 3.0 V. On a surface whose floor ToF stays invalid the ToF becomes silent after about 2 s and FLYING is shown, so this scene does not hold | `Battery emergency in TAKEOFF without evidence of the air (the healthy ToF has not shown the craft off the floor) → DISARM`. Motors stop, IDLE_GROUND |
| TAKEOFF, evidence of leaving the floor | ARM at 3.6 V (STABILIZE, throttle at minimum). Lift the vehicle to about 10 cm (5-15 cm) above the floor and confirm the state is TAKEOFF. Lower the voltage below 3.0 V | `Battery emergency → LANDING`. **The motors are not cut**. Put the vehicle slowly on the floor and `Landing complete` appears |
| FLYING | Lift it to about 20 cm (above 15 cm), confirm the state is FLYING and lower the voltage | `Battery emergency → LANDING`. The motors are not cut |
| Floor, IDLE_GROUND (after DISARM) | DISARM with the voltage still below 3.0 V and wait about 5 s | `Battery emergency: no transition in IDLE_GROUND` (log only). Trying ARM is refused: `ARM rejected: battery X.XXV (USB/low — unsafe to fly)`, `prearm  : BLOCKED (battery low / USB power)`, an error sound |

**1h-3 A landing that begins during a hand-held climb (C11). Optional (same condition as 1h-2).** With the same set-up as "TAKEOFF, evidence of leaving the floor" of 1h-2, have the vehicle moving by hand at the instant LANDING begins. Start lifting the vehicle slowly from the floor at about 0.2-0.4 m/s and lower the voltage below 3.0 V while it passes 5-15 cm. Then move it slowly up and down within 5-15 cm for about 2 s and put it on the floor at the end. Read `sf log stats events <bundle>`. Pass: the motors stop only after the vehicle is put on the floor (`last valid ToF before stop` is 5 cm or less). They do not stop while the height is above 5 cm.

### 1i Review of all records (C6, C15)

| Item | Content |
|------|---------|
| Entry | 1a to 1h are done and all bundles and console records (1d's B included) remain in `logs/checkout/` |
| Action C6 | 1. `grep -n -i "panic\|stack overflow\|Guru Meditation" logs/checkout/console_*.txt` (ESP-IDF's standard messages; the actual wording on this vehicle is unconfirmed).<br>2. `sf log stats boot <bundle>` on every bundle except those of 1h. |
| Pass C6 | No panic line. `reset reasons` only `1 POWERON` and `3 SW` (4 PANIC, 5 INT_WDT, 6 TASK_WDT and 9 BROWNOUT are failures). The remaining stack can be read with `stack` while disarmed (`min free` of every task, in B; refused while armed). There is no pass figure, so it is **recorded only**: copy the full output of `stack` at the end of stage 1 and after the last flight of stages 2 and 3 (after DISARM). The free internal memory is the `heap` line of `status` (free, minimum, largest block); copy it at the same times and with the WiFi capture and the WiFi CLI connected (the values on the floor with USB only are in chapters 6 and 8 of `docs/hardware/mcu-memory.md`). The designed sizes are in "memory layout" of `detailed_design.md` (ImuTask 16 KB, ControlTask 8 KB, LogTask 6 KB, ...) |
| Action C15 | On every bundle of stage 1 except those of 1h: `for f in logs/checkout/s1_*.sflog.zip; do case "$f" in *s1_1h_*) continue;; esac; echo "$f"; sf log info "$f" --text --grep "re-init"; done`. For the console records `grep -n "re-init" logs/checkout/console_*.txt`. As support, `ToF output stopped` of `sf log stats loop <bundle>` is 0 rows, and the recorded `status` outputs have no `tof     : SILENT` |
| Pass C15 (normal operation) | No line containing `re-init`. The function acts only when samples have been missing for 2 s or more (`kTofReinitSilenceUs`). An attempt on a sensor that works is itself a defect (record it and report to the owner) |
| When the sensor stops on its own | Stopping the downward ToF is not part of this procedure (no document tells whether the downward ToF of this vehicle sits where it can be detached safely without soldering, so removing or blocking the sensor cannot be given as an action). When the sensor stops on its own (`tof     : SILENT`, `sensor tof` invalid, `re-init` lines), keep **the motors unable to spin**, record the following, report to the owner and do not go on to stage 2.<br>- The time of the first attempt line `VL53L3CX bottom re-init: attempt 1 (no sample for N ms)` and N (by design N is 2000 or more).<br>- The interval between later attempts (by design about 10 s; `kTofReinitRetryIntervalUs`).<br>- The result line of each attempt: success `... succeeded (attempt N, X ms)`, failure `... failed (attempt N, X ms): <error name>` (a failure line appears for the 1st attempt and then every 6th = one per minute). For every failed attempt the driver adds two lines, `I2C read failed at 0x00E5` and `WaitDeviceBooted failed`.<br>- The time until `sensor tof` is `valid=yes` again after a success.<br>- Meanwhile `prearm  : BLOCKED (downward ToF silent / not re-acquired - ...)`, and `ARM rejected: downward ToF is silent (...)` if ARM was tried |
| Pass C15 (when it stopped, **provisional**) | The first attempt about 2 s after the silence began, later ones about every 10 s, and after a success `valid=yes` returns. The basis is the design values (2 s, 10 s, 1 s, 6 in `flight_limits.hpp` lines 196-199); the duration on the real vehicle is unconfirmed even in the design document |

**Condition to proceed**: C1 to C5 pass. C6 to C8, C10 to C12, C14 and C15 pass or are approved by the owner. **Even if stage 2 is on another day, do the "Entry of stage 2" below every time.**

## 6. Stage 2: Low Free Flight with the Propellers Fitted

### Entry of stage 2 (check all of it at the start of every flight day)

| # | Check | Content |
|---|-------|---------|
| 1 | Results of stage 1 | C1 to C5 pass. 1g-1 (C8) and 1g-4 (C10) pass. Fly over a surface for which C3 passed. 1h-1 passes. **When 1h-2 is "not done"**: the owner approves, all flights of stage 2 stay at 0.5 m or lower, and the vehicle lands at once when a battery notice (the cyan blink and the warning sound) appears (the critical-voltage action is unconfirmed on the real vehicle) |
| 2 | Propellers | Fit four undamaged, undeformed propellers matching the motor positions (checked in 0-12) and the rotation directions, and check the guards for damage (items 1 and 2 of the pre-flight checklist in `safety.md`). No mix-up of the clockwise and counterclockwise types (the correct correspondence is **unconfirmed**: the owner checks). **After fitting them, do not use the motor-running command (`motor`)** |
| 3 | Parameters | `param get safety.battery.low_v` is `= 3.4 (float)` and `flip.enable` is `= 0 (int)` (the table "Parameters that are changed" in chapter 3) |
| 4 | Battery | Nearly full, and **every flight starts at 4.0 V or more** (provisional; basis: the table of `safety.md` calls 3.7-4.0 V normal and ready to fly, and this is its lower bound 3.7 V plus a margin for the voltage drop of a flight). If the voltage is below 3.7 V after a flight, or a warning sound appears, end with that flight and change the battery (`safety.md`: 3.5-3.7 V is land recommended, 3.5 V or below is land at once. `safety.battery.low_v` 3.4 V is the warning) |
| 5 | Crew, terminals, area | The crew of the table in chapter 1. T1 to T3. A ceiling of 2.0 m or higher, a netted area of at least 2 m x 2 m, marks at 1.0 m and 1.2 m on a wall or pole. Everyone has read the abort ladder |
| 6 | Transmitter | Powered on and paired (`pairing : Paired`). The mode as each step specifies (check by the LED color) |

**Purpose**: fly in order of increasing risk (the table in chapter 1), one purpose per flight. When a flight fails, do not go on to the next. **Abort**: "Abort ladder" and "Abort Criteria per Stage" in chapter 2.

### Common procedure (every time)

| Order | Action |
|-------|--------|
| Before | Re-insert the battery and wait for the calibration to finish (steady green, beep x3). `status` on the WiFi CLI: `prearm  : ready`, `pairing : Paired`; record the `battery` voltage (4.0 V or more) |
| Before | In T1 run `sf log wifi -d <seconds> -o logs/checkout/s2_<step>_<content>_<run>.sflog.zip` (seconds from the table in chapter 3). If a second person is present, type `sf emergency` into T3 |
| After | After landing and DISARM, wait for the capture to end. Do the "Common check after each flight" of chapter 3 and read the state transitions with `sf log info <bundle> --text --grep Transition`. Read the error and warning lines with `--grep "E ("` and `--grep "W ("` (log lines of ESP-IDF start with `E (` and `W (`; the form on the real vehicle is unconfirmed) |

Steps flown with `sf` (2-3, 2-6, 2-9, 2-10, 3) are operated with these commands (`--ip` changes the vehicle's IP; the default is 192.168.10.1). **`sf takeoff` ARMs the vehicle by itself and takes off in POS_HOLD** (no ARM press on the transmitter; "Operation map" in chapter 1):

```bash
sf takeoff     # auto takeoff (from ARM). Climbs to the fixed 0.5 m, waits until FLYING and prints the reply (no altitude argument. The wait is 15 s by default)
sf up 50       # climb 50 cm (10-300 cm). Waits until it arrives (14 s). A command beyond the 1.2 m ceiling is refused by the PC
sf cw 90       # clockwise 90 degrees (1-360 degrees). Waits until the turn is done. sf ccw turns counterclockwise
sf stop        # hold the present position (height and heading). Sent from another terminal, it also ends a command that is waiting
sf land        # land. Waits until touchdown (35 s by default)
sf emergency   # stop all motors at once (sent without waiting)
sf height      # estimated height [cm] (sf tof is the downward ToF [cm], sf baro the barometric height [m])
```

Exit codes: 0 success, 1 the vehicle's `error ...` or the PC's refusal of an argument, 3 no reply, 130 Ctrl-C. Interrupting a waiting `sf takeoff`, `sf up`, `sf cw` etc. with Ctrl-C sends `stop` automatically (it is refused during the climb of a takeoff, and the takeoff goes on). Interrupting `sf land` sends nothing. **`emergency` is never sent automatically.** Measure the elapsed time with `time sf cw 90`. The pilot holds the transmitter neutral and is ready to DISARM (moving a stick releases the API guidance).

**Prepare timed actions as one line** (do not operate two terminals in flight). Example (start `sf up 50` and send `sf stop` 1 s later): `sf up 50 & sleep 1 && sf stop; wait`. The `sf up` and the `sf stop` of that line reach the vehicle through separate sockets. **How to take a window** (`--since` and `--until` of `sf log stats`): `sf log info <bundle> --text --grep "cmd: "` prints the vehicle times of the commands that reached the vehicle from the PC (`cmd: takeoff`, `cmd: cw 90`, ...). The window of a command runs from its time to the time of the next command. The hover window runs from the time of `Transition: TAKEOFF → FLYING` to the time of `cmd: land`. **Judging "settled"** (common, provisional): `sf height` is inside 0.5 +- 0.07 m (the hold band; the default of `sf log stats events`) twice in a row, and by eye the swing is below 0.15 m.

### 2-1 Detection of a manual low-throttle takeoff (C8)

| Item | Content |
|------|---------|
| Purpose | When the vehicle rises at low throttle, does the ToF evidence move ARMED_GROUND to TAKEOFF? The decision was checked in 1g-1; here it is checked with a real lift |
| Entry | Claude Code has started the capture. STABILIZE (the body LED is yellow-green). Crew: the pilot (the developer) alone |
| Action | Press ARM, raise the throttle slowly, and when the vehicle lifts hold about 10-20 cm for 2-3 s, lower the throttle and land. Stay within 20 cm. **If it does not lift, repeat once**; if it still does not lift, record it and finish |
| Record | The throttle value at lift-off (the `throttle` column of `pilot.csv`, 0 to 1; whether it is the same scale as the threshold 0.5 (the thrust command inside the vehicle) is **unconfirmed**). The times from `--grep Transition` |
| Pass | When the throttle at lift-off is 0.5 or less, TAKEOFF follows after ToF above 5 cm has lasted 1 s. When it lifted above 0.5, TAKEOFF follows by the throttle threshold, so record that **the check did not hold** (judge C8 by the result of 1g) |
| Abort | Height above 20 cm, a growing tilt (abort ladder rung 1) |

### 2-2 DISARM during the takeoff of an altitude mode (C10)

| Item | Content |
|------|---------|
| Purpose | Does a DISARM during the takeoff of an altitude mode stop the motors at once as designed? (Why it is not a landing: a landing started near the floor can lift the vehicle.) **It is done after checking it on the bench in 1g-4 (an entry condition)** |
| Entry | 1g-4 passed. On a soft mat for which C3 passed. ALT_HOLD (the body LED is orange). Crew: the pilot (the developer) alone |
| Action | Press ARM. After about 0.3 s of spool-up the climb begins. Press DISARM **about 1 s after ARM** (height below 0.3 m). After the DISARM the vehicle falls from that height |
| Record | `--grep Transition`: the time of TAKEOFF → IDLE_GROUND (without passing LANDING). `sf log stats events <bundle>` |
| Pass | Same as 1g-4 (without passing LANDING, all duties 0) |
| Abort | DISARM does not take effect after 1 s (remove the battery; when a second person is present, Enter on `sf emergency` in T3) |

### 2-3 Altitude hold after takeoff, normal landing, observation of barometric height (C9, C11, C14)

| Item | Content |
|------|---------|
| Purpose | Does `sf takeoff` hold 0.5 m? SILS overshoots to about 0.84 m and needs about 20 s to return to 0.5 m. Record the real values. Then `sf land` shows the touchdown decision on the real vehicle for the first time. The same capture observes the barometric height (observation only) |
| Entry | 2-2 passed. Crew: the pilot (the developer) + Claude Code (the capture and sending the commands; the pilot stops it). POS_HOLD is switched on by `sf takeoff` |
| Action | `time sf takeoff`. When the reply returns, watch a 30 s hover (the pilot is ready to DISARM; abort ladder rung 2). Then `sf land` |
| Read | `sf log stats events <bundle>`, `--grep "Landing detected"`, for the hover window `sf log stats baro <bundle> --since <t0> --until <t1>`, `sensor baro` on the WiFi CLI (`offset age`) |
| Record | `peak ToF height`, `last time outside 0.50 +- 0.07 m` (the difference from the start of FLYING), the time in the `LANDING` line, `last valid ToF before stop`, the elapsed time of `time sf takeoff`, the touchdown path, rms and maximum of `baro height - ToF`, `learned offset` |
| Pass | The peak is **1.0 m or less** (provisional; basis: the abort line at the 1.0 m mark; the altitude limit 1.2 m and the SILS overshoot 0.84 m). A pass line for the hold and the settling is not defined in a design document (compare the measured values with SILS and the owner judges). `sf land` ends successfully and the motors stop after touchdown. **Barometric height is observation only and has no pass criterion**: it has learned (`learned`) in the hover. Past records show an offset of about +0.2 to +0.9 m and `baro height - ToF` rms 0.22 m and maximum about 1.2 m (reference values, not criteria) |
| Abort | Above the 1.0 m mark, yellow fast blinking, `sf takeoff` returns `error ...` (check the state with `status`). Abort ladder rungs 2 and 3 |

### 2-4 Landing from hover and the touchdown decision path (C11)

| Item | Content |
|------|---------|
| Purpose | Do the three touchdown decision paths (firm ground: ToF 5 cm or less plus at rest; stalled descent: the descent stops by ground effect; ToF dead zone: 0.2 s of invalid readings after a valid reading near the floor) stop the motors only on the floor? |
| Entry | 2-3 passed. Do it **three times or more per surface** (one capture each, with the run number). ALT_HOLD. Crew: the pilot (the developer) alone |
| Action | Press ARM (automatic takeoff) and hold 0.5 m. Press DISARM (automatic landing). **If the motors do not stop after touchdown, wait 12 s from the press and press DISARM a second time** (basis: the first half, 12 s, of the time `sf land` waits for the landing to finish, `vehicle_api.py` line 64. A normal landing takes about 2 s from 0.5 m at about 0.3 m/s, so 12 s is more than five times that) |
| Read | `--grep "Landing detected"`: one of `(ground)` / `(stalled descent / ground effect)` / `(ToF dead zone: only invalid samples for N ms after a valid X m)`. `sf log stats events <bundle>` |
| Record | The wording of the path, the time from the landing start to the motor stop, `last valid ToF before stop`, the number of invalid samples in the last 0.5 s. Whether the vehicle was on the floor when the motors stopped (the person watching, video) |
| Pass | The vehicle is on the floor at the instant the motors stop. The last valid ToF is 5 cm or less. Above 5 cm up to 15 cm it is the stalled-descent path and the owner decides (record it) |
| Abort | The motors do not stop after touchdown (a second DISARM after the 12 s above), or the motors stopped in the air |

### 2-5 Landing from a low hover and ToF scatter (C11)

There is no way to create on the real vehicle, safely, a situation where LANDING begins within about 1 s of the start of a takeoff (a landing that begins during a climb). The only roads from TAKEOFF into LANDING are communication loss, a battery emergency (with evidence of leaving the floor), the ToF rule and a floor evidence that does not release (`detailed_design.md` §3.1). A transmitter DISARM stops at once, `sf land` is held while TAKEOFF, the communication-loss grace is 3 s, and an unreleased floor evidence needs 5 s. The battery emergency is done in 1h-3 (bench). So on the real vehicle a landing from a low hover stands in for it.

| Item | Content |
|------|---------|
| Purpose | (a) Even a landing that begins at a low hover does not stop the motors above 5 cm. (b) The rest decision of the touchdown takes the floor ToF readings staying within a 0.02 m width for 1 s (`kLandingRestBandM`, `kLandingRestHoldMs` 1000 ms) as evidence of rest. Does a resting real vehicle fit in this width? (A past real record gave 0.022 m, slightly above the band.) |
| Entry | 2-4 passed. ALT_HOLD. Crew: the pilot + a person who reads the height with `sf tof` and tells it. **Do not do alone** (Claude Code's reading can be several seconds late, so it is not left to it; the crew table in chapter 1) |
| Action | Press ARM (automatic takeoff), the pilot lowers the throttle stick to a height of **0.10-0.12 m** and holds it for 2 s or more (check with `sf tof`). Note the vehicle times of the start and the end of that 2 s stretch. Press DISARM (automatic landing). Whether the stick can go down that far is **unconfirmed** (the API's lower limit is 0.2 m; the stick's lower limit was not found in the code). If it cannot, record that and skip 2-5 |
| Read | `--grep "Landing detected"`, `sf log stats events <bundle>`, and for **that 2 s stretch** (`<t0>` to `<t1>`) `sf log stats spread <bundle> --since <t0> --until <t1>` (take a stretch of 2 s or more) |
| Record | The path, the time in the `LANDING` line, `last valid ToF before stop`, `largest 1 s spread` and the `spread` [cm] of each window |
| Pass | (a) The last valid ToF at the motor stop is 5 cm or less (record the path and the time). (b) No criterion (record only). If `largest 1 s spread` exceeds 2 cm, report it to the owner |
| Abort | Same as 2-4 |

### 2-6 Altitude hold by `sf stop` after takeoff (C9)

| Item | Content |
|------|---------|
| Purpose | Does `sf stop` hold the altitude it was sent at? A `stop` while waiting for `takeoff` is refused (`error not flying or guidance released - takeoff/go to re-engage`), so check it in a climb after the takeoff is done |
| Entry | 2-3 passed. Crew: the pilot (the developer) + Claude Code. Prepare the following line in T2 |
| Action | `sf takeoff`. After "settled" (the common judgement), `sf up 50 & sleep 1 && sf stop; wait` (stop about 1 s after the climb begins). Then `sf land`. The target of `sf up 50` is 1.0 m; if the stop does not work it heads for 1.0 m (the vehicle stops it at the 1.2 m ceiling) |
| Record | The reply of `sf stop` (`ok`), the reply of `sf up 50` (`error ...` when cut off), the stopped altitude and its later hold with `sf log viz <bundle> --mode position`. The expected stop height is about 0.8 m (0.5 m + 1 s of climb; the climb speed is a design value near 0.3 m/s, unconfirmed on the real vehicle) |
| Pass (**provisional**) | At the stopped altitude, the altitude changes by 0.07 m or less during the 5 s after the stop (basis: the same width as the hold band 0.5 +- 0.07 m). The altitude does not exceed 1.2 m |
| Abort | `sf stop` is refused, or it does not hold and keeps climbing toward the 1.2 m mark (abort ladder rungs 2 and 3) |

### 2-7 Communication loss in flight of an altitude mode (C12)

**During the communication loss the pilot's DISARM does not arrive. The only way to stop is `sf emergency` in T3.**

| Item | Content |
|------|---------|
| Purpose | When the transmitter is lost in flight of an altitude mode, does the vehicle land? |
| Entry | 2-3 passed. ALT_HOLD. Crew: 2 people (the pilot + the PC operator waiting with `sf emergency` typed). **Not done by the developer alone plus Claude Code** (once the transmitter is off the person has no way to stop, and Claude Code's `emergency` can be several seconds late. Until a simulation of communication loss is added, done only when a second person is present. Planned for later) |
| Action | Press ARM (automatic takeoff) and hold 0.5 m. The PC operator signals "switch off" by voice and the pilot switches the transmitter off. **Do not switch the transmitter on again until you have confirmed the vehicle has landed and DISARMed (the motors stopped)** (rule 12; ARM toggles with every press, so do not touch the stick right after the reconnection). On an abnormality press Enter in T3 |
| Read | `--grep "Comm lost"`, `--grep Transition`, `sf log stats events <bundle>`, `sf log viz <bundle> --mode attitude` |
| Record | The time from the loss to LANDING, the length of LANDING, touchdown, motor stop |
| Pass | LANDING in about 3.5 s, descends level, stops at touchdown. The motors do not stop in the air |
| Abort | It does not land, or the tilt grows (`sf emergency` in T3). Abort ladder rungs 3 and 4 |

### 2-8 Communication loss after a manual lift (C8, C12)

| Item | Content |
|------|---------|
| Purpose | When communication is lost after a manual lift into TAKEOFF/FLYING, does the vehicle land without cutting the motors? (Cutting them would drop it.) |
| Entry | 2-1 and 2-7 passed. STABILIZE. A mat on the floor. Crew: 2 people (the pilot + the PC operator waiting with `sf emergency` typed). **Not done by the developer alone plus Claude Code** (the same reason as 2-7). The pilot holds the stick with one hand and switches the transmitter's power off with the other (**switching off while holding the stick needs both hands**; if the height cannot be held with one hand, do not ask the PC operator to hold it, and do not do this step) |
| Action | Hold about 0.3 m and switch the transmitter off after the signal. From then on as in 2-7 (rule 12) |
| Read | `--grep "Comm lost"`, `--grep Transition`, `sf log stats events <bundle>` |
| Record | The time from the loss to LANDING (about 3.5 s by design), the duties during LANDING, touchdown, motor stop |
| Pass | It becomes LANDING. The motors are not cut until touchdown. The landing attitude is level |
| Abort | A large tilt, no touchdown (`sf emergency` in T3). Abort ladder rung 4 |

### 2-9 Turns: 90 degrees x 4, 360 degrees, the opposite way 270 degrees (C13)

`cw` and `ccw` turn any angle from 1 to 360 degrees in the commanded direction by the commanded angle. The wait for completion is "angle / 1.0 rad/s (about 57.3 deg/s) + 6 s". SILS has no coupling between turning and altitude or position, so the real values of the altitude drop and the horizontal position drift during a turn are **unknown**, and this flight measures them for the first time. Make 2-9 and 2-10 separate flights.

| Item | Content |
|------|---------|
| Purpose | Does the vehicle turn in the commanded direction by the commanded angle (no reverse turn at large angles, no answering without turning)? Does the altitude not drop and the horizontal position not drift during the turn? |
| Entry | 2-3, 2-6 and 2-7 passed. Crew: the pilot (the developer) + Claude Code. POS_HOLD |
| Action | `sf takeoff`. After "settled", wait 3 s before each turn and run in this order: `time sf cw 90` four times (back to the original heading), `time sf cw 360`, `time sf ccw 270`. Then `sf land` |
| Read | For each turn the time to `ok` (the design value is near angle / 57.3 deg/s). For the window (from `cmd: cw ...` of that turn to the next command) `sf log stats turn <bundle> --since <t0> --until <t1>` |
| Record | `yaw change` (its difference from the command is the heading error), the minimum of `ToF height` (the difference from 0.5 m is the altitude drop), `horizontal drift` (the estimator's horizontal position drift; also check the real drift with a mark on the floor or video), the time to `ok` |
| Pass (**provisional**) | `yaw change` matches the commanded direction and angle (clockwise is positive). Temporary lines: **angle error within 10 degrees**, **lowest ToF height 0.30 m or more** (a drop within 0.2 m of the hold), **horizontal drift within 0.5 m** (basis: the safe-side temporary lines of this document; 0.5 m is half of the 1 m from the center of a 2 m area to the wall). Report to the owner when exceeded. Record the measured values as they are |
| Abort | The ToF height falls below 0.30 m, or the vehicle drifts more than 0.5 m from the center of the area, or the tilt grows. At once `sf stop`, and if that does not stop it, abort ladder rungs 2 and 3 |

### 2-10 `sf stop` in the middle of a turn (C13)

| Item | Content |
|------|---------|
| Purpose | When `sf stop` is sent in the middle of a turn, does the vehicle stop at that heading, without returning to the original heading or turning on? |
| Entry | 2-9 passed. Crew: the pilot (the developer) + Claude Code. POS_HOLD. Prepare the following line in T2 |
| Action | `sf takeoff`. After "settled", `sf cw 360 & sleep 3 && sf stop; wait` (stop about 3 s later = about half of 360 degrees). Then `sf land` |
| Read | The time `<ts>` of `sf log info <bundle> --text --grep "cmd: stop"`. `sf log stats turn <bundle> --since <ts> --until <ts + 5>` |
| Record | The reply of `sf stop` (`ok`) and the reply of `sf cw 360` (`error interrupted by stop`). `yaw change` for the 5 s after the stop |
| Pass (**provisional**) | `yaw change` for the 5 s after the stop is within 10 degrees (basis: the same temporary line as 2-9). The replies are as above |
| Abort | Same as 2-9 |

**Condition to proceed**: all of 2-1 to 2-10 pass (or are approved by the owner).

## 7. Stage 3: Integrated Flight

**Entry condition**: **the "condition to proceed" of stage 2 is met** (every step passed or approved by the owner; the approvals are written in the notes of the results sheet) and the results sheet is filled in. Check the whole entry table of stage 2 (six items) again. The same area, the same crew (the developer + Claude Code) and a fully charged battery. Make this flight **only once**; its purpose is to confirm that nothing appears, when running from takeoff to landing, that the part-by-part checks did not show (C16). **Abort**: the "Abort ladder" and "Abort Criteria per Stage" in chapter 2.

| Order | Action | Check |
|-------|--------|-------|
| Before | The "common procedure (before)" of stage 2. `sf log wifi -d 150 -o logs/checkout/s3_final_1.sflog.zip`. If a second person is present, `sf emergency` typed in T3 | — |
| 1 | `sf takeoff` | The reply returns |
| 2 | **Check the altitude and the downward ToF right after takeoff** (rule 5). The pilot watches the 1.0 m mark and the LEDs. Claude Code reads `sf height` and `sf tof` once each (the reply can be several seconds late, so the judgement is made by the pilot's eye and the values are records) | Altitude near 0.5 m, ToF normal. **If either is abnormal, end here with `sf land` (or DISARM)** |
| 3 | One full turn: `sf cw 90` four times (one `sf cw 360` is also fine if 2-9 passed). Check the altitude before each | Every `sf cw` returns success |
| 4 | `sf land` | The reply returns and the motors stop at touchdown |
| After | Confirm DISARM and wait for the capture to end. Take `status` on the WiFi CLI before powering off | — |

**What to read from the captured bundle** (`<bundle>` is `logs/checkout/s3_final_1.sflog.zip`). In addition to the "Common check after each flight" of chapter 3 (C1, C2, C15):

| What to read | Command | Pass |
|--------------|---------|------|
| Log text: errors and warnings | `sf log info <bundle> --text --grep "E ("` and `--grep "W ("` | No error (`E (`). The warnings (`W (`) include none of the kinds `Downward ToF`, `Comm lost`, `Battery`, `Impact`, `ESKF diverged`, `ARM rejected`, `log sink full`, `set_float`, and each remaining line can be explained |
| State transitions | `--text --grep Transition` | Only `IDLE_GROUND → ARMED_GROUND → TAKEOFF → FLYING → LANDING → IDLE_GROUND` in that order |
| Touchdown and motor stop | `--text --grep "Landing detected"`, `sf log stats events <bundle>` | One touchdown line. Record its wording and the time until the motor stop |
| Loop, log paths, ToF silence | `sf log stats loop <bundle>` | `late` about the same as the runs with capture in 1d and 1e (provisional). All four `log paths` are 0. The `text log` losses are 0. `ToF output stopped` is `tof_silent=1 in 0 of ...` |
| ToF diagnostic values | `ToF bottom:` of `Diagnostics` | Record only |
| Altitude and heading trajectory | `sf log viz <bundle> --mode position --save logs/checkout/s3_position.png`, `--mode attitude`, `sf log analyze <bundle>`, and for the turn windows `sf log stats turn` | The altitude does not exceed the 1.0 m mark (check with video and graph). The hold height is near 0.5 m. The total heading change is 360 degrees clockwise (within 10 degrees; provisional). The vehicle does not lift again after landing |
| Restarts | `sf log stats boot <bundle>` | `reset reasons` only `1 POWERON` or `3 SW` |

## 8. Results Sheet and Wrap-up (Stage 4)

### Wrap-up procedure

| Order | Action |
|-------|--------|
| 4-1 | Confirm that the changed parameters are restored: `param get safety.battery.low_v` (`= 3.4 (float)`), `param get flip.enable` (`= 0 (int)`) |
| 4-2 | Keep the bundles and console records of `logs/checkout/`. `sf log check <bundle>` shows 0 errors on every bundle. Whether to commit them is the owner's decision (`logs/` is outside Git) |
| 4-3 | Fill in the results sheet. Write "not done" or "unconfirmed" for what could not be measured |
| 4-4 | For a failed or pending item write the cause and the next action in the notes |
| 4-5 | Remove the battery and record its charge state |

### Results sheet

Commit tested (ID): ____________　Build date and time (`version`): ____________　Performed by: ____________

One execution is one row. When a step is repeated, add a row with a larger "Run". "Start V" is the `battery` [V] before the flight or the ARM. "Result" is one of "pass", "fail", "not done", "unconfirmed". The parentheses are the IDs of the check items.

| Step (items) | Run | Start V | Criterion | Measured value | Result | Bundle / record file | Notes |
|--------------|-----|---------|-----------|----------------|--------|----------------------|-------|
| 0-1 | 1 | — | All four propellers removed | | | | |
| 0-2 | 1 | — | No differences | Commit ID: | | | |
| 0-3 to 0-4 | 1 | — | No `sf doctor` errors, build succeeds | | | | |
| 0-5 to 0-6 | 1 | — | It boots | `version`: | | | |
| 0-7 | 1 | — | The four settings equal the table | | | | |
| 0-8 to 0-9 | 1 | — | `mode : ap`, packets arrive | | | | |
| 0-10 to 0-11 | 1 | | Paired, a "ready to fly" voltage | | | | |
| 0-12 | 1 | — | The four motor positions match | Rotation directions: | | | |
| 1a (C1, C2) | 1 | | `late` of the `all` lines is 0 | | | | |
| 1b surface 1 (C3) | | | `valid above 5 cm` is 0 | min / max / valid count / invalid count: | | | |
| 1b surface 2 (C3) | | | Same | | | | |
| 1b diagnostics (C3) | | | Record only | signal / ambient / sigma: | — | | |
| 1c (C14) | 1 | | Near 0, `valid`, offset not learned | | | | |
| 1d condition A (C1, C2, C4) | 1 | | `period max` < 10 ms, `busy max` < 2.5 ms, `over budget` 0, N > 0, no `log sink full` | | | | |
| 1d condition B (C1, C2, C4) | 1 | | Same. `late`/`cycles` not larger than A (provisional) | | | | |
| 1d condition C (C1, C2) | 1 | | Same | | | | |
| 1e without capture (C1, C4) | 1-3 | | Record the spread (minimum, maximum) | | — | | |
| 1e with capture (C1, C4) | 1-3 | | Within the spread without capture (provisional). `text_dropped_lines` 0 | | | | |
| 1f (C5) | 1 | | Warning on the TCP side only. `reboot` refused while ARMed | | | | |
| 1g-1 (C8) | 1 | | ToF above 5 cm for 1 s, then TAKEOFF | | | | |
| 1g-2 (C8, C12) | 1 | | LANDING in about 3.5 s, motors not cut | | | | |
| 1g-3 (C11) | 1 | | `Landing detected`, motors stopped | | | | |
| 1g-4 (C10) | 1 | | Stops without passing LANDING | | | | |
| 1h-1 (C7) | 1 | | Notice and re-raise about every 5 s. Restored | | | | |
| 1h-2 floor, ARMED_GROUND (C7) | 1 | | Log only, motors not cut | | | | |
| 1h-2 TAKEOFF, no evidence (C7) | 1 | | Motors stop | | | | |
| 1h-2 TAKEOFF, evidence (C7) | 1 | | LANDING, motors not cut | | | | |
| 1h-2 FLYING (C7) | 1 | | LANDING, motors not cut | | | | |
| 1h-2 IDLE_GROUND (C7) | 1 | | Log only. ARM refused | | | | |
| 1h-3 (C11) | 1 | | Motors do not stop while above 5 cm | | | | |
| 1i (C6) | 1 | | No panic, `reset reasons` only 1 and 3 | | | | |
| 1i (C15) | 1 | | 0 `re-init` lines | | | | |
| 2-1 (C8) | 1-2 | | With lift-off at 0.5 or less, TAKEOFF after ToF above 5 cm for 1 s | | | | |
| 2-2 (C10) | 1 | | Stops without passing LANDING | | | | |
| 2-3 (C9, C11, C14) | 1 | | Peak <= 1.0 m (provisional). `sf land` succeeds | peak / settling / path / time: | | | |
| 2-4 surface 1 (C11) | 1-3 | | Last valid ToF <= 5 cm | path / time: | | | |
| 2-4 surface 2 (C11) | 1-3 | | Same | | | | |
| 2-5 (C11) | 1 | | Last valid ToF <= 5 cm. Scatter is record only | path / time / largest scatter: | | | |
| 2-6 (C9) | 1 | | Altitude change in 5 s after stop <= 0.07 m (provisional) | stopped height: | | | |
| 2-7 (C12) | 1 | | LANDING in about 3.5 s, no stop in the air | | | | |
| 2-8 (C8, C12) | 1 | | LANDING, motors not cut | | | | |
| 2-9 90 degrees x 4 (C13) | 1 | | Heading returns. Record error, drop, drift (provisional 10 degrees, 0.30 m, 0.5 m) | error / lowest ToF / drift: | | | |
| 2-9 360 degrees (C13) | 1 | | 360 degrees clockwise | time / error / lowest ToF / drift: | | | |
| 2-9 counterclockwise 270 degrees (C13) | 1 | | 270 degrees counterclockwise | time / error / lowest ToF / drift: | | | |
| 2-10 (C13) | 1 | | `yaw change` in 5 s after stop <= 10 degrees (provisional) | | | | |
| After each flight of stages 2 and 3 (C1, C2, C15) | per flight | | The common check of chapter 3 | | | | |
| 3 (C16) | 1 | | As in the table of chapter 7 | | | | |
| 4-1 to 4-5 | 1 | — | Parameters restored, `sf log check` 0 errors | | | | |

## 9. Appendix: Where Commands and Output Forms Were Confirmed

| Command / output | Form | Confirmed in (inside the repository) |
|------------------|------|---------------------------------------|
| Lines of `status` | `state   :`, `armed   :`, `pairing :`, `battery : %.2f V, %.0f mA`, `prearm  :`, `tof     : SILENT - ...`, `loop    :`, `imu/control all/armed`, `finite  :` | `firmware/vehicle/tasks/cli_task.cpp` lines 283-402 |
| `sensor tof`, `sensor baro`, `sensor power`, `param get/set/save`, `version`, `wifi show`, `reboot`, the TCP CLI banner | The form of each line | Same, lines 80-105, 136, 192, 472-514, 1168, 1268-1272, 1321. The out-of-range warning: `components/sf_core/params.cpp` line 1130 |
| `log sink full` | `W (<ms>) LogTask: log sink full: <N> lines dropped` | `components/sf_core/log_sink.cpp` line 143 |
| Battery notices | `Low battery: %.2fV`, `Critical battery: %.2fV`, `Low battery warning`, `Battery emergency ...`, `ARM rejected: battery ...`. A warning is re-raised about every 5 s | `components/sf_failsafe/failsafe.cpp` lines 283-287, `battery_alert_policy.hpp` line 67, `components/sf_state/state_manager.cpp` lines 203 and 775-797 |
| State transitions, DISARM, communication loss | `Transition: A → B`, `DISARM accepted (...)`, `Comm lost → hover 3000 ms then LANDING`, `Comm lost → LANDING (hover grace elapsed)`, `Takeoff complete → FLYING/...` | `components/sf_state/state_manager.cpp` lines 331, 367, 750, 1054, 1167 |
| Touchdown decision | `Landing detected (...)` | `components/sf_takeoff_landing/takeoff_landing.cpp` lines 327, 331 |
| ToF re-initialisation | `VL53L3CX bottom re-init: attempt %u (no sample for %u ms)`, `... succeeded ...`, `... failed ...: <error name>`. Constants 2 s, 10 s, 1 s, 6 | `tasks/tof_task.cpp` lines 330-347, `components/sf_state/include/tof_silence_policy.hpp` line 309, `components/sf_core/include/flight_limits.hpp` lines 196-199 |
| ARM operation and modes | ARM is a toggle on the stick press; `sf takeoff` ARMs by itself and uses POS_HOLD | `tasks/state_task.cpp` lines 791-803 and 910, `tasks/api_task.cpp` lines 543-551, `docs/guides/controller.md` chapter 9 |
| How DISARM acts, LEDs, sounds | The abort ladder, the table of LEDs and sounds | `docs/operation_manual.md` lines 53-103, 136-140, 210 |
| Single-motor test | `motor test <1-4> <0-100> [seconds]`, M1 = FR, M2 = RR, M3 = RL, M4 = FL | `docs/operation_manual.md` line 322 |
| Thresholds | `kTakeoffGroundBandM` 0.05 m, `kTakeoffAirborneBandM` 0.15 m, `kAltitudeCeilingM` 1.2 m, `kAutoTakeoffTargetAltitudeM` 0.5 m, `kLandingRestBandM` 0.02 m, `kLandingRestHoldMs` 1000 ms, `kAutoTakeoffNoLiftAbortUs` 5 s, communication-loss grace 3 s, `critical_battery_v` 3.0 V, `low_battery_v` 3.4 V, `TAKEOFF_THROTTLE_THRESH` 0.5 | `flight_limits.hpp` lines 55, 72, 291-296, 326-377, `state_manager.cpp` line 49, `failsafe.hpp` lines 38-39, `main/config.hpp` line 278 |
| Turn and move wait times | Turn = angle / 57.3 + 6 s (+ 3 s of network margin), move = distance / 10 cm/s + 6 s (+ 3 s), takeoff 15 s, landing 35 s | `components/sf_command/include/rotate_guidance.hpp` line 98, `lib/sfcli/utils/vehicle_api.py` lines 59-122 |
| `sf` flight, query and log commands | `--ip`, `--timeout`, exit codes 0/1/3/130, `emergency` and `stop` are sent without a handshake (a separate socket per call) | `lib/sfcli/commands/flight.py`, `query.py`, `log.py`, `lib/sfcli/utils/vehicle_api.py` lines 160-200, `docs/commands/sf-log.md` |
| API replies | `error not flying or guidance released ...`, `error interrupted by stop`, `cmd: <line>` | `tasks/api_task.cpp` lines 453, 795, 1930 |
| Columns of the flight log | `flight_phase`, `motor`, `attitude`, `posvel`, `tof_bottom`, `baro_height`, `status`, `loop_timing`, `nonfinite`, `link_health`, `tof_silence`, `text_log`, `pilot` | `protocol/spec/flight_log.yaml` |
| Battery guide | Full 4.2 V, normal 3.7-4.0 V ready to fly, 3.5-3.7 V land recommended | `docs/guides/safety.md` chapter 4 |

**What could not be confirmed**: the actual wording of a panic message, how to connect a power supply to the battery connector, the behavior on the real vehicle of the `sf takeoff` family of commands and of the stall of USB by `kill -STOP`, the form on the real vehicle of the `E (` and `W (` filters of the log text, whether the ALT_HOLD stick can go down to 0.10 m, the duration of the real re-initialisation when the downward ToF stops on its own, the time to reconnect the transmitter and the mode behavior right after it, which wins between the mode the API switched to and the mode given by the transmitter, the correspondence of the clockwise and counterclockwise propellers to their mounting positions, whether the USB-first order is needed, the scale correspondence of `throttle` in `pilot.csv` to the threshold 0.5, the upper limit of the ToF range, and the form of the `sf log stats` output on a bundle of the real vehicle (confirmed on SILS and on past real logs).
