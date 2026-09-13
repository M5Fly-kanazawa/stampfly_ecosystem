# 用語・言い換え辞書

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. この文書の位置づけ

**この文書は `stampfly_ecosystem` リポジトリだけで通用する基準文書である。** 他リポジトリと共有する仕組み（symlink・git submodule・外部参照スクリプト等）は作らない。他プロジェクト（`micromouse_rl`、`m5stamp_uwb_localizer` 等）にも似た辞書があるが、それぞれ別リポジトリの正本であり、本書はそれらを**参考に内容を取り込んだだけ**で、以後の同期は行わない。

**役割分担**: 用語の**意味**（略語が何を指すか・専門語の定義）は [`../guides/glossary.md`](../guides/glossary.md) を基準とする。用語の**言い方**（言い換え・禁止語・訳語・表記・語法・文体）は本書を基準とする。両方確認したいときは両方を見る。

**改訂の考え方**: ユーザー（このプロジェクトの責任者）からの指摘は 1 回で本書に反映し、以後の全セッションに効かせる。指摘された語は最優先で該当章に追記する。改訂の手順は 11 章を参照。

**このプロジェクトの前提**: このリポジトリの主担当者はソフトウェア開発の専門家ではなく、専門は制御工学・ハードウェアである（`../../CLAUDE.md` 参照）。本書の言い換え・禁止語の多くは「ソフト業界の内輪語・直訳語が読者に通じない」という理由による。

## 2. 原則

1. **略語・記号名は初出時に必ず内容を括弧書きで添える。** 例:「ESKF（誤差状態カルマンフィルタ、Error-State Kalman Filter）」「M2（マイルストーン 2）」。長い文書では離れた再出時にも短く再掲する。記号の羅列は読み手には意味のない文字列になる。
2. **分野の内輪語・借用語を避け、日本の技術者の日常語で言う。** ソフトウェア業界のスラング・他分野（IT・医学・経済・法律等）の直訳を、制御工学・機械・航空宇宙以外の分野からそのまま持ち込まない。
3. **範囲・集合・系列を呼ぶときは、具体（番号範囲・件数・中身）を添える。** 「検証用のログ」ではなく「検証用の 20 件のログ（seed 7000〜7019 相当）」のように書く。
4. **同じ語が複数の対象を指すなら呼び分ける。** 6 章「多義語の呼び分け」を参照。
5. **ユーザーから指摘された語は最優先で記録する。** 指摘 1 回で以後の全セッションに効く状態を保つ（11 章）。
6. **ソフトウェア固有の用語は初出で一文解説する。** 例:「スキーマ（データの構造・形式の定義）」。制御工学・電気回路・物理・ハードウェアなど、このプロジェクトの主担当者の専門領域の用語は解説不要でそのまま使ってよい。
7. **数学の説明は大学 3 年までの範囲を前提にする。** それを超える概念（測度論的確率、関数解析等）を使うときは、定義・直感的な意味・具体例を添える。

## 3. 使わない語 → 言い換え

禁止語・直訳語・借用語・IT 俗語をまとめた統合表。**装置やソフトウェアを人・動物のように言う表現（擬人化）は 4 章にまとめてあるので、ここには含めない。** 学術論文だけで使う訳語は 7 章、複数の対象を指す多義語は 6 章、regression・SILS/HILS・モデルの表記は個別ルールがあるため 5 章にまとめた。

| 使わない語 | 言い換え | 分類 | 分野 | 制定日・出典 |
|---|---|---|---|---|
| サグ（電圧サグ・電池サグ・sag） | 「電池電圧の低下」「放電に伴う電圧低下」「負荷時の電圧降下」。補償は「電圧低下の補償」 | 禁止語 | 電気 | 2026-08-02 ユーザー明示 |
| bob（高度の bob） | 「高度の上下動」 | 禁止語 | 制御・飛行 | 2026-06-27 |
| ゲート（合否ゲート・モデル一致ゲート） | 「合否判定」「合格基準」「判定条件」 | 禁止語 | 汎用 | 2026-09-05 ユーザー明示（多義語としての扱いは6章参照） |
| 解禁（層の解禁順・解禁される） | 「順に使えるようになる」「段階的に開放される」 | 禁止語 | 汎用 | 2026-09-08 ユーザー明示 |
| 疎通（疎通確認・疎通テスト） | 「接続の確認」「つながっているかの確認」「受信できるかの確認」 | 禁止語 | 通信 | 2026-09-08 ユーザー明示 |
| 家事（housekeeping） | 補助処理・付随処理（統計出力・コンソール・LED 等、具体名を添える） | 禁止語 | ソフト | 2026-08-30 |
| 律速・律速段階（rate-determining／bottleneck） | ボトルネック／時間の大半を占める処理／〜が先に頭打ちになる | 訳語 | 汎用 | 2026-07-13, 2026-08-14 |
| 露出（学習・出題の遭遇回数の意、exposure） | 出題回数・提示回数 | 訳語 | 機械学習 | 2026-08-14 |
| 発火（fire／trigger） | 作動（警報・トリガーが働くこと）／成立（条件・条文について。例:「打ち切り条文が発火」→「打ち切りが成立」） | 訳語 | 汎用 | 2026-08-14 |
| 機会（occasion の直訳） | 場合・タイミング・時点 | 訳語 | 汎用 | 2026-09-05 |
| ラップする（wrap around） | 桁あふれで 0 に戻る・折り返す | 訳語 | ソフト | 2026-09-05 |
| 一発で | 一度で・一回で | 訳語 | 汎用 | 2026-09-05 |
| ゾンビ接続 | 残留接続（初出のみ「（いわゆる）ゾンビ接続」併記可。詳細は4章「残してよい定訳」） | 許容併記 | 通信・ソフト | 2026-09-05 |
| 迷子（ファイルの意） | 残存した・取り残された | 訳語 | ソフト | 2026-09-05 |
| デグレ（った） | 既存動作の破壊（regression の扱いは5章参照） | IT俗語 | ソフト | micromouse_rl辞書 |
| エッジケース・コーナーケース | 極端な条件・めったに起きない条件 | IT俗語 | ソフト | micromouse_rl辞書 |
| スモークテスト | 「最小動作確認」（起動して基本動作が壊れていないかだけを見る試験）。動詞的には「ざっと動作を確認する」 | IT俗語 | ソフト | micromouse_rl辞書（2026-09-13 に名詞形を追加） |
| ドライラン | 予行・実行せずに手順確認 | IT俗語 | ソフト | micromouse_rl辞書 |
| ボイラープレート | 定型コード・お決まりの部分 | IT俗語 | ソフト | micromouse_rl辞書 |
| イテレーション | 反復・繰り返し（の1周） | IT俗語 | ソフト | micromouse_rl辞書 |
| ローンチ | 開始・公開（ロケットは「打ち上げ」） | IT俗語 | ソフト | micromouse_rl辞書 |
| フィックスする | 修正する・直す | IT俗語 | ソフト | micromouse_rl辞書 |
| ペンディング | 保留 | IT俗語 | ソフト | micromouse_rl辞書 |
| アサイン | 割り当て・担当にする | IT俗語 | ソフト | micromouse_rl辞書 |
| ハーネス（test harness） | 試験プログラム（本体を動かして入出力を流す側のコード） | 訳語 | ソフト | 2026-05-31 |
| 界面（ソフトの接続点の意） | インターフェース | 訳語 | ソフト | 2026-05-31 |
| 継ぎ目（同上） | 境界・インターフェース | 訳語 | ソフト | 2026-05-31 |
| 算法 | アルゴリズム | 訳語 | ソフト | feedback_plain_japanese_terms |
| 圏外・射程外 | 対象外 | 訳語 | 汎用 | feedback_plain_japanese_terms |
| 素体 | 土台・基盤 | 訳語 | ソフト | feedback_plain_japanese_terms |
| 北極星（比喩、最重要目標の意） | 最重要の目標・核心 | 比喩 | 汎用 | feedback_plain_japanese_terms |
| 差替可能 | 差し替え可能・交換可能 | 訳語 | 汎用 | feedback_plain_japanese_terms |
| 潰す（まとめる意） | まとめる | 訳語 | 汎用 | feedback_plain_japanese_terms |
| 唯一の正 | 「〜を基準とする」・基準となる文書・大元 | 訳語 | 汎用 | feedback_plain_japanese_terms |
| 正本（〜が正本・ローカル正本・正本は X） | 「基準となる文書」「基準ファイル」「〜を基準とする」「元となる定義」「元データ」。`PROJECT_PLAN.md` を指す「原典」はオーナーが用いた語なので可 | 禁止語 | 汎用 | 2026-09-13 ユーザー明示「日本人は普段使わない」（本表末尾の注記も参照） |
| 比力（specific force） | 加速度計が測る加速度・加速度計の測定値 | 訳語 | 航法 | 2026-05-31 |
| 整流（rectify、絶対値化の比喩） | 正側へ折り返す・負を正に畳む・絶対値化（で符号が消える） | 訳語 | 信号処理 | 2026-06-01 |
| 崖（cliff） | 臨界・不連続な破綻・（閾値を境に）一気に発散する・破綻の境目 | 比喩 | 制御 | 2026-06-08 |
| 音・鳴る（信号処理の tone の直訳） | 正弦波・励振している・周波数点・励振周波数 | 比喩 | 制御・信号処理 | 2026-06-21 |
| スタブ（stub） | 代替実装・置き換え用の簡易実装 | 訳語 | ソフト | 2026-08-06 |
| 憲章（charter） | 研究計画書・本計画・基本方針・実施要領 | 借用語 | 汎用 | 2026-08-10（micromouse_rl辞書） |
| 崩壊した（collapse） | 破綻した・限界を超えた・成り立たなくなった | 訳語 | 汎用 | micromouse_rl辞書 |
| 汚染された値・データ汚染（contaminated） | 当てにならない値・異常値が混入した・外れ値まじりの | 訳語 | 計測 | micromouse_rl辞書 |
| 病的な（pathological） | 定義上破綻する・極端な・意味をなさない | 訳語 | 汎用 | micromouse_rl辞書 |
| 走行包絡・性能包絡（envelope） | 性能範囲・運用領域・（航空では）飛行領域 | 訳語 | 航空・制御 | micromouse_rl辞書 |
| 参照値・参照信号（reference） | 目標値・指令値・基準値 | 訳語 | 制御 | micromouse_rl辞書 |
| 攻撃的な（aggressive、制御・走行の形容） | 思い切った・応答の速い・強気の | 訳語 | 制御 | micromouse_rl辞書 |
| 頑強な・堅牢な（robust、制御文脈の訳） | ロバストな（制御分野の定訳。統計文脈は「頑健な」も可） | 訳語 | 制御 | micromouse_rl辞書 |
| 正気度確認（sanity check） | 検算・妥当性の簡易確認 | 訳語 | 汎用 | micromouse_rl辞書 |
| 素朴な実装・ナイーブな（naive） | 単純な・工夫のない・そのままの | 訳語 | ソフト | micromouse_rl辞書 |
| バニラの（vanilla） | 素の・標準のままの | 訳語 | ソフト | micromouse_rl辞書 |
| 現金化する（性能・投資を、cash in on） | 引き出す・活かす・実際の速さに変える | 比喩 | 汎用 | 2026-08-20（micromouse_rl辞書） |
| トンネル視野（tunnel vision） | 視野狭窄・そこしか見えなくなる | 比喩 | 汎用 | 2026-08-20（micromouse_rl辞書） |
| 裁定（技術的決定の意、ruling） | 決定・判断・方針・決めたこと | 訳語 | 汎用 | 2026-08-20（micromouse_rl辞書） |
| 些末でない・非自明な（non-trivial、乱用） | 一筋縄でいかない・手間のかかる（数学の「非自明」はそのまま可） | 訳語 | 汎用 | micromouse_rl辞書 |
| 封筒の裏の計算（back-of-envelope） | 概算・ざっくり見積り | 比喩 | 汎用 | micromouse_rl辞書 |
| 教師-学生モデル（teacher-student） | 教師-生徒モデル | 訳語 | 機械学習 | micromouse_rl辞書 |
| 切除実験（ablation） | アブレーション（要素抜き比較実験）・要素抜き実験 | 訳語 | 機械学習 | micromouse_rl辞書 |
| パラレルアクシス合成（parallel axis） | 平行軸の定理（による合成） | 訳語 | 機械 | micromouse_rl辞書 |
| ボアサイト（boresight） | （センサ・光学系の）光軸 | 訳語 | 計測 | micromouse_rl辞書 |
| レンジファインダ（rangefinder） | 距離センサ・測距センサ | 訳語 | 計測 | micromouse_rl辞書 |
| プラギング（plugging） | 逆転制動（「プラギング」と併記は可） | 訳語 | 機械 | micromouse_rl辞書 |
| ストールトルク（stall torque） | 停動トルク（モータ定格の意） | 訳語 | 機械 | micromouse_rl辞書 |
| アーティファクト（計測の artifact） | 計測系に起因する見かけの現象・偽の信号 | 訳語 | 計測 | micromouse_rl辞書 |
| グラウンドトゥルース（ground truth） | 真値 | 訳語 | 計測 | micromouse_rl辞書 |
| クリーンな（コンタミ以外の文脈での clean） | 素性のよい・混入のない | 訳語 | 計測 | micromouse_rl辞書 |
| 隔離する（isolate、問題の切り分けの意） | 切り分ける | 訳語 | 汎用 | micromouse_rl辞書 |
| 免罪する（exonerate） | 無実と確認する・原因から除外する | 訳語 | 汎用 | micromouse_rl辞書 |
| 迅速な反復（rapid iteration） | 試行の回転が速い・短い周期で回す | 訳語 | 汎用 | micromouse_rl辞書 |
| てこ入れする・レバレッジする（leverage） | 活用する | 訳語 | 汎用 | micromouse_rl辞書 |
| 最適な（optimal、目的関数の最適化以外での乱用） | 適切な・ちょうどよい（真に最適性を主張するときのみ「最適」） | 訳語 | 汎用 | micromouse_rl辞書 |
| 梯子・速度梯子（ladder、試験手順の意） | 速度水準を下から順に上げて走らせる一連の試験（初出はこの説明つき、以後は「速度引き上げ試験」と略記可） | 借用語 | 試験手順 | micromouse_rl辞書 |
| 腕（実験条件の意、arm） | 条件・群（対照群・処置群） | 借用語 | 実験計画 | 2026-08-13（micromouse_rl辞書） |

**許容語（そのまま使ってよい）**: 「ベンチ」「テストベンチ」は制御工学で標準的な語としてそのまま使ってよい（2026-05-31 確認）。

**表の注記（「正本」の経緯）**: micromouse_rl 辞書は「正本」を法律用語（裁判所交付文書の正本・謄本・抄本）だとして 2026-08-13 に使用禁止にしていた。本リポジトリでは `../../CLAUDE.md` をはじめ既存文書が「〜が正本」「ローカル正本」を技術文書の語として使っており、本書の制定時（2026-09-13 午前）はいったん「禁止しない」と記録した。同日午後、オーナーが「正本という言葉も日本人は普段使わないので言い換えて」と指示したため、**本リポジトリでも「正本」は使わない**ことにし、両辞書の食い違いは解消した。言い換えは上表のとおり（基準となる文書／基準ファイル／〜を基準とする／元となる定義）。

## 4. 装置・プログラムの擬人化を避ける

**原則**: 装置・タスク・ソフトウェアが主語のときは、比喩ではなく実際の動作を書く。人や動物のように言わない。

| 使わない語 | 言い換え |
|---|---|
| 叩く | 呼び出す／直接操作する／送る |
| 吐く | 出力する |
| 食わせる | 入力する／渡す |
| 走る・走らせる | 動く・実行する・動かす |
| 殺す | 停止させる／終了させる |
| 気づく | 検出する／検知する／分かる |
| 諦める | 打ち切る／断念する／〜へ切り替える |
| 期待する | 想定する／見込む／前提とする |
| 知っている・知らない | 情報を持っている・持っていない／判別できる・できない |
| 信じる | 重みを置く／信頼度を高く扱う／観測を重視する |
| 死んでいる・死ぬ／生きている | 故障している・応答しない・停止している／動作している・応答している |
| 黙る・黙っている | 出力が止まる／応答が無い |
| 喋る | 送信する／通信する |
| 眠る・目覚める・起こす | スリープする／待機する／復帰する／起動する |
| 頑張る・嫌がる・喜ぶ・機嫌 | 状況に即して平易に言い換える |
| 面倒を見る・お守り | 管理する／扱う |
| 相手（装置を人のように） | 相手局／相手側／通信相手 |
| 自分（装置を人のように） | 自局／自分自身の |
| 〜してくれる・〜してしまう（装置が主語） | 〜する／〜になる |
| 素直に（実装・値の性格付け） | そのまま／無理なく／単純に |
| 正直な（値の性格付け） | 実態に合った／妥当な |
| 飢える・飢餓（処理が回ってこない意） | 処理が回ってこない状態（初出のみ「スタベーション」併記可） |

### 残してよい定訳（初出で説明を添える）

| 語 | 扱い |
|---|---|
| 相手局・自局 | 通信の標準用語。裸の「相手」「自分」は使わない |
| スリープ | OS の標準用語（「眠る」「目覚める」は使わない） |
| スタベーション | 初出で「処理が回ってこない状態（スタベーション）」と書き、以後は「処理が回ってこない」 |
| ゾンビ接続 | 初出で「切断されたのに残り続ける接続（いわゆるゾンビ接続）」と書き、以後は「残留接続」 |
| ボトルネック | 「律速」の代わりに使う（3章） |
| フック・トリガ・ハンドル | コードの識別子（例: `RangingSampleHook`）や API 名を指すときだけ。文中の動作説明では「呼び出し」「作動」「操作」を使う |

出典: [m5stamp_uwb_localizer](https://github.com/kouhei1970/m5stamp_uwb_localizer) `docs/JA_ENGINEERING_TERMS.md`（2026-09-05 制定）を全面的に取り込んだ。

### 現象の説明に比喩を使わない（2026-09-13 追加）

**原則**: StampFly は工学的な対象である。機体・センサ・推定器・制御器で**起きた現象を説明するとき**、擬人化（人のように）・擬動物化（動物のように）・他の物からの比喩（乗り物・機械・地形・生き物などに見立てる言い方）を用いず、**どの軸・どの量が、どの向きに、どれだけ、どの時間スケールで変化したか**を書く。比喩は読み手ごとに思い浮かべる像が異なり、再現・計測・比較ができないため。

| 使わない比喩 | 言い換え（物理量と観測で書く） |
|---|---|
| 暴走する（機体・速度指令・推定値が） | 発散する／制御が効かず増大する／意図しない方向へ加速し続ける（量と向きを添える） |
| 暴れる（値・機体が） | 大きく変動する／±30 m/s² まで振れる／制御されずに傾く（振幅・周波数を添える） |
| 跳ねる・跳ね上がる（値が） | 急増する／一時的に大きくなる（何倍・何 ms か） |
| 蹴る・蹴ってしまう（出力・ループを） | 出力に急変を与える／ループを呼び出す・駆動する |
| 蹴り返し（反トルクの説明） | 反作用トルク（角運動量保存による） |
| 噛み合う（制御ループ・帯域が） | 整合する／時間スケールが分離している／両立する |
| 崖（しきい値を越えると一斉に変わる様） | しきい値を超えると一斉に無効になる／急激に低下する（しきい値の値を添える） |
| お辞儀する・首を振る・尻もちをつく・つんのめる・よろける | ピッチ軸の一時的な前傾（角度・時間）／ヨー軸の往復振動（周波数・振幅）／着地時の後傾／前方への急な傾き／姿勢の不規則な変動 |
| 息をする・脈打つ・呼吸（周期的な変動） | 周期 T の変動／〜Hz の振動 |
| 元気がない・疲れる・弱る（モータ・電池が） | 出力が低下している／電圧が低下している／推力係数が小さい |
| びっくりする・パニックになる（推定器・制御器が） | 外れ値で推定が大きく動く／制御出力が飽和する |

**残してよい定訳**（制御・電子・推定分野の教科書用語。初出で一文説明を添える）: オーバーシュート、ハンチング、リミットサイクル、ドリフト、飽和、発振、ワインドアップ、微分キック（derivative kick）、ラッチアップ、スパイク、リンギング、デッドバンド、ヒステリシス、チャタリング。これらは比喩由来でも分野で定義が定まっている語であり、言い換えると不正確になる。

**判断待ち**: 「ふらつき」（位置・姿勢の小さな不規則変動。研究名「ふらつき最小化」にも使用中）。日常語として物にも使うが、書くときは初出で対象の量（位置 RMS・姿勢 RMS 等）を添える。

## 5. 訳語・表記の決まり

### regression（統計の回帰／ソフトウェアの再確認試験）

- **統計的な回帰**（regression analysis・回帰直線・回帰係数）は「回帰」のままでよい。
- **ソフトウェアの regression test（変更で既存の動作が壊れていないかを自動で確かめる試験）は「再確認試験」と呼ぶ。** 初出時は「再確認試験（変更で既存の動作が壊れていないかを自動で確かめる試験）」と書く。
- 「機能が壊れること」自体は「既存動作の破壊」と呼ぶ（例:「退行が起きた」ではなく「既存動作の破壊が起きた」）。
- **「退行」「退行試験」「回帰テスト」は使わない**（2026-09-13 に本リポジトリのローカル規約として変更。下記「経緯」参照）。
- 英語名は訳さない: `sf sils regression`（コマンド名）、CI ワークフロー名 "SILS scenario regression"、コミットメッセージの "regression" は英語のまま書く。

**経緯**:

| 時期 | 決定 | 状態 |
|---|---|---|
| 2026-06-07 | 「回帰（テスト／スイート）」は使わず「検証スイート」「テストスイート」「既存の動作が壊れていないこと」と言い換える案 | 廃止（次の決定に上書きされた） |
| 2026-06-22 | ソフトウェアの regression は「退行」、統計は「回帰」のまま、という決定（グローバル `~/.claude/CLAUDE.md` に「必ず守る」と明記） | 本リポジトリでは 2026-09-13 に下記へ変更（他リポジトリでは 2026-06-22 の決定が現在も有効） |
| 2026-09-13 | 本リポジトリに限り「再確認試験」「既存動作の破壊」へ変更。理由: オーナーから「『退行』という言葉は分かりにくいので、分かりやすい言葉に置き換えて」との指摘。何をする試験かが語だけで伝わる、説明的な言い方を採った | 現行（本リポジトリのローカル規約） |

### SIL → SILS、HIL → HILS

- Software-in-the-Loop の呼称は **SILS（Software In the Loop Simulation）**。初出時は「SILS（Software In the Loop Simulation）」と併記する。
- Hardware-in-the-Loop の呼称は **HILS（Hardware In the Loop Simulation）**。初出時に同様に併記する。
- **SITL は ArduPilot/PX4 のシミュレータの固有名としてのみ使う**（例:「ArduPilot の SITL」）。一般概念の呼称としては使わない。
- コード側の識別子（`sf sils`、`simulator/sils/`、`SILS_EMU_*` 環境変数、`namespace sils`、CI `sils-regression.yml` 等）も同じ表記に揃える。旧 `sf sil` は当面エイリアスとして動作する。
- 凍結パス（旧表記のまま。改名の対象外）: `firmware/vehicle_old`（削除済み。過去の参照のみ残る）、`analysis/reports/`、`analysis/scripts/verify_hikoki64/`。
- **実装状況の注記**: HILS はファームウェア側が未実装（受信処理なし）。Python 側のインタフェースはあるが呼び出し元がない。実装状況に触れるときは「Python 側のみで、実機との接続は未実装」と明記する。

### モデル（model）

- **「モデル」は常にカタカナで書く。漢字「模型」と書かない。** センサモデル・モータモデル・物理モデル・ノイズモデル・デバイスモデルなど、数学・ソフトウェア・シミュレーションの「モデル」はすべてカタカナ。
- 「模型」は物理的な縮尺模型・プラモデル・鉄道模型の意味であり、抽象・数理モデルの表記としては誤り。
- 制御工学の標準語（モデル・ゲイン・フィルタ・オブザーバ等）は無理に漢字化・言い換えをしない。平易化の対象はソフト固有のジャーゴンに限る（3章）。

### コマンド名・識別子は訳さない

`sf` コマンド名（`sf build`、`sf sils regression` 等）、環境変数名、クラス名・関数名などのコード識別子、CI ワークフロー名は、文章中でも**英語表記のまま**扱う。日本語の地の文に混ぜるときは、初出でその機能を一言説明してから使う（例:「`sf doctor`（環境診断コマンド）」）。

### 英語のまま残す語と日本語化する語の判定基準

以下の基準で判定する（学会原稿の査読で確認された基準を統合。原稿・査読者名は伏せる）。

- **確立した日本語の定訳がある専門語は日本語にする**。例: Hessian → ヘッセ行列、rank-1 → ランク1、Box 制約 → ボックス制約。
- **定訳が無い・分野で英語表記が定着している語はそのまま使い、初出で説明を添える**。例: Real-Time Iteration (RTI) は定訳がなく、初出で説明してから使えばそのままでよい。warm-start も許容（「ウォームスタート」と書けばより自然）。
- **コードの識別子・固有名はそのまま**。例: `core0` / `core1`、`EIGEN_NO_MALLOC` のような変数名・マクロ名。
- **定訳の無い操作は、無理に一語の訳語を当てず、操作をそのまま説明する**。例:「凝縮（condensing）」のような借用語ではなく、「動力学の等式制約で状態変数を消去し、入力のみの密な二次計画に帰着させる（condensing と呼ばれる）」のように操作を書き下す。

## 6. 多義語の呼び分け

同じ字面が複数の対象を指す語は、対象ごとに呼び分けるか、具体を添える。

### 回帰・退行・再確認試験（regression）

5章参照。統計は「回帰」、ソフトウェアの regression test は本リポジトリでは「再確認試験」。

### 段（stage／rung）

単独の「段」は使わず、文脈で呼び分ける。カリキュラムの学習区間は**学習段階**（例: 学習段階1〜4）、速度引き上げ試験の刻みは**速度水準**（例: 速度水準0.45・全7水準）のように、対象ごとに違う語を当てる。

### 帯（band）

「◯◯帯」は使わず、**用途と具体的な番号範囲を添えて書く**。例:「検証帯」→「検証用の20件（seed 7000〜7019）」。初出時は必ず範囲を括弧書きし、同一文書内の再出は用途の略記（「検証用の一式」等）でよい。

### 面

「面」を「対象そのもの（迷路・盤面等）」の意味で単独使用する用法は、他プロジェクト（マイクロマウス競技）由来の内輪語であり、本リポジトリには直接の対応物がない。本リポジトリで「面」を使うときは、PCB の実装面・水平面・ロータ回転面のような**物理的な面**の意味に限定し、対象物の略称として単独で使わない。

### オブザーバ

**フィードバック制御に使う状態推定器の意味に限定する。** 制御に接続しない推定（ログを見るためだけの推定等）は「オープンループ推定」と呼び、「オブザーバ」「オブザーバモード」とは呼ばない。詳細は9章「語法」。

### ゲート

3章の禁止語表のとおり、**「合否ゲート」「モデル一致ゲート」「PASS/FAIL ゲート」のような「判定の関門」の意味では使わない**（→「合否判定」「合格基準」「判定条件」）。オーナーの指摘（2026-09-05）はこの用法に対するもの。一方、**電子回路・デジタル論理の正式用語（MOSFET のゲート端子、論理ゲート AND/OR）はそのまま使ってよい**——分野の標準語であり言い換えると不正確になる。状態推定の外れ値棄却に使う「χ² ゲート」（カルマンフィルタの測定更新で残差の χ² 値がしきい値を超えた観測を捨てる処理）は、**「χ² 判定」（カイ二乗判定）と書く**（2026-09-13 オーナー決定）。派生形も同様: 「χ² ゲーティング」→「χ² 判定」、「ゲート閾値」→「判定しきい値」、「ゲートを通過した／ゲートで棄却された」→「判定を通過した／判定で棄却された」、「静止ゲート」（起動時校正の静止判定）→「静止判定」。コマンド名・パラメータ名・識別子（`sf sils sysid-gate`、`eskf.gate.*`、`chi2_gate`、`mag_calib_gate_*` 等）は英語なのでそのまま。

### 汎用の多義語対応表

| 英語 | 使い分け |
|---|---|
| stall | 航空=失速／モータ定格=停動（トルク）／誘導機試験=拘束 |
| constraint | 力学=拘束（拘束力・拘束条件）／最適化=制約（制約条件） |
| tracking | 制御=追従（追従制御）／レーダ・目標=追尾／日常=追跡 |
| trajectory／path | 宇宙・弾道=軌道／描いた跡=軌跡／計画する道筋=経路（経路計画） |
| calibration | 計量法・JIS=校正／防衛・航空系慣用=較正（文書内で統一） |
| preload／pressurization | 軸受・ばね=予圧（初期荷重）／機体・キャビン=与圧 |
| tolerance | 寸法=公差／計測・判定=許容誤差・許容値 |
| damping／attenuation | 振動・制御=減衰／信号の減りも「減衰」だが増幅の逆は「利得低下」とも言う |
| identification | システム同定=同定／個体の判別=識別 |
| frame | 座標の話=座標系（機体座標系・慣性座標系）。「フレーム」は構造部材か動画のコマの意 |
| nominal | 仕様書の値=公称（公称値）／定格の話=定格 |

出典: [micromouse_rl](https://github.com/kouhei1970/micromouse_rl) `docs/JA_ENGINEERING_TERMS.md` §3 を土台にした。

## 7. 学術論文の採用表記

**この章は学会原稿・対外的な学術文書を書くときの表記規約であり、通常の開発文書（README・ログ解析ノート・チャット報告等）の用語を置き換えるものではない。** 例えば「フライトログ」は本リポジトリの開発文書・sf CLI（`sf log` コマンド群）で標準的に使う語であり、下表の「学術論文では『飛行データ』」という規約は論文執筆時にのみ適用する。

専門用語は日本語論文での慣用表記を J-STAGE・CiNii・学会講演集等で確認してから使う（[[feedback_paper_terminology_check]]）。以下は論文執筆時の規約から統合した検証済み表記。出典（原稿名・査読者名）は伏せ、訳語対のみを記載する。

### ドローン・制御・推定分野

| 概念 | 採用表記 | 備考 |
|---|---|---|
| 機体分類 | クアッドロータ（quadrotor）※初出で英語併記 | 表記の揺れが大きいので初出定義後に統一使用 |
| 無人機の総称 | 無人航空機（ドローン）※初出併記、以後「機体」等で受ける | 本文で「ドローン」単独は避ける |
| GPS が使えない環境 | 非GPS環境 | 「GPS非利用環境」は学術実例未発見 |
| 光学式流れセンサ | オプティカルフロー／オプティカルフローセンサ（中黒なし） | 中黒入りは少数派 |
| ToF 測距 | ToF（Time of Flight）測距センサ ※初出併記 | 大文字/小文字の揺れは初出併記で吸収 |
| 位置を保つ制御 | 位置制御／位置保持制御／ホバリング | 「定点保持」は学術実例未発見のため使わない |
| 実際に効くゲイン | 定訳なし。説明的表現（「実際に得られる応答のゲイン」）とし、初出定義してから用語として使う | 「実効ゲイン」の学術実例なし |
| パラメータを求める | システム同定／パラメータ同定／同定 | 「飛行データに基づく同定」の組み立ては可 |
| 頑健性 | ロバスト性／ロバスト設計／ロバスト安定 | 3章「頑強な・堅牢な→ロバストな」と整合 |
| Software-in-the-Loop | SILS（Software In the Loop Simulation）※初出併記 | 5章のSILS規約と同一 |
| SITL | 一般語としては使わない。ArduPilot/PX4 のシミュレータに言及する場合のみ固有名として使う | 6章・5章と同一の判断 |
| 電池の電圧降下現象 | 「電池電圧の低下」「放電に伴う電圧低下」「負荷時の電圧降下」 | 「サグ」は3章の禁止語 |
| 多重ループ制御 | カスケード制御／カスケード構造 | 確立語 |
| 慣性センサ | 慣性計測装置（IMU）※初出併記、以後 IMU | — |
| 飛行記録 | 飛行データ | 「飛行ログ」は学術実例未確認のため学術論文では使わない（開発文書では「フライトログ」を使ってよい） |
| 姿勢推定の誤差状態フィルタ | 誤差状態カルマンフィルタ（Error-State Kalman Filter, ESKF）※初出併記、以後 ESKF | 和訳語の査読実例は少ないため英字定義で導入 |
| 相補フィルタ | 相補フィルタ | 慣用確認済み |
| 外乱推定に基づく補償 | 外乱オブザーバ | 慣用確認済み |
| 積分飽和対策 | アンチワインドアップ | 現象自体は「ワインドアップ」 |
| 微分項の LPF | 不完全微分 | 慣用確認済み（PID教科書） |
| モータへの推力配分 | 定訳なし。初出で「各モータへの推力配分（以下ミキサ）」と説明的に導入する | 「制御配分／コントロールアロケーション」のマルチロータでの学術実例は未発見 |
| 内外ループの応答速度差 | 「時間スケール分離」は実例未発見。「内側ループの応答を外側ループより十分高速にする」等の説明的表現にする | — |
| 発振の閾値 | 「安定境界」は実例未発見。「安定限界」（閾値）／「安定余裕」（マージン）を使う | — |
| 持続振動 | リミットサイクル | 慣用確認済み |
| モータ種別 | コアレスモータ | 慣用確認済み |
| PWM のオン比率 | デューティ比 | 慣用確認済み |
| 回転の4元数表現 | クォータニオン（数理系では「四元数」も慣用） | — |
| 同定結果をモデルへ反映する | 「反映する」「モデルを更新する」 | 「還流」は開発文書の内輪語なので論文では使わない |

### 最適化・MPC（モデル予測制御）分野

学会原稿の査読で確認された訳語（原稿名・査読者名は伏せる）。

| 直訳・借用語（使わない） | 採用表記 |
|---|---|
| 段階コスト | ステージコスト |
| 多レート構成 | マルチレート構成 |
| 二重コア構成 | デュアルコア構成 |
| Cholesky ソルブ | コレスキー分解による求解（前進・後退代入） |
| 準ニュートン差分更新 | 準ニュートン低ランク更新（ランク1更新） |
| 力比 c_tau | トルク・推力比（torque-to-thrust ratio。C_Q/C_T の統一和名はないため英語併記が安全） |
| failsafe 制御 | フェールセーフ制御（JIS Z8115 の正式表記） |
| モーメント腕 | モーメントアーム |
| スタック化した | 「全ステップの状態・入力を縦に並べたベクトルを導入し…」等、書き下し表記に変える |
| Hessian | ヘッセ行列 |
| rank-1／rank-2 | ランク1／ランク2 |
| Box 制約／Box 射影 | ボックス制約／ボックス集合への射影 |
| 凝縮・凝縮形 | 状態消去（condensing）。初出で一度だけ英語を併記する |
| ADMM（展開なしの初出） | 交互方向乗数法 (ADMM)。初出で「制約付き最適化を、制約なし最小化と制約集合への射影の交互反復に分解する解法」と一文説明する |
| TR1 / SR2（展開なしの初出） | TR1 = 両側ランク1更新（two-sided rank-one）、SR2 = 対称ランク2更新（symmetric rank-two）。初出で展開する |
| ZOH | 初出で「零次ホールド (ZOH)」 |
| ESC | 「ESC（モータ駆動アンプ）」等の一言を添える |
| FDI の3ステップ混同 | 検出（detection: 故障が起きたか）・分離（isolation: どの部位が故障したか）・同定（identification: 効率等の値の推定）を区別して書く |

## 8. 文体（AI生成文の癖の排除）

学術論文・対外文書を書くときは、AI（コーディングエージェント）が生成した文章に特有の癖を推敲で取り除く。通常の開発文書・チャット報告では必須ではないが、意識して悪くはない。

### 日本語

| 項目 | 具体例 | 対策 |
|---|---|---|
| ダッシュ（——）の多用 | 「すべて——ファームウェア...——を」 | 「すなわち」「から〜まで」に置き換え、または句点で文を切る |
| 断定回避の連続 | 全段落末尾が「〜と考えられる」「〜が示唆される」 | 断定すべき箇所では断定する。著者の意志は「〜したい」「〜と考えている」 |
| 抽象動詞 | 「活用する」「促進する」「実現する」「統合的に」 | 具体的な行為を書く |
| テンプレ構造 | 導入→列挙→まとめの繰り返し、均一な段落長 | 議論に応じて段落の長さや密度を変える |
| 太字の乱用 | 本文中の強調 | 学会論文では使わない。1文書あたり数個までに絞る |
| 括弧の乱用 | 「シミュレータで試す（実機不要）」 | 括弧なしで意味が通る文に書き直す |
| 自画自賛 | 「設計思想は明快である」 | 客観的な記述に |
| 「これにより」の多用 | AI文章に頻出、人間はあまり使わない | 「そのため」「この結果」等と使い分けるか、削除 |
| 大げさな副詞 | 「飛躍的に」「劇的に」 | 「大幅に」「大きく」、または削除 |

### 英語（Abstract 等）

| 避ける語 | 代替案 |
|---|---|
| delve, explore | investigate, study |
| crucial, vital, pivotal | important, key |
| comprehensive, robust | 不要なら削除 |
| Furthermore, Moreover | 接続詞なしでつなげる |
| dramatically | significantly, greatly |
| maximize | draw out, exploit |
| amplify | extend, broaden |
| not only...but also | 別の構文に |

### チェック方法（自問）

1. 「この文は人間が書いた論文に見えるか」
2. 「ダッシュを使っていないか」
3. 「全段落が同じトーン・同じ長さになっていないか」
4. 「断定すべき箇所で逃げていないか」
5. 「読者の前提知識を仮定した表現になっていないか」

出典: 論文執筆時の規約から統合。

## 9. 語法

意味は正しくても、使い方に制約がある語をまとめる。

### オブザーバ

**制御にフィードバックしない推定を「オブザーバ」「オブザーバモード」と呼んではいけない。** 制御工学でオブザーバはフィードバック制御のための状態推定器を指す語であり、制御に接続しない推定（ログ確認だけ・診断だけの推定）には「オープンループ推定」を使う。

例:「推定だけ動かして制御に使わない」→「オープンループ推定」（誤:「オブザーバモードで動かす」）。

### ゲート

3章・6章のとおり、「判定の関門」の意味（合否ゲート・モデル一致ゲート・PASS/FAIL ゲート）では使わず「合否判定」「合格基準」「判定条件」と言う。電子回路・デジタル論理の正式用語（MOSFET のゲート端子、論理ゲート）はそのまま使ってよい。状態推定の「χ² ゲート」は「χ² 判定」と書く（2026-09-13 決定、6章）。識別子 `sf sils sysid-gate`・`eskf.gate.*` 等は訳さない。

## 10. 提出前チェックリスト

対外文書・重要な報告・コミットメッセージを書いたら、送信前に以下を確認する。

1. **禁止語スキャン**: 3章・4章の左列（使わない語・擬人化表現・現象説明の比喩）が残っていないか。機体で起きた現象は、軸・量・向き・大きさ・時間スケールで書いたか
2. **略語・記号名の初出展開**: `M2`、`sf sils regression`、`exp_020` のような記号名を裸で使っていないか。初出で内容を括弧書きしたか
3. **regression／SILS・HILS／モデルの表記統一**: 5章の規約と一致しているか（特に「再確認試験」「モデル」のカタカナ表記）
4. **多義語チェック**: 段・帯・面・オブザーバ・ゲート・回帰/再確認試験が対象ごとに正しく呼び分けられているか（6章）
5. **学術論文の採用表記**: 学会原稿・対外的な学術文書であれば7章の表記を確認したか（開発文書では不要）
6. **AI生成文の癖のスキャン**: 対外文書・論文では8章のチェックリストを適用したか
7. **専門用語の初出解説**: ソフト用語に一文解説を添えたか。大学3年を超える数学概念を噛み砕いたか（2章）
8. **カタカナ表記の長音ルール**: モータ・センサ・フィルタ・コントローラ等、3音以上の長音省略で統一されているか
9. **指摘された語の記録**: 今回新たに指摘された語があれば、該当章に追記してから提出したか

## 11. 改訂手順と経緯

### 改訂手順

- 追加すべき語に気づいたセッション・エージェントは、本ファイルに追記してからコミットする。
- ユーザーからの指摘は最優先で反映し、指摘された語は必ず該当章（多くは3章か4章）に収載する。
- 他プロジェクトの辞書の更新を機械的に追いかける仕組みは作らない（1章のとおり、同期はしない）。取り込みが必要になったら、そのときに改めて内容を読んで反映する。

### 経緯

| 日付 | できごと |
|---|---|
| 2026-05-31〜2026-08-06 | `feedback_plain_japanese_terms` 系の指摘が蓄積（ハーネス・界面・比力・整流・崖・音/鳴る・スタブ 等） |
| 2026-06-22 | regression → 「退行」の決定（グローバル `~/.claude/CLAUDE.md` に必ず守るルールとして明記） |
| 2026-08-02 | 「サグ」使用禁止 |
| 2026-08-07 | SIL → SILS 統一 |
| 2026-08-10 | `micromouse_rl` に `JA_ENGINEERING_TERMS.md` 新設（本書の雛形の一つ） |
| 2026-08-14 | グローバル `~/.claude/CLAUDE.md` に「用語運用」の5原則（略語展開・内輪語回避・範囲明示・多義語呼び分け・指摘の即時記録）を制定 |
| 2026-08-20 | HIL → HILS 追加統一 |
| 2026-08-30 | 「家事」使用禁止 |
| 2026-09-05 | 「ゲート」使用禁止。`m5stamp_uwb_localizer` に `JA_ENGINEERING_TERMS.md` 新設（擬人化排除、本書のもう一つの雛形） |
| 2026-09-08 | 「解禁」「疎通」使用禁止 |
| 2026-09-13 | 本書 `docs/contributing/terminology.md` を新規制定。`micromouse_rl`・`m5stamp_uwb_localizer` の辞書、記憶ファイル群、論文執筆時の規約を統合。**本リポジトリに限り regression の訳語を「退行」から「再確認試験」へ変更**（5章参照） |
| 2026-09-13 | 「χ² ゲート」は「χ² 判定」と書くと決定（6章）。電子回路・デジタル論理のゲートは従来どおり可。同日、本書の使わない語（ゲート・比力・サグ・律速・解禁・模型・疎通）をリポジトリ全体（文書・コードの日本語コメント・SILS シナリオ・GUI 表示・SCI スライド）で一括是正 |
| 2026-09-13 | 4章に「現象の説明に比喩を使わない」（擬人化・擬動物化・他の物からの比喩を用いず、軸・量・向き・大きさ・時間スケールで書く）を追加。「正本」を使わない語に追加（→ 基準となる文書・基準ファイル・〜を基準とする）。いずれもオーナー指摘 |

---

<a id="english"></a>

## 1. About This Document

This document is the **local source of truth for word choice in the `stampfly_ecosystem` repository only**. No cross-repository sharing mechanism (symlinks, submodules, sync scripts) is created; it draws on similar dictionaries in other repositories (`micromouse_rl`, `m5stamp_uwb_localizer`) as reference material, one-time, without ongoing synchronization.

**Division of labor**: term *meaning* (what an abbreviation stands for) is owned by [`../guides/glossary.md`](../guides/glossary.md); *phrasing* (word choice, banned terms, translation conventions, notation, usage constraints, style) is owned by this document.

## 2. Principles

Abbreviations and symbolic names must be spelled out in parentheses on first use; industry jargon and words borrowed from unrelated fields should be replaced with everyday engineering Japanese; ranges/sets should be described concretely; polysemous words must be disambiguated by context; user-flagged terms are recorded immediately. Software-specific terms get a one-sentence gloss on first use (the project owner's expertise is control engineering and hardware, not software); math explanations assume up to an undergraduate junior level.

## 3. Words to Avoid

A merged table (see the Japanese section) of banned words, direct-translation jargon, borrowed terms, and IT slang, each with a replacement, a category, a field, and the date/source it was established. Roughly 70 entries, consolidated from this project's own feedback history plus the `micromouse_rl` and `m5stamp_uwb_localizer` dictionaries. A noted exception: `micromouse_rl` bans the word "正本" itself, but this repository uses it routinely as a normal technical term, so it is not banned here — the conflict is recorded rather than forced into agreement.

## 4. Avoiding Anthropomorphism of Devices and Software

Devices, tasks, and software should be described by what they actually do, not by human or animal metaphors (dying, sleeping, giving up, believing, etc.). A verb-substitution table and a short list of acceptable idioms (used with a one-time gloss) are given in the Japanese section, taken from `m5stamp_uwb_localizer`'s dictionary.

Added 2026-09-13: phenomena observed on the StampFly (an engineering object) are described by axis, quantity, direction, magnitude, and time scale — never by personification, animal imagery, or metaphors borrowed from other objects (runaway, thrashing, kicking, bowing, cliff, breathing, ...). Established textbook terms of control, electronics, and estimation (overshoot, hunting, limit cycle, drift, saturation, wind-up, derivative kick, latch-up, spike, ringing, dead band, hysteresis, chattering) remain, with a one-sentence gloss at first use.

## 5. Translation and Notation Conventions

- **regression**: statistical regression stays "回帰"; a software regression test is called "再確認試験" (re-verification test) in this repository as of 2026-09-13, superseding the earlier "退行" convention (history table in the Japanese section). "既存動作の破壊" describes breakage itself. English identifiers (`sf sils regression`, the "SILS scenario regression" CI workflow name, "regression" in commit messages) are left untranslated.
- **SIL/HIL → SILS/HILS**: spelled out on first use; SITL is reserved for ArduPilot/PX4's own tool name.
- **model**: always written in katakana (モデル), never with the kanji 模型 (which means a physical scale model).
- Command names, identifiers, and CI workflow names are never translated.
- A general rule of thumb for what to translate vs. keep in English is given, based on conventions confirmed during academic paper review.

## 6. Disambiguating Polysemous Words

Covers regression/re-verification-test, 段 (stage vs. speed level), 帯 (always spell out the concrete range), 面 (restricted to physical surfaces here, not borrowed from the maze-robotics sense), オブザーバ (see Usage, section 9), and ゲート (banned outright in this repository, including its electronics sense — see the Japanese section for the reasoning), plus a general table of engineering homonyms (stall, constraint, tracking, trajectory/path, calibration, preload/pressurization, tolerance, damping/attenuation, identification, frame, nominal).

## 7. Adopted Academic Phrasing

**This chapter applies only when writing for academic conferences/journals** — it does not override everyday project terminology (e.g. "フライトログ" remains standard in this repository's own tooling docs). Two tables list phrasing verified against real Japanese-language publications (J-STAGE/CiNii) for (a) drone/control/estimation topics and (b) optimization/MPC topics, consolidated from paper-writing conventions without naming the specific manuscript or reviewer.

## 8. Style: Removing Signs of AI-Generated Prose

For papers and external documents, remove habits typical of AI-drafted text (overuse of em-dashes, chains of hedging phrases, abstract verbs, boilerplate structure, excessive bold, "これにより" overuse, inflated adverbs), and the English-abstract equivalents (delve, crucial, dramatically, etc.). Not mandatory for everyday chat/dev notes, but good practice.

## 9. Usage Notes

"オブザーバ" (observer) is reserved for a state estimator used in feedback control; an estimate that is not fed back to control is an "オープンループ推定" (open-loop estimate), never an "observer mode." "ゲート" is avoided outright (see section 6), including in its electronics sense.

## 10. Pre-Submission Checklist

Nine checks covering: banned-word/anthropomorphism scan, abbreviation expansion, regression/SILS/HILS/model notation consistency, polysemous-word disambiguation, academic phrasing (for papers only), AI-prose habits (for external documents), one-sentence glosses for software jargon, katakana long-vowel consistency, and recording any newly flagged word before sending.

## 11. Revision Process and History

Anyone who notices a term to add should append it here and commit. User feedback is reflected immediately. No mechanism automatically pulls in updates from other repositories' dictionaries — that stays a manual, occasional act. See the Japanese section for the full timeline, including the 2026-09-13 change from "退行" to "再確認試験" for this repository only.
