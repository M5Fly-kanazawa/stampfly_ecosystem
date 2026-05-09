# StampFly モータ動作確認チュートリアル（臨時用）

> このドキュメントは「StampFly のモータをハードコードした任意の Duty で回したい」という単一目的のための **臨時手順書** です。Mac の開発環境がゼロの状態から、リポジトリの取得 → Workshop Lesson 1 のカスタマイズ → ペアリング → 動作確認 → 工場出荷状態への復元 → 環境のアンインストールまでを通しで行います。

---

## 0. 必要なもの

| 項目 | 備考 |
|------|------|
| Mac | macOS 12 (Monterey) 以降を推奨（古い版でも動く可能性あり） |
| StampFly 本体 | 機体 |
| StampFly コントローラ | 送信機（M5 製） |
| USB-C ケーブル × 2 | データ通信対応のもの。充電専用ケーブルでは書き込めない |
| バッテリー | 満充電 |
| インターネット接続 | ESP-IDF やライブラリ取得に必要 |

---

## 1. ツールのインストール（環境ゼロから）

### 1-1. Xcode Command Line Tools

```bash
xcode-select --install
```

ダイアログが出たら "Install" をクリック。完了まで数分。

### 1-2. Homebrew

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

インストール後、ターミナルに表示される指示に従って `brew` を PATH に追加（M1/M2 Mac では `eval "$(/opt/homebrew/bin/brew shellenv)"` を `~/.zshrc` に追記する案内が出る）。

### 1-3. ビルドツール一式

```bash
brew install cmake ninja dfu-util ccache
```

### 1-4. Python 3.8+（既に入っていれば不要）

macOS には Xcode CLT 経由などで Python が入っていることが多いので、まずバージョンを確認します:

```bash
python3 --version
```

`Python 3.8.0` 以上が表示されればこの節はスキップして 1-5 へ。表示されない、または `3.7` 以下の場合のみインストール:

```bash
brew install python@3.12
```

### 1-5. エディタ

`sf lesson edit` は VSCode → vi/vim の順で探して起動します。`vi` は macOS 標準なので追加インストール不要、VSCode を使いたい場合は以下でインストールします。

#### VSCode

既に入っているか確認:

```bash
code --version
```

バージョンが表示されればこの節は完了。表示されなければインストール:

```bash
# Homebrew で（推奨）
brew install --cask visual-studio-code
```

または公式サイト [https://code.visualstudio.com/](https://code.visualstudio.com/) から `.dmg` をダウンロードして `/Applications/` に配置。

`brew install --cask` でインストールした場合は `code` コマンドが自動で PATH に入ります。`.dmg` から手動インストールした場合は、VSCode を起動して **コマンドパレット (`Cmd+Shift+P`)** から `Shell Command: Install 'code' command in PATH` を実行してください。

#### vim（任意）

macOS 標準の `vi` で十分です。新しい vim を使いたい場合のみ:

```bash
brew install vim
```

### 1-6. 動作確認

```bash
xcode-select -p
brew --version
cmake --version
ninja --version
python3 --version   # 3.8 以上であること
code --version      # VSCode を入れた場合
```

---

## 2. リポジトリの取得

```bash
mkdir -p ~/work
cd ~/work
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
```

以降、断りのない限りカレントディレクトリは `~/work/stampfly_ecosystem` です。

---

## 3. ESP-IDF のインストール

リポジトリ付属のインストーラが ESP-IDF v5.5.2 を `~/esp/esp-idf` に自動セットアップします。

```bash
./install.sh
```

5〜15 分ほどかかります。途中で確認プロンプトが出たら `Y` で続行。完了したら **新しいターミナルタブを開く**（環境変数を反映するため）。

---

## 4. 開発環境のアクティブ化

ターミナルを開く度に必要です。

```bash
cd ~/work/stampfly_ecosystem
source setup_env.sh
```

成功すると `[OK] StampFly development environment ready.` と表示されます。続けて環境診断:

```bash
sf doctor
```

すべて OK なら次へ。エラーが出た場合は `docs/setup/macos.md` の「9. トラブルシューティング」を参照。

---

## 5. コントローラのファームウェア書き込み

> 以降の作業はコントローラと機体の両方を扱います。**書き込み対象を間違えないように、片方ずつ USB 接続して進めてください。**

1. コントローラを USB-C ケーブルで Mac に接続する
2. コントローラの電源を入れる
3. 書き込み:
   ```bash
   sf flash controller
   ```
4. 完了したらコントローラを USB から外す

> ポートが自動検出できない場合は `-p /dev/tty.usbserial-XXXX` のように明示できます。

---

## 6. 機体ファームウェアのカスタマイズ（Workshop Lesson 1）

### 6-1. Lesson 1 にスイッチ

```bash
sf lesson switch 1
```

これで編集対象のファイルが Lesson 1 の雛形に切り替わります。

### 6-2. user_code.cpp を開く

```bash
sf lesson edit
```

VSCode（なければ vi/vim）が新規ウィンドウで開きます。

### 6-3. 任意の Duty でモータを回すコードに書き換える

`user_code.cpp` を以下に置き換えてください。各モータごとに 1 行ずつ並んでいます。**回したいモータの数値（Duty）を 0.0 〜 0.15 の間で設定**すればそのモータが回り、**0.0 のままならそのモータは止まったまま**です。

```cpp
#include "workshop_api.hpp"

// =========================================================================
// Motor duty test (hardcoded)
// 各モータの Duty をハードコードする
// =========================================================================
//
// Motor IDs / モータ ID:
//   1 = FR (右前)   2 = RR (右後)
//   3 = RL (左後)   4 = FL (左前)
//
// Duty range: 0.0 - 1.0
//   0.0  = stop / 止める
//   0.10 = spin at 10% / 10% で回す
//   このチュートリアルでは 0.15 を上限とする

void setup()
{
    ws::print("Motor duty test");

    // Do NOT auto-arm in code. Arming is controlled by the controller's
    // ARM button so motors never spin until you intentionally press it.
    // コードでは自動 ARM しない。アーム/解除はコントローラの ARM ボタンで
    // 行うので、意図的に押すまでモータは絶対に回らない
}

void loop_400Hz(float dt)
{
    // ----- Set each motor's duty here / 各モータの Duty をここで設定 -----
    ws::motor_set_duty(1, 0.10f);   // FR (右前)
    ws::motor_set_duty(2, 0.00f);   // RR (右後)
    ws::motor_set_duty(3, 0.00f);   // RL (左後)
    ws::motor_set_duty(4, 0.00f);   // FL (左前)
    // -------------------------------------------------------------------
}
```

回したいモータの行の数値（例: `0.00f`）を `0.10f` などに書き換えるだけです。複数モータを同時に回すこともできます。

保存してエディタを閉じて構いません（書き込み時にエディタは不要）。

### 6-4. ビルドして機体に書き込む

機体を USB-C で Mac に接続し、機体の電源を入れたうえで:

```bash
sf lesson build
sf lesson flash
```

`sf lesson flash` はフラッシュ後にシリアルモニタを開きます。

- 起動時に `Motor duty test` のログが流れれば成功
- モニタ終了は `Ctrl+]`

---

## 7. ペアリング（機体 ↔ コントローラ）

> このチュートリアルではコントローラの ARM ボタンを押した時にモータが回り出す運用なので、**ペアリングは必須**です。先にここで完了させてください。

ESP-NOW モード（出荷既定値）の手順:

1. **コントローラ**: M5 ボタン（画面下のボタン）を **押しながら電源を入れる**
   - LCD に `Pairing mode...` と表示され、ビープが鳴ってペアリング受付状態になる
2. **機体**: そのまま StampFly の電源を入れる
   - 起動中にコントローラ側を見つけて自動でペアリングが成立する
3. 両機ともビープが鳴ればペアリング完了
4. ペアリング情報は自動保存され、次回以降は自動接続

うまくいかない時の対処:
- **機体側の長押しを試す**: コントローラを上記 1 のペアリング受付状態にしたまま、機体の StampFly ボタンを約 2 秒長押しして機体側からもペアリングモードに入る
- 両機をいったん電源 OFF → 1 からやり直す
- USB は外しておく（給電だけしての電源 ON でも可）
- 周囲に他の StampFly がない場所でやる

---

## 8. モータ動作確認

1. **機体上部の M5StampS3（基板モジュール）を上から指で押さえる** — 4 モータが同時に回ると機体が一気に飛び上がるため、突然の離陸を物理的に止められる状態にしてから次へ進むこと。机上に置いた機体の M5StampS3 を真上から押さえつけるイメージ
2. バッテリーを接続して機体の電源を入れる（USB 給電のままでも可）。この時点ではアームしていないのでモータは回らない
3. シリアルモニタに `Motor duty test` が出ることを確認
4. コントローラの電源を入れ、ペアリング済みであれば機体と自動接続される
5. **押さえる手を保ったまま**、コントローラの **ARM ボタンを 1 回押す** → 設定した Duty でモータが回転開始
6. 指定したモータが指定の Duty で回ることを確認

止め方:
- コントローラの **ARM ボタンをもう一度押す** → モータが停止
- 完全に切る: バッテリ／USB を外す、または機体電源 OFF

Duty や回すモータを変えたいとき:
- 6-3 のコードに戻り、対象モータの行（例: `ws::motor_set_duty(1, 0.10f);`）の数値を書き換える
- 6-4 (`sf lesson build && sf lesson flash`) を再実行

---

## 9. 工場出荷状態に戻す

検証が終わったら、機体・送信機を純正ファームへ戻します。

### 9-1. 機体を戻す

機体を USB-C で接続したうえで:

```bash
sf flash vehicle --legacy
```

### 9-2. コントローラを戻す

コントローラを USB-C で接続したうえで（機体は外す）:

```bash
sf flash controller --legacy
```

> 各コマンド実行前に `source setup_env.sh` 済みであることを確認してください。新しいターミナルから始めた場合は再度 `source` する必要があります。

### 9-3. 戻したあとの再ペアリング

戻したあとは **7. ペアリング** をもう一度実施してください。

---

## 10. 環境のアンインストール

検証が終わって開発環境ごと片付けたい場合の手順です。**戻すレベルに応じて段階的に削除**できます。他のプロジェクトで Homebrew や Python を使っている場合は 10-3 以降には踏み込まないでください。

### 10-1. リポジトリと ESP-IDF を消す（StampFly 関連だけ削除）

最低限これだけで「StampFly の開発環境」は消えます。Homebrew や Python は残るので、他用途には影響しません。

```bash
# リポジトリ
rm -rf ~/work/stampfly_ecosystem

# ESP-IDF 本体（install.sh が ~/esp/esp-idf に置いたもの）
rm -rf ~/esp/esp-idf
rmdir ~/esp 2>/dev/null   # ~/esp に他のものが無ければ削除される

# ESP-IDF のツールチェイン
rm -rf ~/.espressif
```

### 10-2. シェル設定の掃除

`~/.zshrc` 等に `~/esp/esp-idf/export.sh` を読み込む行を自分で追記していた場合は、その行を削除してターミナルを開き直してください。`source setup_env.sh` だけで使っていた場合は何もしなくて構いません。

### 10-3. Homebrew で入れたツールを消す（任意）

このチュートリアル用に新しく入れたツールだけ消す場合:

```bash
# ビルドツール（StampFly 専用に入れた場合のみ）
brew uninstall cmake ninja dfu-util ccache

# Python（他で使っていないことを確認してから）
brew uninstall python@3.12

# VSCode（必要なら）
brew uninstall --cask visual-studio-code

# vim を追加インストールした場合のみ
brew uninstall vim
```

> **注意:** これらは他のプロジェクトでも一般的に使うツールです。他で使っていないか確認してから消してください。

### 10-4. Homebrew 自体を消す（任意）

Mac を完全に元の状態に戻したい場合のみ実行:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/uninstall.sh)"
```

### 10-5. Xcode Command Line Tools（通常残す）

CLT は git や他の開発でも使うため、残しておくのが無難です。どうしても削除したい場合のみ:

```bash
sudo rm -rf /Library/Developer/CommandLineTools
```

再導入は `xcode-select --install`。

---

## 関連ドキュメント

| ドキュメント | 内容 |
|--------------|------|
| `docs/setup/macos.md` | Mac セットアップ詳細 |
| `firmware/workshop/lessons/lesson_01_motor/README.md` | Lesson 1 解説 |
| `firmware/workshop/lessons/lesson_02_controller/README.md` | コントローラ入力を使うレッスン |
| `docs/getting-started.md` | 全体のスタートガイド（ペアリング・通信モード詳細含む） |
| `docs/guides/safety.md` | 安全ガイドライン |
