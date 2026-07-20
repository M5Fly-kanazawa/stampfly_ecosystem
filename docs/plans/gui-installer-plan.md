# エコシステム GUI インストーラ（StampFly Setup）実装計画

作成: 2026-07-20。発案: ユーザー「CLIのインストーラーは今後も修正発展させる。それに
基づいたGUIインストーラが今後の普及の鍵」。全OS（Windows / macOS ARM / macOS Intel /
Linux）版を用意する。

## 1. 設計方針

### 単一実体の原則（CLI が本体、GUI は皮）

インストールの実ロジックは今後も `scripts/installer.py`（CLI）だけが持つ。GUI は
**ウィザード画面と進捗表示だけ**を担い、実処理は「クローンしたリポジトリの
`installer.py` をプロセス内 import して実行」する。これにより:

- CLI 側の修正・発展が**そのまま GUI に反映される**（GUI の再リリース不要。GUI は
  常に「いまクローンした最新の installer.py」を実行するため）
- テスト・保守の対象が1つに保たれる

### 実行モデル

| 項目 | 決定 |
|------|------|
| 配布形式 | PyInstaller --onefile（フラッシャと同じ4OSマトリクス）。**利用者のPCに Python が無くても起動できる** |
| GUI ツールキット | tkinter（フラッシャと同じ。追加依存なし） |
| リポジトリ取得 | GUI 自身が `git clone`（既定 `~/stampfly_ecosystem`、変更可）。既存 checkout 検出時は「そのまま使う（修復）/ アンインストール」を提示 |
| installer.py 実行 | クローン先の `scripts/installer.py` を `importlib` でプロセス内 import し、`Installer().run(...)` をワーカースレッドで実行。stdout/stderr をキュー経由でログ画面へ |
| 非対話化の契約 | 環境変数 `SF_INSTALLER_NONINTERACTIVE=1` を installer.py の `prompt()`/`prompt_choice()` が見て既定値を即返す。加えて全 prompt を EOF 安全化（stdin が無い環境では既定値）— **古い installer.py と新しい GUI が混ざっても劣化許容**（環境変数を知らない旧版でも EOF 既定値で進む） |
| 進捗表示の契約 | installer.py の `Step N/4:` ヘッダ行を GUI がパースしてステップインジケータを進める（この書式を契約としてコメントに明記） |
| 文字コード | GUI 内部は UTF-8、subprocess 捕捉は `encoding="utf-8", errors="replace"`（cp932 教訓） |

### 前提条件の扱い

| 前提 | GUI での扱い |
|------|-------------|
| git | 起動後の環境チェック画面で検査。無ければ OS 別の導入コマンド（winget / brew / apt）をコピー ボタン付きで提示し「再チェック」 |
| Python 3.8+（システム） | installer.py 自体は GUI 同梱の Python で動くため**不要になる**。ただし ESP-IDF 自身のインストーラが必要とする場合があるため、無ければ警告+導入案内（エラーにはしない） |
| ディスク空き・ネットワーク | チェック画面で目安を表示（ESP-IDF 含め数GB） |

## 2. 画面構成（ウィザード）

1. **ようこそ** — 何が入るか（sf CLI / ESP-IDF / フラッシャ）、目安サイズ・時間
2. **環境チェック** — git / Python / ディスク / ネットワーク。NG項目は対処法+再チェック
3. **オプション** — インストール先、フラッシャアプリ（既定ON）、ショートカット（既定ON）、
   minimal（シミュレータ省略、既定OFF）。既存インストール検出時は 修復/アンインストール を選択可
4. **実行** — ステップインジケータ（clone → Step1/4〜4/4）+ ライブログ
5. **完了** — 成功: 次にやること（`sf doctor` 等）/ 失敗: ログ保存ボタン+CLI 復旧手順

## 3. 成果物・配布

| 項目 | 内容 |
|------|------|
| ソース | `tools/installer_gui/stampfly_installer.py`（stdlib+tkinter のみ） |
| アセット名 | `StampFlySetup_<tag>_windows-x64.exe` / `_macos-arm64.zip` / `_macos-x64.zip` / `_linux-x64` |
| CI | release.yml にフラッシャと同型の 4OS マトリクスを追加。ビルド後に凍結バイナリで `--selftest` を実行（後述） |
| リリースアセット数 | 9 → **13**（versioning.md §3/§5、release-workflow.md の記載も更新） |
| アイコン | フラッシャで確立した 3D レンダパイプラインを流用（稲妻の代わりに下向き矢印=導入のメタファー） |

## 4. 検証計画

| 層 | 内容 |
|----|------|
| selftest（CI・凍結バイナリで実行） | Tk を開かずに: installer.py のプロセス内 import 契約（`Installer` クラスと `run/uninstall/clean` の署名）、`SF_INSTALLER_NONINTERACTIVE` の応答、引数組み立て、前提条件プローブを検証。**凍結バイナリで走らせることで stdlib 同梱漏れも検出** |
| check スクリプト | `tools/ci/check_installer_gui.py`（依存ゼロ、4OS レッグで常時実行） |
| 手元 E2E | macOS でビルド→起動→環境チェック/オプション画面の動作確認（フル install E2E は ESP-IDF ダウンロード数GBを伴うため実機・ユーザー確認と併せて実施） |
| 実機 E2E | まっさらな Windows / macOS で配布バイナリから完走（リリース前のユーザー確認項目） |

## 5. 制約・注意

- installer.py は **stdlib のみ**を維持する（GUI の凍結 Python で import するため）。
  新しい stdlib import を installer.py に足す場合は
  `tools/installer_gui/stampfly_installer.py` の hidden-import 一覧にも追記する（契約を
  両ファイルのコメントに明記）
- GUI からのキャンセルは「フェーズ境界で停止」（installer.py 実行中の強制中断はしない）
- クローン対象は `main`（以後の更新は `sf upgrade` に接続する、既存のライフサイクル通り）

## 6. リリース載せ先

実装完了・CI 4OS 緑・実機 E2E 後に、v2026.07.2（タグ未発行のため同梱可）または
v2026.07.3 として発行するかはタグ時点のユーザー判断。
