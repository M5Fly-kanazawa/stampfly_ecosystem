# よくある問題と解決方法

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 環境セットアップ

| 問題 | 原因 | 解決策 |
|------|------|--------|
| `ModuleNotFoundError: stampfly_edu` | パッケージ未インストール | `pip install -e ".[edu]"` を実行 |
| `ImportError: ipywidgets` | 教育用依存関係なし | `pip install -e ".[edu]"` を実行 |
| `FileNotFoundError: sample data` | サンプルデータ未生成 | `python -m stampfly_edu.generate_samples` を実行 |
| Jupyter が起動しない | jupyter 未インストール | `pip install jupyter` を実行 |
| ウィジェットが表示されない | ipywidgets の設定 | `jupyter nbextension enable --py widgetsnbextension` |

## 2. WiFi 接続

| 問題 | 原因 | 解決策 |
|------|------|--------|
| StampFly の WiFi が見つからない | 電源が入っていない | バッテリーを確認、電源を入れ直す |
| 接続しても通信できない | IP アドレスが違う | `192.168.10.1` を確認 |
| 接続が不安定 | 距離が遠い | StampFly に近づく（1m以内推奨）|
| タイムアウトエラー | ファームウェアが応答しない | StampFly を再起動 |

## 3. 飛行関連

| 問題 | 原因 | 解決策 |
|------|------|--------|
| モーターが回らない | バッテリー切れ | バッテリーを充電 |
| 離陸しない | キャリブレーション未実施 | `sf cal gyro` を実行 |
| ドリフトする | 磁気キャリブレーション未実施 | `sf cal mag` を実行 |
| 急に落ちる | バッテリー電圧低下 | バッテリーを交換・充電 |
| 振動する | PID ゲインが高すぎる | ゲインを下げる |

## 4. ノートブック実行

| 問題 | 原因 | 解決策 |
|------|------|--------|
| プロットが表示されない | バックエンドの設定 | `%matplotlib inline` をセルの先頭に追加 |
| sympy の数式が表示されない | LaTeX 未設定 | `from IPython.display import Math` を使用 |
| カーネルが落ちる | メモリ不足 | 大きなデータセットを分割して処理 |
| グラフが文字化け | フォント設定 | `plt.rcParams['font.family'] = 'sans-serif'` |

## 5. シミュレータ関連

| 問題 | 原因 | 解決策 |
|------|------|--------|
| シミュレータに自動フォールバック | WiFi 未接続 | 意図的ならそのまま使用可 |
| VPython が起動しない | vpython 未インストール | `pip install vpython` |
| シミュレーションが遅い | データ点数が多い | dt を大きくする |

## 6. グラフ表示（matplotlib）

| 症状 | 原因 | 解決策 |
|------|------|--------|
| `sf log viz` / `sf sysid fit --plot` でウィンドウが開かず、`FigureCanvasAgg is non-interactive, and thus cannot be shown` という警告だけが出て終わる | この Python の matplotlib（グラフ描画ライブラリ）に使える GUI バックエンド（ウィンドウ表示の仕組み）が無く、自動的に画面を出さない "Agg" バックエンドが選ばれている | 何もしなくても sf が自動でログの隣に `<ログ名>.png` を保存し、既定の画像ビューアで開く（下記参照）。恒久的に直したい場合は下表の対処を行う |
| `Can't find a usable init.tcl in the following directories: ...` という `TclError` で終了する | pyenv-win 等でインストールした Python の Tcl/Tk（Tkウィンドウ表示に必要な一式）が不完全 | 同上（sf が自動で PNG 保存に切り替える）。恒久的に直すには下表の「pyenv-win」の行を参照 |

**sf が自動で行うこと（2026-09 対応）:** `lib/sfcli/utils/plotting.py` が、matplotlib の GUI バックエンドを候補順（macOS: macosx → Qt → Tk、Windows: Tk → Qt → wx、Linux: Qt → GTK → Tk → wx）に1つずつ試し、実際に使える物だけを選ぶ。全て使えない場合だけ画面なしの "Agg" に切り替え、ログファイルと同じフォルダに `<ログ名>.png` を保存して OS 標準の画像ビューアで自動的に開く。この仕組みは `sf log viz` と `sf sysid fit/noise --plot` の両方に入っている。**さらに（2026-09 追加の根本対処）** インストーラ（`install.bat` / `install.sh`）と `sf upgrade` は、GUI バックエンドが使える状態かを毎回確認し、Tk（Tcl/Tk 一式）が使えない環境では sf の Python 環境に自動で PyQt6（Qt バインディング、GUI ウィンドウ表示の仕組みの一種）を導入する。既にインストール済みの環境でこれを今すぐ反映したい場合は `sf doctor --fix` を実行する。

**事前確認:** `sf doctor` を実行し、「Checking plot window support (matplotlib)」の項目を見る。`GUI backend: macosx`（等）と出れば正常、`GUI backend: NONE` ならこの節の対処が必要（`sf doctor --fix` で自動修復を試みる）。

**恒久的に直す方法:**

| 環境 | 対処 |
|------|------|
| まず試す | `sf doctor --fix`（sf の Python 環境に PyQt6 を自動導入する） |
| Windows（新規インストール） | python.org から Python を再インストールする際、「tcl/tk and IDLE」にチェックを入れる。その後 `install.bat` を再実行する |
| Windows / macOS 共通 | sf の Python 環境に Qt バックエンドを追加する: `pip install "PyQt6>=6.5,<7"` |
| Linux | `sudo apt install python3-tk`。PyQt6 導入済みなのに起動しない場合は `libxcb-cursor0`（Qt6 のプラットフォームプラグインが必要とするライブラリ。Ubuntu 22.04 以降で必要になることがある） |
| pyenv-win（`Can't find a usable init.tcl` の場合） | 環境変数 `TCL_LIBRARY` / `TK_LIBRARY` を、ベースにした Python 本体の `tcl\tcl8.6` / `tcl\tk8.6` フォルダに設定する |
| バックエンドを自分で固定したいとき | 環境変数 `MPLBACKEND` で明示指定する（例: `MPLBACKEND=agg`）|
| そもそもウィンドウ不要で保存だけでよいとき | `--save <ファイル名>`（`sf log viz`）や `--plot-output <ファイル名>`（`sf sysid fit/noise`）を付けて実行する |

---

<a id="english"></a>

## 1. Environment Setup

| Issue | Cause | Solution |
|-------|-------|----------|
| `ModuleNotFoundError: stampfly_edu` | Package not installed | Run `pip install -e ".[edu]"` |
| `ImportError: ipywidgets` | Missing edu dependencies | Run `pip install -e ".[edu]"` |
| `FileNotFoundError: sample data` | Sample data not generated | Run `python -m stampfly_edu.generate_samples` |

## 2. WiFi Connection

| Issue | Cause | Solution |
|-------|-------|----------|
| Cannot find StampFly WiFi | Not powered on | Check battery, restart |
| Connected but no communication | Wrong IP | Verify `192.168.10.1` |
| Unstable connection | Too far | Move closer (within 1m) |

## 3. Flight Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| Motors don't spin | Dead battery | Charge battery |
| Won't take off | Not calibrated | Run `sf cal gyro` |
| Drifts | Mag cal needed | Run `sf cal mag` |

## 4. Notebook Execution

| Issue | Cause | Solution |
|-------|-------|----------|
| No plots | Backend config | Add `%matplotlib inline` |
| Kernel crashes | Out of memory | Process data in chunks |

## 5. Simulator

| Issue | Cause | Solution |
|-------|-------|----------|
| Automatic fallback to the simulator | WiFi not connected | Fine to keep using it if intended |
| VPython does not start | vpython not installed | `pip install vpython` |
| Simulation is slow | Too many data points | Increase dt |

## 6. Plot Window (matplotlib)

| Symptom | Cause | Solution |
|---------|-------|----------|
| `sf log viz` / `sf sysid fit --plot` opens no window, and only prints a `FigureCanvasAgg is non-interactive, and thus cannot be shown` warning before exiting | This Python's matplotlib (the plotting library) has no usable GUI backend (window-display mechanism), so it silently picked the headless "Agg" backend | sf now saves a `<log>.png` next to the log and opens it with the default image viewer automatically (see below). To fix it permanently, see the table below |
| Exits with `TclError: Can't find a usable init.tcl in the following directories: ...` | The Python install (often via pyenv-win) has an incomplete Tcl/Tk (the toolkit Tk windows need) | Same as above -- sf automatically switches to saving a PNG. For a permanent fix, see the "pyenv-win" row below |

**What sf now does automatically (added 2026-09):** `lib/sfcli/utils/plotting.py` tries each GUI backend in order (macOS: macosx -> Qt -> Tk; Windows: Tk -> Qt -> wx; Linux: Qt -> GTK -> Tk -> wx) and picks the first one that actually works. Only when none of them work does it fall back to the headless "Agg" backend, save a `<log>.png` next to the log file, and open it with the OS default image viewer. This applies to both `sf log viz` and `sf sysid fit/noise --plot`. **Also (2026-09 root fix):** the installer (`install.bat` / `install.sh`) and `sf upgrade` both check for a working GUI backend every time they run, and install PyQt6 (a Qt binding, one kind of GUI window-display mechanism) into the sf Python environment automatically when Tk (the Tcl/Tk toolkit) is unusable. For an existing install, run `sf doctor --fix` to apply this right now.

**To check:** run `sf doctor` and look at "Checking plot window support (matplotlib)". `GUI backend: macosx` (or similar) means it is working; `GUI backend: NONE` means this section applies (try `sf doctor --fix` for an automatic repair).

**Permanent fixes:**

| Environment | Fix |
|-------------|-----|
| Try this first | `sf doctor --fix` (installs PyQt6 into the sf Python environment automatically) |
| Windows (fresh install) | Reinstall Python from python.org with "tcl/tk and IDLE" checked, then run `install.bat` again |
| Windows / macOS | Install a Qt backend into the sf Python environment: `pip install "PyQt6>=6.5,<7"` |
| Linux | `sudo apt install python3-tk`. If PyQt6 is installed but still fails to start, also install `libxcb-cursor0` (a library the Qt6 platform plugin needs, sometimes missing on Ubuntu 22.04+) |
| pyenv-win (`Can't find a usable init.tcl`) | Set the `TCL_LIBRARY` / `TK_LIBRARY` environment variables to the base Python install's `tcl\tcl8.6` / `tcl\tk8.6` folders |
| To pin the backend yourself | Set the `MPLBACKEND` environment variable (e.g. `MPLBACKEND=agg`) |
| When a window is not needed at all | Pass `--save <file>` (`sf log viz`) or `--plot-output <file>` (`sf sysid fit/noise`) |
