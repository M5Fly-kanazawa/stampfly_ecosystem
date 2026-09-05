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
