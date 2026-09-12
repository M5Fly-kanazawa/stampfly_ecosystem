# notebooks/

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

探索的解析用の Jupyter Notebook を置く。`education/` には大学講義向けの連番ノートブック
（Session 1〜15、`lib/stampfly_edu` を使用）がある。

## 2. education/ の一覧

| ファイル | 内容 |
|---|---|
| `01_hello_stampfly.ipynb` | StampFly とドローン制御の世界 — Python SDK で StampFly を操作し、初飛行を体験する |
| `02_autonomous_flight.ipynb` | プログラムで自律飛行 — 矩形パスを自律飛行し、ログを取得して軌跡をプロットする |
| `03_feedback_basics.ipynb` | フィードバック制御入門 — P 制御で1次系の位置制御を体験し、フィードバックの概念を理解する |
| `04_pid_theory.ipynb` | PID 制御の理論と実装 — P/I/D 各項の役割を理解し、工学形式（Kp, Ti, Td）で PID を設計する |
| `05_rate_control_tuning.ipynb` | 実機で PID を感じる（角速度制御） — レート PID ゲインを変更し、ステップ応答を比較して効果を体感する |
| `06_drone_dynamics.ipynb` | ドローンの数学モデル — 6DoF 運動方程式、DC モータモデル、ホバー条件を理解する |
| `07_system_identification.ipynb` | システム同定 — 実機データから Allan 分散、センサノイズ特性、慣性モーメントを同定する |
| `08_sensor_fusion.ipynb` | センサフュージョンとカルマンフィルタ — ジャイロ積分 vs ESKF を比較し、R パラメータの感度を分析する |
| `09_cascade_attitude.ipynb` | 姿勢制御（カスケードの概念） — 内側ループ（Rate）と外側ループ（Angle）のカスケード構造を理解する |
| `10_altitude_control.ipynb` | 高度制御（フィードフォワードとアンチワインドアップ） — ホバー推力フィードフォワードの効果を体感する |
| `11_position_control.ipynb` | 位置制御（座標変換と外乱抑制） — NED/Body 座標変換と 4 段カスケードの全体像を理解する |
| `12_waypoint_mission.ipynb` | ウェイポイント飛行 — 複数点を経由する自律飛行を実装し、軌跡精度を定量評価する |
| `13_custom_controller.ipynb` | カスタムコントローラ — `send_rc_control()` + `get_telemetry()` で外部 PID ループを実装する |
| `14_project_template.ipynb` | 最終プロジェクト テンプレート — Session 14-15 用のプロジェクト雛形 |
| `15_analysis_toolkit.ipynb` | ログ解析ユーティリティ集 — フライトログ解析に使える関数とプロットのコレクション |

## 3. 使い方

```bash
pip install -e ".[edu]"
cd analysis/notebooks/education
jupyter notebook
```

`edu` extra（`pyproject.toml`）に `jupyter`・`ipywidgets`・`sympy`・`python-control` が含まれる。

---

<a id="english"></a>

## 1. Overview

This directory holds Jupyter notebooks for exploratory analysis. `education/` contains a
numbered series of university course notebooks (Session 1-15, using `lib/stampfly_edu`).

## 2. education/ Listing

| File | Content |
|---|---|
| `01_hello_stampfly.ipynb` | The World of StampFly and Drone Control — first flight using the Python SDK |
| `02_autonomous_flight.ipynb` | Autonomous Flight with Programming — fly a rectangular path, capture logs, plot the trajectory |
| `03_feedback_basics.ipynb` | Introduction to Feedback Control — position control of a first-order system with P control |
| `04_pid_theory.ipynb` | PID Control Theory and Implementation — design PID in engineering form (Kp, Ti, Td) |
| `05_rate_control_tuning.ipynb` | Feel PID on Real Hardware (Angular Rate Control) — compare step responses across rate PID gains |
| `06_drone_dynamics.ipynb` | Drone Mathematical Model — 6DoF equations of motion, DC motor model, hover condition |
| `07_system_identification.ipynb` | System Identification — Allan variance, sensor noise, and moments of inertia from real data |
| `08_sensor_fusion.ipynb` | Sensor Fusion and Kalman Filter — gyro integration vs ESKF, R parameter sensitivity |
| `09_cascade_attitude.ipynb` | Attitude Control (The Cascade Concept) — inner Rate loop and outer Angle loop structure |
| `10_altitude_control.ipynb` | Altitude Control (Feedforward and Anti-Windup) — effect of hover thrust feedforward |
| `11_position_control.ipynb` | Position Control (Coordinate Transform and Disturbance Rejection) — the full 4-stage cascade |
| `12_waypoint_mission.ipynb` | Waypoint Mission — autonomous multi-waypoint flight with quantitative accuracy evaluation |
| `13_custom_controller.ipynb` | Custom Controller — external PID loop using `send_rc_control()` + `get_telemetry()` |
| `14_project_template.ipynb` | Final Project Template — project template for Session 14-15 |
| `15_analysis_toolkit.ipynb` | Log Analysis Toolkit — a collection of functions and plots for flight log analysis |

## 3. Usage

```bash
pip install -e ".[edu]"
cd analysis/notebooks/education
jupyter notebook
```

The `edu` extra (`pyproject.toml`) includes `jupyter`, `ipywidgets`, `sympy`, and `python-control`.
