# Lesson 12: Python SDK Flight

> **Note:** The `stampfly_edu` package this lesson imports below was removed on 2026-09-13 (tag `archive/2026-09-13`). This procedure will be rewritten when the Workshop is upgraded to the current architecture.
> **注記:** このレッスンが下記でインポートしている `stampfly_edu` は 2026-09-13（タグ `archive/2026-09-13`）に削除された。Workshop の現行基盤へのアップグレード時に手順を作り直す。

## Goal / 目標
Control StampFly from Python using the SDK. Send flight commands from a Jupyter notebook.

Python SDKを使ってStampFlyを操作する。Jupyterノートブックからフライトコマンドを送信する。

## Overview / 概要

This lesson shifts from C++ firmware to **Python-based control**. The firmware side runs a
stable PID controller (from Lesson 8). You will use the Python SDK and `sf` CLI to send
high-level commands like takeoff, hover, and land.

このレッスンではC++ファームウェアからPythonベースの制御に移行する。ファームウェア側は
Lesson 8の安定したPID制御を実行する。Python SDKと`sf` CLIを使って離陸、ホバリング、
着陸などの高レベルコマンドを送信する。

## Architecture / アーキテクチャ
```
┌──────────────────┐     WiFi      ┌──────────────────┐
│  PC / Notebook   │ ◄──────────► │    StampFly      │
│                  │               │                  │
│  Python SDK      │   commands    │  PID Controller  │
│  sf CLI          │ ──────────►  │  Motor Mixer     │
│  Jupyter         │   telemetry  │  Sensors         │
│                  │ ◄──────────  │                  │
└──────────────────┘               └──────────────────┘
```

## Prerequisites / 前提条件

| Item | Description |
|------|-------------|
| Firmware | Flash Lesson 12 (`sf lesson switch 12`) |
| WiFi | Connect PC to StampFly AP (`StampFly_XXXX`) |
| Python | `pip install stampfly-edu` (or `pip install -e ".[edu]"`) |

## Steps / 手順

### 1. Flash stable firmware / 安定ファームウェアを書き込む
```bash
sf lesson switch 12
sf flash vehicle -m
```

### 2. Connect to StampFly WiFi / WiFiに接続する
Connect your PC to the StampFly WiFi access point (SSID: `StampFly_XXXX`).

### 3. Use sf CLI for flight / sf CLIでフライト
```bash
# Takeoff to 0.5m / 0.5mまで離陸
sf flight takeoff

# Hover for 5 seconds / 5秒間ホバリング
sf flight hover --duration 5

# Land / 着陸
sf flight land
```

### 4. Python SDK in Jupyter / JupyterでPython SDKを使う

Open a Jupyter notebook and try the following:

```python
from stampfly_edu import connect_or_simulate

# Connect to real drone or use simulator
# 実機に接続またはシミュレータを使用
drone = connect_or_simulate()

# Takeoff / 離陸
drone.takeoff(altitude=0.5)

# Wait / 待機
import time
time.sleep(3)

# Read telemetry / テレメトリ読み取り
print(f"Altitude: {drone.altitude:.2f} m")
print(f"Roll:     {drone.roll:.1f} deg")
print(f"Pitch:    {drone.pitch:.1f} deg")

# Land / 着陸
drone.land()

# Disconnect / 切断
drone.close()
```

### 5. Capture and plot telemetry / テレメトリを取得してプロットする
```python
import matplotlib.pyplot as plt
from stampfly_edu import connect_or_simulate

drone = connect_or_simulate()
drone.takeoff(altitude=0.5)

# Record data for 5 seconds / 5秒間データを記録
data = drone.record(duration=5.0)

drone.land()
drone.close()

# Plot altitude / 高度をプロット
plt.figure(figsize=(10, 4))
plt.plot(data["time"], data["altitude"], label="Altitude")
plt.xlabel("Time [s]")
plt.ylabel("Altitude [m]")
plt.title("StampFly Altitude during Hover")
plt.legend()
plt.grid(True)
plt.show()
```

## Firmware (C++ Side) / ファームウェア（C++側）

For this lesson, both `student.cpp` and `solution.cpp` contain the same PID controller from
Lesson 8. The focus is on the Python side, not the C++ side.

このレッスンでは `student.cpp` と `solution.cpp` はどちらもLesson 8のPID制御と同じ内容。
焦点はPython側であり、C++側ではない。

## Useful Commands / 便利なコマンド
| Command | Description |
|---------|-------------|
| `sf flight takeoff` | Takeoff to default altitude |
| `sf flight land` | Land and disarm |
| `sf flight hover --duration N` | Hover for N seconds |
| `sf log wifi` | Stream real-time telemetry |
| `sf monitor` | Serial debug output |

## Key Concepts / キーコンセプト
- Python SDK enables rapid prototyping and data analysis
- `connect_or_simulate()` works with both real drone and simulator
- WiFi telemetry provides real-time data at 400Hz
- High-level commands abstract away low-level PID control
- Jupyter notebooks combine code, visualization, and documentation
