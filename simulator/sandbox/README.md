# Sandbox - STL Viewer Experiment

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このフォルダについて
STLファイルの分割とWebGLベースの3Dビューアの実験用フォルダです。

### 機能
- STLファイルの自動分割（連結成分分析）
- STLファイルの手動分割（インデックスベース）
- WebGLによる3Dモデル表示
- パーツごとの表示/非表示切り替え

## 2. 使用方法

### STL分割ツール

```bash
cd stl_splitter

# 両方の方式で分割（推奨）
python split_stl.py --method both

# 自動分割のみ
python split_stl.py --method auto

# 手動分割のみ
python split_stl.py --method manual
```

### WebGLビューア

```bash
# サーバー起動（ブラウザが自動で開きます）
python serve.py
```

または手動で:
```bash
python -m http.server 8080
# ブラウザで http://localhost:8080/webgl_viewer/ を開く
```

## 3. フォルダ構成

```
sandbox/
├── README.md           # このファイル
├── serve.py            # ローカルサーバー
├── stl_splitter/       # STL分割ツール
│   ├── split_stl.py    # 分割スクリプト
│   └── parts/          # 分割結果
│       ├── auto/       # 自動分割結果
│       └── manual/     # 手動分割結果
└── webgl_viewer/       # WebGLビューア
    ├── index.html      # メインHTML
    └── js/
        └── viewer.js   # Three.jsビューアロジック
```

## 4. 分割方式の比較

| 方式 | 説明 | 利点 | 欠点 |
|------|------|------|------|
| 自動（auto） | 連結成分分析で物理的に分離したパーツを検出 | CAD知識不要、再現性あり | パーツ名は自動生成 |
| 手動（manual） | インデックス範囲で分割 | パーツ名を指定可能 | STL変更時に要修正 |

---

<a id="english"></a>

## 1. Overview

### About This Folder
Experimental folder for STL file splitting and WebGL-based 3D viewer.

### Features
- Automatic STL splitting (connected component analysis)
- Manual STL splitting (index-based)
- WebGL 3D model display
- Per-part visibility toggle

## 2. Usage

### STL Splitter Tool

```bash
cd stl_splitter

# Split with both methods (recommended)
python split_stl.py --method both

# Automatic split only
python split_stl.py --method auto

# Manual split only
python split_stl.py --method manual
```

### WebGL Viewer

```bash
# Start server (browser opens automatically)
python serve.py
```

Or manually:
```bash
python -m http.server 8080
# Open http://localhost:8080/webgl_viewer/ in browser
```

## 3. Folder Structure

```
sandbox/
├── README.md           # This file
├── serve.py            # Local server
├── stl_splitter/       # STL splitter tool
│   ├── split_stl.py    # Split script
│   └── parts/          # Split results
│       ├── auto/       # Automatic split results
│       └── manual/     # Manual split results
└── webgl_viewer/       # WebGL viewer
    ├── index.html      # Main HTML
    └── js/
        └── viewer.js   # Three.js viewer logic
```

## 4. Split Method Comparison

| Method | Description | Pros | Cons |
|--------|-------------|------|------|
| Auto | Detect physically separated parts via connected component analysis | No CAD knowledge needed, reproducible | Part names are auto-generated |
| Manual | Split by index range | Custom part names | Needs update when STL changes |
