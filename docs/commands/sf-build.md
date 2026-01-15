# sf build

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

ファームウェアをビルドします。ESP-IDF の `idf.py build` をラップし、プロジェクトルートからの実行を可能にします。

## 2. 構文

```bash
sf build [target] [options]
```

### 引数

| 引数 | 説明 | デフォルト |
|------|------|-----------|
| `target` | ビルド対象（vehicle または controller） | vehicle |

### オプション

| オプション | 説明 |
|-----------|------|
| `-c, --clean` | クリーンビルド（fullclean 後にビルド） |
| `-j, --jobs N` | 並列ジョブ数 |
| `-v, --verbose` | 詳細出力 |

## 3. 使用例

```bash
# vehicleファームウェアをビルド
sf build vehicle

# controllerファームウェアをビルド
sf build controller

# クリーンビルド
sf build vehicle --clean

# 並列ジョブ数を指定
sf build vehicle -j 8
```

## 4. 出力

ビルド成功時：
```
[INFO] Building vehicle firmware...
  Directory: /path/to/firmware/vehicle
[INFO] Running: idf.py build
...
[OK] Build successful: vehicle
  Binary: /path/to/firmware/vehicle/build/vehicle.bin
  Size: 850.2 KB
```

---

<a id="english"></a>

## 1. Overview

Build firmware. Wraps ESP-IDF's `idf.py build` and enables execution from project root.

## 2. Syntax

```bash
sf build [target] [options]
```

### Arguments

| Argument | Description | Default |
|----------|-------------|---------|
| `target` | Build target (vehicle or controller) | vehicle |

### Options

| Option | Description |
|--------|-------------|
| `-c, --clean` | Clean build (fullclean before build) |
| `-j, --jobs N` | Number of parallel jobs |
| `-v, --verbose` | Verbose output |

## 3. Examples

```bash
# Build vehicle firmware
sf build vehicle

# Build controller firmware
sf build controller

# Clean build
sf build vehicle --clean

# Specify parallel jobs
sf build vehicle -j 8
```
