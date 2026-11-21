# StampFly Schematics

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

StampFly の回路図（PDF）の置き場所と、各ファイルの内容を示す。回路図は M5Stack が作成した図面で、ここには受け取ったファイルを変更せずに置いている。ファームウェアのピン割り当て・電源系統・センサの接続を確かめるときは、ここの図面を基準とする。

### 対象読者

- 機体ファームウェア（`firmware/vehicle/`）の BSP・ドライバを読む人、書く人
- 電源・センサの挙動を回路から確かめたい人

## 2. 回路図の一覧

### ファイル

| ファイル | 基板 | 図面の作成日（PDF の情報） | ページ | SHA-256（先頭 16 桁） |
|----------|------|---------------------------|--------|----------------------|
| [`Stamp_Fly_v1.0.pdf`](Stamp_Fly_v1.0.pdf) | 主基板（M5StampS3 を載せる基板。IMU・地磁気・気圧・下向き ToF・電流計・モータ駆動） | 2024-06-05 | 1 | `cd723159b898d840` |
| [`StampS3_Fly_Hat.pdf`](StampS3_Fly_Hat.pdf) | 主基板のコネクタ P3 に付く小基板（前方 ToF VL53L3CX、電池コネクタ、電池電圧から 3.3 V を作るレギュレータ HT7533） | 2023-09-22 | 1 | `bdd7b6ae25cb69f9` |
| [`Sch_PMW3901MB_SPI.pdf`](Sch_PMW3901MB_SPI.pdf) | オプティカルフローセンサ PMW3901 の基板（SPI） | 2024-06-06 | 1 | `f8a7d9e79c124a9e` |

### 取り込みの記録

| 項目 | 内容 |
|------|------|
| 取り込み日 | 2026-11-21 |
| 作成者 | M5Stack（図面は Altium Designer の出力） |
| 公開の扱い | StampFly はオープンソースハードウェアであり、回路図は公開してよい資料（開発者の確認、2026-11-21）。ライセンスの条文はこの文書では確かめていない |
| 公開 URL | M5Stack が同じ 3 枚を公開している: `https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1069/Stamp_Fly_v1.0.pdf`、`…/1069/StampS3_Fly_Hat.pdf`、`…/1069/Sch_PMW3901MB_SPI.pdf`（M5Stack 製品文書 StampFly より、閲覧 2026-11-21） |
| 変更 | なし（受け取ったファイルのまま） |
| 版の表示 | 図面の中に表題欄は無い。版はファイル名による |

### 図面を読むときの注意

- PDF は図面で、文字の検索だけでは配線は分からない。ネット名は線のつながりを追って確かめる。
- 製品の基板（v1.1 を含む）がこの図面と同じかは、この文書では確かめていない。
- 図面から読み取った内容をほかの文書に書くときは、ファイル名を添える（例: `hardware_init.md` 5 章の「前方 ToF と電池の接続」）。

---

<a id="english"></a>

## 1. Overview

### About This Document

This directory holds the StampFly schematics (PDF) and lists what each file is. The schematics are drawings made by M5Stack and are kept here as received, without changes. Use them as the reference when checking pin assignments, the power tree and sensor connections of the firmware.

### Target Audience

- People who read or write the BSP and drivers of the vehicle firmware (`firmware/vehicle/`)
- People who want to check power and sensor behaviour against the circuit

## 2. List of Schematics

### Files

| File | Board | Drawing date (PDF metadata) | Pages | SHA-256 (first 16 digits) |
|------|-------|-----------------------------|-------|---------------------------|
| [`Stamp_Fly_v1.0.pdf`](Stamp_Fly_v1.0.pdf) | Main board (carries the M5StampS3; IMU, magnetometer, barometer, downward ToF, power monitor, motor drivers) | 2024-06-05 | 1 | `cd723159b898d840` |
| [`StampS3_Fly_Hat.pdf`](StampS3_Fly_Hat.pdf) | Small board that plugs into connector P3 of the main board (front ToF VL53L3CX, battery connector, regulator HT7533 making 3.3 V from the battery) | 2023-09-22 | 1 | `bdd7b6ae25cb69f9` |
| [`Sch_PMW3901MB_SPI.pdf`](Sch_PMW3901MB_SPI.pdf) | Optical-flow sensor PMW3901 board (SPI) | 2024-06-06 | 1 | `f8a7d9e79c124a9e` |

### Import Record

| Item | Content |
|------|---------|
| Imported | 2026-11-21 |
| Author | M5Stack (the drawings are Altium Designer output) |
| Publication | StampFly is open-source hardware and the schematics may be published (confirmed by the developer, 2026-11-21). The licence text is not checked in this document |
| Public URLs | M5Stack publishes the same three drawings: `https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1069/Stamp_Fly_v1.0.pdf`, `…/1069/StampS3_Fly_Hat.pdf`, `…/1069/Sch_PMW3901MB_SPI.pdf` (from the M5Stack StampFly product documentation, viewed 2026-11-21) |
| Changes | None (files as received) |
| Revision marking | The drawings have no title block; the revision comes from the file name |

### Notes for Reading the Drawings

- The PDFs are drawings; a text search alone does not show the wiring. Confirm a net by following the lines.
- Whether the production boards (including v1.1) match these drawings is not checked in this document.
- When writing what was read from a drawing into another document, name the file (for example "front ToF and the battery" in chapter 5 of `hardware_init.md`).
