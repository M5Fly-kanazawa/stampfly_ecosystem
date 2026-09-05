#!/usr/bin/env python3
"""Generate StampFly Workshop slides (Lesson 0-13) as PowerPoint files.

Usage:
    python generate_slides.py              # Generate all lessons
    python generate_slides.py --lesson 0   # Generate specific lesson

Requires: python-pptx, Pillow
"""

from __future__ import annotations

import argparse
from pathlib import Path
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SLIDE_WIDTH = Inches(13.333)   # 16:9
SLIDE_HEIGHT = Inches(7.5)

# StampFly brand colors
SF_BLUE = RGBColor(0, 120, 200)
SF_DARK = RGBColor(0, 60, 100)
SF_LIGHT = RGBColor(220, 240, 255)
SF_RED = RGBColor(220, 50, 30)
SF_GREEN = RGBColor(40, 160, 40)
SF_GRAY = RGBColor(80, 80, 80)
WHITE = RGBColor(255, 255, 255)
BLACK = RGBColor(0, 0, 0)

# Fonts — Meiryo for Japanese, Consolas for code
FONT_JP = "Meiryo"
FONT_CODE = "Consolas"

# Directories
SCRIPT_DIR = Path(__file__).resolve().parent
IMAGES_DIR = SCRIPT_DIR.parent / "images"
OUTPUT_DIR = SCRIPT_DIR / "output"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def new_presentation() -> Presentation:
    """Create a blank 16:9 presentation."""
    prs = Presentation()
    prs.slide_width = SLIDE_WIDTH
    prs.slide_height = SLIDE_HEIGHT
    return prs


def add_title_slide(prs: Presentation, title: str, subtitle: str) -> None:
    """Add a title slide with blue background."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])  # blank layout

    # Blue background
    bg = slide.background
    fill = bg.fill
    fill.solid()
    fill.fore_color.rgb = SF_BLUE

    # Title text
    left, top = Inches(1), Inches(2.5)
    txBox = slide.shapes.add_textbox(left, top, Inches(11), Inches(1.5))
    tf = txBox.text_frame
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.text = title
    p.font.size = Pt(44)
    p.font.bold = True
    p.font.color.rgb = WHITE
    p.font.name = FONT_JP
    p.alignment = PP_ALIGN.CENTER

    # Subtitle
    txBox2 = slide.shapes.add_textbox(left, Inches(4.2), Inches(11), Inches(1))
    tf2 = txBox2.text_frame
    tf2.word_wrap = True
    p2 = tf2.paragraphs[0]
    p2.text = subtitle
    p2.font.size = Pt(24)
    p2.font.color.rgb = RGBColor(200, 230, 255)
    p2.font.name = FONT_JP
    p2.alignment = PP_ALIGN.CENTER


def add_content_slide(
    prs: Presentation,
    title: str,
    bullets: list[str],
    *,
    image_path: Path | None = None,
) -> None:
    """Add a content slide with title, bullet points, and optional image."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])  # blank

    # Title bar
    title_shape = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE, Inches(0), Inches(0),
        SLIDE_WIDTH, Inches(1.1),
    )
    title_shape.fill.solid()
    title_shape.fill.fore_color.rgb = SF_BLUE
    title_shape.line.fill.background()
    tf = title_shape.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    p.text = f"  {title}"
    p.font.size = Pt(28)
    p.font.bold = True
    p.font.color.rgb = WHITE
    p.font.name = FONT_JP

    # Content area
    if image_path and image_path.exists():
        text_width = Inches(6.5)
        img_left = Inches(7.5)
    else:
        text_width = Inches(11)
        img_left = None

    # Bullet text
    txBox = slide.shapes.add_textbox(
        Inches(0.8), Inches(1.5), text_width, Inches(5.5),
    )
    tf = txBox.text_frame
    tf.word_wrap = True
    for i, bullet in enumerate(bullets):
        if i == 0:
            p = tf.paragraphs[0]
        else:
            p = tf.add_paragraph()
        p.text = bullet
        p.font.size = Pt(20)
        p.font.name = FONT_JP
        p.font.color.rgb = SF_GRAY
        p.space_after = Pt(8)
        p.level = 0

    # Image
    if img_left and image_path and image_path.exists():
        slide.shapes.add_picture(
            str(image_path), img_left, Inches(1.5),
            width=Inches(5),
        )


def add_code_slide(
    prs: Presentation,
    title: str,
    code: str,
    *,
    filename: str = "user_code.cpp",
) -> None:
    """Add a slide with code listing."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])

    # Title bar
    title_shape = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE, Inches(0), Inches(0),
        SLIDE_WIDTH, Inches(1.1),
    )
    title_shape.fill.solid()
    title_shape.fill.fore_color.rgb = SF_BLUE
    title_shape.line.fill.background()
    tf = title_shape.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    p.text = f"  {title}"
    p.font.size = Pt(28)
    p.font.bold = True
    p.font.color.rgb = WHITE
    p.font.name = FONT_JP

    # Filename label
    lbl = slide.shapes.add_textbox(
        Inches(0.8), Inches(1.3), Inches(3), Inches(0.5),
    )
    lp = lbl.text_frame.paragraphs[0]
    lp.text = filename
    lp.font.size = Pt(14)
    lp.font.bold = True
    lp.font.color.rgb = SF_BLUE
    lp.font.name = FONT_CODE

    # Code box background
    code_bg = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE,
        Inches(0.6), Inches(1.8),
        Inches(12), Inches(5.2),
    )
    code_bg.fill.solid()
    code_bg.fill.fore_color.rgb = SF_LIGHT
    code_bg.line.color.rgb = RGBColor(180, 210, 240)
    code_bg.line.width = Pt(1)

    # Code text
    txBox = slide.shapes.add_textbox(
        Inches(0.8), Inches(1.9), Inches(11.6), Inches(5.0),
    )
    tf = txBox.text_frame
    tf.word_wrap = True
    lines = code.strip().split("\n")
    for i, line in enumerate(lines):
        if i == 0:
            p = tf.paragraphs[0]
        else:
            p = tf.add_paragraph()
        p.text = line
        p.font.size = Pt(15)
        p.font.name = FONT_CODE
        p.font.color.rgb = SF_GRAY
        p.space_after = Pt(2)


def add_table_slide(
    prs: Presentation,
    title: str,
    headers: list[str],
    rows: list[list[str]],
) -> None:
    """Add a slide with a table."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])

    # Title bar
    title_shape = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE, Inches(0), Inches(0),
        SLIDE_WIDTH, Inches(1.1),
    )
    title_shape.fill.solid()
    title_shape.fill.fore_color.rgb = SF_BLUE
    title_shape.line.fill.background()
    tf = title_shape.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    p.text = f"  {title}"
    p.font.size = Pt(28)
    p.font.bold = True
    p.font.color.rgb = WHITE
    p.font.name = FONT_JP

    # Table
    n_rows = len(rows) + 1
    n_cols = len(headers)
    table_shape = slide.shapes.add_table(
        n_rows, n_cols,
        Inches(0.8), Inches(1.6),
        Inches(11.5), Inches(0.6 * n_rows),
    )
    table = table_shape.table

    # Header row
    for j, h in enumerate(headers):
        cell = table.cell(0, j)
        cell.text = h
        cell.fill.solid()
        cell.fill.fore_color.rgb = SF_BLUE
        for paragraph in cell.text_frame.paragraphs:
            paragraph.font.size = Pt(16)
            paragraph.font.bold = True
            paragraph.font.color.rgb = WHITE
            paragraph.font.name = FONT_JP

    # Data rows
    for i, row in enumerate(rows):
        for j, val in enumerate(row):
            cell = table.cell(i + 1, j)
            cell.text = val
            if i % 2 == 0:
                cell.fill.solid()
                cell.fill.fore_color.rgb = SF_LIGHT
            for paragraph in cell.text_frame.paragraphs:
                paragraph.font.size = Pt(14)
                paragraph.font.name = FONT_JP
                paragraph.font.color.rgb = SF_GRAY


def add_checkpoint_slide(
    prs: Presentation,
    checks: list[str],
    next_lesson: str,
) -> None:
    """Add a checkpoint / summary slide."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])

    # Title bar
    title_shape = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE, Inches(0), Inches(0),
        SLIDE_WIDTH, Inches(1.1),
    )
    title_shape.fill.solid()
    title_shape.fill.fore_color.rgb = SF_BLUE
    title_shape.line.fill.background()
    tf = title_shape.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    p.text = "  チェックポイント / Checkpoint"
    p.font.size = Pt(28)
    p.font.bold = True
    p.font.color.rgb = WHITE
    p.font.name = FONT_JP

    # Checks
    txBox = slide.shapes.add_textbox(
        Inches(0.8), Inches(1.6), Inches(11), Inches(3.5),
    )
    tf = txBox.text_frame
    tf.word_wrap = True
    for i, check in enumerate(checks):
        if i == 0:
            p = tf.paragraphs[0]
        else:
            p = tf.add_paragraph()
        p.text = f"\u2610  {check}"
        p.font.size = Pt(20)
        p.font.name = FONT_JP
        p.font.color.rgb = SF_GRAY
        p.space_after = Pt(6)

    # Next lesson box
    box = slide.shapes.add_shape(
        MSO_SHAPE.ROUNDED_RECTANGLE,
        Inches(0.8), Inches(5.5),
        Inches(11.5), Inches(1.2),
    )
    box.fill.solid()
    box.fill.fore_color.rgb = SF_LIGHT
    box.line.color.rgb = SF_BLUE
    box.line.width = Pt(2)
    btf = box.text_frame
    btf.vertical_anchor = MSO_ANCHOR.MIDDLE
    bp = btf.paragraphs[0]
    bp.text = f"次のレッスン: {next_lesson}"
    bp.font.size = Pt(22)
    bp.font.bold = True
    bp.font.color.rgb = SF_BLUE
    bp.font.name = FONT_JP
    bp.alignment = PP_ALIGN.CENTER


# ---------------------------------------------------------------------------
# Lesson builders
# ---------------------------------------------------------------------------

def build_lesson_00() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 0: 環境セットアップ", "Environment Setup")

    # P2: Workshop curriculum
    add_table_slide(prs, "ワークショップカリキュラム / Workshop Curriculum", [
        "Day", "Lesson", "テーマ",
    ], [
        ["Day 1", "0 - 2", "環境セットアップ + モータ + コントローラ"],
        ["Day 2", "3 - 5", "LED + IMU + 初フライト"],
        ["Day 3", "6 - 8", "モデリング + システム同定 + PID制御"],
        ["Day 4", "9 - 11", "姿勢推定 + Python SDK + 競技会準備"],
        ["Day 5 AM", "", "ホバリングタイム競技会"],
    ])

    # P3: StampFly hardware specs
    add_table_slide(prs, "StampFly ハードウェア / Hardware", [
        "項目", "仕様",
    ], [
        ["MCU", "ESP32-S3 (M5Stamp S3)"],
        ["IMU", "BMI270 (6軸 400Hz)"],
        ["気圧", "BMP280"],
        ["ToF", "VL53L3CX"],
        ["質量", "37g（バッテリ含む）"],
        ["通信", "ESP-NOW + WiFi"],
        ["バッテリ", "LiPo 1S 3.7V"],
    ])

    # P4: Ecosystem overview
    add_content_slide(prs, "エコシステム全体像 / Ecosystem Overview", [
        "StampFly Ecosystem の6層構造:",
        "",
        "• firmware/ --- 機体・送信機の組込みコード",
        "• control/ --- 制御設計（PID, MPC）",
        "• simulator/ --- 3D 物理シミュレータ",
        "• analysis/ --- データ解析（Jupyter）",
        "• tools/ --- 開発ツール（sf CLI）",
        "• protocol/ --- 通信仕様（SSOT）",
    ])

    # P5: sf CLI development tool
    add_table_slide(prs, "開発ツール sf CLI / Development Tool", [
        "カテゴリ", "コマンド例", "説明",
    ], [
        ["ビルド", "sf build / sf flash", "ファームウェア開発"],
        ["診断", "sf doctor / sf monitor", "デバッグ"],
        ["ログ", "sf log wifi / sf log viz", "テレメトリ取得・可視化"],
        ["シミュレータ", "sf sim run", "仮想環境で練習"],
        ["キャリブレーション", "sf cal gyro/accel/mag", "センサ校正"],
    ])

    # P6: Windows setup (1/2) — Prerequisites & install
    add_content_slide(prs, "Windows 環境構築 (1/2) / Windows Setup", [
        "【前提条件の確認】（CMD で実行）",
        "  git --version → git version 2.x（なければ winget install Git.Git）",
        "  python --version → Python 3.10 以上（なければ winget install Python.Python.3.12）",
        "",
        "【インストール手順】（CMD で実行）",
        "  1. リポジトリをクローン",
        "  2. install.bat を実行（ESP-IDF + sfcli 自動インストール）",
        "  3. setup_env.bat で開発環境をアクティベート",
        "",
        "  git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git",
        "  cd stampfly_ecosystem && install.bat && setup_env.bat",
    ])

    # P7: Windows setup (2/2) — Driver & verification
    add_content_slide(prs, "Windows 環境構築 (2/2) / Driver & Verification", [
        "【USB シリアルドライバ】",
        "• CH9102F ドライバをインストール（M5Stack 製品用）",
        "• https://docs.m5stack.com/en/download",
        "",
        "【動作確認】",
        "• sf doctor で環境診断 — すべて OK になれば完了",
        "",
        "【macOS / Linux ユーザー】",
        "• install.sh + setup_env.sh を使用してください",
    ])

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "ワークショップファームウェアをビルド・書き込み・動作確認する",
        "",
        "1. sf lesson build でビルド",
        "2. sf lesson flash で書き込み",
        "3. シリアルモニタで \"Hello StampFly!\" を確認",
    ])

    add_content_slide(
        prs, "開発フロー / Development Flow",
        ["sf CLI でビルド → フラッシュ → モニタの流れ"],
        image_path=IMAGES_DIR / "build_flash_flow.png",
    )

    add_table_slide(prs, "sf CLI コマンド一覧", [
        "コマンド", "説明",
    ], [
        ["sf doctor", "環境診断"],
        ["sf lesson build", "ワークショップビルド"],
        ["sf lesson flash", "書き込み＋モニタ"],
        ["sf lesson switch N", "レッスン切り替え"],
        ["sf monitor", "シリアルモニタ"],
    ])

    add_content_slide(prs, "ワークショップの構造 / Workshop Structure", [
        "学生が実装する関数は2つだけ！",
        "",
        "• setup() --- 起動時に1回呼ばれる",
        "• loop_400Hz(dt) --- 400Hz（2.5ms毎）で呼ばれる",
        "",
        "ハードウェアの複雑さは ws:: 名前空間で隠蔽",
        "FreeRTOS, SPI/I2C, センサーフュージョンを意識する必要なし",
    ])

    # P12: Firmware architecture
    add_content_slide(
        prs, "ファームウェアの全体像 / Firmware Architecture",
        [
            "5層構造（下から上へ）:",
            "",
            "  5. user_code.cpp ← 皆さんのコードはここ！",
            "  4. ws:: API（gyro_x(), motor_set_duty() 等）",
            "  3. Vehicle Components（IMU, baro, motor driver）",
            "  2. ESP-IDF / FreeRTOS（tasks, timers, queues）",
            "  1. Hardware（ESP32-S3, BMI270, BMP280）",
            "",
            "ws::gyro_x() と書くだけで SPI通信→フィルタ→バイアス補正を全部やってくれる",
            "下のレイヤーはワークショップ中意識する必要なし",
        ],
        image_path=IMAGES_DIR / "firmware_layers.png",
    )

    # P13: Behind the 400Hz loop
    add_content_slide(
        prs, "400Hz ループの裏側 / Behind the 400Hz Loop",
        [
            "loop_400Hz() が呼ばれるまでの流れ:",
            "",
            "  1. ESP Timer（2500μs）→ ハードウェアタイマ割り込み",
            "  2. IMU Task（Read BMI270）→ SPI読み取り + バイアス補正",
            "  3. Go!（data ready）→ データ準備完了",
            "  4. loop_400Hz (Your Code) → user_code.cpp が実行される",
            "",
            "→ 2.5ms ごとに繰り返し = 400Hz",
            "",
            "【ポイント】loop_400Hz(dt) は常にIMUデータ更新後に呼ばれる",
            "学生はタイマー管理不要、ただ関数を書くだけ！",
        ],
        image_path=IMAGES_DIR / "loop_400hz_timeline.png",
    )

    add_code_slide(prs, "実習: Hello StampFly", """
#include "workshop_api.hpp"

void setup()
{
    ws::print("Hello StampFly!");
}

void loop_400Hz(float dt)
{
    // Nothing to do in Lesson 0
}
""")

    # P15: Workflow
    add_content_slide(prs, "作業の進め方 / Workflow", [
        "【ファイル構造】",
        "  firmware/workshop/",
        "    ├── lessons/",
        "    │   ├── lesson_00/student.cpp  ← テンプレート",
        "    │   ├── lesson_01/student.cpp",
        "    │   └── ...",
        "    └── main/",
        "        └── user_code.cpp  ← ここを編集！",
        "",
        "【手順】",
        "  1. sf lesson switch N → テンプレートを user_code.cpp にコピー",
        "  2. user_code.cpp を編集",
        "  3. sf lesson build",
        "  4. sf lesson flash",
        "",
        "⚠ student.cpp はテンプレート — 直接編集しない！ switch で何度でもリセット可能",
    ])

    # P16: Controller setup
    add_content_slide(prs, "コントローラのセットアップ / Controller Setup", [
        "【コントローラのビルドと書き込み】",
        "  1. コントローラを USB 接続",
        "  2. sf build controller",
        "  3. sf flash controller",
        "",
        "【確認】",
        "  LCD に起動画面が表示されれば OK",
        "  Lesson 2 でペアリングして使用する",
    ])

    # P17: Simulator
    add_content_slide(prs, "シミュレータで遊ぶ / Try the Simulator", [
        "【USB HID モードに切替】",
        "  1. 画面タッチでメニューを開く",
        "  2. Comm: を選択",
        "  3. USB HID に切替 → 自動再起動",
        "",
        "【コマンド】",
        "  sf sim run                  ← デフォルト（VPython + ボクセルワールド）",
        "  sf sim run -w ringworld     ← 軽量ワールド",
        "",
        "【操作】スロットル → 上昇 / ロール・ピッチ → 傾き / ヨー → 回転",
        "",
        "アクロモード（角速度制御）で飛行 → まずは感覚をつかもう",
        "⚠ 終了後は Comm: を ESP-NOW に戻すこと（実機飛行用）",
    ])

    add_checkpoint_slide(prs, [
        "sf doctor がエラーなしで通る",
        "ビルドが成功し \"Hello StampFly!\" が表示される",
        "コントローラのビルド・書き込みが完了した",
        "シミュレータが起動し操縦を試した",
    ], "Lesson 1: モータ制御")

    return prs


def build_lesson_01() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 1: モータ制御", "Motor Control")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "モータの番号と配置を理解し、PWM で個別制御する",
        "",
        "• モータ番号 M1-M4 の位置を覚える",
        "• ARM / DISARM の仕組みを理解する",
        "• PWM デューティ比でモータ回転数を制御",
    ])

    add_content_slide(
        prs, "モータ配置 / Motor Layout",
        [
            "対角のモータが同じ方向に回転（トルクバランス）",
            "M1(FR), M3(RL) = CCW（反時計回り）",
            "M2(RR), M4(FL) = CW（時計回り）",
        ],
        image_path=IMAGES_DIR / "motor_layout.png",
    )

    add_content_slide(
        prs, "PWM とは / What is PWM?",
        [
            "Pulse Width Modulation（パルス幅変調）",
            "Duty 0% = 停止 / 100% = 最大回転",
            "StampFly API: motor_set_duty(id, 0.0~1.0)",
        ],
        image_path=IMAGES_DIR / "pwm_waveform.png",
    )

    add_content_slide(prs, "ARM / DISARM とは / What is ARM?", [
        "【安全スイッチ】",
        "• ARM = モーター出力を有効化（回転可能になる）",
        "• DISARM = モーター出力を無効化（強制停止）",
        "• コードから ws::arm() で ARM できる",
        "",
        "⚠ arm() を呼ばないとモーターは回らない！",
        "  setup() 内で最初に呼ぶこと",
    ])

    add_table_slide(prs, "モータ制御 API", [
        "関数", "説明", "引数",
    ], [
        ["arm()", "モーター出力を有効化", "---"],
        ["disarm()", "モーター出力を無効化", "---"],
        ["motor_set_duty(id, duty)", "個別モータ設定", "id=1-4, duty=0.0-1.0"],
        ["motor_set_all(duty)", "全モータ一括", "duty=0.0-1.0"],
        ["motor_stop_all()", "全モータ停止", "---"],
    ])

    add_code_slide(prs, "実習: モータを順番に回す", """
#include "workshop_api.hpp"

static uint32_t timer = 0;

void setup() {
    ws::print("Lesson 1: Motor Control");
    ws::arm();  // Enable motor output
}

void loop_400Hz(float dt) {
    timer++;
    // 800 ticks = 2 seconds at 400Hz
    int motor_id = (timer / 800) % 4 + 1;

    ws::motor_stop_all();
    ws::motor_set_duty(motor_id, 0.1f);  // 10% duty
}
""")

    add_checkpoint_slide(prs, [
        "arm() なしではモータが回らないことを確認した",
        "M1 が FR（右前）のプロペラを回す",
        "M1→M2→M3→M4 の順に回転する",
    ], "Lesson 2: コントローラ入力")

    return prs


def build_lesson_02() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 2: コントローラ入力", "Controller Input")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "スティック値を変数に読み取り、演算でモータを個別制御する",
        "",
        "• ESP-NOW 無線通信の仕組みを理解",
        "• チャンネル設定で他の受講生との混信を回避",
        "• 変数と四則演算でスティック → モータ Duty を計算",
        "• オープンループ手動操縦の限界を体感",
    ])

    add_content_slide(
        prs, "ESP-NOW 通信フロー / Communication Flow",
        [
            "コントローラ → 2.4GHz 無線 → StampFly",
            "スティック値は ws:: API で取得可能",
        ],
        image_path=IMAGES_DIR / "espnow_dataflow.png",
    )

    add_content_slide(
        prs, "コントローラ各部 / Controller Layout (MODE 3)",
        [
            "左スティック: Pitch ↑↓ / Roll ←→（押込み: Flip 予定）",
            "右スティック: Throttle ↑↓ / Yaw ←→（押込み: ARM）",
            "黄ボタン左: Alt Hold / Manual 切替（予定）",
            "黄ボタン右: STABILIZE / ACRO 切替",
            "M5ボタン = LCDパネル: タップでペアリング等",
        ],
        image_path=IMAGES_DIR / "controller.jpg",
    )

    add_table_slide(prs, "コントローラ API", [
        "関数", "説明", "値域",
    ], [
        ["set_channel(ch)", "WiFiチャンネル設定", "1, 6, 11"],
        ["rc_throttle()", "スロットル", "0.0 -- 1.0"],
        ["rc_roll()", "ロール", "-1.0 -- +1.0"],
        ["rc_pitch()", "ピッチ", "-1.0 -- +1.0"],
        ["rc_yaw()", "ヨー", "-1.0 -- +1.0"],
        ["is_armed()", "ARM状態", "true / false"],
        ["motor_set_duty(id, v)", "モータ個別制御", "id=1-4, v=0.0-1.0"],
    ])

    add_content_slide(prs, "ペアリング手順 / Pairing", [
        "Step 1: コントローラの M5ボタンを押しながら電源ON → LCD に \"Pairing mode...\" 表示",
        "Step 2: StampFly のボタンを 3秒長押し → 青LED高速点滅 + ビープ音",
        "Step 3: ペアリング完了 → ビープ音が鳴り、次回から自動接続",
        "",
        "⚠ 教室では一組ずつペアリングする（ブロードキャスト通信のため近くの機体と干渉する可能性）",
        "⚠ うまくいかない場合: 両方を再起動して Step 1 からやり直す",
    ])

    add_content_slide(prs, "チャンネル設定 / Channel Setting", [
        "【なぜチャンネルを変えるのか？】",
        "教室で複数の StampFly を同時に飛ばすと、同じチャンネル同士で混信が起きる。",
        "チャンネルを分けることで回避できる。",
        "",
        "チャンネル 1 = グループ A（緑）/ 6 = グループ B（黄）/ 11 = グループ C（赤）",
        "",
        "⚠ set_channel(ch) を setup() 内で呼ぶ",
        "⚠ 機体とコントローラで同じチャンネルにすること！",
    ])

    # Motor layout + mixing sign table
    add_table_slide(prs, "モータ配置とミキシング / Motor Layout & Mixing", [
        "Motor", "T", "Roll", "Pitch", "Yaw",
    ], [
        ["M1 FR", "+", "-", "+", "+"],
        ["M2 RR", "+", "-", "-", "-"],
        ["M3 RL", "+", "+", "-", "+"],
        ["M4 FL", "+", "+", "+", "-"],
    ])

    add_content_slide(
        prs, "オープンループ制御 / Open-Loop Control",
        [
            "フィードバックなしの直接制御",
            "問題: 外乱（風・重心ずれ）に対応できない",
            "→ Lesson 5-6 で PID 制御を導入して解決",
        ],
        image_path=IMAGES_DIR / "open_loop.png",
    )

    add_code_slide(prs, "実習: 手動ミキシング (1/2) Setup", """
#include "workshop_api.hpp"
static uint32_t tick = 0;

void setup() {
    ws::print("L2: Open-Loop Control");
    ws::arm();              // Enable motor output
    ws::set_channel(1);     // 1, 6, or 11
}
""")

    add_code_slide(prs, "実習: 手動ミキシング (2/2) Loop", """
void loop_400Hz(float dt) {
    tick++;
    float t = ws::rc_throttle();
    float r = ws::rc_roll()  * 0.3f;
    float p = ws::rc_pitch() * 0.3f;
    float y = ws::rc_yaw()   * 0.3f;
    ws::motor_set_duty(1, t - r + p + y); // FR
    ws::motor_set_duty(2, t - r - p - y); // RR
    ws::motor_set_duty(3, t + r - p + y); // RL
    ws::motor_set_duty(4, t + r + p - y); // FL
    if (tick % 80 == 0)
        ws::print("T=%.2f R=%.2f P=%.2f Y=%.2f",
            t, r, p, y);
}
""")

    add_checkpoint_slide(prs, [
        "指定チャンネルで混信なく通信できた",
        "コントローラとペアリングできた",
        "スロットルで全モータが均等に回る",
        "ピッチスティックで前後モータの回転差が出る",
    ], "Lesson 3: LED 制御")

    return prs


def build_lesson_03() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 3: LED 制御", "LED Control")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "LED でシステムの状態（ARM / DISARM）とバッテリー残量を可視化する",
        "",
        "• WS2812 アドレサブル LED の制御",
        "• 状態遷移の考え方（ステートマシン）",
        "• バッテリー電圧のモニタリング",
    ])

    add_content_slide(
        prs, "LED 状態遷移 / LED State Machine",
        [
            "DISARM → バッテリー電圧グラデーション（赤〜緑）、ARM → 緑",
            "is_armed() と battery_voltage() で状態判定",
        ],
        image_path=IMAGES_DIR / "led_state_machine.png",
    )

    add_table_slide(prs, "LED 制御 API", [
        "関数", "説明", "引数",
    ], [
        ["disable_led_task()", "システム LED 更新を無効化", "---"],
        ["enable_led_task()", "システム LED 更新を再有効化", "---"],
        ["led_color(r, g, b)", "LED 色設定（裏表両方）", "各 0-255"],
        ["is_armed()", "ARM 状態確認", "true / false"],
        ["battery_voltage()", "バッテリー電圧", "3.0-4.2 V"],
    ])

    add_content_slide(prs, "バッテリー電圧と色 / Battery & Color", [
        "LiPo バッテリー: 3.3V（空）〜 4.2V（満充電）",
        "",
        "電圧 → 色のマッピング:",
        "  3.3V = 赤（危険！）",
        "  3.75V = 黄",
        "  4.2V = 緑（満充電）",
        "",
        "⚠ 3.3V 以下で使用禁止！バッテリーが損傷します",
    ])

    add_code_slide(prs, "実習: ARM 状態で LED 色を変える", """
#include "workshop_api.hpp"

void setup() {
    ws::print("Lesson 3: LED Control");
    ws::disable_led_task();  // Student LED control
}

void loop_400Hz(float dt) {
    if (ws::is_armed()) {
        ws::led_color(0, 255, 0);    // Green
    } else {
        // Battery level as color gradient
        float v = ws::battery_voltage();
        float level = (v - 3.3f) / (4.2f - 3.3f);
        if (level < 0) level = 0;
        if (level > 1) level = 1;
        uint8_t r = 255 * (1.0f - level);
        uint8_t g = 255 * level;
        ws::led_color(r, g, 0);
    }
}
""")

    add_checkpoint_slide(prs, [
        "DISARM 時に赤〜緑のグラデーション表示",
        "ARM すると緑に変わる",
        "バッテリー残量に応じて色が変化する",
    ], "Lesson 4: IMU センサー")

    return prs


def build_lesson_04() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 4: IMU センサ", "IMU Sensor")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "IMU（ジャイロ+加速度）データを読み取り、シリアルモニタと WiFi テレメトリで確認する",
        "",
        "• NED 座標系（北-東-下）の理解",
        "• ジャイロスコープで角速度を取得",
        "• 加速度センサで並進加速度を取得",
        "• WiFi テレメトリでデータ取得・可視化",
    ])

    add_content_slide(
        prs, "NED 座標系 / NED Coordinate System",
        [
            "X = 前方 (Forward)",
            "Y = 右方 (Right)",
            "Z = 下方 (Down)",
            "",
            "BMI270: 6軸 IMU（3軸ジャイロ + 3軸加速度）400Hz",
        ],
        image_path=IMAGES_DIR / "imu_axes.png",
    )

    add_table_slide(prs, "IMU API", [
        "関数", "説明", "単位",
    ], [
        ["gyro_x/y/z()", "角速度 (Roll/Pitch/Yaw)", "rad/s"],
        ["accel_x/y/z()", "加速度 (X/Y/Z)", "m/s²"],
    ])

    add_code_slide(prs, "実習: 6軸読み取り + シリアル表示", """
#include "workshop_api.hpp"
static uint32_t tick = 0;
void setup() {
    ws::print("Lesson 4: IMU Sensor");
}
void loop_400Hz(float dt) {
    tick++;
    float gx = ws::gyro_x();
    float gy = ws::gyro_y();
    float gz = ws::gyro_z();
    float ax = ws::accel_x();
    float ay = ws::accel_y();
    float az = ws::accel_z();
    // Print every 100ms (40 ticks)
    if (tick % 40 == 0) {
        ws::print("G=(%.3f,%.3f,%.3f) A=(%.2f,%.2f,%.2f)",
                  gx, gy, gz, ax, ay, az);
    }
}
""")

    add_content_slide(prs, "WiFi テレメトリ受信 / Receiving WiFi Telemetry", [
        "システムが IMU・姿勢・センサデータを自動的に 400Hz で送信しています",
        "",
        "手順:",
        "1. シリアルモニタで SSID を確認（StampFly_XXXX）",
        "2. PC の WiFi で StampFly に接続（IP: 192.168.4.1）",
        "3. sf log wifi -d 30 で30秒キャプチャ → CSV 自動保存",
        "",
        "→ logs/stampfly_wifi_20260303T143000.csv に保存されました",
        "",
        "-o name.csv でファイル名指定、--no-save で保存なし（統計のみ）",
    ])

    add_content_slide(prs, "データの可視化 / Data Visualization", [
        "保存した CSV を可視化: sf log viz logs/stampfly_wifi_*.csv",
        "",
        "可視化モード:",
        "• --mode all … 全パネル（デフォルト）",
        "• --mode sensors … IMU + 高度センサ",
        "• --mode attitude … 姿勢推定（Roll/Pitch/Yaw）",
        "",
        "• --save plot.png でファイル保存可能",
        "• StampFly を手で揺らしながらキャプチャし、ジャイロの変化をグラフで確認しよう",
    ])

    add_checkpoint_slide(prs, [
        "シリアルモニタでジャイロ・加速度の値を確認できる",
        "静止時に accel_z ≈ -9.81 を確認（反力）",
        "sf log wifi でセンサデータを受信できる",
        "sf log viz でデータをグラフ表示できる",
    ], "Lesson 5: レート P 制御 + 初フライト")

    return prs


def build_lesson_05() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 5: レート P 制御 + 初フライト", "Rate P-Control + First Flight")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "比例フィードバック制御を実装し、初の制御飛行を行う",
        "",
        "• 閉ループ制御の考え方",
        "• P 制御で角速度を安定化",
        "• ARM/DISARM による安全管理",
    ])

    add_content_slide(
        prs, "閉ループ制御 / Closed-Loop Control",
        [
            "目標値とセンサ値の差（誤差）を計算",
            "誤差に比例ゲイン Kp を掛けて制御出力",
            "ジャイロセンサが帰還（フィードバック）を提供",
        ],
        image_path=IMAGES_DIR / "feedback_block.png",
    )

    add_table_slide(prs, "制御 API", [
        "関数", "説明", "値域",
    ], [
        ["gyro_x/y/z()", "ジャイロ角速度", "rad/s"],
        ["rc_roll/pitch/yaw()", "スティック入力", "-1.0 -- +1.0"],
        ["rc_throttle()", "スロットル", "0.0 -- 1.0"],
        ["motor_mixer(T,R,P,Y)", "モーターミキサー", "---"],
        ["is_armed()", "ARM 状態確認", "true / false"],
    ])

    add_code_slide(prs, "実習: 3軸 P 制御", """
#include "workshop_api.hpp"
void setup() { ws::print("Lesson 5: Rate P-Control"); }
void loop_400Hz(float dt) {
    if (!ws::is_armed()) {
        ws::motor_stop_all();
        ws::led_color(50, 0, 0);  // Red = DISARM
        return;
    }
    ws::led_color(0, 50, 0);  // Green = ARM
    float Kp_rp = 0.5f, Kp_yaw = 2.0f;
    float rate_max = 1.0f, yaw_max = 5.0f;
    float roll_e  = ws::rc_roll()*rate_max  - ws::gyro_x();
    float pitch_e = ws::rc_pitch()*rate_max - ws::gyro_y();
    float yaw_e   = ws::rc_yaw()*yaw_max   - ws::gyro_z();
    ws::motor_mixer(ws::rc_throttle(),
        Kp_rp*roll_e, Kp_rp*pitch_e, Kp_yaw*yaw_e);
}
""")

    add_content_slide(prs, "安全注意 / Safety", [
        "⚠ フライト前チェック:",
        "",
        "☐ 保護メガネを着用",
        "☐ 低スロットルから徐々に上げる（最初は 30% 以下）",
        "☐ プロペラの回転方向を確認（M1=CCW, M2=CW, M3=CCW, M4=CW）",
        "☐ 異常時はすぐにスロットルを下げて DISARM",
    ])

    add_checkpoint_slide(prs, [
        "ARM 後スロットルを上げて安定ホバリング",
        "スティック操作で機体が応答する",
        "DISARM で即座にモーターが停止する",
    ], "Lesson 6: システムモデリング")

    return prs


def build_lesson_06() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 6: システムモデリング", "System Modeling")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "プラント伝達関数を導出し、モデルに基づいて P ゲインを設計する",
        "",
        "1. モータ+機体のプラント伝達関数 G_p(s) を理解",
        "2. P 制御の閉ループ特性（ωn, ζ）を導出",
        "3. ζ から Kp を設計 — L05 の初フライト経験をモデルで裏付け",
    ])

    add_content_slide(
        prs, "物理メカニズム / Physical Mechanism",
        [
            "duty 信号がどうやって機体の角速度になるか — 4 つのステージ",
            "",
            "  d → [Motor] → ωm → [Propeller] → F → [Geometry] → τ → [Body] → ω",
            "",
            "• Motor: PWM → モータ回転数（1次遅れ）",
            "• Propeller: 回転 → 推力（F = Ct·ωm²）",
            "• Geometry: 差動推力 → トルク（×4L or ×4κ）",
            "• Rigid Body: トルク → 角速度（積分器 1/(I·s)）",
        ],
        image_path=IMAGES_DIR / "plant_detail.png",
    )

    add_content_slide(
        prs, "プラントモデル（簡略化）/ Plant Model (Simplified)",
        [
            "ホバー近傍で線形化 → 2ブロックに集約:",
            "",
            "  u → [Mixer + Motor: Km/(τm·s+1)] → τ → [Body: 1/(I·s)] → ω",
            "",
            "統合伝達関数: G_p(s) = K / (s·(τm·s + 1)),  K = Km/I",
            "",
            "• Mixer + Motor: duty → トルク — 1次遅れ Km/(τm·s+1)",
            "• Body: トルク → 角速度 — 積分器 1/(I·s)",
        ],
        image_path=IMAGES_DIR / "plant_model.png",
    )

    add_table_slide(prs, "パラメータ一覧 / Parameter Summary", [
        "パラメータ", "記号", "Roll", "Pitch", "Yaw",
    ], [
        ["慣性モーメント", "I", "9.16e-6", "13.3e-6", "20.4e-6 kg·m²"],
        ["トルクゲイン", "Km", "9.3e-4", "9.3e-4", "3.9e-4 N·m"],
        ["モータ時定数", "τm", "0.02 s", "0.02 s", "0.02 s"],
        ["プラントゲイン", "K=Km/I", "102", "70", "19 rad/s²"],
    ])

    add_content_slide(
        prs, "P 制御の閉ループ / P Control Closed Loop",
        [
            "閉ループ伝達関数:",
            "  G_cl(s) = Kp·K / (τm·s² + s + Kp·K)  ← 2次系！",
            "",
            "固有振動数: ωn = √(Kp·K / τm)",
            "減衰比: ζ = 1 / (2·√(Kp·K·τm))",
            "",
            "設計式: Kp = 1 / (4·ζ²·K·τm)",
        ],
        image_path=IMAGES_DIR / "closed_loop_model.png",
    )

    add_content_slide(
        prs, "開ループボード線図 / Open-Loop Bode Plot",
        [
            "開ループ伝達関数:",
            "  L(s) = Kp · G_p(s) = Kp·K / (s·(τm·s + 1))",
            "",
            "位相余裕 PM:",
            "• ωgc: |L(jωgc)| = 1 となる周波数",
            "• PM = 90° − arctan(τm·ωgc)",
            "",
            "Kp を上げると ωgc ↑ → PM ↓ → 振動的に",
        ],
        image_path=IMAGES_DIR / "bode_loop_shaping.png",
    )

    add_content_slide(
        prs, "無駄時間の影響 / Dead Time Effect",
        [
            "制御ループの無駄時間 τd ≈ 5ms:",
            "  センサ処理 + 制御演算 + PWM更新",
            "",
            "L_delay(s) = L(s) · e^(−τd·s)",
            "• ゲインは変わらない（|e^(−jωτd)| = 1）",
            "• 位相が −τd·ω だけ追加で低下",
            "",
            "PM 比較（Roll）:",
            "  Kp=0.5:  モデル 51° → 実機 ≈40° (60° 未達!)",
            "  Kp=0.25: モデル 65° → 実機 ≈59° (ギリギリ)",
            "",
            "→ 実用上 PM ≥ 60° が必要",
        ],
        image_path=IMAGES_DIR / "bode_dead_time.png",
    )

    add_table_slide(prs, "ゲイン設計表 / Gain Design Table", [
        "軸", "K", "Kp=0.5 の ζ", "ζ=0.7 の Kp", "ζ=1.0 の Kp",
    ], [
        ["Roll", "102", "0.50", "0.25", "0.12"],
        ["Pitch", "70", "0.60", "0.36", "0.18"],
        ["Yaw", "19", "1.15", "1.34", "0.66"],
    ])

    add_code_slide(prs, "実習: モデルベースゲイン設計", """
// Model-based Kp design (target zeta = 0.7)
const float tau_m = 0.02f;
const float K_roll  = 102.0f;
const float K_pitch =  70.0f;
const float K_yaw   =  19.0f;
float zeta = 0.7f;

float Kp_roll  = 1.0f/(4*zeta*zeta*K_roll *tau_m);
float Kp_pitch = 1.0f/(4*zeta*zeta*K_pitch*tau_m);
float Kp_yaw   = 1.0f/(4*zeta*zeta*K_yaw  *tau_m);

// Apply per-axis Kp in control loop
float roll_cmd  = Kp_roll  * (target_roll  - ws::gyro_x());
float pitch_cmd = Kp_pitch * (target_pitch - ws::gyro_y());
float yaw_cmd   = Kp_yaw   * (target_yaw   - ws::gyro_z());
""")

    add_checkpoint_slide(prs, [
        "G_p(s) = K/(s(τm·s+1)) を説明できる",
        "モデルベース Kp と L05 の Kp=0.5 を比較飛行",
        "Roll/Pitch/Yaw の ζ 差を体感で理解",
    ], "Lesson 7: システム同定")

    return prs


def build_lesson_07() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 7: システム同定",
                    "System Identification")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "フライトデータからプラントモデルのパラメータ K, τm を同定する",
        "  G_p(s) = K / (s·(τm·s + 1))",
        "",
        "• システム同定（SysID）の考え方",
        "• WiFi テレメトリでデータ取得",
        "• sf sysid fit でモデルフィッティング",
        "• 同定値と L6 理論値を比較",
    ])

    add_content_slide(
        prs, "システム同定とは / What is System Identification?",
        [
            "入出力データからプラントのパラメータ（K, τm）を推定する",
            "L6 の理論モデルと実機データが一致するかを検証する",
            "",
            "u(t) → [Plant G_p(s)] → y(t) → [Parameter Estimation] → K̂, τ̂m",
        ],
        image_path=IMAGES_DIR / "sysid_concept.png",
    )

    add_content_slide(prs, "WiFi テレメトリ / WiFi Telemetry", [
        "システムが IMU・姿勢・センサデータを 400Hz で自動送信",
        "sf log wifi で受信 → CSV 保存 → sf log viz で可視化",
        "",
        "sf log wifi             — WiFi テレメトリ取得",
        "sf log wifi -o data.csv — ファイル名指定で保存",
        "sf log analyze data.csv — フライトデータ分析",
        "sf log viz data.csv     — 波形可視化",
    ])

    add_content_slide(prs, "プラント入出力の復元 / Reconstructing Plant I/O", [
        "テレメトリの記録: ctrl_roll (スティック), gyro_x (角速度)",
        "",
        "Kp 既知 → プラントへの入力を計算できる:",
        "  target = ctrl × rate_max",
        "  u(t) = Kp × (target − gyro)  ← プラント入力",
        "  y(t) = gyro                   ← プラント出力",
        "",
        "→ 閉ループデータでも開ループモデルを直接同定可能",
        "",
        "必要な情報: Kp (例: 0.5), rate_max (例: 1.0 rad/s)",
    ])

    add_content_slide(prs, "フィッティングの仕組み / How Fitting Works", [
        "最小二乗法によるパラメータ推定",
        "",
        "1. 復元した u(t) をモデルに入力: u(t) → G_p(s) → ŷ(t)",
        "2. 実測 y(t) との二乗誤差を計算: J(K, τm) = Σ(ŷ(t) - y(t))²",
        "3. J を最小化する K, τm を数値最適化で求める",
        "",
        "sf sysid fit が自動で行うこと:",
        "  • ホバリング区間の自動検出",
        "  • データを短いセグメントに分割してフィッティング",
        "  • 結果の統計処理（中央値・不確かさ）",
    ])

    add_content_slide(prs, "実習: データ取得 / Hands-on: Data Acquisition", [
        "L5 の P 制御（Kp=0.5）で飛行し、テレメトリデータを取得",
        "",
        "1. sf lesson switch 7 でテンプレートをコピー",
        "2. user_code.cpp に Kp をセット（例: 0.5）",
        "3. sf lesson build → sf lesson flash",
        "4. ホバリング中にスティック操作（2〜3回で十分）",
        "5. PC でデータ受信: sf log wifi -o flight.csv",
        "",
        "ポイント: Kp と rate_max の値を記録しておくこと",
        "  （sf sysid fit に渡す必要がある）",
    ])

    add_content_slide(prs, "sf sysid fit / Model Fitting Tool", [
        "コマンド: sf sysid fit flight.csv --kp 0.5 --plot",
        "",
        "出力例:",
        "  Roll   K =  98.5 (ref: 102.0, err: 3.4%)",
        "         tau_m = 0.019 (ref: 0.020)  R² = 0.94",
        "  Pitch  K =  72.1 (ref:  70.0, err: 3.0%)",
        "         tau_m = 0.021 (ref: 0.020)  R² = 0.91",
        "",
        "オプション:",
        "  --axis roll  特定軸のみ分析",
        "  --rate-max   rate_max (yaw は 5.0)",
        "  -o result.yaml  結果をファイルに保存",
    ])

    add_table_slide(prs, "モデル検証 / Model Verification", [
        "軸", "K (同定)", "K (L6理論)", "τm (同定)", "τm (理論)", "R²",
    ], [
        ["Roll", "?", "102.0", "?", "0.020", "?"],
        ["Pitch", "?", "70.0", "?", "0.020", "?"],
        ["Yaw", "?", "19.0", "?", "0.020", "?"],
    ])

    add_content_slide(prs, "設計 Kp の計算 / Design Kp", [
        "同定した K, τm から設計 Kp を逆算",
        "",
        "  Kp = 1 / (4·ζ²·K·τm)    (ζ = 0.7)",
        "",
        "考察:",
        "  • 同定値と理論値のずれ → モデル誤差・無駄時間が原因",
        "  • 同定 K, τm で計算した Kp は L5 で使った値と近いか?",
    ])

    add_checkpoint_slide(prs, [
        "sf log wifi でフライトデータを取得できた",
        "sf sysid fit で K, τm を同定した",
        "同定値と L6 理論値を比較した",
    ], "Lesson 8: PID 制御")

    return prs


def build_lesson_08() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 8: PID 制御", "PID Control")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "モデルベースの Kp を起点に、I 項・D 項を追加して制御を改善する",
        "",
        "• P 制御の限界: モデル不確かさ・外乱に弱い",
        "• 工学形式 Kp / Ti / Td とは",
        "• 不完全微分: ノイズに強い微分項",
        "• ログから調整する方法",
    ])

    add_content_slide(
        prs, "PID 制御器 / PID Controller",
        [
            "P（比例）: 現在の誤差に比例した出力",
            "I（積分）: Kp/Ti で偏差の蓄積を補正",
            "D（不完全微分）: Td·s/(η·Td·s+1) で高周波をカット",
            "",
            "C(s) = Kp(1 + 1/(Ti·s) + Td·s/(η·Td·s+1))",
        ],
        image_path=IMAGES_DIR / "pid_block.png",
    )

    add_content_slide(prs, "なぜ不完全微分か / Why Incomplete Derivative?", [
        "【問題】",
        "• ジャイロセンサにはノイズがある",
        "• 理想微分 de/dt はノイズを増幅",
        "• 高周波振動 → モータに悪影響",
        "",
        "【解決策】",
        "• 不完全微分フィルタ η で高周波をカット",
        "• η = 0.1〜0.2（小→フィルタ弱い、大→フィルタ強い）",
        "• 微分の効きとノイズ除去のトレードオフ",
    ])

    add_table_slide(prs, "工学形式パラメータ / Engineering Form", [
        "パラメータ", "意味", "効果",
    ], [
        ["Kp", "比例ゲイン", "大→応答速い、大きすぎ→振動"],
        ["Ti", "積分時間 [s]", "小→積分強い、小さすぎ→ワインドアップ"],
        ["Td", "微分時間 [s]", "大→微分強い、大きすぎ→ノイズ増幅"],
        ["η", "フィルタ係数", "0.1〜0.2、大→ノイズに強い"],
    ])

    add_table_slide(prs, "ログから調整する / Log-Based Tuning", [
        "ログで見える症状", "原因", "調整",
    ], [
        ["振動が収まらない", "Kp 過大", "Kp↓"],
        ["応答が遅い", "Kp 過小", "Kp↑"],
        ["定常偏差が残る", "I項不足", "Ti↓（積分強化）"],
        ["オーバーシュート大", "D項不足", "Td↑（微分強化）"],
        ["高周波ノイズ", "η 過小", "η↑（0.1→0.2）"],
        ["ゆっくり発散", "ワインドアップ", "Ti↑"],
    ])

    add_table_slide(prs, "推奨ゲイン / Recommended Gains", [
        "軸", "Kp", "Ti [s]", "Td [s]", "η",
    ], [
        ["Roll", "0.25", "1.67", "0.01", "0.125"],
        ["Pitch", "0.36", "1.67", "0.01", "0.125"],
        ["Yaw", "1.34", "4.0", "0.005", "0.125"],
    ])

    add_code_slide(prs, "実習 Step 1: 理想 PID", """
float Kp=0.25f, Ti=1.67f, Td=0.01f;
float integral=0, prev_err=0;
// inside loop_400Hz(dt)
float error = target - ws::gyro_x();
float P = Kp * error;
if (Ti > 0)  // I term (trapezoidal)
    integral += (dt/(2*Ti)) * (error + prev_err);
float I = Kp * integral;
float D = Kp*Td*(error - prev_err)/dt; // ideal D
prev_err = error;
float roll_out = P + I + D;
""")

    add_code_slide(prs, "実習 Step 2: 不完全微分", """
// Replace ideal D with incomplete derivative filter
float eta = 0.125f;
float d_filt = 0;  // persistent state
// inside loop_400Hz(dt)
float alpha = 2*eta*Td / dt;
float a = (alpha - 1) / (alpha + 1);
float b = 2*Td / ((alpha + 1) * dt);
d_filt = a * d_filt + b * (error - prev_err);
float D = Kp * d_filt;  // Filtered!
""")

    add_checkpoint_slide(prs, [
        "理想微分 vs 不完全微分で高周波振動の違いを確認",
        "定常偏差がなくなった（P のみと比較）",
        "ゲイン調整表を使って応答を改善できた",
    ], "Lesson 9: 姿勢推定")

    return prs


def build_lesson_09() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 9: 姿勢推定", "Attitude Estimation")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "相補フィルタを実装し、ESKFの推定値と比較する",
        "",
        "• ジャイロ積分のドリフト問題を理解",
        "• 相補フィルタで加速度センサとジャイロを融合",
        "• Teleplot で CF vs ESKF をリアルタイム比較",
    ])

    add_content_slide(prs, "姿勢の表現 — ZYX オイラー角", [
        "オイラー角: 3次元の姿勢（回転）を 3 つの角度で表現する方法の一つ",
        "",
        "φ (Roll): X'' 軸回り — 左右の傾き",
        "θ (Pitch): Y' 軸回り — 前後の傾き",
        "ψ (Yaw): Z 軸回り — 機首方向",
        "",
        "StampFly は ZYX オイラー角（Tait-Bryan 角）を使用",
        "回転順序: Z軸(ψ) → Y'軸(θ) → X''軸(φ)",
        "ジンバルロック (θ = ±90°) に注意",
    ])

    add_content_slide(prs, "オイラー角微分とジャイロ角速度の関係", [
        "ジャイロの測定値 ω ≠ オイラー角の時間微分",
        "",
        "φ̇ = ωx + ωy sinφ tanθ + ωz cosφ tanθ",
        "θ̇ = ωy cosφ − ωz sinφ",
        "ψ̇ = ωy (sinφ / cosθ) + ωz (cosφ / cosθ)",
        "",
        "離散積分（毎ステップ更新）:",
        "φ_{k+1} = φ_k + φ̇_k Δt,  θ_{k+1} = θ_k + θ̇_k Δt,  ψ_{k+1} = ψ_k + ψ̇_k Δt",
        "",
        "注意: 現在の (φ,θ) に依存 → 毎ステップ再計算。θ=±90° でジンバルロック",
    ])

    add_content_slide(prs, "加速度からのオイラー角算出 (1) — 座標変換", [
        "静止時、加速度センサは重力の反力を測定する",
        "NED 座標系の重力 [0, 0, g]ᵀ に対し、センサ出力は Rᵀ [0, 0, -g]ᵀ:",
        "",
        "[ax, ay, az]ᵀ = Rᵀ [0, 0, -g]ᵀ = [g sinθ, -g sinφ cosθ, -g cosφ cosθ]ᵀ",
        "",
        "• ax はピッチ角 θ のみに依存（φ を含まない）",
        "• ay と az はロール角 φ とピッチ角 θ の両方に依存",
        "• ヨー角 ψ はどの成分にも現れない（重力は鉛直 → ψ の情報なし）",
        "",
        "前提: 機体が静止または等速運動中であること",
    ])

    add_content_slide(prs, "加速度からのオイラー角算出 (2) — Roll と Pitch の導出", [
        "【Roll φ】(-ay)/(-az) をとると g cosθ が約分され θ が消える:",
        "(-ay)/(-az) = (g sinφ cosθ)/(g cosφ cosθ) = sinφ/cosφ = tanφ",
        "→ φ = atan2(-ay, -az)",
        "",
        "【Pitch θ】ay²+az² = g² cos²θ (sin²φ+cos²φ) = g² cos²θ より:",
        "ax / √(ay²+az²) = g sinθ / g cosθ = tanθ",
        "→ θ = atan2(ax, √(ay²+az²))",
        "",
        "Yaw ψ: 加速度のみでは算出不可（磁気センサが必要）",
        "長所: ドリフトなし  短所: 振動ノイズ大",
    ])

    add_content_slide(
        prs, "ジャイロドリフト問題 / Gyro Drift Problem",
        [
            "ジャイロは角速度 ω を測定 → 角度 = ∫ω dt",
            "微小なバイアスが時間と共に蓄積（ドリフト）",
            "ジャイロ単体では長時間の姿勢推定は不可能",
            "",
            "→ 加速度センサと組み合わせて解決",
        ],
        image_path=IMAGES_DIR / "gyro_drift.png",
    )

    add_content_slide(
        prs, "相補フィルタ / Complementary Filter",
        [
            "θ̂ = α·(θ̂_prev + ω·dt) + (1−α)·θ_accel",
            "",
            "α = 0.98（ジャイロ 98% + 加速度 2%）",
            "",
            "ジャイロ: 短期精度◎ / 長期ドリフト✕",
            "加速度: 長期安定◎ / 振動ノイズ✕",
            "→ 両者の長所を融合",
        ],
        image_path=IMAGES_DIR / "comp_filter.png",
    )

    add_table_slide(prs, "姿勢推定 API",
        ["関数", "説明", "単位"],
        [
            ["gyro_x/y/z()", "角速度（Roll/Pitch/Yaw）", "rad/s"],
            ["accel_x/y/z()", "加速度（X/Y/Z）", "m/s²"],
            ["estimated_roll/pitch/yaw()", "ESKF 推定姿勢角", "rad"],
            ["print(fmt, ...)", "シリアル出力（Teleplot 対応）", "---"],
        ],
    )

    add_code_slide(prs, "実習: 相補フィルタ + Teleplot", """
#include "workshop_api.hpp"
#include <cmath>
static float cf_roll = 0.0f, cf_pitch = 0.0f;

void setup() { ws::print("Lesson 9: Attitude Estimation"); }

void loop_400Hz(float dt) {
    float gx = ws::gyro_x(), gy = ws::gyro_y();
    float ax = ws::accel_x(), ay = ws::accel_y(),
          az = ws::accel_z();
    float accel_roll  = atan2f(ay, az);
    float accel_pitch = atan2f(-ax, az);

    constexpr float alpha = 0.98f;
    cf_roll  = alpha*(cf_roll  + gx*dt)
             + (1-alpha)*accel_roll;
    cf_pitch = alpha*(cf_pitch + gy*dt)
             + (1-alpha)*accel_pitch;

    // Teleplot output (VSCode Teleplot extension)
    ws::print(">cf_roll:%.1f", cf_roll*57.3f);
    ws::print(">eskf_roll:%.1f",
              ws::estimated_roll()*57.3f);
}
""")

    add_content_slide(prs, "Teleplot によるリアルタイム可視化", [
        "Teleplot フォーマット: >変数名:値 をシリアルに出力するだけ",
        "",
        "ws::print(\">cf_roll:%.2f\", cf_roll * 57.3f);",
        "ws::print(\">eskf_roll:%.2f\", ws::estimated_roll() * 57.3f);",
        "",
        "セットアップ:",
        "1. VSCode 拡張: alexnesnes.teleplot をインストール",
        "2. Teleplot パネルでシリアルポートを選択",
        "3. 自動でグラフ化される",
        "",
        "注意: 400Hz 全 tick で出力するとシリアル帯域に負荷。4 tick 毎（100Hz）にデシメーション推奨",
    ])

    add_content_slide(prs, "Madgwick フィルタ", [
        "勾配降下法によるクォータニオン推定",
        "チューニングパラメータが β の1つだけ",
        "",
        "状態: q = [q₀, q₁, q₂, q₃]",
        "予測: q̇ = ½ q ⊗ ω",
        "補正勾配: ∇f = Jᵀ(q̂, d̂) · f(q̂, d̂)",
        "更新: q_{t+1} = q_t + dt·(½ q_t ⊗ ω − β · ∇f / |∇f|)",
        "",
        "f: 加速度/磁気の目的関数、J: そのヤコビアン、β: 補正ゲイン (≈ 0.04)",
    ])

    add_content_slide(prs, "EKF（拡張カルマンフィルタ）", [
        "非線形カルマンフィルタの5ステップ",
        "状態ベクトル例: x = [φ, θ, ψ, b_gx, b_gy, b_gz]ᵀ",
        "",
        "1. 状態予測: x̂⁻ = f(x̂, u)  ← ジャイロで姿勢を積分",
        "2. 共分散予測: P⁻ = FPFᵀ + Q  ← 不確かさの伝播",
        "3. カルマンゲイン: K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹",
        "4. 状態更新: x̂ = x̂⁻ + K(z − h(x̂⁻))  ← 観測で補正",
        "5. 共分散更新: P = (I − KH)P⁻  ← 不確かさを縮小",
    ])

    add_content_slide(prs, "ESKF（誤差状態カルマンフィルタ）", [
        "StampFly で実際に使われている 15 状態 ESKF",
        "名目状態 + 誤差状態の分離アーキテクチャ",
        "",
        "誤差状態ベクトル（15 states）:",
        "δx = [δp, δv, δθ, δb_g, δb_a]ᵀ",
        "",
        "予測: 名目状態はジャイロ/加速度で直接積分、誤差共分散のみ伝播",
        "更新: 気圧/ToF/磁気/光学フローの各観測で誤差状態を補正",
        "",
        "利点: 誤差状態は常に小さいため線形化が正確",
    ])

    add_table_slide(prs, "推定手法の比較 / Estimator Comparison",
        ["", "相補フィルタ", "Madgwick", "EKF", "ESKF"],
        [
            ["計算量", "極小", "小", "中", "中〜大"],
            ["パラメータ数", "1 (α)", "1 (β)", "Q, R 行列", "Q, R 行列"],
            ["推定対象", "Roll/Pitch", "Roll/Pitch/Yaw", "姿勢+バイアス", "位置/速度/姿勢/バイアス"],
            ["使用センサ", "Gyro+Accel", "Gyro+Accel+Mag", "Gyro+Accel+Mag", "全センサ（6種）"],
            ["精度", "○", "◎", "◎", "◎◎"],
        ],
    )

    add_checkpoint_slide(prs, [
        "手で傾けると CF のロール・ピッチが変化する",
        "CF と ESKF の値が概ね一致する（Teleplot で確認）",
        "α を変えて応答の違いを観察した",
        "Madgwick / EKF / ESKF の特徴を説明できる",
    ], "Lesson 10: API 総覧とアプリケーション開発")

    return prs


def build_lesson_10() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 10: ws:: API リファレンス",
                    "ws:: API Reference")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "ws:: API 全 43 関数をカテゴリ別に理解し、各関数の使い方を把握する",
        "",
        "• ws:: API 全 43 関数をカテゴリ別に理解",
        "• 各 API の使い方をコード例で確認",
        "• 環境・距離センサ API で全センサに直接アクセス",
    ])

    add_table_slide(prs, "ws:: API 一覧 / ws:: API Reference",
        ["カテゴリ", "関数", "説明"],
        [
            ["Motor", "motor_set_duty(m, d), motor_mixer(t,r,p,y)", "モータ制御"],
            ["RC Input", "rc_throttle/roll/pitch/yaw()", "スティック値 [-1,1]"],
            ["Buttons", "rc_throttle_yaw_button(), rc_stabilize_acro_mode()", "ボタン/モード"],
            ["LED", "led_color(r,g,b), disable/enable_led_task()", "LED 制御"],
            ["IMU", "gyro_x/y/z(), accel_x/y/z()", "角速度, 加速度"],
            ["Env/Distance", "baro_altitude(), mag_x(), tof_bottom(), flow_vx()", "環境・距離 (10)"],
            ["Estimation", "estimated_roll/pitch/yaw/altitude()", "ESKF 推定値"],
            ["Utility", "millis(), battery_voltage(), print()", "時刻, 電圧, 出力"],
        ],
    )

    add_table_slide(prs, "推定値 API / Estimation API",
        ["関数", "説明", "単位"],
        [
            ["estimated_roll()", "ESKF ロール推定角", "rad"],
            ["estimated_pitch()", "ESKF ピッチ推定角", "rad"],
            ["estimated_yaw()", "ESKF ヨー推定角", "rad"],
            ["estimated_altitude()", "ESKF 推定高度", "m"],
        ],
    )

    add_table_slide(prs, "ハードウェアセンサ仕様 / Hardware Sensors",
        ["センサ", "型番", "サンプルレート", "測定量"],
        [
            ["IMU", "BMI270", "400 Hz", "加速度 + ジャイロ"],
            ["気圧", "BMP280", "50 Hz", "気圧 → 高度"],
            ["磁気", "BMM150", "10-30 Hz", "磁気ベクトル"],
            ["ToF（下方）", "VL53L3CX", "30 Hz", "対地距離（0-2 m）"],
            ["ToF（前方）", "VL53L3CX", "30 Hz", "前方距離（0-2 m）"],
            ["光学フロー", "PMW3901", "100 Hz", "対地速度"],
        ],
    )

    add_table_slide(prs, "Environmental / Distance Sensors API（1/2）",
        ["関数", "単位", "説明"],
        [
            ["baro_altitude()", "m", "気圧高度（BMP280）"],
            ["baro_pressure()", "Pa", "気圧値（BMP280）"],
            ["mag_x()", "uT", "磁気 X 成分（BMM150）"],
            ["mag_y()", "uT", "磁気 Y 成分（BMM150）"],
            ["mag_z()", "uT", "磁気 Z 成分（BMM150）"],
        ],
    )

    add_table_slide(prs, "Environmental / Distance Sensors API（2/2）",
        ["関数", "単位", "説明"],
        [
            ["tof_bottom()", "m", "下方 ToF 距離（VL53L3CX, 0-2m）"],
            ["tof_front()", "m", "前方 ToF 距離（-1 = 未接続）"],
            ["flow_vx()", "m/s", "光学フロー速度 X（PMW3901）"],
            ["flow_vy()", "m/s", "光学フロー速度 Y（PMW3901）"],
            ["flow_quality()", "0-255", "光学フロー品質（高い = 良好）"],
        ],
    )

    add_content_slide(prs, "Teleplot: リアルタイムグラフ化", [
        "Teleplot 形式 (>name:value) でシリアル出力すると、",
        "VS Code 拡張 Teleplot がリアルタイムグラフを描画する",
        "",
        'ws::print(">baro_alt:%.2f", ws::baro_altitude());',
        'ws::print(">tof_bottom:%.3f", ws::tof_bottom());',
        'ws::print(">eskf_alt:%.2f", ws::estimated_altitude());',
        "",
        "セットアップ: VSCode 拡張 alexnesnes.teleplot をインストール",
        "→ Teleplot パネルでシリアルポートを選択 → 自動グラフ化",
    ])

    add_code_slide(prs, "実習: 全センサ Teleplot 出力", """
#include "workshop_api.hpp"
void setup() { ws::print("Lesson 10: Sensor API"); }
void loop_400Hz(float dt) {
    static uint32_t tick = 0; tick++;
    if (tick % 8 != 0) return;  // 50 Hz
    // Environmental sensors (ws:: API)
    ws::print(">baro_alt:%.2f", ws::baro_altitude());
    ws::print(">mag_x:%.1f", ws::mag_x());
    ws::print(">tof_bottom:%.3f", ws::tof_bottom());
    ws::print(">flow_vx:%.3f", ws::flow_vx());
    // ESKF estimation
    ws::print(">eskf_alt:%.2f", ws::estimated_altitude());
    ws::print(">eskf_roll:%.1f", ws::estimated_roll()*57.3f);
}
""")

    add_checkpoint_slide(prs, [
        "ws:: API 全 43 関数のカテゴリと役割を把握した",
        "Motor / RC / IMU / Sensor / Estimation の使い方を理解した",
        "気圧・磁気・ToF・光学フロー API で値を取得できた",
        "Teleplot で複数センサのグラフを同時表示した",
    ], "Lesson 11: 独自ファームウェア開発")

    return prs


def build_lesson_11() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 11: 独自ファームウェア開発",
                    "Custom Firmware Development")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "sf app new で独自プロジェクトを作り、ネイティブ API で全センサにアクセスする",
        "",
        "• sf app new で独自ファームウェアプロジェクトを作成",
        "• テンプレートの構造（app_main, ControlTask）を理解",
        "• StampFlyState で全センサ・推定値に直接アクセス",
        "• Teleplot でセンサデータをリアルタイム可視化",
    ])

    add_content_slide(prs, "作業フローの変更", [
        "⚠ このレッスンでは sf lesson switch は使いません",
        "ワークショップ（firmware/workshop/）の外に、独立したプロジェクトを作ります。",
        "",
        "【L0-L10（これまで）】",
        "• 開始: sf lesson switch N",
        "• 編集先: firmware/workshop/main/user_code.cpp",
        "• ビルド: sf lesson build → sf lesson flash",
        "",
        "【L11（このレッスン）】",
        "• 開始: sf app new my_drone",
        "• 編集先: firmware/my_drone/main/main.cpp",
        "• ビルド: sf build my_drone → sf flash my_drone -m",
    ])

    add_content_slide(prs, "ワークショップ → ネイティブ開発", [
        "【ws:: ワークショップ】",
        "• ws::gyro_x(), ws::alt()",
        "• ws::print(\">tag:%.2f\", v)",
        "• setup() / loop_400Hz(dt)",
        "• 1 関数 = 1 センサ値",
        "",
        "【ネイティブ開発】",
        "• state.getIMUData(a, g)",
        "• printf(\">tag:%.2f\\n\", v)",
        "• app_main() / ControlTask()",
        "• 構造体で複数値を一括取得",
        "",
        "ネイティブ = vehicle ファームウェアと同じ構成。あらゆるアルゴリズムを実装可能。",
    ])

    add_content_slide(prs, "sf app new でプロジェクト作成", [
        "vehicle と同じ環境を独自プロジェクトとして生成",
        "",
        "# sf app new my_drone    → firmware/my_drone/ が生成",
        "# sf build my_drone      → ビルド",
        "# sf flash my_drone -m   → 書き込み + モニタ",
        "",
        "【テンプレートの特徴】",
        "• vehicle と同じセンサタスク・初期化コードを再利用",
        "• ControlTask() のみを独自実装",
        "• IMU, 気圧, ToF, 光学フロー, モーター等すべて利用可能",
    ])

    add_content_slide(prs, "プロジェクト構成 / Project Structure", [
        "sf app new my_drone が生成するファイル:",
        "",
        "firmware/my_drone/",
        "  CMakeLists.txt      → vehicle/components, common を参照",
        "  main/",
        "    CMakeLists.txt    → vehicle タスク・初期化を直接コンパイルに含める",
        "    main.cpp          → ← ここを編集",
        "",
        "• CMakeLists.txt: 全センサ・通信・推定コンポーネントを利用可能にする",
        "• main/CMakeLists.txt: IMU, Baro, ToF 等のタスクと init.cpp を再利用",
        "• main/main.cpp: ユーザーが編集する唯一のファイル（ControlTask + コールバック）",
    ])

    add_content_slide(prs, "main.cpp の全体構造", [
        "【app_main() — ブートシーケンス】",
        "NVS → センサ初期化 → ESKF 起動 → タスク起動（自動生成済み）",
        "",
        "【ControlTask() — 400 Hz ユーザーコード ← ここを編集】",
        "センサ読み取り・制御計算・モーター出力を記述するメインループ",
        "",
        "【コールバック関数】",
        "• onButtonEvent() — ボタン押下で ARM / DISARM",
        "• handleControlInput() — コントローラからの操縦入力を処理",
    ])

    add_code_slide(prs, "ControlTask の中身", """
void ControlTask(void* pvParameters) {
    while (true) {
        xSemaphoreTake(g_control_semaphore, ...);
        // 1. センサ読み取り
        state.getIMUData(accel, gyro);    // 400 Hz
        state.getAttitudeEuler(r, p, y);  // ESKF
        state.getBaroData(alt, p);        // 50 Hz
        state.getToFData(bot, fnt);       // 30 Hz
        // 2. 制御計算（ユーザー実装）
        // 3. モーター出力
        // g_motor.setThrust(0, thrust);
        // 4. Teleplot 出力（50 Hz）
        if (tick % 8 == 0) printf(...);
    }
}
""")

    add_table_slide(prs, "StampFlyState API",
        ["メソッド", "ソース", "取得データ"],
        [
            ["getIMUData(a, g)", "BMI270", "加速度 + ジャイロ"],
            ["getBaroData(alt, p)", "BMP280", "高度 [m], 気圧 [Pa]"],
            ["getMagData(mag)", "BMM150", "磁気 x,y,z [uT]"],
            ["getToFData(b, f)", "VL53L3CX", "下方/前方距離 [m]"],
            ["getFlowData(vx, vy)", "PMW3901", "速度 vx,vy [m/s]"],
            ["getAttitudeEuler(r, p, y)", "ESKF", "roll, pitch, yaw [rad]"],
            ["getPowerData(v, i)", "INA3221", "電圧 [V], 電流 [A]"],
        ],
    )

    add_code_slide(prs, "実習: 全センサ Teleplot 可視化\n"
                        "（編集: firmware/my_viz/main/main.cpp の ControlTask 内）", """
auto& state = stampfly::StampFlyState::getInstance();
stampfly::Vec3 accel, gyro;
state.getIMUData(accel, gyro);
float alt, p;
state.getBaroData(alt, p);
float roll, pitch, yaw;
state.getAttitudeEuler(roll, pitch, yaw);

static uint32_t tick = 0; tick++;
if (tick % 8 == 0) {  // 50 Hz
    printf(">baro_alt:%.2f\\n", alt);
    printf(">roll_deg:%.1f\\n", roll * 57.3f);
    printf(">gyro_x:%.3f\\n", gyro.x);
}
""")

    add_checkpoint_slide(prs, [
        "sf app new でプロジェクトを作成しビルドできた",
        "プロジェクト構成（3 ファイルの役割）を理解した",
        "app_main() と ControlTask() の役割を理解した",
        "StampFlyState で全センサ値を直接取得できた",
        "Teleplot でリアルタイムグラフを確認した",
    ], "Lesson 12: Python SDK プログラム飛行")

    return prs


def build_lesson_12() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 12: Python SDK プログラム飛行",
                    "Python SDK Programmatic Flight")

    add_content_slide(prs, "今日のゴール / Today's Goal", [
        "Python SDK の設計思想と将来の自律飛行の姿を理解する",
        "",
        "• SDK アーキテクチャ（TCP CLI + WebSocket）を理解",
        "• Python API で離陸・移動・着陸をプログラム化",
        "• Tello SDK 互換設計の利点を知る",
    ])

    add_content_slide(
        prs, "SDK アーキテクチャ / SDK Architecture",
        [
            "PC (Python SDK) ↔ WiFi AP ↔ StampFly ESP32-S3",
            "",
            "TCP CLI (port 23): コマンド送受信（takeoff, land, move...）",
            "WebSocket (port 81): テレメトリ受信（400 Hz リアルタイム）",
            "",
            "WiFi AP アドレス: 192.168.4.1",
        ],
        image_path=IMAGES_DIR / "python_sdk_arch.png",
    )

    add_table_slide(prs, "Python SDK API", ["関数", "説明", "備考"], [
        ["StampFly(host)", "インスタンス生成", "デフォルト 192.168.4.1"],
        ["connect()", "WiFi 接続", "TCP CLI + WS"],
        ["takeoff()", "離陸", "ブロッキング"],
        ["land()", "着陸", "ブロッキング"],
        ["move_forward(cm)", "前進", "20-200 cm"],
        ["rotate_clockwise(deg)", "時計回り回転", "1-360°"],
        ["send_rc_control(lr,fb,ud,yaw)", "RC 制御値送信", "-100〜+100"],
        ["get_telemetry()", "テレメトリ取得", "400 Hz dict"],
        ["end()", "切断", "リソース解放"],
    ])

    add_content_slide(prs, "Tello SDK との互換性", [
        "djitellopy 互換設計",
        "Tello のコードをほぼそのまま StampFly に移植可能",
        "",
        "• 同じ API 名（takeoff, land, move_forward...）",
        "• connect_or_simulate() でオフライン開発も可能",
        "• 大学の Tello 教材を流用可能",
        "",
        "【StampFly の利点】",
        "• 内部の PID ゲインを自由に変更可能",
        "• テレメトリを 400 Hz で取得",
        "• 制御理論の実験プラットフォーム",
    ])

    add_code_slide(prs, "コード例: 基本フライト", """
from stampfly import StampFly

drone = StampFly()
drone.connect()

drone.takeoff()
drone.move_forward(50)    # 50 cm forward
drone.rotate_clockwise(90)
drone.move_forward(50)
drone.land()

drone.end()
""", filename="flight_basic.py")

    add_code_slide(prs, "コード例: テレメトリ取得", """
import time
from stampfly import StampFly

drone = StampFly()
drone.connect()
drone.takeoff()

# Hover + read telemetry for 5 seconds
for _ in range(50):
    drone.send_rc_control(0, 0, 0, 0)
    t = drone.get_telemetry()
    print(f"alt={t.get('alt',0):.1f}")
    time.sleep(0.1)

drone.land()
drone.end()
""", filename="flight_telemetry.py")

    add_table_slide(prs, "開発ロードマップ / Development Roadmap",
        ["状態", "機能"],
        [
            ["✅", "TCP CLI 通信（port 23）"],
            ["✅", "WebSocket テレメトリ（port 81）"],
            ["✅", "ws:: ワークショップ API"],
            ["✅", "sf CLI ツール群"],
            ["🔧", "Python SDK パッケージ"],
            ["🔧", "connect_or_simulate()"],
            ["📋", "Jupyter Notebook 連携"],
            ["📋", "自律ミッション（ウェイポイント）"],
        ],
    )

    add_checkpoint_slide(prs, [
        "SDK アーキテクチャ（TCP + WebSocket）を理解した",
        "Python API の基本関数を把握した",
        "Tello 互換の設計意図を理解した",
        "開発ロードマップを確認した",
    ], "Lesson 13: 精密着陸競技会 ルール説明")

    return prs


def build_lesson_13() -> Presentation:
    prs = new_presentation()

    add_title_slide(prs, "Lesson 13: 精密着陸競技会",
                    "Precision Landing Competition")

    add_content_slide(prs, "競技会概要 / Competition Overview", [
        "精密着陸競技",
        "パイロットは定位置から操縦し、3m 先のヘリポートに精密着陸する",
        "",
        "• 種目: 精密着陸（パイロット定位置、3m 先のヘリポートに着陸）",
        "• ヘリポート: 約 40cm × 40cm",
        "• 評価: 着陸までのタイム（ベストタイム採用）",
    ])

    add_table_slide(prs, "競技ルール詳細 / Competition Rules",
        ["項目", "内容"],
        [
            ["種目", "精密着陸（パイロット定位置操縦）"],
            ["距離", "パイロットからヘリポートまで 3m"],
            ["ヘリポート", "40cm × 40cm"],
            ["制限時間", "60 秒"],
            ["試行回数", "3 回（ベストタイム採用）"],
            ["判定", "ARM → 離陸 → ヘリポート着陸までの時間"],
            ["パイロット", "定位置から動かない"],
            ["失格条件", "安全エリア外飛行、手による介入、パイロットの移動"],
        ],
    )

    add_table_slide(prs, "タイムスケジュール / Schedule",
        ["時間", "内容"],
        [
            ["Day 4: 13:00-13:30", "ルール説明（このスライド）"],
            ["Day 4: 13:30-16:00", "準備・自由練習・予選"],
            ["Day 5: 9:00-9:15", "ルール確認・機体チェック"],
            ["Day 5: 9:15-9:45", "最終チューニング"],
            ["Day 5: 9:45-10:45", "競技本番"],
            ["Day 5: 10:45-11:30", "結果発表・表彰・振り返り"],
        ],
    )

    add_content_slide(prs, "攻略のヒント / Tips for Success", [
        "【PID ゲイン調整】",
        "• 安定性重視（急旋回よりも穏やかな応答）",
        "• ヨー制御で方向を合わせる",
        "• スロットル微調整で高度を安定させる",
        "",
        "【練習のポイント】",
        "• まず安定ホバリングを確立",
        "• 前進 → 停止 → 降下の手順を練習",
        "• バッテリ・プロペラを毎回チェック",
        "",
        "テレメトリ活用: Teleplot でリアルタイムデータを確認",
    ])

    add_checkpoint_slide(prs, [
        "競技ルールと失格条件を理解した",
        "バッテリがフル充電されている",
        "PID ゲインの調整方法を把握している",
        "安全チェックリストを確認した",
    ], "さあ、練習を始めよう！ 13:30 から自由練習")

    return prs


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

BUILDERS = {
    0: build_lesson_00,
    1: build_lesson_01,
    2: build_lesson_02,
    3: build_lesson_03,
    4: build_lesson_04,
    5: build_lesson_05,
    6: build_lesson_06,
    7: build_lesson_07,
    8: build_lesson_08,
    9: build_lesson_09,
    10: build_lesson_10,
    11: build_lesson_11,
    12: build_lesson_12,
    13: build_lesson_13,
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate StampFly workshop PPTX")
    parser.add_argument("--lesson", type=int, help="Lesson number (0-13)")
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if args.lesson is not None:
        lessons = [args.lesson]
    else:
        lessons = sorted(BUILDERS.keys())

    for n in lessons:
        builder = BUILDERS.get(n)
        if builder is None:
            print(f"Unknown lesson: {n}")
            continue
        prs = builder()
        out_path = OUTPUT_DIR / f"lesson_{n:02d}.pptx"
        prs.save(str(out_path))
        print(f"Generated: {out_path}")


if __name__ == "__main__":
    main()
