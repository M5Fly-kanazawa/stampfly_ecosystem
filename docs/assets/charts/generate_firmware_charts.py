"""
Drone firmware origin / domestic ratio visualization for invited lecture.

Sources:
  - Dronecode Foundation "The 2024 Year in Review"
    https://dronecode.org/the-2024-year-in-review/
    (PX4 contributor country share: US 21%, Switzerland 16%, Auterion 37.95%)
  - インプレス Drone Journal "ArduPilot と PX4 の違い" (2025)
    https://drone-journal.impress.co.jp/docs/special/1186463.html
    "日本製ドローンの多くが ArduPilot に代表されるオープンソース FW を採用"
  - 春原久徳 "日本の産業用ドローンで急速に浸透するArdupilot" (2021)
    https://drone.jp/column/2021070616000046984.html
  - DRONE MEDIA / techblitz — ACSL は世界でも数えるほどしかない
    「制御ソフトウェアをソースコードから自社で保有」する企業
  - 東洋経済 (2025) — 国内市場の国産シェア約3%
  - 日経 (2026/4) — 国内市場の中国製シェア9割超
  - CSIS (2025) — レアアース・電池の中国依存
"""

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

JP_FONT_PATH = "/System/Library/Fonts/ヒラギノ角ゴシック W6.ttc"
font_manager.fontManager.addfont(JP_FONT_PATH)
matplotlib.rcParams["font.family"] = "Hiragino Sans"
matplotlib.rcParams["axes.unicode_minus"] = False

OUT_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/docs/assets/charts"
DPI = 300

plt.rcParams.update({
    "axes.titlesize": 18,
    "axes.labelsize": 14,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.fontsize": 11,
})

COLOR_US = "#2E5EAA"
COLOR_CH = "#D62728"
COLOR_OTHER = "#888888"
COLOR_JP = "#F5A623"
COLOR_CHINA = "#D62728"
COLOR_DOMESTIC = "#2E5EAA"


# ---- Chart 7: PX4 open-source FC contributor share by country ----
# Slice order chosen so the small Japan wedge lands at lower-right (away from title)
fig, ax = plt.subplots(figsize=(10, 6.8))

labels = [
    "米国",
    "スイス\n(Auterion中心)",
    "日本\n(推定)",
    "その他\n(EU・豪・中・他)",
]
sizes = [21, 16, 2, 61]
colors = [COLOR_US, COLOR_CH, COLOR_JP, COLOR_OTHER]
explode = (0.0, 0.0, 0.20, 0.0)

wedges, texts, autotexts = ax.pie(
    sizes,
    labels=labels,
    colors=colors,
    autopct="%.0f%%",
    startangle=90,
    counterclock=False,
    explode=explode,
    pctdistance=0.72,
    labeldistance=1.18,
    wedgeprops=dict(edgecolor="white", linewidth=2),
    textprops=dict(fontsize=12),
)
for at in autotexts:
    at.set_color("white")
    at.set_fontweight("bold")
    at.set_fontsize(13)

ax.set_title("PX4(主要オープンソースFC)の開発貢献 国別シェア", pad=24)

fig.text(0.5, 0.04,
         "オープンソースFCの開発主体は欧米 — 日本のコミット率は数%以下",
         ha="center", va="bottom", fontsize=12.5, color="#333", fontweight="bold")
fig.text(0.99, 0.005,
         "出所: Dronecode Foundation『The 2024 Year in Review』(2024年データ)",
         ha="right", va="bottom", fontsize=9, color="gray")

fig.tight_layout(rect=[0, 0.08, 1, 1])
fig.savefig(f"{OUT_DIR}/07_px4_contributor_country.png", dpi=DPI, bbox_inches="tight")
plt.close(fig)
print("Saved: 07_px4_contributor_country.png")


# ---- Chart 8: Japan drone industry domestic-content by layer ----
fig, ax = plt.subplots(figsize=(11, 6.8))

# Layers (top → bottom of stack), with approximate domestic-content % and the dominant foreign source
layers = [
    "機体ブランド\n(国内市場における国産シェア)",
    "フライトファームウェア\n(完全自社開発の比率, 推定)",
    "フライトコントローラ基板\n(国産設計・製造の比率, 推定)",
    "リチウムイオン電池\n(国内生産比率, 推定)",
    "ネオジム磁石(モータ用)\n(国内生産比率, 推定)",
]
domestic = [3, 8, 15, 15, 10]
foreign = [100 - v for v in domestic]
foreign_source = [
    "中国メーカー約90% (DJI等)",
    "海外OSS(ArduPilot/PX4)が大半",
    "海外設計のPixhawk派生が大半",
    "中国 約85% (CATL等)",
    "中国 約90%",
]

y_pos = np.arange(len(layers))
bars_d = ax.barh(y_pos, domestic, color=COLOR_DOMESTIC, edgecolor="black",
                  linewidth=0.5, label="国産・自社開発")
bars_f = ax.barh(y_pos, foreign, left=domestic, color=COLOR_CHINA,
                  alpha=0.55, edgecolor="black", linewidth=0.5,
                  label="海外発(中国 / OSS / その他)")

for i, (d, f, src) in enumerate(zip(domestic, foreign, foreign_source)):
    if d >= 8:
        # Wide enough — place inside bar in white
        ax.text(d / 2, i, f"{d}%", va="center", ha="center",
                fontsize=11, fontweight="bold", color="white")
    else:
        # Very narrow bar — place at the bar's vertical center, just outside its right edge
        ax.text(d + 0.6, i, f"{d}%", va="center", ha="left",
                fontsize=11, fontweight="bold", color=COLOR_DOMESTIC)
    ax.text(d + f / 2, i, src, va="center", ha="center",
            fontsize=10.5, color="#1a1a1a")

ax.set_yticks(y_pos)
ax.set_yticklabels(layers, fontsize=11)
ax.invert_yaxis()
ax.set_xlim(0, 100)
ax.set_xlabel("構成比 [%]")
ax.set_title("日本のドローン産業 — 階層別 国産シェアの実態", pad=12)
ax.legend(loc="lower right", framealpha=0.95)
ax.grid(True, alpha=0.3, axis="x")

fig.text(0.99, 0.005,
         "出所: 東洋経済(2025)・日経(2026/4)・Drone Journal・Dronecode Foundation・CSIS・IEA "
         "／ ※自社FW・基板の比率は産業構造から推定",
         ha="right", va="bottom", fontsize=8.5, color="gray")

fig.tight_layout(rect=[0, 0.05, 1, 1])
fig.savefig(f"{OUT_DIR}/08_japan_layer_domestic_share.png", dpi=DPI, bbox_inches="tight")
plt.close(fig)
print("Saved: 08_japan_layer_domestic_share.png")

print("\nFirmware charts generated in:", OUT_DIR)
