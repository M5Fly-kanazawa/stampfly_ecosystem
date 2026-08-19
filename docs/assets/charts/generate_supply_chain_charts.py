"""
Drone supply-chain dependency visualization for invited lecture.

Sources:
  - 東洋経済オンライン (2025) "ドローン国産6割は幻想なのか" — 国産シェア約3%
    https://toyokeizai.net/articles/-/939975
  - 日本経済新聞 (2026/4) "国内市場9割が中国製"
  - JUIDA (2020) — 国産普及率 3.8%
  - 36Kr / Strainer — DJI 世界シェア 約76%
  - CSIS (2025) "Drone Supply Chain War" — レアアース・電池・FC の中国依存
  - DroneXL / FCC (2025) — ドローン用電池の中国シェア
  - USGS / IEA — ネオジム磁石/リチウム電池の中国生産比率
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
    "legend.fontsize": 12,
})

COLOR_CHINA = "#D62728"
COLOR_JAPAN = "#2E5EAA"
COLOR_OTHER = "#A0A0A0"
COLOR_REFERENCE = "#444444"


# ---- Chart 5: Japan domestic drone market by maker country ----
fig, ax = plt.subplots(figsize=(9, 6.5))

labels = ["中国製\n約90%", "国産\n約3%", "その他\n(米欧等)\n約7%"]
sizes = [90, 3, 7]
colors = [COLOR_CHINA, COLOR_JAPAN, COLOR_OTHER]
explode = (0.0, 0.12, 0.0)  # pop out the Japan slice

wedges, texts, autotexts = ax.pie(
    sizes,
    labels=labels,
    colors=colors,
    autopct="%.0f%%",
    startangle=90,
    counterclock=False,
    explode=explode,
    pctdistance=0.72,
    labeldistance=1.12,
    wedgeprops=dict(edgecolor="white", linewidth=2),
    textprops=dict(fontsize=13),
)
for at in autotexts:
    at.set_color("white")
    at.set_fontweight("bold")
    at.set_fontsize(14)

ax.set_title("日本国内のドローン市場シェア(メーカー国別, 2025年)", pad=20)

# Highlight commentary
fig.text(0.5, 0.04,
         "国産ドローンは国内市場でわずか 3% — 機体の9割超が中国メーカー製",
         ha="center", va="bottom", fontsize=13, color=COLOR_CHINA, fontweight="bold")
fig.text(0.99, 0.005,
         "出所: 東洋経済 (2025)、日本経済新聞 (2026/4)、JUIDA",
         ha="right", va="bottom", fontsize=9, color="gray")

fig.tight_layout(rect=[0, 0.08, 1, 1])
fig.savefig(f"{OUT_DIR}/05_japan_market_by_country.png", dpi=DPI, bbox_inches="tight")
plt.close(fig)
print("Saved: 05_japan_market_by_country.png")


# ---- Chart 6: China share of drone-related strategic items ----
fig, ax = plt.subplots(figsize=(11, 6.5))

# Items and Chinese share (%)
items = [
    "ドローン用リチウム電池\n(FCC調査)",
    "ネオジム磁石\n(高性能モータ)",
    "リチウムイオン電池\n(世界生産)",
    "民生用ドローン世界シェア\n(DJI単独)",
    "中国メーカーの世界\nドローン市場シェア",
    "日本国内ドローン市場\n中国製シェア",
    "日本国内市場 国産シェア\n(参考)",
]
shares = [99, 90, 85, 76, 75, 90, 3]
# Color: red bars for China share, blue bar for Japan reference (last)
bar_colors = [COLOR_CHINA] * 6 + [COLOR_JAPAN]

y_pos = np.arange(len(items))
bars = ax.barh(y_pos, shares, color=bar_colors, edgecolor="black", linewidth=0.5)

for bar, v in zip(bars, shares):
    label_x = v + 1.5
    ax.text(label_x, bar.get_y() + bar.get_height() / 2,
            f"{v}%", va="center", ha="left",
            fontsize=12, fontweight="bold")

ax.set_yticks(y_pos)
ax.set_yticklabels(items, fontsize=11)
ax.invert_yaxis()  # top item first
ax.set_xlabel("シェア / 依存度 [%]")
ax.set_xlim(0, 110)
ax.set_title("ドローン関連の戦略物資 — 中国シェアと日本の現状")
ax.grid(True, alpha=0.3, axis="x")

# 50% reference line
ax.axvline(x=50, color=COLOR_REFERENCE, linestyle="--", linewidth=1, alpha=0.5)
ax.text(50, -0.7, "50%", ha="center", va="bottom", fontsize=10, color=COLOR_REFERENCE)

# Legend / commentary
fig.text(0.99, 0.005,
         "出所: CSIS (2025)、DroneXL/FCC (2025)、IEA、USGS、36Kr、東洋経済、日経",
         ha="right", va="bottom", fontsize=9, color="gray")

fig.tight_layout(rect=[0, 0.04, 1, 1])
fig.savefig(f"{OUT_DIR}/06_supply_chain_dependency.png", dpi=DPI, bbox_inches="tight")
plt.close(fig)
print("Saved: 06_supply_chain_dependency.png")

print("\nSupply-chain charts generated in:", OUT_DIR)
