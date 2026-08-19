"""
Drone market visualization for invited lecture.
Sources:
  - Japan: Impress Research Institute, "Drone Business Survey Report 2025"
    https://research.impress.co.jp/topics/list/drone/710
  - World: SkyQuest Technology Consulting (Drone Market Report)
    https://www.skyquestt.com/report/drone-market
  - Cross-check: Grand View Research, MarketsandMarkets, Yano Research

Unit: 兆円 (trillion JPY).
Currency conversion: 1 USD = 150 JPY (approx, 2024-2025 average).
"""

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

# Japanese font setup (macOS Hiragino)
JP_FONT_PATH = "/System/Library/Fonts/ヒラギノ角ゴシック W6.ttc"
font_manager.fontManager.addfont(JP_FONT_PATH)
matplotlib.rcParams["font.family"] = "Hiragino Sans"
matplotlib.rcParams["axes.unicode_minus"] = False

OUT_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/docs/assets/charts"
DPI = 300
JPY_PER_USD = 150.0

# ---- Data ----
# Years (FY)
years = np.array([2024, 2025, 2026, 2027, 2028, 2029, 2030])

# World drone market (SkyQuest): $54.83B (2024) → $117.62B (2030), CAGR ~12.54%
world_usd_b = np.array([54.83, 65.14, 73.31, 82.51, 92.86, 104.50, 117.62])
# USD billion → 兆円: $1B × 150 JPY/USD = 1.5×10^11 JPY = 0.15 兆円 → divisor 1000
world_trln_jpy = world_usd_b * JPY_PER_USD / 1000.0

# Japan drone market (Impress): 4,371億円 (2024) → 10,195億円 (2030), CAGR 15.2%
japan_oku_jpy = np.array([4371, 4987, 5743, 6616, 7621, 8779, 10195])
japan_trln_jpy = japan_oku_jpy / 10000.0  # 億円 → 兆円

# Japan composition (Impress): 機体 / サービス / 周辺サービス, in 億円
# 2024: 1134 / 2295 / 942 = 4371
# 2030: 2746 / 5288 / 2161 = 10195 (CAGR 15.9 / 14.9 / 14.8 %)
cagr_kitai, cagr_service, cagr_peri = 0.159, 0.149, 0.148
kitai_2024, service_2024, peri_2024 = 1134, 2295, 942
kitai = np.array([kitai_2024 * (1 + cagr_kitai) ** (y - 2024) for y in years])
service = np.array([service_2024 * (1 + cagr_service) ** (y - 2024) for y in years])
peri = np.array([peri_2024 * (1 + cagr_peri) ** (y - 2024) for y in years])

# Japan share of world market
share_pct = 100.0 * japan_trln_jpy / world_trln_jpy

# ---- Common style ----
COLOR_WORLD = "#2E5EAA"
COLOR_JAPAN = "#D62728"
COLOR_KITAI = "#4C72B0"     # 機体 (hardware)
COLOR_SERVICE = "#DD8452"   # サービス
COLOR_PERI = "#55A868"      # 周辺サービス

plt.rcParams.update({
    "axes.titlesize": 18,
    "axes.labelsize": 14,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.fontsize": 12,
    "figure.titlesize": 20,
})


def annotate_points(ax, xs, ys, fmt="{:.2f}", color="black", offset=(0, 8)):
    for x, y in zip(xs, ys):
        ax.annotate(fmt.format(y), (x, y), textcoords="offset points",
                    xytext=offset, ha="center", color=color, fontsize=11)


# ---- Chart 1: Time series comparison (world vs Japan) ----
fig, ax = plt.subplots(figsize=(10, 6.5))
ax.plot(years, world_trln_jpy, "-o", color=COLOR_WORLD, linewidth=2.8,
        markersize=9, label="世界市場(SkyQuest)")
ax.plot(years, japan_trln_jpy, "-s", color=COLOR_JAPAN, linewidth=2.8,
        markersize=9, label="日本市場(インプレス)")

annotate_points(ax, years, world_trln_jpy, fmt="{:.1f}", color=COLOR_WORLD,
                offset=(0, 12))
# Japan line sits low on the shared axis; label above the markers to clear the line
annotate_points(ax, years, japan_trln_jpy, fmt="{:.2f}", color=COLOR_JAPAN,
                offset=(0, 14))

ax.set_xlabel("年度")
ax.set_ylabel("市場規模 [兆円]")
ax.set_title("ドローン市場規模の推移(2024-2030)")
ax.grid(True, alpha=0.3)
ax.legend(loc="upper left", framealpha=0.95)
ax.set_ylim(0, max(world_trln_jpy) * 1.22)

# CAGR box (upper-right, away from data)
ax.text(0.98, 0.78,
        "世界 CAGR 12.5%\n日本 CAGR 15.2%",
        transform=ax.transAxes, ha="right", va="top",
        fontsize=12,
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#F5F5F5", edgecolor="gray"))

# Source caption — below the plot area
fig.text(0.99, 0.01,
         "為替: 1 USD = 150 JPY 換算  /  出所: インプレス総合研究所『ドローンビジネス調査報告書2025』、SkyQuest",
         ha="right", va="bottom", fontsize=9, color="gray")

fig.tight_layout(rect=[0, 0.04, 1, 1])
fig.savefig(f"{OUT_DIR}/01_market_size_comparison.png", dpi=DPI, bbox_inches="tight")
plt.close(fig)
print("Saved: 01_market_size_comparison.png")


# ---- Chart 2: CAGR comparison ----
fig, ax = plt.subplots(figsize=(10, 6))
labels = ["世界全体\n(SkyQuest)", "世界全体\n(Grand View)", "アジア太平洋\n(GVR)",
         "日本\n(インプレス)", "北米\n(GVR)", "欧州\n(GVR)"]
cagrs = [12.5, 14.3, 18.4, 15.2, 10.5, 12.0]
colors = [COLOR_WORLD, COLOR_WORLD, "#7AA2D9", COLOR_JAPAN, "#888888", "#888888"]

bars = ax.bar(labels, cagrs, color=colors, edgecolor="black", linewidth=0.5)
for bar, v in zip(bars, cagrs):
    ax.text(bar.get_x() + bar.get_width() / 2, v + 0.5, f"{v:.1f}%",
            ha="center", fontsize=12, fontweight="bold")

ax.set_ylabel("年平均成長率 (CAGR) [%]")
ax.set_title("ドローン市場の年平均成長率比較(2024-2030予測)")
ax.set_ylim(0, max(cagrs) * 1.22)
ax.grid(True, alpha=0.3, axis="y")

# Reference line at world (SkyQuest) baseline — highlights regions outpacing the world
ax.axhline(y=cagrs[0], color=COLOR_WORLD, linestyle="--", linewidth=1.2, alpha=0.6)
# Place label in the gap between Japan and North America bars to avoid overlap
ax.text(3.5, cagrs[0] + 0.4,
        f"世界基準 {cagrs[0]:.1f}%", color=COLOR_WORLD, fontsize=10,
        ha="center", va="bottom",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white",
                  edgecolor=COLOR_WORLD, alpha=0.9, linewidth=0.8))

fig.text(0.99, 0.01,
         "出所: インプレス総合研究所、SkyQuest、Grand View Research",
         ha="right", va="bottom", fontsize=9, color="gray")

fig.tight_layout(rect=[0, 0.04, 1, 1])
fig.savefig(f"{OUT_DIR}/02_cagr_comparison.png", dpi=DPI, bbox_inches="tight")
plt.close(fig)
print("Saved: 02_cagr_comparison.png")


# ---- Chart 3: Japan market composition (stacked bar) ----
fig, ax = plt.subplots(figsize=(10, 6))
width = 0.65
# Convert 億円 → 兆円
kitai_t = kitai / 10000.0
service_t = service / 10000.0
peri_t = peri / 10000.0

p1 = ax.bar(years, kitai_t, width, color=COLOR_KITAI, label="機体", edgecolor="white", linewidth=0.5)
p2 = ax.bar(years, service_t, width, bottom=kitai_t, color=COLOR_SERVICE,
            label="サービス", edgecolor="white", linewidth=0.5)
p3 = ax.bar(years, peri_t, width, bottom=kitai_t + service_t, color=COLOR_PERI,
            label="周辺サービス", edgecolor="white", linewidth=0.5)

# Total labels on top
totals = kitai_t + service_t + peri_t
for x, t in zip(years, totals):
    ax.text(x, t + 0.025, f"{t:.2f}", ha="center", fontsize=11, fontweight="bold")

ax.set_xlabel("年度")
ax.set_ylabel("市場規模 [兆円]")
ax.set_title("日本のドローン市場の構成推移(機体・サービス・周辺)")
ax.legend(loc="upper left", framealpha=0.95)
ax.grid(True, alpha=0.3, axis="y")
ax.set_ylim(0, max(totals) * 1.18)

fig.text(0.99, 0.01,
         "出所: インプレス総合研究所『ドローンビジネス調査報告書2025』",
         ha="right", va="bottom", fontsize=9, color="gray")

fig.tight_layout(rect=[0, 0.04, 1, 1])
fig.savefig(f"{OUT_DIR}/03_japan_market_composition.png", dpi=DPI, bbox_inches="tight")
plt.close(fig)
print("Saved: 03_japan_market_composition.png")


# ---- Chart 4: Japan share of world market ----
fig, ax = plt.subplots(figsize=(10, 6))
bars = ax.bar(years, share_pct, color=COLOR_JAPAN, alpha=0.85,
              edgecolor="black", linewidth=0.5, width=0.65)
for bar, v in zip(bars, share_pct):
    ax.text(bar.get_x() + bar.get_width() / 2, v + 0.08, f"{v:.2f}%",
            ha="center", fontsize=11, fontweight="bold")

ax.set_xlabel("年度")
ax.set_ylabel("世界市場に対するシェア [%]")
ax.set_title("日本のドローン市場の世界シェア推移")
ax.grid(True, alpha=0.3, axis="y")
ax.set_ylim(0, max(share_pct) * 1.40)

# Reference line at average — label on the right side, away from bars
avg = share_pct.mean()
ax.axhline(y=avg, color="#444", linestyle="--", linewidth=1, alpha=0.6)
ax.text(years[-1] + 0.2, avg, f"平均 {avg:.2f}%",
        ha="left", va="center", fontsize=11, color="#444")

fig.text(0.99, 0.01,
         "為替: 1 USD = 150 JPY 換算  /  出所: 世界=SkyQuest、日本=インプレス",
         ha="right", va="bottom", fontsize=9, color="gray")

fig.tight_layout(rect=[0, 0.04, 1, 1])
fig.savefig(f"{OUT_DIR}/04_japan_world_share.png", dpi=DPI, bbox_inches="tight")
plt.close(fig)
print("Saved: 04_japan_world_share.png")

print("\nAll 4 charts generated in:", OUT_DIR)
