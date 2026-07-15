# Yaw-Axis Dynamics Model — Reaction-Torque Zero (LHP / Minimum-Phase, RESOLVED)
# ヨー軸ダイナミクスモデル — 反トルク零点（LHP・最小位相＝決着済み）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

> ## ⚠️ 最終決着（2026-06-21）— この問題はクローズ。再オープンしないこと
>
> 実飛行ログ（`sf log wifi`）の per-tone 解析で決着。タイトル「RHP 零点」は**誤り**（符号が逆）。以下が最終結論：
>
> ### 真のモデルの「形」（物理で確定）
> $$G_\text{yaw}(s)=\frac{b\,(1+\tau_z s)\,e^{-Ls}}{s\,(T_m s+1)},\quad \tau_z=\frac{I_r}{2k_Q\omega_0}>0$$
> ヨートルク＝抗力 $2k_Q\omega_0\,\delta\omega$ ＋反トルク $I_r\dot{\delta\omega}$。CW プロペラで両者**同符号**ゆえ $(1+\tau_z s)$ ＝**左半平面（LHP）＝最小位相零点＝位相リード**（RHP・非最小位相ではない）。roll/pitch は推力差動で反トルク項が無く $G=b e^{-Ls}/(s(T_m s+1))$（零点なし）。ここで $I_r$＝**回転子+プロペラの合成慣性 Jmp=2.01e-8 kg·m²**（モータ単体 Jm≈1e-9 ではない。プロペラが支配的）。
>
> **物理計算による独立検証**（2026-06-21）: 計測パラメータ($C_q$=9.71e-11, $K_m$=6.125e-4, $R_m$=0.34, $J_{mp}$=2.01e-8, $\omega_0$=2930 rad/s)から $T_m=J_{mp}/(K_m^2/R_m+D_m+2C_q\omega_0)=11.8$ms(13.5Hz)・$\tau_z=J_{mp}/(2C_q\omega_0)=35.3$ms(4.5Hz)。**比 $\tau_z/T_m=1+(K_m^2/R_m+D_m)/(2C_q\omega_0)=3.0$ は慣性に無関係**＝零点は常に極の3倍低域＝必ず低域リード。交差周波数(2.9Hz)で零点/極因子は **+20.5°リード**で、**フライトデータの +22〜32°リードと一致**。→ 反トルク零点は理論とデータが独立に確認。**設計を積分器+遅れで行うのは保守的（+20°リードはタダの位相余裕）。**
>
> **【追記 2026-07-15】パラメータ再測定による数値更新**: コーストダウン試験・プロペラ写真の画素直接積分・回転子実測諸元により $J_{mp}$=**1.375e-8**（旧2.01e-8）、$C_q$=**4.10e-11**（旧9.71e-11）、$\omega_0$=**3670 rad/s**（旧2930）と確定（multicopter_introduction/notes/qa_log.md Q4-9..13）。再計算: **$\tau_z = J_{mp}/(2C_q\omega_0) = 45.7$ ms (3.5 Hz)**。$K_e$ は 2026-07-15 に**開回路EMFで直接測定: 6.82e-4**（$R$=0.593 は論文LCR値を採用）。γ=1 で $T_m$=12.7 ms、飛行同定の 20〜34 ms は実効導通率 γ<1 に相当。交差 2.9 Hz のリード予測は +26.8° となり、フライト実測 +22〜32° の中央と**一致**。結論（LHP・設計は積分器+遅れ）は不変。
>
> ### 同定の限界（実データで証明）
> - 実飛行 per-tone データは積分器+遅れより**LHP零点+極**で良く合う（resid 0.128→0.018）。低域位相リード（2Hz で −58°）がその証拠で、漏れでも積分初期化でもない**本物**。
> - **しかし個別の零点/極は同定不能**（プロファイル：$\tau_z$ を 0.2→1.0 と 5 倍動かしても resid 0.018–0.020 で平坦・$T_m$ が補償）。理由＝零点(~0.6Hz)が**最低励振 2 Hz より下**＋零点と極が近接。
> - 低域励振（0.3–1Hz チャープ）すれば**同定は可能**（「不可能」ではない）。ただしヨー角の振れ ∝1/ω・要ヘディングホールド停止・長い滞在＋ドリフトのコスト。
>
> ### 設計の決定（これを使う）
> - **設計モデル＝積分器+遅れ `b·e^{-Ls}/s`（T→0）**。理由：交差周波数 wc≈18 rad/s=2.9 Hz は零点(0.6Hz)・極(2.5Hz)より**上**で、設計に要る FRF はそこで測定済み。零点/極の個別値は**設計に不要**（制御帯域外で実質相殺）。
> - **安全ゲートは yaw のみ T→0 を許容**（`api_task.cpp` 物理境界で `t_lo=(axis==2)?0:0.002`）。roll/pitch は実モータ極~31ms ゆえ T≥2ms を維持。
> - 低域同定は「今の制御には不要だが、物理モデル $\tau_z=I_r/(2k_Q\omega_0)$ を確証したい場合の将来実験」と位置づけ。**制御目的で零点を追ってはならない（同定不能＋不要）。**
>
> **§2–§3 の物理導出は有効（形は正しい）。§4 以降の「零点を同定して設計する」手順は無効**（同定不能）。コードに `fitPlantYaw`・`tau_z`・wc 上限は存在しない。経緯の詳細は auto-memory `project_vehicle_wobble_study` 参照。

## 1. 概要

### このドキュメントについて

本文書は **StampFly のヨー（yaw）軸の制御対象（プラント）モデル**を定義する。ロール/ピッチ軸が単純な「積分器＋1次遅れ＋むだ時間」で表せるのに対し、ヨー軸は**モータの反トルク（反作用トルク）**により**零点**を持つ（当初 RHP＝非最小位相と考えられたが、冒頭注記の通り 2026-06-21 に **LHP＝最小位相・位相リード**と決着）。この零点が未モデルであることが、オンボード自動チューニング（autotune）でヨーの同定が退化（tau が下限に張り付き残差が悪化）した原因であり、本文書はその物理・伝達関数・同定法・PID設計への影響をまとめる。

### 対象読者

- 制御則・autotune を扱う開発者
- ヨー軸の挙動・余裕の薄さの物理的理由を知りたい人

### 関連

- 実装: `firmware/vehicle/components/sf_autotune/`（`fitPlantYaw`, `plantResponse`）, `firmware/vehicle/tasks/api_task.cpp`（`cmdAutotune`）
- 研究: `wobble_minimization_study.md`
- 同定の3原則: `development_roadmap.md`（Model Identity）

## 2. ヨートルクの物理（2つの機構）

X配置クアッドのヨートルクは、ロール/ピッチとは**生成機構が異なる**。

| 軸 | トルク生成 | 遅れの源 |
|----|-----------|---------|
| ロール/ピッチ | **推力差動**（前後・左右モータの推力差）| 推力立ち上がりの遅れ（モータ＋空力） |
| **ヨー** | **反トルク＋抗力トルク**（CW/CCW モータの回転差）| 2機構の混合（下記） |

ヨートルクは2成分の和：

```
            ┌─ 空力抗力トルク  τ_aero ∝ Σ sᵢ k_Q ωᵢ²    （定常・モータ速度で遅れる）
  τ_yaw  = ─┤
            └─ 反トルク        τ_react = −I_r Σ sᵢ ω̇ᵢ   （モータ「加速時」だけ・瞬時）
```

- $s_i = \pm 1$：モータ $i$ の回転方向（CCW/CW）
- $k_Q$：抗力係数、$\omega_i$：モータ角速度、$I_r$：回転子＋プロペラの慣性
- **抗力**：モータが速く回るほど大きい（定常的）。モータ速度に比例 → モータ立ち上がりで遅れる。
- **反トルク**：モータを**加速する瞬間**に機体が受ける蹴り返し（角運動量保存）。**∝ 角加速度 $\dot\omega$**。

ロール/ピッチの推力差動には反トルク成分が無いため、この零点は**ヨー固有**。

## 3. 伝達関数の導出

### 3.1 機体のヨー方程式

$$ I_z\,\dot r = \tau_{yaw} = \tau_{aero} + \tau_{react} $$

- $I_z$：機体ヨー慣性、$r$：ヨーレート

### 3.2 モータの動特性

ヨー指令 $\delta$ は各モータに差動速度 $\delta\omega_i = s_i\,\delta$ を与える。モータ/ESC は1次遅れ（時定数 $\tau_m$＝**全軸共通**、同じモータ）：

$$ \delta\omega(s) = \frac{\delta(s)}{\tau_m s + 1} $$

### 3.3 各トルク成分（ホバー線形化）

$$ \tau_{aero}(s) = \frac{g_a\,\delta}{\tau_m s + 1}, \qquad
   \tau_{react}(s) = \frac{+\,g_r\,s\,\delta}{\tau_m s + 1} $$

【符号訂正 2026-06-21】反トルクは抗力と**同符号**（ヨーを助ける向き）。旧版の負号が RHP 誤りの源。

- $g_a$：抗力ゲイン（$\propto k_Q\,\omega_0$）、$g_r$：反トルクゲイン（$\propto I_r$）

### 3.4 ヨーレート伝達関数

剛体の積分 $1/(I_z s)$ を込めて：

$$ \boxed{\;G_{yaw}(s) = \frac{r(s)}{\delta(s)}
   = K\,\frac{1 + \tau_z\,s}{s\,(\tau_m s + 1)}\;}
   \qquad K = \frac{g_a}{I_z},\;\; \tau_z = \frac{g_r}{g_a} = \frac{I_r}{2\,k_Q\,\omega_0} $$

- **$\tau_z > 0$** で零点は **$s = -1/\tau_z$（左半平面＝最小位相・位相リード）**【2026-06-21 訂正】。
- 実機ではセンサ＋処理＋駆動の**むだ時間 $L$（〜5 ms、全軸共通）**も加わる：

$$ G_{yaw}(s) = \frac{b\,(1 + \tau_z s)\,e^{-Ls}}{s\,(T s + 1)} $$

（$b=K$、$T=\tau_m$ と同一視。同定ではこの4パラメータ形を使う。）

## 4. ロール/ピッチとの比較

| | ロール/ピッチ | ヨー |
|---|---|---|
| モデル | $\dfrac{b\,e^{-Ls}}{s(Ts+1)}$ | $\dfrac{b\,(1+\tau_z s)\,e^{-Ls}}{s(Ts+1)}$ |
| 構造 | 積分＋極＋むだ時間 | 積分＋極＋**LHP零点（リード）**＋むだ時間 |
| ゲイン $b$ | 大（推力差動が強い） | **小（約1/4〜1/5）**（反トルクは弱い） |
| 時定数 $T$ | 〜20〜34 ms | **同程度**（モータ共通） |
| 零点 $\tau_z$ | なし（$=0$） | **あり（LHP・リード）** |

→ **「$T$ はモータ共通、$b$ はヨーで小、＋ヨーだけ反トルクのLHPリード零点」** が要点。

## 5. リード零点（最小位相）の意味【2026-06-21 改稿 — 旧RHP記述を訂正】

### 5.1 初期応答（蹴り返しは無い）

ステップ応答のトルクは**初期から正**で、むしろ定常より大きい：

$$ \tau_{yaw}(0^+) = +\,K\,\tau_z/\tau_m > K = \tau_{yaw}(\infty) $$

＝加速反トルクが立ち上がりを先行して担い（定常比 $\tau_z/\tau_m$ 倍）、モータ回転数の
収束とともに抗力トルクへ引き継ぐ。**逆向きの蹴り返しは起きない**。

### 5.2 帯域への影響（制限しない）

LHPリード零点は位相を**進める**ため帯域を制限しない（交差 2.9 Hz で +20〜30° のリード＝タダの位相余裕）。
ただし零点と極の**個別値**は現行の飛行励振帯域では同定不能（冒頭注記）。設計は積分器＋遅れで保守的に行う。

### 5.3 「ヨーは制御しやすい」と両立

ヨーは**高帯域を要しない**（並進と非干渉・遅くてよい）ため、RHP零点で帯域が制限されても実用上困らない。＝**低めの $\omega_c$ で保守的に組む**のが正解。これが「ヨーは余裕が薄い軸」（実測 PM22°/GM3.8dB 等）の物理的背景でもある。

## 6. システム同定（4パラメータ）【無効 — 冒頭注記参照: fitPlantYaw は不採用・コードに存在しない】

### 6.1 3パラメータ同定が退化する理由

零点なしモデル $b e^{-Ls}/(s(Ts+1))$ をヨーに当てると、零点の位相遅れ・振幅形状を表せず、フィットは

- 零点の位相を **むだ時間 $L$ に押し込む**（$L$ が膨らむ）、または
- **$T \to 0$ に崩す**

→ 残差が悪化（実機 0.15〜0.21、ロール/ピッチの約5倍）。**tau が下限に張り付く**のはこの退化。

### 6.2 4パラメータ同定（採用）

$\{b, T, \tau_z\}$ を自由、**$L$ は共通遅れ 5 ms に固定**して同定する。

| 判断 | 理由 |
|---|---|
| **L を固定** | 位相だけでは $L$ と $\tau_z$ を分離不能（$e^{-Ls}\approx 1-Ls$ で混同）。**振幅**で分離可能（$\lvert 1-\tau_z j\omega\rvert$ は高域で上昇＝むだ時間が真似できない指紋）。$L$ を既知値に固定すれば $(b,T,\tau_z)$ は良条件（条件数 3.5）。 |
| $\tau_z \ge 0$（RHPのみ） | 物理的に反トルクはRHP。下限0で零点なしへ自然退化。 |
| 境界 $\tau_z \in [0, 30]\,\text{ms}$ | 零点 $\ge 5.3$ Hz＝掃引帯域内。広すぎる境界は不良フィットを隠す。 |
| 種 $T\approx25\,\text{ms}$, $\tau_z\in[0,15]\,\text{ms}$ | v0 は $\tau_z=0$（既知良の3パラ最適へ退化）。 |

実装：`fitPlantYaw()`（`fitPlant` のNelder-Meadループを複製＝ロール/ピッチは一字一句不変）。

## 7. PID設計への影響【無効 — 冒頭注記参照: 採用された設計は積分器+遅れ（yaw のみ T→0 許容）】

| 項目 | 扱い |
|---|---|
| `tunePid` / `evalMargins` | **アルゴリズム不変**。零点は `plantResponse` の位相として自動的に効く。RHP零点を打ち消す経路は存在しない（不正相殺は起きない）。 |
| ヨー $\omega_c$ 上限 | **$\omega_{c,cap} = k_z/\tau_z$（$k_z=0.3$）**。上限で PM=60°/GM≈7 dB。$0.5/\tau_z$ で PM が負になり既存ゲートが自動棄却。 |
| ヨー既定 $\omega_c$ | **25 → 18 rad/s**（上限が滅多に効かず予測的）。 |
| ヨー GM下限 | **8 → 6 dB**（全軸6dBに統一）。8dBは未モデル零点を補う保守値だった。零点をモデル化し $\omega_c$ 上限で帯域限界を避けた今、GMは信頼でき、8dBは健全な設計（上限で~7-8dB）を弾くだけ。 |

## 8. パラメータと実装

### 8.1 同定結果パラメータ（読み出し専用）

| param | 意味 | 単位 |
|---|---|---|
| `autotune.<軸>.b` | ゲイン $b$ | — |
| `autotune.<軸>.tau` | モータ極 $T$ | s |
| `autotune.<軸>.tauz` | **反トルク零点 $\tau_z$**（RHP） | s |
| `autotune.<軸>.delay` | むだ時間 $L$（ヨーは5 ms固定） | s |
| `autotune.<軸>.resid` | フィット残差（小さいほど良） | — |
| `autotune.<軸>.wc/pm/gm` | 達成余裕（実効ゲインに対して） | rad/s, deg, dB |

ロール/ピッチは $\tau_z=0$（反トルク零点なし）。

### 8.2 主なコード

| 場所 | 役割 |
|---|---|
| `autotune.hpp` `Plant{... tau_z ...}` | プラント構造体（$\tau_z$ 既定0＝3パラと同一）|
| `autotune.cpp` `plantResponse(...,tau_z=0)` | $\tau_z=0$ でビット同一（ロール/ピッチ不変）|
| `autotune.cpp` `fitPlantYaw()` | 4パラ同定（$L$ 固定）|
| `api_task.cpp` `cmdAutotune` | yaw 分岐・$\omega_c$ 上限・$\tau_z$ 保存 |

## 9. 検証と実機判定基準

### 9.1 ホスト単体テスト

`test_main.cpp::autotune_fit_yaw_rhp_zero`：RHP零点を持つ合成プラント（$\tau_z=14$ ms）で

- 3パラ同定 → **退化**（残差 > 0.08、実機の症状を再現）
- 4パラ `fitPlantYaw` → **復元**（$\tau_z$ 誤差 < 2 ms、$L$ 固定、残差 < 0.05）
- PID設計 → $\omega_c=15$ で実現可能

ロール3パラ回帰テストは不変（byte一致）。

> **注意:** SIL のヨープラントは最小位相（零点なし）ゆえ**この退化を再現できない**。SILは回帰確認のみ、**実機が最終判定**。

### 9.2 実機合格基準

ホバーで `autotune yaw`（既定 $\omega_c=18$）→ 着陸 → `param save`：

| param | 期待 | 旧（退化時） |
|---|---|---|
| `autotune.yaw.resid` | 0.03〜0.06 | 0.15〜0.21 |
| `autotune.yaw.tauz` | (0, 30) ms 内・**境界に張り付かない** | （無）|
| `autotune.yaw.tau`（$T$）| 20〜34 ms・**ロール/ピッチと一致** | 退化 |
| `autotune.yaw.gm` | ≥ 6 dB（適用条件・期待 ~7-8 dB） | 0.92 |

適用に成功すれば**緑LED＋ゲイン変化**（着陸後 `param save`）、棄却なら赤。零点モデル化＋GM下限6dB＋$\omega_c$上限で、**健全な設計は適用されるようになった**（従来は8dB下限が~7-8dBの設計を弾いていた）。

**最重要**：ヨーの $T$ がロール/ピッチの $T$ と数 ms 以内で一致すれば「$L=5$ ms固定が正しい・$\tau_z$ が本物の零点」の確証。大きくズレたら $L_0$ が誤り。$\tau_z$ が境界に張り付いていたら励振不足/モデル不足のサイン。

---

<a id="english"></a>

> ## ⚠️ FINAL RESOLUTION (2026-06-21) — CLOSED. Do not reopen.
>
> Flight-log per-tone analysis settled this. The title's original "RHP zero" was **wrong** (sign error): drag and
> reaction torque act with the **same sign**, so the true model is
> $G_\text{yaw}=b\,(1+\tau_z s)\,e^{-Ls}/(s(T_m s+1))$ — a **left-half-plane (minimum-phase) zero = phase lead**.
> There is no initial reverse kick and no bandwidth limit; the lead (+20–30° at the 2.9 Hz crossover, matching the
> measured +22–32°) is free phase margin. Individual zero/pole values are **not identifiable** from the flight
> excitation (residual profile flat); the adopted design model is **integrator + delay** (T→0 allowed for yaw only).
> `fitPlantYaw`/`tau_z`/wc-cap described in §6–§9 below were **not adopted** and do not exist in the code — those
> sections are retained as history. **Param update (2026-07-15):** re-measured $J_{mp}$=1.375e-8, $C_q$=4.10e-11,
> $\omega_0$=3670 rad/s → $\tau_z$=45.7 ms (3.5 Hz); $T_m$=9.5–17.5 ms (two coexisting electrical parameter sets,
> to be resolved). The Japanese body above has been corrected in place; the English body below is pre-resolution.

## 1. Overview

### About This Document

This document defines the **plant model of the StampFly yaw axis**. Where roll/pitch are a simple "integrator + first-order lag + dead time", the yaw axis carries a **right-half-plane (RHP) zero — non-minimum phase** caused by the motor **reaction torque**. That structure is why the on-board autotune's yaw identification degenerated (tau pinned to its floor, residual ~5× worse). This doc covers the physics, transfer function, identification, and PID-design implications.

### Related

- Code: `sf_autotune/` (`fitPlantYaw`, `plantResponse`), `api_task.cpp` (`cmdAutotune`)
- Study: `wobble_minimization_study.md`

## 2. Yaw-Torque Physics (two mechanisms)

Roll/pitch torque = **thrust differential** (no reaction term). Yaw torque = **aero drag + reaction torque**:

```
            ┌─ aero drag   τ_aero ∝ Σ sᵢ k_Q ωᵢ²    (steady; lags via motor speed)
  τ_yaw  = ─┤
            └─ reaction    τ_react = −I_r Σ sᵢ ω̇ᵢ   (only while motors ACCELERATE; instant)
```

The reaction term (∝ motor angular acceleration) is unique to yaw → it adds a zero.

## 3. Transfer-Function Derivation

Airframe: $I_z\dot r = \tau_{yaw}$. Motor (1st order, $\tau_m$ common to all axes): $\delta\omega = \delta/(\tau_m s+1)$. Aero $\tau_{aero}=g_a\delta\omega$, reaction $\tau_{react}=-g_r s\,\delta\omega$. Hence

$$ G_{yaw}(s) = K\,\frac{1-\tau_z s}{s(\tau_m s+1)},\quad \tau_z=\frac{g_r}{g_a}=\frac{I_r}{2k_Q\omega_0} $$

a **RHP zero at $+1/\tau_z$**. With transport delay $L$ (~5 ms, common):
$G_{yaw}(s)=b(1-\tau_z s)e^{-Ls}/(s(Ts+1))$.

## 4. Roll/Pitch vs Yaw

| | roll/pitch | yaw |
|---|---|---|
| model | $b\,e^{-Ls}/(s(Ts+1))$ | $b(1-\tau_z s)e^{-Ls}/(s(Ts+1))$ |
| gain $b$ | large | small (~1/4–1/5) |
| pole $T$ | 20–34 ms | same (common motors) |
| zero $\tau_z$ | none (0) | RHP zero |

## 5. Why the RHP Zero Matters

- **Initial reverse**: $\tau_{yaw}(0^+)<0$ — the craft yaws briefly the wrong way (reaction kick).
- **Bandwidth limit**: a RHP zero cannot be cancelled and limits the achievable crossover ($\omega_c \lesssim 1/\tau_z$).
- **Compatible with "yaw is easy"**: yaw needs no high bandwidth (decoupled from translation), so a conservative low-$\omega_c$ design is fine — which also explains yaw's historically thin margins.

## 6. 4-Parameter Identification

The 3-param (no-zero) fit is degenerate on yaw (crams the zero into $L$ or collapses $T$; residual 0.15–0.21). Fit $\{b,T,\tau_z\}$ with **$L$ fixed at the common 5 ms**: phase alone cannot separate $L$ from $\tau_z$ (confounded), but the **magnitude rise of $|1-\tau_z j\omega|$** at high frequency does — making the fit well-conditioned (Jacobian condition ~3.5). Bounds $\tau_z\in[0,30]$ ms (RHP-only). Implemented as `fitPlantYaw()` (Nelder-Mead loop duplicated so roll/pitch's 3-param path is byte-identical).

## 7. PID-Design Impact

`tunePid`/`evalMargins` are **algorithmically unchanged** — the zero is just extra phase in `plantResponse`, and a RHP zero cannot be cancelled. A **yaw $\omega_c$ cap $=0.3/\tau_z$** keeps the design clear of the non-minimum-phase bandwidth limit (GM ~7 dB at the cap); over-bandwidth requests self-reject via the existing PM gate. Yaw default $\omega_c$ lowered 25→18.

## 8. Parameters & Code

Result params per axis: `autotune.<axis>.{b, tau(=T), tauz(=τ_z, RHP zero), delay(=L), resid, wc, pm, gm}` (roll/pitch tau_z=0). Code: `Plant.tau_z` (default 0), `plantResponse(...,tau_z=0)` (bit-identical at 0), `fitPlantYaw()`, `cmdAutotune` (yaw branch + wc cap + tau_z persist).

## 9. Validation & Acceptance

Host test `autotune_fit_yaw_rhp_zero`: a synthetic RHP-zero plant ($\tau_z=14$ ms) → the 3-param fit degenerates (residual >0.08), `fitPlantYaw` recovers $b/T/\tau_z$ (τ_z within 2 ms, $L$ fixed, residual <0.05), PID feasible at $\omega_c=15$. The roll 3-param regression test is unchanged. **SIL's yaw plant is minimum-phase and cannot reproduce the degeneracy — hardware is the ground truth.**

Hardware acceptance (`autotune yaw`, default $\omega_c=18$, then `param save`):

| param | expect | was |
|---|---|---|
| `autotune.yaw.resid` | 0.03–0.06 | 0.15–0.21 |
| `autotune.yaw.tauz` | inside (0, 30) ms, NOT pinned | — |
| `autotune.yaw.tau` ($T$) | 20–34 ms, matching roll/pitch | degenerate |
| `autotune.yaw.gm` | ≥ 6 dB (apply floor; expect ~7-8 dB) | 0.92 |

The yaw GM floor was lowered 8→6 dB: the old 8 dB compensated for the unmodelled zero (unreliable margin); with the zero now modelled + the wc cap, a sound yaw design (~7-8 dB GM) applies instead of being blocked.

**Key check:** yaw $T$ matching roll/pitch $T$ (within a few ms) confirms $L_0=5$ ms is the right common delay and $\tau_z$ is a real zero; a large mismatch means $L_0$ is wrong; $\tau_z$ pinned at a bound signals weak excitation / model gaps.
