"""
Motor Steady-State Parameter Identification
モータ定常特性パラメータ同定

Model (steady-state torque balance):
モデル（定常トルク釣り合い）:

  (D + K²/R)·ω + Cq·ω² + qf = (K/R)·e

Dividing by K/R:
K/R で割って整理:

  e = A·ω² + B·ω + C

where:
  A = Cq·R/K       ... aerodynamic torque coefficient (normalized)
  B = (D + K²/R) / (K/R) = D·R/K + K
  C = qf·R/K       ... static friction (normalized)

Given measured R and identified A, B, C, recover physical parameters:
測定済み R と同定した A, B, C から物理パラメータを逆算:

  K  = Cq·R / A
  D  = (B - K) · K / R  = (B·K - K²) / R
  qf = C·K / R

Usage:
  python -m tools.sysid.steady_state analysis/datasets/sysid/motor_steady_state.csv
  python -m tools.sysid.steady_state analysis/datasets/sysid/motor_steady_state.csv --Rm 0.63
  python -m tools.sysid.steady_state analysis/datasets/sysid/motor_steady_state.csv --plot
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass
class SteadyStateResult:
    """Steady-state identification result
    定常特性同定結果"""

    # Least-squares coefficients (e = A·ω² + B·ω + C)
    # 最小二乗法で同定した係数
    A: float  # [V/(rad/s)²]
    B: float  # [V/(rad/s)]
    C: float  # [V]

    # Fit quality
    # フィッティング品質
    r_squared: float
    residual_rms: float  # [V]

    # Derived physical parameters (require Rm)
    # 導出物理パラメータ（Rm が必要）
    Km: Optional[float] = None   # Back-EMF constant [V·s/rad]
    Dm: Optional[float] = None   # Viscous damping [N·m·s/rad]
    Qf: Optional[float] = None   # Static friction torque [N·m]
    Rm: Optional[float] = None   # Motor resistance [Ω] (input)
    Cq: Optional[float] = None   # Torque coefficient [N·m/(rad/s)²] (input)


def fit_steady_state(voltage: np.ndarray, omega: np.ndarray) -> SteadyStateResult:
    """Fit e = A·ω² + B·ω + C by least squares
    最小二乗法で e = A·ω² + B·ω + C をフィッティング

    Args:
        voltage: Measured voltage [V]
        omega: Measured angular velocity [rad/s]

    Returns:
        SteadyStateResult with A, B, C and fit quality
    """
    # Build design matrix: [ω², ω, 1]
    # 計画行列を構築
    n = len(omega)
    X = np.column_stack([omega**2, omega, np.ones(n)])

    # Least squares: X @ [A, B, C]^T = voltage
    # 最小二乗法
    coeffs, residuals, rank, sv = np.linalg.lstsq(X, voltage, rcond=None)
    A, B, C = coeffs

    # Compute fit quality
    # フィッティング品質を計算
    voltage_pred = X @ coeffs
    ss_res = np.sum((voltage - voltage_pred) ** 2)
    ss_tot = np.sum((voltage - np.mean(voltage)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    residual_rms = np.sqrt(ss_res / n)

    return SteadyStateResult(
        A=A, B=B, C=C,
        r_squared=r_squared,
        residual_rms=residual_rms,
    )


def derive_physical_params(
    result: SteadyStateResult,
    Rm: float,
    Cq: float = 4.10e-11,
) -> SteadyStateResult:
    """Derive physical parameters from A, B, C and known Rm, Cq
    A, B, C と既知の Rm, Cq から物理パラメータを導出

    Relations:
      A = Cq·R/K  →  K = Cq·R/A
      B = D·R/K + K  →  D = (B - K)·K/R
      C = qf·R/K  →  qf = C·K/R

    Args:
        result: SteadyStateResult with A, B, C
        Rm: Motor resistance [Ω]
        Cq: Torque coefficient [N·m/(rad/s)²]

    Returns:
        Updated SteadyStateResult with Km, Dm, Qf
    """
    A, B, C = result.A, result.B, result.C

    # K = Cq·R / A
    # 逆起電力定数
    Km = Cq * Rm / A

    # D = (B - K)·K / R
    # 粘性摩擦係数
    Dm = (B - Km) * Km / Rm

    # qf = C·K / R
    # 静止摩擦トルク
    Qf = C * Km / Rm

    result.Km = Km
    result.Dm = Dm
    result.Qf = Qf
    result.Rm = Rm
    result.Cq = Cq

    return result


def load_csv(filepath: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Load voltage-omega CSV data
    電圧-角速度 CSV データを読み込み

    Args:
        filepath: Path to CSV file

    Returns:
        (voltage, omega) arrays
    """
    # Skip comment lines (starting with #) and 1 header line
    # コメント行（#始まり）とヘッダー行をスキップ
    import csv as csv_mod
    rows = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # Try to parse as numbers; skip header
            parts = line.split(",")
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                continue  # header line
    data = np.array(rows)
    voltage = data[:, 0]
    omega = data[:, 1]
    return voltage, omega


def print_results(result: SteadyStateResult, reference: dict | None = None):
    """Print identification results
    同定結果を表示"""

    print("=" * 65)
    print("Motor Steady-State Identification Results")
    print("モータ定常特性同定結果")
    print("=" * 65)

    print("\n--- Least-Squares Coefficients (e = A·ω² + B·ω + C) ---")
    print("--- 最小二乗法係数 ---")
    print(f"  A = {result.A:.4e}  [V/(rad/s)²]")
    print(f"  B = {result.B:.4e}  [V/(rad/s)]")
    print(f"  C = {result.C:.4e}  [V]")
    print(f"  R² = {result.r_squared:.6f}")
    print(f"  Residual RMS = {result.residual_rms:.4e} [V]")

    if result.Km is not None:
        print(f"\n--- Derived Physical Parameters (Rm = {result.Rm} Ω, Cq = {result.Cq:.2e}) ---")
        print("--- 導出物理パラメータ ---")
        print(f"  Km (back-EMF constant)   = {result.Km:.4e}  [V·s/rad]")
        print(f"  Dm (viscous damping)     = {result.Dm:.4e}  [N·m·s/rad]")
        print(f"  Qf (static friction)     = {result.Qf:.4e}  [N·m]")

        # Sanity checks
        # 妥当性チェック
        print("\n--- Sanity Checks ---")
        print("--- 妥当性チェック ---")
        if result.Dm < 0:
            print(f"  ⚠ WARNING: Dm = {result.Dm:.4e} is NEGATIVE (physically invalid)")
            print(f"    Dm が負値です（物理的に不正）。Rm の値を見直してください。")
        else:
            print(f"  ✓ Dm > 0 (OK)")

        if result.Qf < 0:
            print(f"  ⚠ WARNING: Qf = {result.Qf:.4e} is NEGATIVE")
        else:
            print(f"  ✓ Qf > 0 (OK)")

        # Verify: reconstruct A, B, C from physical params
        # 検証: 物理パラメータから A, B, C を再構成
        A_check = result.Cq * result.Rm / result.Km
        B_check = result.Dm * result.Rm / result.Km + result.Km
        C_check = result.Qf * result.Rm / result.Km
        print(f"\n--- Reconstruction Check ---")
        print(f"--- 再構成チェック ---")
        print(f"  A: {result.A:.4e} → {A_check:.4e}  (err={abs(result.A - A_check):.2e})")
        print(f"  B: {result.B:.4e} → {B_check:.4e}  (err={abs(result.B - B_check):.2e})")
        print(f"  C: {result.C:.4e} → {C_check:.4e}  (err={abs(result.C - C_check):.2e})")

        if reference:
            print(f"\n--- Comparison with Reference ---")
            print("--- リファレンス値との比較 ---")
            fmt = "  {:20s}  {:>12s}  {:>12s}  {:>8s}"
            print(fmt.format("Parameter", "Identified", "Reference", "Diff %"))
            print("  " + "-" * 56)
            for name, key in [("Km [V·s/rad]", "Km"), ("Dm [N·m·s/rad]", "Dm"), ("Qf [N·m]", "Qf")]:
                val = getattr(result, key)
                ref = reference.get(key)
                if ref is not None and ref != 0:
                    pct = (val - ref) / abs(ref) * 100
                    print(fmt.format(name, f"{val:.4e}", f"{ref:.4e}", f"{pct:+.1f}%"))
                elif ref is not None:
                    print(fmt.format(name, f"{val:.4e}", f"{ref:.4e}", "N/A"))


def plot_results(
    voltage: np.ndarray,
    omega: np.ndarray,
    result: SteadyStateResult,
    save_path: str | Path | None = None,
):
    """Plot measured data and fitted curve
    実測データとフィッティング曲線をプロット"""
    import matplotlib.pyplot as plt

    omega_fit = np.linspace(0, omega.max() * 1.05, 200)
    voltage_fit = result.A * omega_fit**2 + result.B * omega_fit + result.C

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(omega, voltage, "ko", markersize=8, label="Measured / 実測")
    ax.plot(omega_fit, voltage_fit, "r-", linewidth=2,
            label=f"Fit: e = {result.A:.3e}ω² + {result.B:.3e}ω + {result.C:.3e}")
    ax.set_xlabel("Angular velocity ω [rad/s]")
    ax.set_ylabel("Voltage e [V]")
    ax.set_title("Motor Steady-State Characteristic / モータ定常特性")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Annotate R²
    ax.text(0.05, 0.95, f"R² = {result.r_squared:.6f}",
            transform=ax.transAxes, fontsize=11, verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"\nPlot saved to: {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Motor steady-state parameter identification / モータ定常特性パラメータ同定"
    )
    parser.add_argument("csv", help="Path to voltage-omega CSV file")
    parser.add_argument("--Rm", type=float, default=None,
                        help="Motor resistance [Ω] (default: try both 0.34 and 0.63)")
    parser.add_argument("--Cq", type=float, default=4.10e-11,
                        help="Torque coefficient [N·m/(rad/s)²] (default: 4.10e-11, 2026-07-15実測)")
    parser.add_argument("--plot", action="store_true", help="Show plot")
    parser.add_argument("--save-plot", type=str, default=None, help="Save plot to file")
    args = parser.parse_args()

    # Load data
    # データ読み込み
    voltage, omega = load_csv(args.csv)
    print(f"Loaded {len(voltage)} data points from {args.csv}")
    print(f"  Voltage range: {voltage.min():.2f} - {voltage.max():.2f} V")
    print(f"  Omega range:   {omega.min():.0f} - {omega.max():.0f} rad/s")

    # Fit A, B, C
    # A, B, C をフィッティング
    result = fit_steady_state(voltage, omega)

    # Reference values from motor_model.hpp (Rm=0.34)
    # motor_model.hpp のリファレンス値
    ref_034 = {"Km": 6.125e-4, "Dm": 3.69e-8, "Qf": 2.76e-5}

    if args.Rm is not None:
        # Single Rm specified
        derive_physical_params(result, Rm=args.Rm, Cq=args.Cq)
        print_results(result, reference=ref_034)
    else:
        # Try both Rm values for comparison
        # 両方の Rm で比較
        print_results(result)

        for Rm_val in [0.34, 0.63]:
            print(f"\n{'=' * 65}")
            print(f"  Rm = {Rm_val} Ω の場合")
            print(f"{'=' * 65}")
            import copy
            r = copy.deepcopy(result)
            derive_physical_params(r, Rm=Rm_val, Cq=args.Cq)
            print_results(r, reference=ref_034)

    # Plot
    if args.plot or args.save_plot:
        plot_results(voltage, omega, result, save_path=args.save_plot)


if __name__ == "__main__":
    main()
