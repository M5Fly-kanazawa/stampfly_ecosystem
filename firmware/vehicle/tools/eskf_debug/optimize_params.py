#!/usr/bin/env python3
"""
ESKF Parameter Optimization Tool

Searches for optimal Q/R parameters that minimize position error
for a known motion pattern (e.g., 20cm square).

Usage:
  python3 optimize_params.py <input.bin> [--target_range=0.20]
"""

import subprocess
import json
import numpy as np
import os
import sys
from itertools import product
from concurrent.futures import ProcessPoolExecutor, as_completed

# Path to eskf_replay
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ESKF_REPLAY = os.path.join(SCRIPT_DIR, 'build', 'eskf_replay')

def run_eskf(input_file, params):
    """Run eskf_replay with given parameters and return metrics"""
    args = [
        ESKF_REPLAY,
        input_file,
        '/tmp/opt_temp.csv',
        '--quiet',
    ]

    for key, value in params.items():
        args.append(f'--{key}={value}')

    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=30)
        # Find JSON line in output
        for line in result.stdout.split('\n'):
            if line.startswith('{'):
                return json.loads(line)
    except Exception as e:
        pass
    return None

def compute_cost(metrics, target_range=0.20):
    """
    Compute cost function for optimization.

    Goals:
    1. Position range should match target (20cm square) - penalize undershooting more
    2. Final position should return to origin
    3. Attitude should remain stable (low roll/pitch)
    """
    if metrics is None:
        return float('inf')

    # Position range error (should be close to target_range)
    # Penalize undershooting (< target) more than overshooting
    x_range_err = metrics['pos_x_range'] - target_range
    y_range_err = metrics['pos_y_range'] - target_range

    # Asymmetric penalty: undershooting is 3x worse than overshooting
    if x_range_err < 0:
        x_range_cost = abs(x_range_err) * 3.0
    else:
        x_range_cost = x_range_err

    if y_range_err < 0:
        y_range_cost = abs(y_range_err) * 3.0
    else:
        y_range_cost = y_range_err

    range_cost = (x_range_cost + y_range_cost) / target_range

    # Return-to-origin error (final distance from origin)
    return_cost = metrics['final_dist'] / target_range

    # Attitude stability (roll and pitch should be near zero)
    att_cost = (abs(metrics['roll']) + abs(metrics['pitch'])) / 10.0

    # Weighted sum - range accuracy is most important
    total_cost = range_cost * 4.0 + return_cost * 2.0 + att_cost * 0.5

    return total_cost

def evaluate_params(args):
    """Worker function for parallel evaluation"""
    input_file, params, target_range = args
    metrics = run_eskf(input_file, params)
    cost = compute_cost(metrics, target_range)
    return params, metrics, cost

def grid_search(input_file, target_range=0.20, n_jobs=4, extended=False):
    """
    Perform grid search over parameter space.
    """
    # Parameter ranges (logarithmic scale for noise parameters)
    # flow_rad_per_pixel: 基準値 0.00205 (= 0.71674/35*0.1)
    # 範囲: 0.8x ~ 1.2x でキャリブレーション誤差を探索
    base_flow_cal = 0.00205

    if extended:
        param_grid = {
            'flow_noise': [0.05, 0.1, 0.2],
            'accel_noise': [0.05, 0.1, 0.2],
            'gyro_noise': [0.001, 0.002, 0.004],
            'tof_noise': [0.002, 0.005, 0.01],
            'baro_noise': [0.05, 0.1, 0.2],
            'mag_noise': [0.2, 0.3, 0.5],
            'accel_att_noise': [0.5, 1.0, 2.0],
            'flow_rad_per_pixel': [base_flow_cal * 0.9, base_flow_cal, base_flow_cal * 1.1, base_flow_cal * 1.2],
        }
    else:
        param_grid = {
            'flow_noise': [0.01, 0.05, 0.1, 0.2, 0.5],
            'accel_noise': [0.05, 0.1, 0.2, 0.5],
            'gyro_noise': [0.0005, 0.001, 0.002, 0.005],
            'tof_noise': [0.001, 0.002, 0.005, 0.01],
            'flow_rad_per_pixel': [base_flow_cal * 0.9, base_flow_cal, base_flow_cal * 1.1],
        }

    # Generate all combinations
    keys = list(param_grid.keys())
    values = list(param_grid.values())
    combinations = list(product(*values))

    print(f"Grid search: {len(combinations)} combinations")
    print(f"Target range: {target_range*100:.0f}cm")
    print()

    # Prepare tasks
    tasks = []
    for combo in combinations:
        params = dict(zip(keys, combo))
        tasks.append((input_file, params, target_range))

    # Run in parallel
    results = []
    with ProcessPoolExecutor(max_workers=n_jobs) as executor:
        futures = {executor.submit(evaluate_params, task): task for task in tasks}

        completed = 0
        for future in as_completed(futures):
            completed += 1
            params, metrics, cost = future.result()
            results.append((params, metrics, cost))

            if completed % 20 == 0:
                print(f"  Progress: {completed}/{len(combinations)}")

    # Sort by cost
    results.sort(key=lambda x: x[2])

    return results

def fine_tune(input_file, best_params, target_range=0.20, n_jobs=4):
    """
    Fine-tune around the best parameters found.
    """
    # Create fine grid around best parameters
    fine_grid = {}
    for key, value in best_params.items():
        # +/- 50% in 5 steps
        fine_grid[key] = [value * m for m in [0.5, 0.7, 1.0, 1.4, 2.0]]

    keys = list(fine_grid.keys())
    values = list(fine_grid.values())
    combinations = list(product(*values))

    print(f"\nFine-tuning: {len(combinations)} combinations")

    tasks = []
    for combo in combinations:
        params = dict(zip(keys, combo))
        tasks.append((input_file, params, target_range))

    results = []
    with ProcessPoolExecutor(max_workers=n_jobs) as executor:
        futures = {executor.submit(evaluate_params, task): task for task in tasks}

        for future in as_completed(futures):
            params, metrics, cost = future.result()
            results.append((params, metrics, cost))

    results.sort(key=lambda x: x[2])

    return results

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 optimize_params.py <input.bin> [--target_range=0.20] [--extended]")
        sys.exit(1)

    input_file = sys.argv[1]
    target_range = 0.20  # 20cm default
    extended = False

    for arg in sys.argv[2:]:
        if arg.startswith('--target_range='):
            target_range = float(arg.split('=')[1])
        elif arg == '--extended':
            extended = True

    if not os.path.exists(ESKF_REPLAY):
        print(f"Error: eskf_replay not found at {ESKF_REPLAY}")
        print("Please build it first: cmake --build build")
        sys.exit(1)

    print("=== ESKF Parameter Optimization ===")
    print(f"Input: {input_file}")
    print(f"Target motion range: {target_range*100:.0f}cm")
    print()

    # First: run with default parameters
    print("Running with default parameters...")
    default_params = {
        'flow_noise': 0.05,
        'accel_noise': 0.05,
        'gyro_noise': 0.001,
        'tof_noise': 0.007,
        'flow_rad_per_pixel': 0.00205,
    }
    default_metrics = run_eskf(input_file, default_params)
    default_cost = compute_cost(default_metrics, target_range)

    print(f"  Default results:")
    print(f"    Position range: X={default_metrics['pos_x_range']*100:.1f}cm, Y={default_metrics['pos_y_range']*100:.1f}cm")
    print(f"    Final distance: {default_metrics['final_dist']*100:.1f}cm")
    print(f"    Attitude: roll={default_metrics['roll']:.1f}°, pitch={default_metrics['pitch']:.1f}°")
    print(f"    Cost: {default_cost:.4f}")
    print()

    # Grid search
    print(f"Starting grid search (extended={extended})...")
    results = grid_search(input_file, target_range, n_jobs=8, extended=extended)

    # Show top 5 results
    print("\n=== Top 5 Results ===")
    for i, (params, metrics, cost) in enumerate(results[:5]):
        print(f"\n#{i+1} Cost: {cost:.4f}")
        print(f"  Parameters:")
        for k, v in params.items():
            print(f"    {k}: {v}")
        if metrics:
            print(f"  Results:")
            print(f"    Position range: X={metrics['pos_x_range']*100:.1f}cm, Y={metrics['pos_y_range']*100:.1f}cm")
            print(f"    Final distance: {metrics['final_dist']*100:.1f}cm")
            print(f"    Attitude: roll={metrics['roll']:.1f}°, pitch={metrics['pitch']:.1f}°")

    # Fine-tune top result
    best_params = results[0][0]
    fine_results = fine_tune(input_file, best_params, target_range, n_jobs=8)

    # Show best result after fine-tuning
    print("\n=== Best Result After Fine-Tuning ===")
    params, metrics, cost = fine_results[0]
    print(f"Cost: {cost:.4f} (default: {default_cost:.4f})")
    print(f"\nOptimal Parameters:")
    for k, v in params.items():
        print(f"  {k}: {v:.6f}")

    if metrics:
        x_err = (metrics['pos_x_range'] - target_range) * 100
        y_err = (metrics['pos_y_range'] - target_range) * 100
        print(f"\nResults:")
        print(f"  Position range: X={metrics['pos_x_range']*100:.1f}cm (err: {x_err:+.1f}cm), Y={metrics['pos_y_range']*100:.1f}cm (err: {y_err:+.1f}cm)")
        print(f"  Final distance: {metrics['final_dist']*100:.1f}cm")
        print(f"  Attitude: roll={metrics['roll']:.1f}°, pitch={metrics['pitch']:.1f}°")
        print(f"  Yaw: {metrics['yaw']:.1f}°")

    # Generate configuration snippet
    print("\n=== Recommended Configuration ===")
    print("// In eskf.hpp defaultConfig():")
    if 'gyro_noise' in params:
        print(f"cfg.gyro_noise = {params['gyro_noise']:.6f}f;")
    if 'accel_noise' in params:
        print(f"cfg.accel_noise = {params['accel_noise']:.6f}f;")
    if 'flow_noise' in params:
        print(f"cfg.flow_noise = {params['flow_noise']:.6f}f;")
    if 'tof_noise' in params:
        print(f"cfg.tof_noise = {params['tof_noise']:.6f}f;")
    if 'baro_noise' in params:
        print(f"cfg.baro_noise = {params['baro_noise']:.6f}f;")
    if 'mag_noise' in params:
        print(f"cfg.mag_noise = {params['mag_noise']:.6f}f;")
    if 'accel_att_noise' in params:
        print(f"cfg.accel_att_noise = {params['accel_att_noise']:.6f}f;")
    if 'flow_rad_per_pixel' in params:
        print(f"cfg.flow_rad_per_pixel = {params['flow_rad_per_pixel']:.7f}f;  // calibration factor")

    # Improvement summary
    improvement = (default_cost - cost) / default_cost * 100
    print(f"\nImprovement: {improvement:.1f}% reduction in cost")

if __name__ == "__main__":
    main()
