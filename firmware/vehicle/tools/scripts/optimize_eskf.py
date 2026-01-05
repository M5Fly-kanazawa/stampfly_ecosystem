#!/usr/bin/env python3
"""
optimize_eskf.py - ESKF Q/R Parameter Optimization Tool

Unified optimization tool supporting multiple methods and datasets.

Usage:
    python optimize_eskf.py data.bin [options]
    python optimize_eskf.py data1.bin data2.bin [options]

Methods:
    --method sa         Simulated Annealing (default, recommended)
    --method gd         Gradient Descent
    --method grid       Grid Search (for single parameter)

Options:
    --iter N            Max iterations (default: 500 for SA, 80 for GD)
    --roll-weight W     Weight for roll error in cost (default: 0.3)
    --output FILE       Output optimized parameters to file
    --apply             Apply to eskf.hpp directly
    --quiet             Less output

Examples:
    python optimize_eskf.py flow01.bin
    python optimize_eskf.py flow01.bin flow_sa.bin --method sa --iter 1000
    python optimize_eskf.py flow01.bin --method gd --iter 100
    python optimize_eskf.py flow01.bin --apply
"""

import argparse
import subprocess
import json
import numpy as np
import os
import sys
from datetime import datetime

# Try to import scipy for SA
try:
    from scipy.optimize import dual_annealing
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("Warning: scipy not available, SA method disabled")

ESKF_REPLAY = "/Users/kouhei/tmp/stampfly_rtos_skelton/tools/eskf_debug/build/eskf_replay"
ESKF_HPP = "/Users/kouhei/tmp/stampfly_rtos_skelton/components/stampfly_eskf/include/eskf.hpp"

# Parameter definitions
PARAMS = {
    'gyro_noise':       {'min': 0.0001, 'max': 0.01,  'default': 0.001},
    'accel_noise':      {'min': 0.01,   'max': 0.5,   'default': 0.1},
    'gyro_bias_noise':  {'min': 1e-5,   'max': 0.1,   'default': 0.00005},
    'accel_bias_noise': {'min': 1e-5,   'max': 0.1,   'default': 0.001},
    'flow_noise':       {'min': 0.005,  'max': 1.0,   'default': 0.1},
    'tof_noise':        {'min': 0.001,  'max': 0.1,   'default': 0.002},
    'accel_att_noise':  {'min': 0.1,    'max': 5.0,   'default': 1.0},
}

PARAM_NAMES = list(PARAMS.keys())


class ESKFOptimizer:
    def __init__(self, data_files, roll_weight=0.3, quiet=False):
        self.data_files = data_files
        self.roll_weight = roll_weight
        self.quiet = quiet
        self.eval_count = 0
        self.best_cost = float('inf')
        self.best_params = None
        self.best_results = None

    def run_eskf(self, params_dict):
        """Run ESKF replay with given parameters"""
        cmd = [ESKF_REPLAY, self.data_files[0], "/tmp/opt_test.csv", "--quiet"]
        for name, value in params_dict.items():
            cmd.append(f"--{name}={value}")

        result = subprocess.run(cmd, capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            if 'final_dist' in line:
                return json.loads(line)
        return None

    def cost_function_dict(self, params_dict):
        """Compute cost for all datasets"""
        self.eval_count += 1
        total_cost = 0
        results = []

        for data_file in self.data_files:
            cmd = [ESKF_REPLAY, data_file, "/tmp/opt_test.csv", "--quiet"]
            for name, value in params_dict.items():
                cmd.append(f"--{name}={value}")

            proc = subprocess.run(cmd, capture_output=True, text=True)
            result = None
            for line in proc.stdout.split('\n'):
                if 'final_dist' in line:
                    result = json.loads(line)
                    break

            if result is None:
                return float('inf'), []

            dist = result['final_dist'] * 100  # cm
            roll = abs(result['roll'])
            cost = dist + roll * self.roll_weight
            total_cost += cost
            results.append({
                'file': os.path.basename(data_file),
                'cost': cost,
                'roll': roll,
                'dist': dist
            })

        # Update best
        if total_cost < self.best_cost:
            self.best_cost = total_cost
            self.best_params = params_dict.copy()
            self.best_results = results

        # Progress output
        if not self.quiet and self.eval_count % 50 == 0:
            print(f"  [{self.eval_count:4d}] cost={total_cost:.2f}")
            for r in results:
                print(f"         {r['file']}: Roll={r['roll']:.1f}deg, dist={r['dist']:.1f}cm")

        return total_cost, results

    def cost_function_array(self, x):
        """Cost function for scipy optimizers (log-scale input)"""
        params_dict = {}
        for i, name in enumerate(PARAM_NAMES):
            val = np.exp(x[i])
            val = np.clip(val, PARAMS[name]['min'], PARAMS[name]['max'])
            params_dict[name] = val
        cost, _ = self.cost_function_dict(params_dict)
        return cost

    def optimize_sa(self, max_iter=500):
        """Simulated Annealing optimization"""
        if not HAS_SCIPY:
            print("Error: scipy required for SA method")
            return None

        print(f"\n=== Simulated Annealing Optimization ===")
        print(f"Datasets: {[os.path.basename(f) for f in self.data_files]}")
        print(f"Max iterations: {max_iter}")
        print("-" * 50)

        bounds = [(np.log(PARAMS[name]['min']), np.log(PARAMS[name]['max']))
                  for name in PARAM_NAMES]

        result = dual_annealing(
            self.cost_function_array,
            bounds,
            maxiter=max_iter,
            seed=42,
            no_local_search=False,
        )

        # Convert result
        best_params = {}
        for i, name in enumerate(PARAM_NAMES):
            best_params[name] = np.clip(np.exp(result.x[i]),
                                        PARAMS[name]['min'],
                                        PARAMS[name]['max'])

        return best_params

    def optimize_gd(self, max_iter=80, learning_rate=0.08):
        """Gradient Descent optimization"""
        print(f"\n=== Gradient Descent Optimization ===")
        print(f"Datasets: {[os.path.basename(f) for f in self.data_files]}")
        print(f"Max iterations: {max_iter}")
        print("-" * 50)

        # Initialize with default values (log scale)
        log_params = {name: np.log(info['default']) for name, info in PARAMS.items()}

        for iteration in range(max_iter):
            # Compute gradient using central difference
            gradient = {}
            epsilon = 0.05

            for name in log_params:
                log_plus = log_params.copy()
                log_minus = log_params.copy()
                log_plus[name] += epsilon
                log_minus[name] -= epsilon

                params_plus = {n: np.clip(np.exp(v), PARAMS[n]['min'], PARAMS[n]['max'])
                              for n, v in log_plus.items()}
                params_minus = {n: np.clip(np.exp(v), PARAMS[n]['min'], PARAMS[n]['max'])
                               for n, v in log_minus.items()}

                cost_plus, _ = self.cost_function_dict(params_plus)
                cost_minus, _ = self.cost_function_dict(params_minus)
                gradient[name] = (cost_plus - cost_minus) / (2 * epsilon)

            # Current cost
            params = {n: np.clip(np.exp(v), PARAMS[n]['min'], PARAMS[n]['max'])
                     for n, v in log_params.items()}
            cost, results = self.cost_function_dict(params)

            if not self.quiet and iteration % 10 == 0:
                print(f"  Iter {iteration}: cost={cost:.2f}")

            # Line search
            alpha = learning_rate
            for _ in range(5):
                new_log_params = {}
                for name in log_params:
                    new_log_params[name] = log_params[name] - alpha * gradient[name]
                    log_min = np.log(PARAMS[name]['min'])
                    log_max = np.log(PARAMS[name]['max'])
                    new_log_params[name] = np.clip(new_log_params[name], log_min, log_max)

                new_params = {n: np.clip(np.exp(v), PARAMS[n]['min'], PARAMS[n]['max'])
                             for n, v in new_log_params.items()}
                new_cost, _ = self.cost_function_dict(new_params)

                if new_cost < cost:
                    log_params = new_log_params
                    break
                alpha *= 0.5
            else:
                # Small step if line search fails
                for name in log_params:
                    log_params[name] -= 0.01 * gradient[name]
                    log_min = np.log(PARAMS[name]['min'])
                    log_max = np.log(PARAMS[name]['max'])
                    log_params[name] = np.clip(log_params[name], log_min, log_max)

        return self.best_params

    def print_results(self, params):
        """Print optimization results"""
        print("\n" + "=" * 60)
        print("OPTIMIZATION COMPLETE")
        print("=" * 60)
        print(f"Total evaluations: {self.eval_count}")
        print(f"Best total cost: {self.best_cost:.3f}")

        print("\n" + "-" * 60)
        print("Results per dataset:")
        print("-" * 60)
        for r in self.best_results:
            print(f"  {r['file']}: Roll={r['roll']:.2f}deg, dist={r['dist']:.2f}cm")

        print("\n" + "-" * 60)
        print("Optimized Parameters:")
        print("-" * 60)
        for name in PARAM_NAMES:
            print(f"  {name:<20} = {params[name]:.6f}")

        print("\n" + "-" * 60)
        print("C++ format (for eskf.hpp):")
        print("-" * 60)
        for name in PARAM_NAMES:
            print(f"cfg.{name} = {params[name]:.6f}f;")

    def apply_to_hpp(self, params):
        """Apply parameters to eskf.hpp"""
        with open(ESKF_HPP, 'r') as f:
            content = f.read()

        # Find and replace defaultConfig section
        date_str = datetime.now().strftime("%Y-%m-%d")

        # Build results comment
        results_comment = []
        for r in self.best_results:
            results_comment.append(f"        // {r['file']}: Roll={r['roll']:.2f}deg, dist={r['dist']:.2f}cm")
        results_str = '\n'.join(results_comment)

        new_config = f'''        // デフォルト設定
        // 最適化実行日: {date_str}
{results_str}
        static Config defaultConfig() {{
            Config cfg;
            // プロセスノイズ (Q)
            cfg.gyro_noise = {params['gyro_noise']:.6f}f;
            cfg.accel_noise = {params['accel_noise']:.6f}f;
            cfg.gyro_bias_noise = {params['gyro_bias_noise']:.6f}f;
            cfg.accel_bias_noise = {params['accel_bias_noise']:.6f}f;

            // 観測ノイズ (R)
            cfg.baro_noise = 0.1f;
            cfg.tof_noise = {params['tof_noise']:.6f}f;
            cfg.mag_noise = 0.1f;
            cfg.flow_noise = {params['flow_noise']:.6f}f;
            cfg.accel_att_noise = {params['accel_att_noise']:.6f}f;'''

        # Find the section to replace
        import re
        pattern = r'        // デフォルト設定.*?cfg\.accel_att_noise = [^;]+;'
        match = re.search(pattern, content, re.DOTALL)

        if match:
            content = content[:match.start()] + new_config + content[match.end():]
            with open(ESKF_HPP, 'w') as f:
                f.write(content)
            print(f"\nApplied to {ESKF_HPP}")
            return True
        else:
            print("Warning: Could not find defaultConfig section in eskf.hpp")
            return False


def main():
    parser = argparse.ArgumentParser(description='ESKF Q/R Parameter Optimization Tool')
    parser.add_argument('inputs', nargs='+', help='Input data files (.bin)')
    parser.add_argument('--method', choices=['sa', 'gd'], default='sa',
                        help='Optimization method: sa (Simulated Annealing), gd (Gradient Descent)')
    parser.add_argument('--iter', type=int, default=None, help='Max iterations')
    parser.add_argument('--roll-weight', type=float, default=0.3, help='Roll error weight')
    parser.add_argument('--output', type=str, help='Output file for parameters')
    parser.add_argument('--apply', action='store_true', help='Apply to eskf.hpp')
    parser.add_argument('--quiet', action='store_true', help='Less output')

    args = parser.parse_args()

    # Validate inputs
    for f in args.inputs:
        if not os.path.exists(f):
            print(f"Error: File not found: {f}")
            sys.exit(1)

    # Default iterations
    if args.iter is None:
        args.iter = 500 if args.method == 'sa' else 80

    # Create optimizer
    optimizer = ESKFOptimizer(args.inputs, args.roll_weight, args.quiet)

    # Run optimization
    if args.method == 'sa':
        best_params = optimizer.optimize_sa(args.iter)
    else:
        best_params = optimizer.optimize_gd(args.iter)

    if best_params is None:
        print("Optimization failed")
        sys.exit(1)

    # Print results
    optimizer.print_results(best_params)

    # Save to file
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(best_params, f, indent=2)
        print(f"\nSaved to {args.output}")

    # Apply to eskf.hpp
    if args.apply:
        optimizer.apply_to_hpp(best_params)


if __name__ == '__main__':
    main()
