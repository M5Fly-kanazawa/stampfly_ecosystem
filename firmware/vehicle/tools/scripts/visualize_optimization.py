#!/usr/bin/env python3
"""
Q/R最適化過程の詳細可視化
"""

import subprocess
import json
import numpy as np
import matplotlib.pyplot as plt
import os

ESKF_REPLAY = "/Users/kouhei/tmp/stampfly_rtos_skelton/tools/eskf_debug/build/eskf_replay"
DATA_FILE = "/Users/kouhei/tmp/stampfly_rtos_skelton/tools/scripts/flow01.bin"

# 最適化履歴を再現
PARAMS = {
    'gyro_noise':       {'min': 0.0001, 'max': 0.01,  'default': 0.001},
    'accel_noise':      {'min': 0.01,   'max': 0.5,   'default': 0.1},
    'gyro_bias_noise':  {'min': 1e-5,   'max': 0.1,   'default': 0.00005},
    'accel_bias_noise': {'min': 1e-5,   'max': 0.1,   'default': 0.001},
    'flow_noise':       {'min': 0.005,  'max': 0.5,   'default': 0.1},
    'tof_noise':        {'min': 0.001,  'max': 0.1,   'default': 0.002},
    'accel_att_noise':  {'min': 0.1,    'max': 5.0,   'default': 1.0},
}

def run_eskf(params):
    cmd = [ESKF_REPLAY, DATA_FILE, "/tmp/opt_test.csv", "--quiet"]
    for name, value in params.items():
        cmd.append(f"--{name}={value}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    for line in result.stdout.split('\n'):
        if 'final_dist' in line:
            return json.loads(line)
    return None

def cost_function(log_params):
    params = {}
    for name, log_val in log_params.items():
        val = np.exp(log_val)
        val = np.clip(val, PARAMS[name]['min'], PARAMS[name]['max'])
        params[name] = val
    result = run_eskf(params)
    if result is None:
        return float('inf'), None, params
    dist = result['final_dist'] * 100
    roll = abs(result['roll'])
    cost = dist + roll * 0.3
    return cost, result, params

def central_difference_gradient(log_params, epsilon=0.05):
    gradient = {}
    for name in log_params:
        log_plus = log_params.copy()
        log_minus = log_params.copy()
        log_plus[name] += epsilon
        log_minus[name] -= epsilon
        cost_plus, _, _ = cost_function(log_plus)
        cost_minus, _, _ = cost_function(log_minus)
        gradient[name] = (cost_plus - cost_minus) / (2 * epsilon)
    base_cost, _, _ = cost_function(log_params)
    return gradient, base_cost

def run_optimization(max_iter=80):
    """最適化を実行して履歴を返す"""
    initial_log_params = {name: np.log(info['default']) for name, info in PARAMS.items()}
    log_params = initial_log_params.copy()
    history = []
    learning_rate = 0.08

    print("最適化実行中...")

    for i in range(max_iter):
        gradient, cost = central_difference_gradient(log_params)
        _, result, params = cost_function(log_params)

        if result is None:
            break

        history.append({
            'iter': i,
            'params': params.copy(),
            'cost': cost,
            'roll': result['roll'],
            'dist': result['final_dist'] * 100,
            'gradient': gradient.copy(),
            'pos_x': result['pos_x'] * 100,
            'pos_y': result['pos_y'] * 100,
        })

        if i % 10 == 0:
            print(f"  Iter {i}: cost={cost:.2f}, Roll={result['roll']:.2f}°, dist={result['final_dist']*100:.2f}cm")

        # 線形探索
        alpha = learning_rate
        for _ in range(5):
            new_log_params = {}
            for name in log_params:
                new_log_params[name] = log_params[name] - alpha * gradient[name]
                log_min = np.log(PARAMS[name]['min'])
                log_max = np.log(PARAMS[name]['max'])
                new_log_params[name] = np.clip(new_log_params[name], log_min, log_max)
            new_cost, _, _ = cost_function(new_log_params)
            if new_cost < cost:
                log_params = new_log_params
                break
            alpha *= 0.5
        else:
            for name in log_params:
                log_params[name] -= 0.01 * gradient[name]
                log_min = np.log(PARAMS[name]['min'])
                log_max = np.log(PARAMS[name]['max'])
                log_params[name] = np.clip(log_params[name], log_min, log_max)

    return history

def main():
    # 最適化実行
    history = run_optimization(max_iter=80)

    if len(history) < 2:
        print("履歴が不足")
        return

    # データ抽出
    iters = [h['iter'] for h in history]
    costs = [h['cost'] for h in history]
    rolls = [h['roll'] for h in history]
    dists = [h['dist'] for h in history]
    pos_x = [h['pos_x'] for h in history]
    pos_y = [h['pos_y'] for h in history]

    # パラメータ
    gn = [h['params']['gyro_noise'] for h in history]
    an = [h['params']['accel_noise'] for h in history]
    gbn = [h['params']['gyro_bias_noise'] for h in history]
    abn = [h['params']['accel_bias_noise'] for h in history]
    fn = [h['params']['flow_noise'] for h in history]
    tn = [h['params']['tof_noise'] for h in history]
    aan = [h['params']['accel_att_noise'] for h in history]

    # 勾配
    grad_norms = [np.sqrt(sum(g**2 for g in h['gradient'].values())) for h in history]

    # === 可視化 ===
    fig = plt.figure(figsize=(16, 14))

    # 1. コスト関数の推移
    ax1 = fig.add_subplot(3, 3, 1)
    ax1.plot(iters, costs, 'b-', linewidth=2)
    ax1.fill_between(iters, costs, alpha=0.3)
    ax1.axhline(y=min(costs), color='r', linestyle='--', label=f'Best={min(costs):.2f}')
    ax1.set_xlabel('Iteration')
    ax1.set_ylabel('Cost')
    ax1.set_title('Cost Function Convergence')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_yscale('log')

    # 2. Roll誤差の推移
    ax2 = fig.add_subplot(3, 3, 2)
    ax2.plot(iters, np.abs(rolls), 'r-', linewidth=2)
    ax2.axhline(y=np.abs(rolls[-1]), color='g', linestyle='--', label=f'Final={abs(rolls[-1]):.2f}°')
    ax2.set_xlabel('Iteration')
    ax2.set_ylabel('|Roll| [deg]')
    ax2.set_title('Roll Error Convergence')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. 位置誤差の推移
    ax3 = fig.add_subplot(3, 3, 3)
    ax3.plot(iters, dists, 'g-', linewidth=2)
    ax3.axhline(y=dists[-1], color='r', linestyle='--', label=f'Final={dists[-1]:.2f}cm')
    ax3.set_xlabel('Iteration')
    ax3.set_ylabel('Position Error [cm]')
    ax3.set_title('Position Error Convergence')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Qパラメータ（プロセスノイズ）
    ax4 = fig.add_subplot(3, 3, 4)
    ax4.semilogy(iters, gn, 'r-', linewidth=2, label=f'gyro_noise ({gn[-1]:.5f})')
    ax4.semilogy(iters, an, 'g-', linewidth=2, label=f'accel_noise ({an[-1]:.4f})')
    ax4.semilogy(iters, gbn, 'b-', linewidth=2, label=f'gyro_bias_noise ({gbn[-1]:.6f})')
    ax4.semilogy(iters, abn, 'm-', linewidth=2, label=f'accel_bias_noise ({abn[-1]:.5f})')
    ax4.set_xlabel('Iteration')
    ax4.set_ylabel('Value (log scale)')
    ax4.set_title('Q Parameters (Process Noise)')
    ax4.legend(fontsize=8, loc='right')
    ax4.grid(True, alpha=0.3)

    # 5. Rパラメータ（観測ノイズ）
    ax5 = fig.add_subplot(3, 3, 5)
    ax5.semilogy(iters, fn, 'r-', linewidth=2, label=f'flow_noise ({fn[-1]:.4f})')
    ax5.semilogy(iters, tn, 'g-', linewidth=2, label=f'tof_noise ({tn[-1]:.5f})')
    ax5.semilogy(iters, aan, 'b-', linewidth=2, label=f'accel_att_noise ({aan[-1]:.4f})')
    ax5.set_xlabel('Iteration')
    ax5.set_ylabel('Value (log scale)')
    ax5.set_title('R Parameters (Observation Noise)')
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)

    # 6. 勾配ノルムの推移
    ax6 = fig.add_subplot(3, 3, 6)
    ax6.semilogy(iters, grad_norms, 'purple', linewidth=2)
    ax6.set_xlabel('Iteration')
    ax6.set_ylabel('Gradient Norm (log)')
    ax6.set_title('Gradient Norm (Convergence Indicator)')
    ax6.grid(True, alpha=0.3)

    # 7. パラメータ空間の軌跡 (gyro_noise vs accel_noise)
    ax7 = fig.add_subplot(3, 3, 7)
    scatter = ax7.scatter(gn, an, c=iters, cmap='viridis', s=30, alpha=0.7)
    ax7.plot(gn[0], an[0], 'go', markersize=15, label='Start', zorder=5)
    ax7.plot(gn[-1], an[-1], 'r*', markersize=20, label='End', zorder=5)
    ax7.set_xlabel('gyro_noise')
    ax7.set_ylabel('accel_noise')
    ax7.set_title('Parameter Space: gyro_noise vs accel_noise')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax7, label='Iteration')

    # 8. パラメータ空間の軌跡 (flow_noise vs accel_att_noise)
    ax8 = fig.add_subplot(3, 3, 8)
    scatter2 = ax8.scatter(fn, aan, c=iters, cmap='plasma', s=30, alpha=0.7)
    ax8.plot(fn[0], aan[0], 'go', markersize=15, label='Start', zorder=5)
    ax8.plot(fn[-1], aan[-1], 'r*', markersize=20, label='End', zorder=5)
    ax8.set_xlabel('flow_noise')
    ax8.set_ylabel('accel_att_noise')
    ax8.set_title('Parameter Space: flow_noise vs accel_att_noise')
    ax8.legend()
    ax8.grid(True, alpha=0.3)
    plt.colorbar(scatter2, ax=ax8, label='Iteration')

    # 9. 最終位置の変化
    ax9 = fig.add_subplot(3, 3, 9)
    scatter3 = ax9.scatter(pos_x, pos_y, c=iters, cmap='coolwarm', s=30, alpha=0.7)
    ax9.plot(pos_x[0], pos_y[0], 'go', markersize=15, label='Start', zorder=5)
    ax9.plot(pos_x[-1], pos_y[-1], 'r*', markersize=20, label='End', zorder=5)
    ax9.plot(0, 0, 'k+', markersize=20, mew=3, label='Target (0,0)', zorder=5)
    ax9.set_xlabel('Final Position X [cm]')
    ax9.set_ylabel('Final Position Y [cm]')
    ax9.set_title('Final Position Evolution')
    ax9.legend()
    ax9.grid(True, alpha=0.3)
    ax9.axis('equal')
    plt.colorbar(scatter3, ax=ax9, label='Iteration')

    plt.suptitle('ESKF Q/R Parameter Optimization Process\n'
                 f'Final: Roll={abs(rolls[-1]):.2f}°, dist={dists[-1]:.2f}cm (from Roll=52.5°, dist=58.6cm)',
                 fontsize=14, fontweight='bold')

    plt.tight_layout()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_file = os.path.join(script_dir, 'optimization_process_detailed.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nSaved to {output_file}")

    plt.show()

if __name__ == '__main__':
    main()
