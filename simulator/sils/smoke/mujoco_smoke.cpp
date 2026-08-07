/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file mujoco_smoke.cpp
 * @brief P1.0 smoke test — load a model into MuJoCo and step it.
 *        P1.0 スモークテスト — MuJoCo にモデルを読み込んでステップする。
 *
 * Proves the MuJoCo dependency builds, links, loads an MJCF, and integrates
 * deterministically on this host. A StampFly-sized box is dropped onto a
 * ground plane; the body height must fall from 0.5 m and settle near the
 * floor. No GUI, no wall clock — headless and reproducible.
 *
 * MuJoCo 依存がビルド・リンクでき、MJCF を読み、このホストで決定論的に
 * 積分できることを示す。StampFly 大のボックスを地面に落とし、機体高度が
 * 0.5 m から床付近まで落ちることを確認する。GUI なし・壁時計なし＝
 * ヘッドレスで再現可能。
 */

#include <cstdio>
#include <cstdlib>

#include <mujoco/mujoco.h>

namespace {
// Number of steps to integrate for the smoke (2 s at 2.5 ms).
// スモークで積分するステップ数（2.5 ms で 2 秒）。
constexpr int kSmokeSteps = 800;
}  // namespace

int main(int argc, char** argv)
{
    // Model path: first argument, or the in-tree default.
    // モデルパス: 第1引数、なければツリー内の既定値。
    const char* model_path =
        (argc > 1) ? argv[1] : "simulator/sils/models/quad_smoke.xml";

    // Load the MJCF model.
    // MJCF モデルを読み込む。
    char error[1024] = {0};
    mjModel* model = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!model) {
        fprintf(stderr, "[mujoco_smoke] failed to load %s: %s\n", model_path, error);
        return 1;
    }

    mjData* data = mj_makeData(model);
    if (!data) {
        fprintf(stderr, "[mujoco_smoke] mj_makeData failed\n");
        mj_deleteModel(model);
        return 1;
    }

    // qpos for a free joint is [x y z qw qx qy qz]; index 2 is the height.
    // free joint の qpos は [x y z qw qx qy qz]; index 2 が高度。
    const double start_z = data->qpos[2];

    for (int step = 0; step < kSmokeSteps; ++step) {
        mj_step(model, data);
    }

    const double end_z = data->qpos[2];
    printf("[mujoco_smoke] MuJoCo %s | steps=%d dt=%.4f\n",
           mj_versionString(), kSmokeSteps, model->opt.timestep);
    printf("[mujoco_smoke] body height: start=%.3f m -> end=%.3f m\n",
           start_z, end_z);

    // The box must have fallen and settled (well below the start height).
    // ボックスは落ちて静定しているはず（開始高度より十分低い）。
    const bool fell = end_z < start_z - 0.1;
    printf("[mujoco_smoke] %s — MuJoCo builds, loads, and steps on this host\n",
           fell ? "OK" : "UNEXPECTED (body did not fall)");

    mj_deleteData(data);
    mj_deleteModel(model);
    return fell ? 0 : 2;
}
