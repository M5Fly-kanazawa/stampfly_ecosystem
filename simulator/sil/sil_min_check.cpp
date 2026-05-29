/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL/test support).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sil_min_check.cpp
 * @brief M1 link smoke test — drives the unmodified StateManager on a PC
 *        M1リンク確認 — 無改変のStateManagerをPCで駆動する
 *
 * Proves the compat shim + sil_topics let the firmware core compile and link
 * on the host. Exercises the first real transition: INIT → IDLE_GROUND → ARM.
 *
 * compatシム + sil_topics により本体コアがホストでコンパイル・リンクできる
 * ことを実証する。最初の実遷移 INIT → IDLE_GROUND → ARM を動かす。
 */

#include <cstdio>
#include "topics.hpp"
#include "state_manager.hpp"

using namespace sf;

int main()
{
    topics_init();

    StateManager sm;
    sm.init();
    printf("init       -> state=%s armed=%d\n",
           flightStateName(sm.getState()), sm.isArmed());

    sm.forceIdle();
    printf("forceIdle  -> state=%s\n", flightStateName(sm.getState()));

    bool ok = sm.requestArm();
    printf("requestArm -> ret=%d state=%s armed=%d\n",
           ok, flightStateName(sm.getState()), sm.isArmed());

    // Expected: INIT → IDLE_GROUND → ARMED_GROUND, armed=1
    // 期待: INIT → IDLE_GROUND → ARMED_GROUND, armed=1
    bool pass = ok && sm.getState() == FlightState::ARMED_GROUND && sm.isArmed();
    printf("%s\n", pass ? "M1 OK" : "M1 FAIL");
    return pass ? 0 : 1;
}
