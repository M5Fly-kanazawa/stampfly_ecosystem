/**
 * @file imu_task.cpp
 * @brief IMUタスク (400Hz) - BMI270読み取り + ESKF更新
 */

#include "tasks_common.hpp"
#include "flight_command.hpp"

static const char* TAG = "IMUTask";

using namespace config;
using namespace globals;

void IMUTask(void* pvParameters)
{
    ESP_LOGI(TAG, "IMUTask started (400Hz via ESP Timer)");

    auto& state = stampfly::StampFlyState::getInstance();

    static uint32_t imu_loop_counter = 0;
    static uint32_t imu_read_fail_counter = 0;

    // ヘルスモニター設定（10連続成功/3連続失敗）
    g_health.imu.setThresholds(10, 3);

    while (true) {
        g_imu_checkpoint = 0;  // セマフォ待ち中

        // Wait for timer semaphore (precise 2.5ms = 400Hz timing)
        if (xSemaphoreTake(g_imu_semaphore, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        imu_loop_counter++;
        g_imu_last_loop = imu_loop_counter;  // タイマーコールバックで監視用
        g_imu_checkpoint = 1;  // ループ開始

        // 10秒ごとにIMUタスク生存確認
        if (imu_loop_counter % 4000 == 0) {
            ESP_LOGI(TAG, "IMUTask alive: loop=%lu, read_fails=%lu, stack_free=%u",
                     imu_loop_counter, imu_read_fail_counter,
                     (unsigned)uxTaskGetStackHighWaterMark(nullptr));
        }

        g_imu_checkpoint = 2;  // isInitialized チェック前

        if (g_imu.isInitialized()) {
            stampfly::AccelData accel;
            stampfly::GyroData gyro;

            g_imu_checkpoint = 3;  // IMU読み取り前
            if (g_imu.readSensorData(accel, gyro) == ESP_OK) {
                g_imu_checkpoint = 4;  // IMU読み取り成功
                g_health.imu.recordSuccess();
                g_imu_task_healthy = g_health.imu.isHealthy();
                imu_read_fail_counter = 0;

                // ============================================================
                // BMI270座標系 → 機体座標系(NED) 変換
                // 図より:
                //   BMI270のX → 機体Y (右方向)
                //   BMI270のY → 機体X (前方)
                //   BMI270のZ → 機体-Z (上向き、NEDでは下が正なので符号反転)
                // 変換式:
                //   機体X = センサY
                //   機体Y = センサX
                //   機体Z = -センサZ
                //
                // 単位変換:
                //   加速度: g → m/s² (×9.81)
                //   ジャイロ: rad/s (変換不要)
                // ============================================================
                float accel_body_x = accel.y * config::eskf::GRAVITY;   // 前方正 [m/s²]
                float accel_body_y = accel.x * config::eskf::GRAVITY;   // 右正 [m/s²]
                float accel_body_z = -accel.z * config::eskf::GRAVITY;  // 下正 (NED) [m/s²]

                float gyro_body_x = gyro.y;     // Roll rate [rad/s]
                float gyro_body_y = gyro.x;     // Pitch rate [rad/s]
                float gyro_body_z = -gyro.z;    // Yaw rate [rad/s]

                // Store pre-LPF raw values for telemetry (with timestamp)
                // テレメトリ用にLPF前の生値を保存（タイムスタンプ付き）
                // All IMU buffers share the same raw_index via push() ordering,
                // so telemetry can read matching data at the same raw_index.
                // 全IMUバッファは push() 順序で同じ raw_index を共有し、
                // テレメトリは同じ raw_index で対応するデータを読める。
                uint32_t imu_ts = static_cast<uint32_t>(esp_timer_get_time());
                g_accel_raw_buf.push(
                    stampfly::math::Vector3(accel_body_x, accel_body_y, accel_body_z), imu_ts);
                g_gyro_raw_buf.push(
                    stampfly::math::Vector3(gyro_body_x, gyro_body_y, gyro_body_z));

                // Apply low-pass filters (機体座標系で)
                float filtered_accel[3] = {
                    g_accel_lpf[0].apply(accel_body_x),
                    g_accel_lpf[1].apply(accel_body_y),
                    g_accel_lpf[2].apply(accel_body_z)
                };
                float filtered_gyro[3] = {
                    g_gyro_lpf[0].apply(gyro_body_x),
                    g_gyro_lpf[1].apply(gyro_body_y),
                    g_gyro_lpf[2].apply(gyro_body_z)
                };

                // Update state
                stampfly::StateVector3 accel_vec(filtered_accel[0], filtered_accel[1], filtered_accel[2]);
                stampfly::StateVector3 gyro_vec(filtered_gyro[0], filtered_gyro[1], filtered_gyro[2]);
                state.updateIMU(accel_vec, gyro_vec);

                // Prepare vectors for estimators
                stampfly::math::Vector3 a(filtered_accel[0], filtered_accel[1], filtered_accel[2]);
                stampfly::math::Vector3 g(filtered_gyro[0], filtered_gyro[1], filtered_gyro[2]);

                // 加速度・ジャイロリングバッファに追加（常時更新）
                g_accel_buf.push(a);
                g_gyro_buf.push(g);

                // Update sensor diagnostics
                // センサ診断情報を更新
                state.updateSensorDiag("imu", g_imu_task_healthy, imu_ts);

                // Signal telemetry task for FFT mode (400Hz sync)
                // FFTモードのテレメトリタスクに新しいIMUデータを通知
                // IMPORTANT: Must be AFTER buffer writes so telemetry reads fresh data
                // 重要: バッファ書き込み後に通知することでテレメトリが最新データを読める
                xSemaphoreGive(g_telemetry_imu_semaphore);

                g_imu_checkpoint = 10;  // ESKF更新前

                // ============================================================
                // Landing Handler update (for level calibration)
                // 着陸キャリブレーション（Disarm時のみ動作）
                // ============================================================
                {
                    // Snapshot flight state once to avoid race condition
                    // フライト状態を1回だけ取得（レースコンディション防止）
                    auto current_fs = state.getFlightState();
                    bool is_disarmed = (current_fs == stampfly::FlightState::IDLE ||
                                        current_fs == stampfly::FlightState::CALIBRATING ||
                                        current_fs == stampfly::FlightState::INIT);

                    float tof_bottom_now = 0.0f;
                    if (g_tof_bottom_buf.count() > 0) {
                        tof_bottom_now = g_tof_bottom_buf.latest();
                    }

                    // Update landing handler with current sensor data
                    // ToF バッファ空は有効なデータとして扱わない
                    bool tof_valid = (g_tof_bottom_buf.count() > 0);
                    g_landing_handler.update(is_disarmed, tof_bottom_now, tof_valid, g, a);

                    // Check for calibration complete (only apply if disarmed)
                    // キャリブレーション完了チェック（DISARM中のみ適用）
                    if (g_landing_handler.justCalibrated() && is_disarmed) {
                        // Set attitude reference from level calibration
                        // Use accel reference for level (roll=0, pitch=0) but keep
                        // current ESKF gyro bias instead of StationaryDetector's value.
                        // StationaryDetector averages raw gyro which includes the
                        // (possibly drifted) ESKF bias estimate, producing incorrect
                        // values after dynamic flight.
                        // 加速度リファレンスで水平基準を設定するが、ジャイロバイアスは
                        // ESKFの現在値を維持する。StationaryDetectorの生ジャイロ平均は
                        // 飛行中にドリフトしたESKFバイアスを含むため不正確。
                        auto eskf_state = g_fusion.getState();
                        g_fusion.setAttitudeReference(
                            g_landing_handler.getAccelReference(),
                            eskf_state.gyro_bias  // Keep ESKF's current gyro bias
                        );
                        // resetForLanding() は justLanded() イベント（行241）で実行済み
                        // ここで再度呼ぶと setAttitudeReference() の freeze_accel_bias=false が
                        // 即座に true に戻される矛盾が発生する
                        ESP_LOGI(TAG, "Level calibration complete - attitude reference set (gyro bias preserved)");
                    }

                    // Update FlightState based on calibration (LED updates automatically via callback)
                    // キャリブレーション状態に基づいてFlightStateを更新（LEDはコールバック経由で自動更新）
                    static stampfly::CalibrationState last_cal_state =
                        stampfly::CalibrationState::NOT_STARTED;
                    auto cal_state = g_landing_handler.getCalibrationState();

                    if (cal_state != last_cal_state) {
                        auto& sys_state_mgr = stampfly::SystemStateManager::getInstance();
                        auto& led_mgr = stampfly::LEDManager::getInstance();
                        if (cal_state == stampfly::CalibrationState::CALIBRATING) {
                            // Transition to CALIBRATING state
                            // CALIBRATING状態に遷移
                            sys_state_mgr.setFlightState(stampfly::FlightState::CALIBRATING);
                            state.setFlightState(stampfly::FlightState::CALIBRATING);  // Legacy compatibility

                            // LED: amber slow blink = calibrating
                            // LED: 黄橙ゆっくり点滅 = キャリブレーション中
                            led_mgr.requestChannel(
                                stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::CALIBRATION,
                                stampfly::LEDPattern::BLINK_SLOW, 0xFFAA00);
                        } else if (cal_state == stampfly::CalibrationState::COMPLETED) {
                            if (is_disarmed && sys_state_mgr.getFlightState() == stampfly::FlightState::CALIBRATING) {
                                sys_state_mgr.setFlightState(stampfly::FlightState::IDLE);
                                state.setFlightState(stampfly::FlightState::IDLE);
                            }

                            // LED: green fast blink 3s then release → mode color shows through
                            // LED: 緑速点滅3��→解除→モード色が表示される
                            led_mgr.requestChannel(
                                stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::CALIBRATION,
                                stampfly::LEDPattern::BLINK_FAST, 0x00FF00, 3000);
                        } else if (cal_state == stampfly::CalibrationState::NOT_STARTED) {
                            if (is_disarmed) {
                                // LED: red slow blink = waiting for landing (disarmed only)
                                // LED: 赤ゆっくり点滅 = 着陸待ち（disarmed時のみ）
                                led_mgr.requestChannel(
                                    stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::CALIBRATION,
                                    stampfly::LEDPattern::BLINK_SLOW, 0xFF0000);
                            } else {
                                // Armed/flying: release calibration LED so mode color shows
                                // Armed/飛行中: キャリブレーションLEDを解除してモード色を表示
                                led_mgr.releaseChannel(
                                    stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::CALIBRATION);
                            }
                        }
                        last_cal_state = cal_state;
                    }
                }

                // Update sensor fusion predict step (400Hz)
                // g_eskf_ready: センサー安定・キャリブレーション完了後にtrue
                if (g_fusion.isInitialized() && g_eskf_ready) {
                    g_imu_checkpoint = 11;  // ESKF入力チェック

                    // 入力値の事前チェック
                    bool eskf_ok = std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z) &&
                                   std::isfinite(g.x) && std::isfinite(g.y) && std::isfinite(g.z);

                    if (eskf_ok) {
                        g_imu_checkpoint = 12;  // predict前

                        // バッファの最新値を取得
                        const auto& accel_latest = g_accel_buf.latest();
                        const auto& gyro_latest = g_gyro_buf.latest();

                        // 接地判定: LandingHandler が唯一の管理者
                        // Ground state: LandingHandler is the single source of truth
                        bool is_landed = g_landing_handler.isLanded();
                        bool has_taken_off = g_landing_handler.hasTakenOff();
                        // Freeze position until takeoff detected (prevent predict-only drift)
                        // 離陸検出まで位置をフリーズ（predict のみのドリフト防止）
                        bool skip_position = (is_landed || !has_taken_off) && eskf::ENABLE_LANDING_RESET;

                        // IMU予測 + 加速度計姿勢補正（predictIMUが両方を実行）
                        // 接地中はskip_position=trueで位置更新をスキップ（ドリフト防止）
                        g_fusion.predictIMU(accel_latest, gyro_latest, 0.0025f, skip_position);

                        // 着陸イベント: キャリブレーション開始トリガー
                        // Landing event: calibration trigger only
                        // (ESKF reset is done at DISARM time in control_task, not here)
                        if (g_landing_handler.justLanded() && g_landing_handler.hasTakenOff()) {
                            ESP_LOGI(TAG, "Landed - calibration sequence will start");
                        }

                        // 接地中 or 離陸前: 位置・速度を0に保持
                        // Grounded or pre-takeoff: hold position/velocity at zero
                        if (is_landed || !has_taken_off) {
                            g_fusion.holdPositionVelocity();
                        }

                        // 離陸イベント: 共分散リセット（armed 後に初めて高度が上がった時）
                        // Takeoff event: reset covariance
                        if (g_landing_handler.justTakenOff()) {
                            g_fusion.resetPositionVelocity();
                            g_fusion.setFreezeAccelBias(false);
                            ESP_LOGI(TAG, "Takeoff - position estimation enabled, accel bias unfrozen");
                        }

                        g_imu_checkpoint = 14;  // フロー更新セクション

                        // オプティカルフロー更新（data_readyフラグで制御、100Hz）
                        // ヘルスチェック: Flow + ToF が両方healthy必要（距離スケーリングに必要）
                        // 接地中: 位置・速度更新を停止するため呼ばない
                        // 飛行中: 実センサ値を使用
                        static int optflow_takeoff_skip_counter = 0;  // 離陸後のスキップカウンタ
                        if (!has_taken_off) {
                            optflow_takeoff_skip_counter = 20;  // 離陸後20回（0.2秒@100Hz）スキップ
                        }

                        if (g_optflow_data_ready) {
                            g_optflow_data_ready = false;

                            // 離陸前・接地中は位置・速度更新を停止（センサー更新を呼ばない）
                            if (has_taken_off && !is_landed && g_optflow_task_healthy && g_tof_task_healthy) {
                                if (optflow_takeoff_skip_counter > 0) {
                                    optflow_takeoff_skip_counter--;
                                    // 共分散リセット直後は過剰補正防止のためスキップ
                                } else if (g_flow_buf.count() > 0 && g_tof_bottom_buf.count() > 0) {
                                    constexpr float dt = 0.01f;  // 100Hz
                                    // バッファの最新値を取得
                                    const auto& flow = g_flow_buf.latest();
                                    float tof_bottom = g_tof_bottom_buf.latest();
                                    // squal/distance チェックは SensorFusion 内部で実行
                                    g_fusion.updateOpticalFlow(flow.dx, flow.dy, flow.squal, tof_bottom, dt,
                                                               gyro_latest.x, gyro_latest.y);
                                }
                            }
                        }

                        g_imu_checkpoint = 15;  // Baro更新セクション

                        // Baro更新（data_readyフラグで制御、50Hz）
                        // ヘルスチェック: Baro healthy必要
                        if (g_baro_data_ready) {
                            g_baro_data_ready = false;
                            if (g_baro_task_healthy && g_baro_buf.count() > 0) {
                                // Baro ON/OFF is controlled by config::eskf::USE_BAROMETER
                                // via ESKF::Config::sensor_enabled[SENSOR_BARO]
                                // 気圧計の有効/無効は config で一元管理
                                g_fusion.updateBarometer(g_baro_buf.latest());
                            }
                        }

                        g_imu_checkpoint = 16;  // ToF更新セクション

                        // ToF更新（data_readyフラグで制御、30Hz）
                        // 接地中: 位置更新を停止するため呼ばない
                        // 飛行中: 実センサ値を使用
                        static int tof_takeoff_skip_counter = 0;
                        if (!has_taken_off) {
                            tof_takeoff_skip_counter = 10;  // 離陸後10回（~0.3秒@30Hz）スキップ
                        }
                        if (g_tof_bottom_data_ready) {
                            g_tof_bottom_data_ready = false;

                            // ToF更新は常に行う（接地中でも高度情報は必要）
                            // 位置のドリフト防止は predictIMU の skip_position フラグで対応
                            if (g_tof_task_healthy) {
                                if (tof_takeoff_skip_counter > 0) {
                                    tof_takeoff_skip_counter--;
                                    // 共分散リセット直後は過剰補正防止のためスキップ
                                } else if (g_tof_bottom_buf.count() > 0) {
                                    // 距離範囲チェックはSensorFusion内部で実行
                                    g_fusion.updateToF(g_tof_bottom_buf.latest());
                                }
                            }
                        }

                        g_imu_checkpoint = 17;  // Mag更新セクション

                        // Mag更新（data_readyフラグで制御、100Hz）
                        // ヘルスチェック: Mag healthy必要
                        static uint32_t mag_update_count = 0;
                        if (g_mag_data_ready && g_mag_ref_set) {
                            g_mag_data_ready = false;
                            if (g_mag_task_healthy && g_mag_buf.count() > 0) {
                                g_fusion.updateMagnetometer(g_mag_buf.latest());
                                mag_update_count++;
                            }
                        }

                        g_imu_checkpoint = 20;  // getState前

                        // Update StampFlyState with estimated state
                        auto eskf_state = g_fusion.getState();

                        g_imu_checkpoint = 21;  // getState後、検証前

                        // NaN チェック（フィルタ実装バグの検出用、リセットはしない）
                        // NaN detection (filter bug indicator — log only, no reset)
                        bool eskf_valid = std::isfinite(eskf_state.roll) && std::isfinite(eskf_state.pitch);

                        if (!eskf_valid) {
                            static uint32_t nan_count = 0;
                            nan_count++;
                            if (nan_count <= 10 || nan_count % 400 == 0) {
                                ESP_LOGE(TAG, "ESKF output NaN detected (count=%lu) — filter bug, no reset",
                                         nan_count);
                            }
                        }

                        {
                            g_imu_checkpoint = 22;  // state更新前

                            // NaN 時は前回値を維持（updateAttitude を呼ばない）
                            if (eskf_valid) {
                                state.updateAttitude(eskf_state.roll, eskf_state.pitch, eskf_state.yaw);
                            }

                            // 接地中は位置・速度を確実にゼロで更新（HTMLView等への表示用）
                            if (skip_position) {
                                state.updateEstimatedPosition(0.0f, 0.0f, 0.0f);
                                state.updateEstimatedVelocity(0.0f, 0.0f, 0.0f);
                            } else {
                                state.updateEstimatedPosition(eskf_state.position.x, eskf_state.position.y, eskf_state.position.z);
                                state.updateEstimatedVelocity(eskf_state.velocity.x, eskf_state.velocity.y, eskf_state.velocity.z);
                            }
                            state.updateGyroBias(eskf_state.gyro_bias.x, eskf_state.gyro_bias.y, eskf_state.gyro_bias.z);
                            state.updateAccelBias(eskf_state.accel_bias.x, eskf_state.accel_bias.y, eskf_state.accel_bias.z);

                            // 姿勢・バイアス定期ログ（4秒ごと）
                            static uint32_t attitude_log_counter = 0;
                            attitude_log_counter++;
                            if (attitude_log_counter % 1600 == 0) {
                                ESP_LOGI(TAG, "Att: R=%.2f P=%.2f Y=%.2f | BA=[%.4f,%.4f,%.4f] %s",
                                         eskf_state.roll * 57.3f, eskf_state.pitch * 57.3f, eskf_state.yaw * 57.3f,
                                         eskf_state.accel_bias.x, eskf_state.accel_bias.y, eskf_state.accel_bias.z,
                                         is_landed ? "(grounded)" : "(flying)");
                            }

                            g_imu_checkpoint = 23;  // state更新後、ロギング前

                            // === Binary logging (400Hz) ===
                            if (g_logger.isRunning()) {
                                stampfly::LogPacket pkt;
                                pkt.header[0] = 0xAA;
                                pkt.header[1] = 0x56;  // V2
                                pkt.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                                // IMU data (filtered, body frame)
                                pkt.accel_x = filtered_accel[0];
                                pkt.accel_y = filtered_accel[1];
                                pkt.accel_z = filtered_accel[2];
                                pkt.gyro_x = filtered_gyro[0];
                                pkt.gyro_y = filtered_gyro[1];
                                pkt.gyro_z = filtered_gyro[2];

                                // Mag data (from state cache)
                                stampfly::Vec3 mag_cached;
                                state.getMagData(mag_cached);
                                pkt.mag_x = mag_cached.x;
                                pkt.mag_y = mag_cached.y;
                                pkt.mag_z = mag_cached.z;

                                // Baro data
                                float baro_alt_cached, pressure_cached;
                                state.getBaroData(baro_alt_cached, pressure_cached);
                                pkt.pressure = pressure_cached;
                                pkt.baro_alt = baro_alt_cached;

                                // ToF data
                                float tof_bottom_cached, tof_front_cached;
                                state.getToFData(tof_bottom_cached, tof_front_cached);
                                pkt.tof_bottom = tof_bottom_cached;
                                pkt.tof_front = tof_front_cached;

                                // OptFlow raw data
                                int16_t flow_dx_cached, flow_dy_cached;
                                uint8_t flow_squal_cached;
                                state.getFlowRawData(flow_dx_cached, flow_dy_cached, flow_squal_cached);
                                pkt.flow_dx = flow_dx_cached;
                                pkt.flow_dy = flow_dy_cached;
                                pkt.flow_squal = flow_squal_cached;

                                // ESKF estimates
                                pkt.pos_x = eskf_state.position.x;
                                pkt.pos_y = eskf_state.position.y;
                                pkt.pos_z = eskf_state.position.z;
                                pkt.vel_x = eskf_state.velocity.x;
                                pkt.vel_y = eskf_state.velocity.y;
                                pkt.vel_z = eskf_state.velocity.z;
                                pkt.roll = eskf_state.roll;
                                pkt.pitch = eskf_state.pitch;
                                pkt.yaw = eskf_state.yaw;
                                pkt.gyro_bias_z = eskf_state.gyro_bias.z;
                                pkt.accel_bias_x = eskf_state.accel_bias.x;
                                pkt.accel_bias_y = eskf_state.accel_bias.y;

                                // Status
                                pkt.eskf_status = 1;  // running
                                pkt.baro_ref_alt = g_baro_reference_altitude;
                                memset(pkt.reserved, 0, sizeof(pkt.reserved));
                                pkt.checksum = 0;  // Will be calculated by Logger

                                g_logger.pushData(pkt);
                            }
                            g_imu_checkpoint = 24;  // ロギング完了
                        }
                    }
                }
                // ESKF is the sole estimator — no fallback
                // ESKF が唯一の推定器 — フォールバックなし

                g_imu_checkpoint = 30;  // ControlTask起動前

                // Wake up ControlTask (runs at same 400Hz rate)
                xSemaphoreGive(g_control_semaphore);

                g_imu_checkpoint = 31;  // ControlTask起動後
            } else {
                g_health.imu.recordFailure();
                g_imu_task_healthy = g_health.imu.isHealthy();
                imu_read_fail_counter++;
                // 連続失敗時にログ出力
                if (imu_read_fail_counter % 400 == 1) {
                    ESP_LOGW(TAG, "IMU read failed, consecutive fails=%lu", imu_read_fail_counter);
                }
            }
        }

        g_imu_checkpoint = 99;  // ループ完了（次のセマフォ待ちへ）
        // No delay here - timing controlled by ESP Timer semaphore
    }
}
