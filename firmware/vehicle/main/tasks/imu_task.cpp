/**
 * @file imu_task.cpp
 * @brief IMUタスク (400Hz) - BMI270読み取り + ESKF更新
 */

#include "tasks_common.hpp"

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
                constexpr float GRAVITY = 9.81f;
                float accel_body_x = accel.y * GRAVITY;   // 前方正 [m/s²]
                float accel_body_y = accel.x * GRAVITY;   // 右正 [m/s²]
                float accel_body_z = -accel.z * GRAVITY;  // 下正 (NED) [m/s²]

                float gyro_body_x = gyro.y;     // Roll rate [rad/s]
                float gyro_body_y = gyro.x;     // Pitch rate [rad/s]
                float gyro_body_z = -gyro.z;    // Yaw rate [rad/s]

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

                // 加速度リングバッファに追加（常時更新）
                // 初期化時は平均計算、ESKF実行時は最新値を使用
                g_accel_buffer[g_accel_buffer_index] = a;
                g_accel_buffer_index = (g_accel_buffer_index + 1) % REF_BUFFER_SIZE;
                if (g_accel_buffer_count < REF_BUFFER_SIZE) {
                    g_accel_buffer_count++;
                }

                // ジャイロも同様にバッファに追加
                g_gyro_buffer[g_gyro_buffer_index] = g;
                g_gyro_buffer_index = (g_gyro_buffer_index + 1) % REF_BUFFER_SIZE;
                if (g_gyro_buffer_count < REF_BUFFER_SIZE) {
                    g_gyro_buffer_count++;
                }

                g_imu_checkpoint = 10;  // ESKF更新前

                // Update sensor fusion predict step (400Hz)
                // g_eskf_ready: センサー安定・キャリブレーション完了後にtrue
                if (g_fusion.isInitialized() && g_eskf_ready) {
                    static uint32_t eskf_error_counter = 0;

                    g_imu_checkpoint = 11;  // ESKF入力チェック

                    // 入力値の事前チェック
                    bool eskf_ok = std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z) &&
                                   std::isfinite(g.x) && std::isfinite(g.y) && std::isfinite(g.z);

                    if (eskf_ok) {
                        g_imu_checkpoint = 12;  // predict前

                        // 接地判定（ToF高度ベース、リングバッファから取得）
                        float tof_bottom_now = 0.0f;
                        if (g_tof_bottom_buffer_count > 0) {
                            int tof_latest_idx = (g_tof_bottom_buffer_index - 1 + REF_BUFFER_SIZE) % REF_BUFFER_SIZE;
                            tof_bottom_now = g_tof_bottom_buffer[tof_latest_idx];
                        }
                        static bool is_grounded = true;        // 起動時は接地状態
                        static bool has_taken_off = false;     // 一度でも離陸したか
                        static int takeoff_counter = 0;        // 離陸判定用カウンタ
                        static int landing_counter = 0;        // 着陸判定用カウンタ
                        constexpr int GROUNDED_TRANSITION_COUNT = 20;  // 50ms @ 400Hz

                        // 接地状態の更新（連続条件付きヒステリシス）
                        bool prev_grounded = is_grounded;
                        if (tof_bottom_now < eskf::LANDING_ALT_THRESHOLD) {
                            landing_counter++;
                            takeoff_counter = 0;
                            if (landing_counter >= GROUNDED_TRANSITION_COUNT) {
                                is_grounded = true;
                            }
                        } else if (tof_bottom_now > eskf::LANDING_ALT_THRESHOLD * 2.0f) {
                            takeoff_counter++;
                            landing_counter = 0;
                            if (takeoff_counter >= GROUNDED_TRANSITION_COUNT) {
                                is_grounded = false;
                            }
                        }
                        // ヒステリシス: 閾値〜2倍の間は状態維持、カウンタもリセットしない

                        // バッファの最新値を取得
                        int accel_latest_idx = (g_accel_buffer_index - 1 + REF_BUFFER_SIZE) % REF_BUFFER_SIZE;
                        int gyro_latest_idx = (g_gyro_buffer_index - 1 + REF_BUFFER_SIZE) % REF_BUFFER_SIZE;
                        const auto& accel_latest = g_accel_buffer[accel_latest_idx];
                        const auto& gyro_latest = g_gyro_buffer[gyro_latest_idx];

                        // 接地中かどうかを判定してskip_positionフラグを設定
                        bool skip_position = is_grounded && eskf::ENABLE_LANDING_RESET;

                        // IMU予測 + 加速度計姿勢補正（predictIMUが両方を実行）
                        // 接地中はskip_position=trueで位置更新をスキップ（ドリフト防止）
                        g_fusion.predictIMU(accel_latest, gyro_latest, 0.0025f, skip_position);

                        // 接地中の処理
                        if (skip_position) {
                            // 着陸遷移時: 位置・速度・加速度バイアスをリセット（姿勢は維持）
                            // resetForLanding()内で加速度バイアス推定もフリーズされる
                            if (!prev_grounded && has_taken_off) {
                                g_fusion.resetForLanding();
                                ESP_LOGI(TAG, "Landed - reset for landing, accel bias frozen (alt=%.3fm)", tof_bottom_now);
                            }
                            // 接地中: 位置・速度を0に保持
                            g_fusion.holdPositionVelocity();
                        } else if (!is_grounded && prev_grounded) {
                            // 離陸遷移時 - 共分散をリセットして安定した推定開始
                            has_taken_off = true;
                            g_fusion.resetPositionVelocity();  // 共分散も適切な初期値に
                            g_fusion.setFreezeAccelBias(false);  // 加速度バイアス推定を再開
                            ESP_LOGI(TAG, "Takeoff - position estimation enabled, accel bias unfrozen (alt=%.3fm)", tof_bottom_now);
                        }

                        g_imu_checkpoint = 14;  // フロー更新セクション

                        // オプティカルフロー更新（data_readyフラグで制御、100Hz）
                        // ヘルスチェック: Flow + ToF が両方healthy必要（距離スケーリングに必要）
                        // 接地中: 位置・速度更新を停止するため呼ばない
                        // 飛行中: 実センサ値を使用
                        static int optflow_takeoff_skip_counter = 0;  // 離陸後のスキップカウンタ
                        if (is_grounded) {
                            optflow_takeoff_skip_counter = 20;  // 離陸後20回（0.2秒@100Hz）スキップ
                        }

                        if (g_optflow_data_ready) {
                            g_optflow_data_ready = false;

                            // 接地中は位置・速度更新を停止（センサー更新を呼ばない）
                            if (!is_grounded && g_optflow_task_healthy && g_tof_task_healthy) {
                                if (optflow_takeoff_skip_counter > 0) {
                                    optflow_takeoff_skip_counter--;
                                    // 共分散リセット直後は過剰補正防止のためスキップ
                                } else if (g_optflow_buffer_count > 0 && g_tof_bottom_buffer_count > 0) {
                                    constexpr float dt = 0.01f;  // 100Hz
                                    // バッファの最新値を取得
                                    int optflow_latest_idx = (g_optflow_buffer_index - 1 + REF_BUFFER_SIZE) % REF_BUFFER_SIZE;
                                    int tof_latest_idx = (g_tof_bottom_buffer_index - 1 + REF_BUFFER_SIZE) % REF_BUFFER_SIZE;
                                    const auto& flow = g_optflow_buffer[optflow_latest_idx];
                                    float tof_bottom = g_tof_bottom_buffer[tof_latest_idx];
                                    // squal/distance チェックは SensorFusion 内部で実行
                                    g_fusion.updateOpticalFlow(flow.dx, flow.dy, flow.squal, tof_bottom, dt,
                                                               gyro_latest.x, gyro_latest.y);
                                }
                            }
                        }

                        g_imu_checkpoint = 15;  // Baro更新セクション

                        // Baro更新（data_readyフラグで制御、50Hz）
                        // ヘルスチェック: Baro healthy必要
                        // TODO: 気圧センサの値が確認できたら有効化
                        if (g_baro_data_ready) {
                            g_baro_data_ready = false;
                            if (g_baro_task_healthy && g_baro_buffer_count > 0) {
                                // バッファの最新値を取得
                                int baro_latest_idx = (g_baro_buffer_index - 1 + REF_BUFFER_SIZE) % REF_BUFFER_SIZE;
                                // g_fusion.updateBarometer(g_baro_buffer[baro_latest_idx]);
                            }
                        }

                        g_imu_checkpoint = 16;  // ToF更新セクション

                        // ToF更新（data_readyフラグで制御、30Hz）
                        // 接地中: 位置更新を停止するため呼ばない
                        // 飛行中: 実センサ値を使用
                        static int tof_takeoff_skip_counter = 0;
                        if (is_grounded) {
                            tof_takeoff_skip_counter = 10;  // 離陸後10回（~0.3秒@30Hz）スキップ
                        }
                        if (g_tof_bottom_data_ready) {
                            g_tof_bottom_data_ready = false;

                            // 接地中は位置更新を停止（センサー更新を呼ばない）
                            if (!is_grounded && g_tof_task_healthy) {
                                if (tof_takeoff_skip_counter > 0) {
                                    tof_takeoff_skip_counter--;
                                    // 共分散リセット直後は過剰補正防止のためスキップ
                                } else if (g_tof_bottom_buffer_count > 0) {
                                    // バッファの最新値を取得
                                    int tof_latest_idx = (g_tof_bottom_buffer_index - 1 + REF_BUFFER_SIZE) % REF_BUFFER_SIZE;
                                    // 距離範囲チェックはSensorFusion内部で実行
                                    g_fusion.updateToF(g_tof_bottom_buffer[tof_latest_idx]);
                                }
                            }
                        }

                        g_imu_checkpoint = 17;  // Mag更新セクション

                        // Mag更新（data_readyフラグで制御、100Hz）
                        // ヘルスチェック: Mag healthy必要
                        static uint32_t mag_update_count = 0;
                        if (g_mag_data_ready && g_mag_ref_set) {
                            g_mag_data_ready = false;
                            if (g_mag_task_healthy && g_mag_buffer_count > 0) {
                                // バッファの最新値を取得
                                int latest_idx = (g_mag_buffer_index - 1 + REF_BUFFER_SIZE) % REF_BUFFER_SIZE;
                                g_fusion.updateMagnetometer(g_mag_buffer[latest_idx]);
                                mag_update_count++;
                            }
                        }

                        g_imu_checkpoint = 20;  // getState前

                        // Update StampFlyState with estimated state
                        auto eskf_state = g_fusion.getState();

                        g_imu_checkpoint = 21;  // getState後、検証前

                        // 出力値の検証（NaNチェック + 発散検出）
                        bool eskf_valid = std::isfinite(eskf_state.roll) && std::isfinite(eskf_state.pitch);

                        // 発散検出: 位置/速度が異常に大きい場合
                        constexpr float MAX_POS = 100.0f;   // 100m以上は異常
                        constexpr float MAX_VEL = 50.0f;    // 50m/s以上は異常
                        bool pos_diverged = std::abs(eskf_state.position.x) > MAX_POS ||
                                           std::abs(eskf_state.position.y) > MAX_POS ||
                                           std::abs(eskf_state.position.z) > MAX_POS;
                        bool vel_diverged = std::abs(eskf_state.velocity.x) > MAX_VEL ||
                                           std::abs(eskf_state.velocity.y) > MAX_VEL ||
                                           std::abs(eskf_state.velocity.z) > MAX_VEL;

                        if (eskf_valid && !pos_diverged && !vel_diverged) {
                            g_imu_checkpoint = 22;  // state更新前
                            state.updateAttitude(eskf_state.roll, eskf_state.pitch, eskf_state.yaw);

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
                                         is_grounded ? "(grounded)" : "(flying)");
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
                        } else {
                            eskf_error_counter++;

                            // 発散時は常にリセット（binlog onと同じ処理）
                            if (pos_diverged || vel_diverged) {
                                g_fusion.reset();
                                g_fusion.setGyroBias(g_initial_gyro_bias);
                                initializeAttitudeFromBuffers();
                            }

                            // ログ出力は100回に1回（スパム防止）
                            if (eskf_error_counter % 100 == 1) {
                                if (!eskf_valid) {
                                    ESP_LOGW(TAG, "ESKF output NaN, errors=%lu", eskf_error_counter);
                                } else {
                                    ESP_LOGW(TAG, "ESKF diverged: pos=[%.1f,%.1f,%.1f] vel=[%.1f,%.1f,%.1f]",
                                             eskf_state.position.x, eskf_state.position.y, eskf_state.position.z,
                                             eskf_state.velocity.x, eskf_state.velocity.y, eskf_state.velocity.z);
                                }
                            }
                        }
                    } else {
                        eskf_error_counter++;
                    }
                }
                // Fallback to simple attitude estimator if ESKF not available
                else if (g_attitude_est.isInitialized()) {
                    g_attitude_est.update(a, g, 0.0025f);
                    auto att_state = g_attitude_est.getState();
                    state.updateAttitude(att_state.roll, att_state.pitch, att_state.yaw);
                }

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
