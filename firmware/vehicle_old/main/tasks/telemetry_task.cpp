/**
 * @file telemetry_task.cpp
 * @brief Dual-mode telemetry: UDP full-data + WebSocket visualization
 *
 * デュアルモードテレメトリ: UDP フルデータ + WebSocket 可視化
 *
 * Architecture:
 *   TelemetryTask (400Hz, IMU-synced, core 0)
 *     Reads sensor data, fills batch accumulators, enqueues to send queue.
 *     Never calls sendto() or broadcast() directly.
 *     センサデータを読み、バッチ蓄積器に充填、送信キューに投入。
 *     sendto() や broadcast() を直接呼ばない。
 *
 *   TelemetrySendTask (independent, core 0, lower priority)
 *     Dequeues packets and sends via UDP or WebSocket.
 *     sendto() blocking (2-3ms) is isolated here.
 *     パケットをデキューし UDP または WebSocket で送信。
 *     sendto() のブロック（2-3ms）はここに隔離される。
 *
 * All large structures are module-static to keep task stack small.
 * 大きな構造体はモジュール static にしてタスクスタックを小さく保つ。
 */

#include "tasks_common.hpp"
#include "sensor_fusion.hpp"
#include "udp_telemetry.hpp"
#include "rate_controller.hpp"
#include "esp_timer.h"

static const char* TAG = "TelemetryTask";

using namespace config;
using namespace globals;
using namespace stampfly::udp_telem;

// =============================================================================
// Module-static state (not on task stack)
// モジュール static 状態（タスクスタック上に置かない）
// =============================================================================

// Send queue
// 送信キュー
struct SendItem {
    uint8_t data[UNIFIED_MAX_SIZE];  // Max: unified packet ~950B
    uint16_t len;
};

static QueueHandle_t s_send_queue = nullptr;
static constexpr int SEND_QUEUE_SIZE = 4;

// Unified packet: accumulate 8 IMU cycles + sensor entries
// 統合パケット: 8 IMUサイクル + センサエントリを蓄積
static ImuEskfSample s_imu_buf[UNIFIED_BATCH_SIZE];
static PosVelSample s_posvel_buf[UNIFIED_BATCH_SIZE];
static RateRefFixed s_rate_ref_buf[UNIFIED_BATCH_SIZE];
static int s_unified_count = 0;  // 0..7, send when reaches 8

// Sensor entries accumulated during 8 cycles (variable part)
// 8サイクル中に蓄積されるセンサエントリ（可変部分）
struct PendingSensor {
    uint8_t id;
    uint8_t data[32];  // max single sensor sample size (CtrlRefSample=30B)
    uint8_t size;
};
static PendingSensor s_pending_sensors[32];  // max entries per unified packet
static int s_pending_count = 0;

// WebSocket batch packet
// WebSocket バッチパケット
static stampfly::TelemetryExtendedBatchPacket s_ws_pkt;

// Reusable SendItem for enqueue (avoid repeated stack allocation)
// enqueue 用の再利用 SendItem（スタック割り当ての繰り返しを避ける）
static SendItem s_send_item;

// =============================================================================
// Reset all UDP state for clean capture start
// クリーンなキャプチャ開始のため全 UDP 状態をリセット
// =============================================================================

static void resetUdpState()
{
    s_unified_count = 0;
    s_pending_count = 0;
    if (s_send_queue) xQueueReset(s_send_queue);
}

// =============================================================================
// Enqueue: copy packet to static SendItem, then to queue (non-blocking)
// エンキュー: パケットを static SendItem にコピーしてキューへ（ノンブロッキング）
// =============================================================================

static uint32_t s_enqueue_drops = 0;

static void enqueue(const void* pkt, size_t len)
{
    if (len > sizeof(s_send_item.data) || !s_send_queue) return;
    memcpy(s_send_item.data, pkt, len);
    s_send_item.len = (uint16_t)len;
    if (xQueueSend(s_send_queue, &s_send_item, 0) != pdTRUE) {
        s_enqueue_drops++;
    }
}

// =============================================================================
// Send task: dequeue and transmit (blocking sendto OK here)
// 送信タスク: デキューして送信（ブロッキング sendto OK）
// =============================================================================

static void TelemetrySendTask(void* pvParameters)
{
    ESP_LOGI(TAG, "TelemetrySendTask started");

    auto& udp_log = UDPLogServer::getInstance();

    SendItem item;
    uint32_t bytes_sent = 0;
    uint32_t stats_time = (uint32_t)(esp_timer_get_time() / 1000);

    while (true) {
        if (xQueueReceive(s_send_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (udp_log.isActive()) {
            int sent = udp_log.send(item.data, item.len);
            if (sent > 0) bytes_sent += sent;
        }

        // Bandwidth stats every 5 seconds
        // 帯域統計（5秒ごと）
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (now_ms - stats_time > 5000) {
            if (bytes_sent > 0) {
                ESP_LOGI(TAG, "UDP: %.1f KB/s, queue free: %d/%d, enq_drops: %lu",
                         (float)bytes_sent / 5.0f / 1024.0f,
                         (int)uxQueueSpacesAvailable(s_send_queue), SEND_QUEUE_SIZE,
                         s_enqueue_drops);
                bytes_sent = 0;
            }
            stats_time = now_ms;
        }
    }
}

// =============================================================================
// Fill helpers (read from ring buffers and state into packet samples)
// 充填ヘルパー（リングバッファと状態からパケットサンプルに読み込む）
// =============================================================================

static void fillImuEskf(ImuEskfSample& s, int idx, uint32_t ts)
{
    s.timestamp_us = ts;

    const auto& gyro = g_gyro_buf.raw_at(idx);
    const auto& accel = g_accel_buf.raw_at(idx);
    s.gyro_x = gyro.x;   s.gyro_y = gyro.y;   s.gyro_z = gyro.z;
    s.accel_x = accel.x;  s.accel_y = accel.y;  s.accel_z = accel.z;

    const auto& gyro_r = g_gyro_raw_buf.raw_at(idx);
    const auto& accel_r = g_accel_raw_buf.raw_at(idx);
    s.gyro_raw_x = gyro_r.x;  s.gyro_raw_y = gyro_r.y;  s.gyro_raw_z = gyro_r.z;
    s.accel_raw_x = accel_r.x; s.accel_raw_y = accel_r.y; s.accel_raw_z = accel_r.z;

    auto st = g_fusion.getESKF().getState();
    s.quat_w = st.orientation.w; s.quat_x = st.orientation.x;
    s.quat_y = st.orientation.y; s.quat_z = st.orientation.z;
    s.gyro_bias_x  = (int16_t)(st.gyro_bias.x  * 10000.0f);
    s.gyro_bias_y  = (int16_t)(st.gyro_bias.y  * 10000.0f);
    s.gyro_bias_z  = (int16_t)(st.gyro_bias.z  * 10000.0f);
    s.accel_bias_x = (int16_t)(st.accel_bias.x * 10000.0f);
    s.accel_bias_y = (int16_t)(st.accel_bias.y * 10000.0f);
    s.accel_bias_z = (int16_t)(st.accel_bias.z * 10000.0f);
}

static void fillPosVel(PosVelSample& s, uint32_t ts)
{
    s.timestamp_us = ts;
    auto st = g_fusion.getESKF().getState();
    s.pos_x = st.position.x; s.pos_y = st.position.y; s.pos_z = st.position.z;
    s.vel_x = st.velocity.x; s.vel_y = st.velocity.y; s.vel_z = st.velocity.z;
}

static void fillWsSample(stampfly::ExtendedSample& s, int idx, uint32_t imu_ts,
                          stampfly::StampFlyState& state)
{
    s.timestamp_us = esp_timer_get_time();

    const auto& gyro = g_gyro_buf.raw_at(idx);
    const auto& accel = g_accel_buf.raw_at(idx);
    s.gyro_x = gyro.x;  s.gyro_y = gyro.y;  s.gyro_z = gyro.z;
    s.accel_x = accel.x; s.accel_y = accel.y; s.accel_z = accel.z;

    stampfly::Vec3 ac, gc;
    state.getIMUCorrected(ac, gc);
    s.gyro_corrected_x = gc.x; s.gyro_corrected_y = gc.y; s.gyro_corrected_z = gc.z;

    state.getControlInput(s.ctrl_throttle, s.ctrl_roll, s.ctrl_pitch, s.ctrl_yaw);

    auto st = g_fusion.getESKF().getState();
    s.quat_w = st.orientation.w; s.quat_x = st.orientation.x;
    s.quat_y = st.orientation.y; s.quat_z = st.orientation.z;
    s.pos_x = st.position.x; s.pos_y = st.position.y; s.pos_z = st.position.z;
    s.vel_x = st.velocity.x; s.vel_y = st.velocity.y; s.vel_z = st.velocity.z;
    s.gyro_bias_x  = (int16_t)(st.gyro_bias.x  * 10000.0f);
    s.gyro_bias_y  = (int16_t)(st.gyro_bias.y  * 10000.0f);
    s.gyro_bias_z  = (int16_t)(st.gyro_bias.z  * 10000.0f);
    s.accel_bias_x = (int16_t)(st.accel_bias.x * 10000.0f);
    s.accel_bias_y = (int16_t)(st.accel_bias.y * 10000.0f);
    s.accel_bias_z = (int16_t)(st.accel_bias.z * 10000.0f);
    s.eskf_status = state.isESKFInitialized() ? 0x01 : 0x00;
    s.accel_corrected_x = accel.x - st.accel_bias.x;
    s.accel_corrected_y = accel.y - st.accel_bias.y;
    s.accel_corrected_z = accel.z - st.accel_bias.z;
    memset(s.padding1, 0, sizeof(s.padding1));

    float ba, bp;
    state.getBaroData(ba, bp);
    s.baro_altitude = ba; s.baro_pressure = bp;

    float tb, tf;
    state.getToFData(tb, tf);
    s.tof_bottom = tb; s.tof_front = tf;
    uint8_t sb, sf_s;
    state.getToFStatus(sb, sf_s);
    s.tof_bottom_status = sb; s.tof_front_status = sf_s;

    int16_t fdx, fdy; uint8_t fsq;
    state.getFlowRawData(fdx, fdy, fsq);
    s.flow_x = fdx; s.flow_y = fdy; s.flow_quality = fsq;

    s.padding2 = 0;
    stampfly::Vec3 mag;
    state.getMagData(mag);
    s.mag_x = mag.x; s.mag_y = mag.y; s.mag_z = mag.z;

    const auto& ar = g_accel_raw_buf.raw_at(idx);
    const auto& gr = g_gyro_raw_buf.raw_at(idx);
    s.gyro_raw_x = gr.x;  s.gyro_raw_y = gr.y;  s.gyro_raw_z = gr.z;
    s.accel_raw_x = ar.x;  s.accel_raw_y = ar.y;  s.accel_raw_z = ar.z;

    s.imu_timestamp_us = imu_ts;
    s.baro_timestamp_us = g_baro_last_timestamp_us;
    s.tof_timestamp_us = g_tof_bottom_last_timestamp_us;
    s.mag_timestamp_us = g_mag_last_timestamp_us;
    s.flow_timestamp_us = g_flow_last_timestamp_us;
    s.padding3 = 0;
}

// =============================================================================
// UDP mode: collect one cycle of sensor data
// UDP モード: 1サイクル分のセンサデータを収集
// =============================================================================

/// Add a sensor entry to pending list
/// センサエントリを保留リストに追加
static void addSensorEntry(uint8_t id, const void* data, uint8_t size)
{
    if (s_pending_count >= 32 || size > sizeof(PendingSensor::data)) return;
    auto& e = s_pending_sensors[s_pending_count++];
    e.id = id;
    e.size = size;
    memcpy(e.data, data, size);
}

/// Build and enqueue the unified packet from accumulated data
/// 蓄積データから統合パケットを構築してキューに投入
static void sendUnifiedPacket()
{
    static UnifiedPacket pkt;
    pkt.reset();

    // Header
    PacketHeader hdr;
    hdr.packet_id = PKT_UNIFIED;
    hdr.sequence = 0;  // TODO: add sequence counter if needed
    hdr.sample_count = UNIFIED_BATCH_SIZE;
    pkt.append(&hdr, sizeof(hdr));

    // Fixed part: 8× IMU+ESKF + 8× PosVel + 8× RateRef
    pkt.append(s_imu_buf, sizeof(s_imu_buf));
    pkt.append(s_posvel_buf, sizeof(s_posvel_buf));
    pkt.append(s_rate_ref_buf, sizeof(s_rate_ref_buf));

    // Entry count
    uint8_t entry_count = (uint8_t)s_pending_count;
    pkt.append(&entry_count, 1);

    // Variable part: sensor entries
    for (int i = 0; i < s_pending_count; i++) {
        SensorEntryHeader eh;
        eh.sensor_id = s_pending_sensors[i].id;
        eh.data_size = s_pending_sensors[i].size;
        pkt.append(&eh, sizeof(eh));
        pkt.append(s_pending_sensors[i].data, s_pending_sensors[i].size);
    }

    // Checksum
    pkt.finalize();

    // Enqueue
    enqueue(pkt.buf, pkt.len);

    // Reset for next cycle
    s_unified_count = 0;
    s_pending_count = 0;
}

static void udpCollectCycle(int read_idx, uint32_t imu_ts,
                            stampfly::StampFlyState& state,
                            int cycle, int& status_cnt,
                            uint32_t& last_flow_ts, uint32_t& last_tof_ts,
                            uint32_t& last_tof_front_ts,
                            uint32_t& last_baro_ts, uint32_t& last_mag_ts)
{
    // 400Hz: accumulate IMU + PosVel + RateRef into buffers
    // 400Hz: IMU + PosVel + RateRef をバッファに蓄積
    fillImuEskf(s_imu_buf[s_unified_count], read_idx, imu_ts);
    fillPosVel(s_posvel_buf[s_unified_count], imu_ts);

    // 400Hz: Rate reference targets from control loop
    // 400Hz: 制御ループからのレート目標値
    {
        float ar, ap, rr, rp, ry;
        state.getControlRef(ar, ap, rr, rp, ry);
        auto& rrf = s_rate_ref_buf[s_unified_count];
        rrf.rate_ref_roll  = static_cast<int16_t>(rr * 1000.0f);
        rrf.rate_ref_pitch = static_cast<int16_t>(rp * 1000.0f);
        rrf.rate_ref_yaw   = static_cast<int16_t>(ry * 1000.0f);
    }

    // Sensor data: detect new data via timestamp change, add as entries
    // センサデータ: タイムスタンプ変化で検出、エントリとして追加

    // Control (~50Hz)
    if ((cycle & 7) == 0) {
        ControlSample cs;
        cs.timestamp_us = imu_ts;
        state.getControlInput(cs.throttle, cs.roll, cs.pitch, cs.yaw);
        addSensorEntry(PKT_CONTROL, &cs, sizeof(cs));

        // Angle reference + flight mode (~50Hz)
        // 角度目標値 + フライトモード（50Hz）
        CtrlRefSample cr;
        cr.timestamp_us = imu_ts;
        cr.flight_mode = static_cast<uint8_t>(state.getFlightMode());
        cr.reserved = 0;
        float ar, ap, rr, rp, ry;
        state.getControlRef(ar, ap, rr, rp, ry);
        cr.angle_ref_roll  = static_cast<int16_t>(ar * 10000.0f);
        cr.angle_ref_pitch = static_cast<int16_t>(ap * 10000.0f);
        cr.total_thrust = state.getTotalThrust();
        state.getMotorDuties(cr.motor_duty);
        addSensorEntry(PKT_CTRL_REF, &cr, sizeof(cr));
    }

    // Optical Flow (~100Hz)
    if (uint32_t ts = g_flow_last_timestamp_us; ts != last_flow_ts && ts != 0) {
        last_flow_ts = ts;
        FlowSample fs;
        fs.timestamp_us = ts;
        int16_t dx, dy; uint8_t sq;
        state.getFlowRawData(dx, dy, sq);
        fs.flow_dx = dx; fs.flow_dy = dy; fs.quality = sq;
        addSensorEntry(PKT_FLOW, &fs, sizeof(fs));
    }

    // ToF Bottom (~30Hz)
    if (uint32_t ts = g_tof_bottom_last_timestamp_us; ts != last_tof_ts && ts != 0) {
        last_tof_ts = ts;
        ToFSingleSample s;
        s.timestamp_us = ts;
        float tb, tf;
        state.getToFData(tb, tf);
        uint8_t sb, sf_s;
        state.getToFStatus(sb, sf_s);
        s.distance = tb; s.status = sb;
        addSensorEntry(PKT_TOF_BOTTOM, &s, sizeof(s));
    }

    // ToF Front (~30Hz)
    if (uint32_t ts = g_tof_front_last_timestamp_us; ts != last_tof_front_ts && ts != 0) {
        last_tof_front_ts = ts;
        ToFSingleSample s;
        s.timestamp_us = ts;
        float tb, tf;
        state.getToFData(tb, tf);
        uint8_t sb, sf_s;
        state.getToFStatus(sb, sf_s);
        s.distance = tf; s.status = sf_s;
        addSensorEntry(PKT_TOF_FRONT, &s, sizeof(s));
    }

    // Barometer (~50Hz)
    if (uint32_t ts = g_baro_last_timestamp_us; ts != last_baro_ts && ts != 0) {
        last_baro_ts = ts;
        BaroSample bs;
        bs.timestamp_us = ts;
        float ba, bp;
        state.getBaroData(ba, bp);
        bs.altitude = ba; bs.pressure = bp;
        addSensorEntry(PKT_BARO, &bs, sizeof(bs));
    }

    // Magnetometer (~25Hz)
    if (uint32_t ts = g_mag_last_timestamp_us; ts != last_mag_ts && ts != 0) {
        last_mag_ts = ts;
        MagSample ms;
        ms.timestamp_us = ts;
        stampfly::Vec3 md;
        state.getMagData(md);
        ms.mag_x = md.x; ms.mag_y = md.y; ms.mag_z = md.z;
        addSensorEntry(PKT_MAG, &ms, sizeof(ms));
    }

    // Advance unified counter, send when 8 cycles accumulated
    // 統合カウンタを進め、8サイクル蓄積で送信
    s_unified_count++;
    if (s_unified_count >= UNIFIED_BATCH_SIZE) {
        sendUnifiedPacket();
    }

    // 1Hz: Status (sent as separate small packet)
    // 1Hz: ステータス（別パケットで送信）
    if (++status_cnt >= 400) {
        status_cnt = 0;
        StatusPacket sp = {};
        sp.header.packet_id = PKT_STATUS;
        sp.header.sample_count = 1;
        sp.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
        sp.voltage = state.getVoltage();
        sp.flight_state = (uint8_t)state.getFlightState();
        sp.eskf_status = state.isESKFInitialized() ? 0x01 : 0x00;

        // PID gains from rate controller (may be adjusted at runtime via CLI)
        // レートコントローラのPIDゲイン（CLI経由でランタイム変更される場合あり）
        if (g_rate_controller_ptr) {
            sp.pid_roll_kp  = g_rate_controller_ptr->roll_pid.getKp();
            sp.pid_roll_ti  = g_rate_controller_ptr->roll_pid.getTi();
            sp.pid_roll_td  = g_rate_controller_ptr->roll_pid.getTd();
            sp.pid_pitch_kp = g_rate_controller_ptr->pitch_pid.getKp();
            sp.pid_pitch_ti = g_rate_controller_ptr->pitch_pid.getTi();
            sp.pid_pitch_td = g_rate_controller_ptr->pitch_pid.getTd();
            sp.pid_yaw_kp   = g_rate_controller_ptr->yaw_pid.getKp();
            sp.pid_yaw_ti   = g_rate_controller_ptr->yaw_pid.getTi();
            sp.pid_yaw_td   = g_rate_controller_ptr->yaw_pid.getTd();
        }

        sp.checksum = computeChecksum(&sp, sizeof(sp));
        enqueue(&sp, sizeof(sp));
    }
}

// =============================================================================
// Main telemetry task
// メインテレメトリタスク
// =============================================================================

void TelemetryTask(void* pvParameters)
{
    ESP_LOGI(TAG, "TelemetryTask started");

    auto& telemetry = stampfly::Telemetry::getInstance();
    auto& state = stampfly::StampFlyState::getInstance();
    auto& udp_log = UDPLogServer::getInstance();

    // Create send queue
    // 送信キュー作成
    s_send_queue = xQueueCreate(SEND_QUEUE_SIZE, sizeof(SendItem));
    if (!s_send_queue) {
        ESP_LOGE(TAG, "Failed to create send queue");
        vTaskDelete(nullptr);
        return;
    }

    // Create send task
    // 送信タスク作成
    xTaskCreatePinnedToCore(TelemetrySendTask, "TelemSend", 4096,
                            nullptr, 12, nullptr, 0);

    // Local state (small — only counters and indices)
    // ローカル状態（小さい — カウンタとインデックスのみ）
    int read_idx = 0;
    int ws_batch_idx = 0;
    int ws_decim = 0;
    int udp_cycle = 0;
    int status_cnt = 0;
    uint32_t last_flow_ts = 0, last_tof_ts = 0, last_tof_front_ts = 0;
    uint32_t last_baro_ts = 0, last_mag_ts = 0;
    bool was_udp_active = false;

    constexpr int WS_DECIMATION = 8;  // 400Hz → 50Hz
    constexpr int MAX_LAG = 40;       // Max samples behind before catch-up

    ESP_LOGI(TAG, "Queue: %d × %dB, send task on core 0",
             SEND_QUEUE_SIZE, (int)sizeof(SendItem));

    // Wait for IMU buffers to fill
    // IMU バッファが埋まるのを待つ
    vTaskDelay(pdMS_TO_TICKS(100));
    read_idx = g_accel_buf.raw_index();

    // =========================================================================
    // Main loop (400Hz, driven by IMU semaphore)
    // メインループ（400Hz、IMU セマフォ駆動）
    // =========================================================================
    while (true) {
        if (xSemaphoreTake(g_telemetry_imu_semaphore, pdMS_TO_TICKS(10)) != pdTRUE) {
            continue;
        }

        // --- Catch-up: prevent read_idx from falling too far behind ---
        // --- 追いつき: read_idx が大きく遅れるのを防ぐ ---
        {
            int head = g_accel_buf.raw_index();
            int lag = (head - read_idx + IMU_BUFFER_SIZE) % IMU_BUFFER_SIZE;
            if (lag > MAX_LAG) {
                read_idx = (head - 4 + IMU_BUFFER_SIZE) % IMU_BUFFER_SIZE;
                ws_batch_idx = 0;
                s_unified_count = 0;
                s_pending_count = 0;
            }
        }

        // --- Overrun fallback ---
        if (g_accel_buf.is_overrun(read_idx)) {
            read_idx = g_accel_buf.safe_read_index(10);
        }

        uint32_t imu_ts = g_accel_raw_buf.raw_timestamp_at(read_idx);
        bool udp_active = udp_log.isActive();

        // --- Mode transition: reset state on UDP start ---
        // --- モード遷移: UDP 開始時に状態をリセット ---
        if (udp_active && !was_udp_active) {
            resetUdpState();
            udp_cycle = 0;
            status_cnt = 0;
            last_flow_ts = last_tof_ts = last_tof_front_ts = last_baro_ts = last_mag_ts = 0;
            ESP_LOGI(TAG, "UDP capture started — state reset");
        }
        was_udp_active = udp_active;

        // --- UDP full-data mode ---
        if (udp_active) {
            udpCollectCycle(read_idx, imu_ts, state, ++udp_cycle, status_cnt,
                            last_flow_ts, last_tof_ts, last_tof_front_ts,
                            last_baro_ts, last_mag_ts);
        }
        // --- WebSocket visualization mode (50Hz) ---
        else if (telemetry.hasClients()) {
            if (++ws_decim >= WS_DECIMATION) {
                ws_decim = 0;
                fillWsSample(s_ws_pkt.samples[ws_batch_idx], read_idx, imu_ts, state);

                if (++ws_batch_idx >= stampfly::FFT_BATCH_SIZE) {
                    s_ws_pkt.header = 0xBD;
                    s_ws_pkt.packet_type = 0x32;
                    s_ws_pkt.sample_count = stampfly::FFT_BATCH_SIZE;
                    s_ws_pkt.reserved = 0;
                    s_ws_pkt.checksum = 0;
                    const uint8_t* d = reinterpret_cast<const uint8_t*>(&s_ws_pkt);
                    constexpr size_t co = offsetof(stampfly::TelemetryExtendedBatchPacket, checksum);
                    uint8_t xv = 0;
                    for (size_t i = 0; i < co; i++) xv ^= d[i];
                    s_ws_pkt.checksum = xv;
                    telemetry.broadcast(&s_ws_pkt, sizeof(s_ws_pkt));
                    ws_batch_idx = 0;
                }
            }
        }

        // Advance read index
        // 読み取りインデックスを進める
        read_idx = (read_idx + 1) % IMU_BUFFER_SIZE;
    }
}
