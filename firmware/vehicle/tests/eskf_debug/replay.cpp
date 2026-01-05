/**
 * @file replay.cpp
 * @brief ESKF Replay Tool - Run ESKF on recorded sensor data
 *
 * デバイスコード（main.cpp + eskf.cpp）準拠の再実装版
 *
 * Usage:
 *   ./eskf_replay <input.bin> <output.csv> [--verbose]
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include "eskf.hpp"

using namespace stampfly;
using namespace stampfly::math;

// ============================================================================
// Binary Log Packet Structures (must match cli.hpp)
// ============================================================================

// ==========================================================================
// V1 Packet Structure - DEPRECATED (commented out for future removal)
// ==========================================================================
#if 0
#pragma pack(push, 1)
struct BinaryLogPacketV1 {
    uint8_t header[2];      // 0xAA, 0x55
    uint32_t timestamp_ms;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float pressure;
    float baro_alt;
    float tof_bottom;
    float tof_front;
    int16_t flow_dx;
    int16_t flow_dy;
    uint8_t flow_squal;
    uint8_t checksum;
};
#pragma pack(pop)
static_assert(sizeof(BinaryLogPacketV1) == 64, "V1 Packet size mismatch");
#endif

#pragma pack(push, 1)
struct BinaryLogPacketV2 {
    uint8_t header[2];      // 0xAA, 0x56 (V2)
    uint32_t timestamp_ms;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float pressure;
    float baro_alt;
    float tof_bottom;
    float tof_front;
    int16_t flow_dx;
    int16_t flow_dy;
    uint8_t flow_squal;
    // ESKF estimates from device
    float pos_x, pos_y, pos_z;
    float vel_x, vel_y, vel_z;
    float roll, pitch, yaw;
    float gyro_bias_z;
    float accel_bias_x, accel_bias_y;
    // Status + metadata
    uint8_t eskf_status;
    float baro_ref_alt;
    uint8_t reserved[11];
    uint8_t checksum;
};
#pragma pack(pop)

static_assert(sizeof(BinaryLogPacketV2) == 128, "V2 Packet size mismatch");

// ============================================================================
// Sensor Data Structure
// ============================================================================

struct SensorData {
    uint32_t timestamp_ms;
    // IMU (フィルタ済み、m/s² と rad/s)
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    // Magnetometer (キャリブレーション済み、uT)
    float mag_x, mag_y, mag_z;
    // Barometer
    float pressure;
    float baro_alt;
    float baro_ref_alt;
    // ToF
    float tof_bottom;
    float tof_front;
    // Optical Flow (raw counts)
    int16_t flow_dx;
    int16_t flow_dy;
    uint8_t flow_squal;
    // Device ESKF estimates (V2 only)
    float dev_pos_x, dev_pos_y, dev_pos_z;
    float dev_vel_x, dev_vel_y, dev_vel_z;
    float dev_roll, dev_pitch, dev_yaw;
    float gyro_bias_z;
    float accel_bias_x, accel_bias_y;
    bool has_device_eskf;
};

// ============================================================================
// Checksum Verification
// ============================================================================

// ==========================================================================
// V1 Checksum - DEPRECATED (commented out for future removal)
// ==========================================================================
#if 0
bool verify_checksum_v1(const BinaryLogPacketV1& pkt)
{
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t checksum = 0;
    for (int i = 2; i < 63; i++) {
        checksum ^= data[i];
    }
    return checksum == pkt.checksum;
}
#endif

bool verify_checksum_v2(const BinaryLogPacketV2& pkt)
{
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t checksum = 0;
    for (int i = 2; i < 127; i++) {
        checksum ^= data[i];
    }
    return checksum == pkt.checksum;
}

// ============================================================================
// Log File Loading
// ============================================================================

int detect_log_format(const char* filename)
{
    FILE* f = fopen(filename, "rb");
    if (!f) return 0;

    uint8_t header[2];
    if (fread(header, 1, 2, f) != 2) {
        fclose(f);
        return 0;
    }
    fclose(f);

    // V1 detection disabled - DEPRECATED
    // if (header[0] == 0xAA && header[1] == 0x55) return 1;
    if (header[0] == 0xAA && header[1] == 0x56) return 2;
    return 0;
}

std::vector<SensorData> load_log_file(const char* filename, int& version, bool quiet = false)
{
    std::vector<SensorData> packets;

    version = detect_log_format(filename);
    if (version == 0) {
        fprintf(stderr, "Error: Unknown log format in %s\n", filename);
        return packets;
    }

    FILE* f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "Error: Cannot open file %s\n", filename);
        return packets;
    }

    if (!quiet) {
        printf("Log format: V%d (%d bytes/packet)\n", version, 128);
    }

    int checksum_errors = 0;

    // ==========================================================================
    // V1 Parsing - DEPRECATED (commented out for future removal)
    // ==========================================================================
#if 0
    if (version == 1) {
        BinaryLogPacketV1 pkt;
        while (fread(&pkt, sizeof(pkt), 1, f) == 1) {
            if (pkt.header[0] == 0xAA && pkt.header[1] == 0x55) {
                if (verify_checksum_v1(pkt)) {
                    SensorData data = {};
                    data.timestamp_ms = pkt.timestamp_ms;
                    data.accel_x = pkt.accel_x;
                    data.accel_y = pkt.accel_y;
                    data.accel_z = pkt.accel_z;
                    data.gyro_x = pkt.gyro_x;
                    data.gyro_y = pkt.gyro_y;
                    data.gyro_z = pkt.gyro_z;
                    data.mag_x = pkt.mag_x;
                    data.mag_y = pkt.mag_y;
                    data.mag_z = pkt.mag_z;
                    data.pressure = pkt.pressure;
                    data.baro_alt = pkt.baro_alt;
                    data.baro_ref_alt = 0.0f;
                    data.tof_bottom = pkt.tof_bottom;
                    data.tof_front = pkt.tof_front;
                    data.flow_dx = pkt.flow_dx;
                    data.flow_dy = pkt.flow_dy;
                    data.flow_squal = pkt.flow_squal;
                    data.has_device_eskf = false;
                    packets.push_back(data);
                } else {
                    checksum_errors++;
                }
            }
        }
    } else
#endif
    {
        BinaryLogPacketV2 pkt;
        while (fread(&pkt, sizeof(pkt), 1, f) == 1) {
            if (pkt.header[0] == 0xAA && pkt.header[1] == 0x56) {
                if (verify_checksum_v2(pkt)) {
                    SensorData data = {};
                    data.timestamp_ms = pkt.timestamp_ms;
                    data.accel_x = pkt.accel_x;
                    data.accel_y = pkt.accel_y;
                    data.accel_z = pkt.accel_z;
                    data.gyro_x = pkt.gyro_x;
                    data.gyro_y = pkt.gyro_y;
                    data.gyro_z = pkt.gyro_z;
                    data.mag_x = pkt.mag_x;
                    data.mag_y = pkt.mag_y;
                    data.mag_z = pkt.mag_z;
                    data.pressure = pkt.pressure;
                    data.baro_alt = pkt.baro_alt;
                    data.baro_ref_alt = pkt.baro_ref_alt;
                    data.tof_bottom = pkt.tof_bottom;
                    data.tof_front = pkt.tof_front;
                    data.flow_dx = pkt.flow_dx;
                    data.flow_dy = pkt.flow_dy;
                    data.flow_squal = pkt.flow_squal;
                    data.dev_pos_x = pkt.pos_x;
                    data.dev_pos_y = pkt.pos_y;
                    data.dev_pos_z = pkt.pos_z;
                    data.dev_vel_x = pkt.vel_x;
                    data.dev_vel_y = pkt.vel_y;
                    data.dev_vel_z = pkt.vel_z;
                    data.dev_roll = pkt.roll;
                    data.dev_pitch = pkt.pitch;
                    data.dev_yaw = pkt.yaw;
                    data.gyro_bias_z = pkt.gyro_bias_z;
                    data.accel_bias_x = pkt.accel_bias_x;
                    data.accel_bias_y = pkt.accel_bias_y;
                    data.has_device_eskf = (pkt.eskf_status != 0);
                    packets.push_back(data);
                } else {
                    checksum_errors++;
                }
            }
        }
    }

    fclose(f);
    if (!quiet) {
        printf("Loaded %zu packets", packets.size());
        if (checksum_errors > 0) {
            printf(" (%d checksum errors)", checksum_errors);
        }
        printf("\n");
    }

    return packets;
}

// ============================================================================
// Main
// ============================================================================

void print_usage(const char* prog)
{
    printf("Usage: %s <input.bin> <output.csv> [options]\n", prog);
    printf("\nRuns ESKF on recorded sensor data.\n");
    printf("\nOptions:\n");
    printf("  --verbose           Print progress\n");
    printf("  --quiet             Minimal output (for optimization)\n");
    printf("  --gyro_noise=N      Gyro noise [rad/s/√Hz] (default: 0.001)\n");
    printf("  --accel_noise=N     Accel noise [m/s²/√Hz] (default: 0.1)\n");
    printf("  --flow_noise=N      Flow noise [m/s] (default: 0.1)\n");
    printf("  --tof_noise=N       ToF noise [m] (default: 0.002)\n");
    printf("  --baro_noise=N      Baro noise [m] (default: 0.1)\n");
    printf("  --mag_noise=N       Mag noise [uT] (default: 0.3)\n");
    printf("  --accel_att_noise=N Accel attitude noise [m/s²] (default: 1.0)\n");
    printf("  --gyro_bias_noise=N Gyro bias RW noise (default: 0.00005)\n");
    printf("  --accel_bias_noise=N Accel bias RW noise (default: 0.001)\n");
    printf("  --flow_rad_per_pixel=N Flow calibration [rad/pixel] (default: 0.00205)\n");
    printf("  --att_update_mode=N Attitude update mode (0=accel_mag, 1=adaptive_R, 2=gyro) (default: 1)\n");
    printf("  --k_adaptive=N      Adaptive R coefficient (mode 1) (default: 100)\n");
    printf("  --gyro_att_threshold=N Gyro threshold [rad/s] (mode 2) (default: 0.5)\n");
    printf("  --flow_tilt_cos_threshold=N Flow tilt cos threshold (default: 0.866 = cos(30deg))\n");
    printf("  --no_mag                Disable magnetometer updates\n");
}

// Helper to parse --key=value arguments
float parse_float_arg(const char* arg, const char* prefix, float default_val)
{
    size_t prefix_len = strlen(prefix);
    if (strncmp(arg, prefix, prefix_len) == 0) {
        return std::atof(arg + prefix_len);
    }
    return default_val;
}

int main(int argc, char* argv[])
{
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    const char* input_file = argv[1];
    const char* output_file = argv[2];
    bool verbose = false;
    bool quiet = false;

    // Default parameters from ESKF::Config::defaultConfig()
    ESKF::Config default_cfg = ESKF::Config::defaultConfig();
    float gyro_noise = default_cfg.gyro_noise;
    float accel_noise = default_cfg.accel_noise;
    float flow_noise = default_cfg.flow_noise;
    float tof_noise = default_cfg.tof_noise;
    float baro_noise = default_cfg.baro_noise;
    float mag_noise = default_cfg.mag_noise;
    float accel_att_noise = default_cfg.accel_att_noise;
    float gyro_bias_noise = default_cfg.gyro_bias_noise;
    float accel_bias_noise = default_cfg.accel_bias_noise;
    float flow_rad_per_pixel = default_cfg.flow_rad_per_pixel;
    float flow_scale_x = default_cfg.flow_cam_to_body[0];
    float flow_scale_y = default_cfg.flow_cam_to_body[3];
    float flow_offset_x = default_cfg.flow_offset[0];
    float flow_offset_y = default_cfg.flow_offset[1];
    int att_update_mode = default_cfg.att_update_mode;
    float k_adaptive = default_cfg.k_adaptive;
    float gyro_att_threshold = default_cfg.gyro_att_threshold;
    float flow_tilt_cos_threshold = default_cfg.flow_tilt_cos_threshold;
    bool mag_enabled = true;

    for (int i = 3; i < argc; i++) {
        if (strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "--quiet") == 0) {
            quiet = true;
        } else if (strncmp(argv[i], "--gyro_noise=", 13) == 0) {
            gyro_noise = std::atof(argv[i] + 13);
        } else if (strncmp(argv[i], "--accel_noise=", 14) == 0) {
            accel_noise = std::atof(argv[i] + 14);
        } else if (strncmp(argv[i], "--flow_noise=", 13) == 0) {
            flow_noise = std::atof(argv[i] + 13);
        } else if (strncmp(argv[i], "--tof_noise=", 12) == 0) {
            tof_noise = std::atof(argv[i] + 12);
        } else if (strncmp(argv[i], "--baro_noise=", 13) == 0) {
            baro_noise = std::atof(argv[i] + 13);
        } else if (strncmp(argv[i], "--mag_noise=", 12) == 0) {
            mag_noise = std::atof(argv[i] + 12);
        } else if (strncmp(argv[i], "--accel_att_noise=", 18) == 0) {
            accel_att_noise = std::atof(argv[i] + 18);
        } else if (strncmp(argv[i], "--gyro_bias_noise=", 18) == 0) {
            gyro_bias_noise = std::atof(argv[i] + 18);
        } else if (strncmp(argv[i], "--accel_bias_noise=", 19) == 0) {
            accel_bias_noise = std::atof(argv[i] + 19);
        } else if (strncmp(argv[i], "--flow_rad_per_pixel=", 21) == 0) {
            flow_rad_per_pixel = std::atof(argv[i] + 21);
        } else if (strncmp(argv[i], "--flow_scale_x=", 15) == 0) {
            flow_scale_x = std::atof(argv[i] + 15);
        } else if (strncmp(argv[i], "--flow_scale_y=", 15) == 0) {
            flow_scale_y = std::atof(argv[i] + 15);
        } else if (strncmp(argv[i], "--flow_offset_x=", 16) == 0) {
            flow_offset_x = std::atof(argv[i] + 16);
        } else if (strncmp(argv[i], "--flow_offset_y=", 16) == 0) {
            flow_offset_y = std::atof(argv[i] + 16);
        } else if (strncmp(argv[i], "--att_update_mode=", 18) == 0) {
            att_update_mode = std::atoi(argv[i] + 18);
        } else if (strncmp(argv[i], "--k_adaptive=", 13) == 0) {
            k_adaptive = std::atof(argv[i] + 13);
        } else if (strncmp(argv[i], "--gyro_att_threshold=", 21) == 0) {
            gyro_att_threshold = std::atof(argv[i] + 21);
        } else if (strncmp(argv[i], "--flow_tilt_cos_threshold=", 26) == 0) {
            flow_tilt_cos_threshold = std::atof(argv[i] + 26);
        } else if (strcmp(argv[i], "--no_mag") == 0) {
            mag_enabled = false;
        }
    }

    if (!quiet) {
        printf("=== ESKF Replay Tool ===\n");
        printf("Input: %s\n", input_file);
        printf("Output: %s\n", output_file);
    }

    // ========================================================================
    // Load log file
    // ========================================================================
    int log_version = 0;
    auto packets = load_log_file(input_file, log_version, quiet);
    if (packets.empty()) {
        fprintf(stderr, "No valid packets found\n");
        return 1;
    }
    bool has_device_eskf = (log_version == 2);

    // ========================================================================
    // Initialize ESKF with custom parameters
    // ========================================================================
    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();

    // Apply command line parameters
    config.gyro_noise = gyro_noise;
    config.accel_noise = accel_noise;
    config.flow_noise = flow_noise;
    config.tof_noise = tof_noise;
    config.baro_noise = baro_noise;
    config.mag_noise = mag_noise;
    config.accel_att_noise = accel_att_noise;
    config.gyro_bias_noise = gyro_bias_noise;
    config.accel_bias_noise = accel_bias_noise;
    config.flow_rad_per_pixel = flow_rad_per_pixel;
    config.flow_cam_to_body[0] = flow_scale_x;  // c2b_xx
    config.flow_cam_to_body[3] = flow_scale_y;  // c2b_yy
    config.flow_offset[0] = flow_offset_x;
    config.flow_offset[1] = flow_offset_y;
    config.att_update_mode = att_update_mode;
    config.k_adaptive = k_adaptive;
    config.gyro_att_threshold = gyro_att_threshold;
    config.flow_tilt_cos_threshold = flow_tilt_cos_threshold;

    // 地磁気更新設定
    config.mag_enabled = mag_enabled;
    eskf.init(config);
    eskf.reset();

    // ========================================================================
    // Restore biases from device log (V2 only)
    // ========================================================================
    if (has_device_eskf && packets.size() > 0) {
        // ジャイロバイアスを復元（デバイスと同じ初期状態にする）
        Vector3 initial_gyro_bias(0.0f, 0.0f, packets[0].gyro_bias_z);
        eskf.setGyroBias(initial_gyro_bias);
        if (!quiet) {
            printf("Gyro bias Z restored: %.6f rad/s\n", packets[0].gyro_bias_z);
        }

        // 加速度バイアスを復元
        Vector3 initial_accel_bias(packets[0].accel_bias_x, packets[0].accel_bias_y, 0.0f);
        eskf.setAccelBias(initial_accel_bias);
        if (!quiet) {
            printf("Accel bias restored: (%.4f, %.4f) m/s²\n",
                   packets[0].accel_bias_x, packets[0].accel_bias_y);
        }
    }

    // ========================================================================
    // Baro reference altitude
    // ========================================================================
    float baro_alt_reference = 0.0f;
    if (has_device_eskf && std::abs(packets[0].baro_ref_alt) > 0.001f) {
        baro_alt_reference = packets[0].baro_ref_alt;
        if (!quiet) {
            printf("Baro reference (from device): %.3f m\n", baro_alt_reference);
        }
    } else {
        // 最初のパケットから計算
        baro_alt_reference = packets[0].baro_alt;
        if (std::abs(baro_alt_reference) < 0.001f && packets[0].pressure > 80000.0f) {
            constexpr float P0 = 101325.0f;
            baro_alt_reference = 44330.0f * (1.0f - std::pow(packets[0].pressure / P0, 0.1903f));
        }
        if (!quiet) {
            printf("Baro reference (calculated): %.3f m\n", baro_alt_reference);
        }
    }

    // ========================================================================
    // Mag reference initialization (device uses 100 samples = 1 second)
    // ========================================================================
    constexpr int MAG_REF_SAMPLES = 100;  // デバイスと同じ
    {
        Vector3 mag_sum = Vector3::zero();
        int mag_sample_count = 0;
        for (size_t i = 0; i < packets.size() && mag_sample_count < MAG_REF_SAMPLES; i++) {
            const auto& pkt = packets[i];
            Vector3 mag(pkt.mag_x, pkt.mag_y, pkt.mag_z);
            float mag_norm = std::sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
            if (mag_norm > 10.0f) {
                mag_sum += mag;
                mag_sample_count++;
            }
        }
        if (mag_sample_count > 0) {
            Vector3 mag_avg = mag_sum * (1.0f / mag_sample_count);
            eskf.setMagReference(mag_avg);
            if (!quiet) {
                printf("Mag reference (%d samples): (%.1f, %.1f, %.1f) uT\n",
                       mag_sample_count, mag_avg.x, mag_avg.y, mag_avg.z);
            }
        } else if (!quiet) {
            printf("Warning: No valid mag samples for reference\n");
        }
    }

    // ========================================================================
    // Open output CSV
    // ========================================================================
    FILE* out = fopen(output_file, "w");
    if (!out) {
        fprintf(stderr, "Error: Cannot create output file %s\n", output_file);
        return 1;
    }

    // CSV header
    fprintf(out, "timestamp_ms,dt_ms,");
    fprintf(out, "pos_x,pos_y,pos_z,");
    fprintf(out, "vel_x,vel_y,vel_z,");
    fprintf(out, "roll_deg,pitch_deg,yaw_deg,");
    fprintf(out, "gyro_bias_x,gyro_bias_y,gyro_bias_z,");
    fprintf(out, "accel_bias_x,accel_bias_y,accel_bias_z,");
    fprintf(out, "raw_accel_x,raw_accel_y,raw_accel_z,");
    fprintf(out, "raw_gyro_x,raw_gyro_y,raw_gyro_z,");
    fprintf(out, "raw_mag_x,raw_mag_y,raw_mag_z,");
    fprintf(out, "raw_baro_alt,raw_tof,raw_flow_squal,");
    fprintf(out, "raw_flow_dx,raw_flow_dy");
    if (has_device_eskf) {
        fprintf(out, ",dev_pos_x,dev_pos_y,dev_pos_z,");
        fprintf(out, "dev_vel_x,dev_vel_y,dev_vel_z,");
        fprintf(out, "dev_roll_deg,dev_pitch_deg,dev_yaw_deg");
    }
    fprintf(out, "\n");

    // ========================================================================
    // Process packets (device-matched timing)
    // ========================================================================
    // デバイスの更新タイミング:
    //   - IMU Predict: 100Hz (ログレートと同じ)
    //   - AccelAtt: 50Hz (predict 2回に1回)
    //   - Baro: 50Hz (data_ready制御)
    //   - ToF: 30Hz (data_ready制御)
    //   - Mag: 10Hz (data_ready制御)
    //   - Flow: 100Hz
    //
    // PCでは固定間隔で近似:
    //   - Baro: 2パケットに1回 (50Hz)
    //   - ToF: 3パケットに1回 (33Hz ≈ 30Hz)
    //   - Mag: 10パケットに1回 (10Hz)

    // Device-matched parameters
    constexpr float FLOW_SCALE = 0.16f;  // main.cpp:367と一致
    constexpr int FLOW_SQUAL_MIN = 30;   // OutlierDetector::isFlowValid相当

    uint32_t last_timestamp = packets[0].timestamp_ms;

    // Flow update tracking (device updates at 100Hz = every 4 IMU packets)
    // デバイスはflow_update_counter >= 4で更新（データ変化は関係ない）
    size_t flow_update_counter = 0;
    constexpr size_t FLOW_UPDATE_INTERVAL = 4;  // 400Hz / 100Hz = 4
    constexpr float FLOW_DT = 0.01f;  // 10ms = 100Hz (matches device)

    // ToF update tracking (30Hz fixed interval like device)
    // デバイスはdata_readyフラグで30Hz更新（値が変化したかどうかは関係ない）
    size_t tof_update_counter = 0;
    constexpr size_t TOF_UPDATE_INTERVAL = 13;  // 400Hz / 30Hz ≈ 13.3

    // Mag update tracking (10Hz fixed interval like device)
    // デバイスはESKF_UPDATE_DIVISOR=10で10Hz更新
    size_t mag_update_counter = 0;
    constexpr size_t MAG_UPDATE_INTERVAL = 40;  // 400Hz / 10Hz = 40

    for (size_t i = 0; i < packets.size(); i++) {
        const auto& pkt = packets[i];

        // Calculate dt
        float dt = (pkt.timestamp_ms - last_timestamp) / 1000.0f;
        if (dt <= 0 || dt > 0.1f) {
            dt = 0.01f;  // 100Hz default
        }
        last_timestamp = pkt.timestamp_ms;

        // IMU data (ログは既にフィルタ済み、m/s²とrad/s)
        Vector3 accel(pkt.accel_x, pkt.accel_y, pkt.accel_z);
        Vector3 gyro(pkt.gyro_x, pkt.gyro_y, pkt.gyro_z);

        // ====================================================================
        // ESKF Predict (100Hz)
        // ====================================================================
        eskf.predict(accel, gyro, dt);

        // ====================================================================
        // Accel Attitude Update (400Hz = every predict, matches device)
        // デバイスは400Hzで加速度姿勢更新を行う
        // ====================================================================
        eskf.updateAccelAttitude(accel);

        // ====================================================================
        // Baro Update (50Hz = every 2nd packet)
        // TODO: 気圧センサの値が確認できたら有効化
        // ====================================================================
        // if (i % 2 == 0) {
        //     float baro_alt = pkt.baro_alt;
        //     if (std::abs(baro_alt) < 0.001f && pkt.pressure > 80000.0f) {
        //         constexpr float P0 = 101325.0f;
        //         baro_alt = 44330.0f * (1.0f - std::pow(pkt.pressure / P0, 0.1903f));
        //     }
        //     float baro_alt_relative = baro_alt - baro_alt_reference;
        //     eskf.updateBaro(baro_alt_relative);
        // }

        // ====================================================================
        // ToF Update (30Hz fixed interval, matches device data_ready timing)
        // デバイスはdata_readyフラグで30Hz更新（値が変化したかどうかは関係ない）
        // ====================================================================
        tof_update_counter++;
        if (tof_update_counter >= TOF_UPDATE_INTERVAL) {
            tof_update_counter = 0;
            if (pkt.tof_bottom > 0.01f && pkt.tof_bottom < 4.0f) {
                eskf.updateToF(pkt.tof_bottom);
            }
        }

        // ====================================================================
        // Mag Update (10Hz fixed interval, matches device ESKF_UPDATE_DIVISOR)
        // デバイスは10Hz固定間隔で更新
        // mag_ref計算に使用した最初の100サンプルはスキップ
        // ====================================================================
        mag_update_counter++;
        if (mag_update_counter >= MAG_UPDATE_INTERVAL) {
            mag_update_counter = 0;
            if (i >= MAG_REF_SAMPLES) {
                Vector3 mag(pkt.mag_x, pkt.mag_y, pkt.mag_z);
                float mag_norm = std::sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
                if (mag_norm > 10.0f) {
                    eskf.updateMag(mag);
                }
            }
        }

        // ====================================================================
        // Flow Update (100Hz fixed interval, matches device flow_update_counter)
        // デバイスは100Hz (= 400Hz IMU / 4) で固定更新
        // データ変化の有無は関係ない
        // ====================================================================
        flow_update_counter++;
        if (flow_update_counter >= FLOW_UPDATE_INTERVAL) {
            flow_update_counter = 0;
            if (pkt.flow_squal >= FLOW_SQUAL_MIN) {
                float distance = pkt.tof_bottom;
                if (distance < 0.02f) distance = 0.02f;
                if (distance > 0.02f && distance < 4.0f) {
                    // 新API: 生カウントとdt、機体ジャイロを渡す
                    // ESKF内部で物理的に正しい変換を行う
                    // dt = 10ms 固定（デバイスと一致）
                    eskf.updateFlowRaw(pkt.flow_dx, pkt.flow_dy, distance, FLOW_DT, gyro.x, gyro.y);
                }
            }
        }

        // ====================================================================
        // Write to CSV
        // ====================================================================
        auto state = eskf.getState();

        fprintf(out, "%u,%.1f,", pkt.timestamp_ms, dt * 1000.0f);
        fprintf(out, "%.6f,%.6f,%.6f,",
                state.position.x, state.position.y, state.position.z);
        fprintf(out, "%.6f,%.6f,%.6f,",
                state.velocity.x, state.velocity.y, state.velocity.z);
        fprintf(out, "%.4f,%.4f,%.4f,",
                state.roll * 180.0f / M_PI,
                state.pitch * 180.0f / M_PI,
                state.yaw * 180.0f / M_PI);
        fprintf(out, "%.8f,%.8f,%.8f,",
                state.gyro_bias.x, state.gyro_bias.y, state.gyro_bias.z);
        fprintf(out, "%.8f,%.8f,%.8f,",
                state.accel_bias.x, state.accel_bias.y, state.accel_bias.z);
        fprintf(out, "%.4f,%.4f,%.4f,",
                pkt.accel_x, pkt.accel_y, pkt.accel_z);
        fprintf(out, "%.6f,%.6f,%.6f,",
                pkt.gyro_x, pkt.gyro_y, pkt.gyro_z);
        fprintf(out, "%.4f,%.4f,%.4f,",
                pkt.mag_x, pkt.mag_y, pkt.mag_z);
        fprintf(out, "%.4f,%.4f,%d,",
                pkt.baro_alt, pkt.tof_bottom, pkt.flow_squal);
        fprintf(out, "%d,%d", pkt.flow_dx, pkt.flow_dy);
        if (has_device_eskf) {
            fprintf(out, ",%.6f,%.6f,%.6f,",
                    pkt.dev_pos_x, pkt.dev_pos_y, pkt.dev_pos_z);
            fprintf(out, "%.6f,%.6f,%.6f,",
                    pkt.dev_vel_x, pkt.dev_vel_y, pkt.dev_vel_z);
            fprintf(out, "%.4f,%.4f,%.4f",
                    pkt.dev_roll * 180.0f / M_PI,
                    pkt.dev_pitch * 180.0f / M_PI,
                    pkt.dev_yaw * 180.0f / M_PI);
        }
        fprintf(out, "\n");

        if (verbose && (i + 1) % 1000 == 0) {
            printf("Processed %zu packets (t=%.2fs)\n",
                   i + 1, pkt.timestamp_ms / 1000.0f);
        }
    }

    fclose(out);

    // ========================================================================
    // Final Summary
    // ========================================================================
    auto final_state = eskf.getState();
    const auto& last_pkt = packets.back();

    // Calculate position range for optimization metrics
    float pos_x_min = 0, pos_x_max = 0, pos_y_min = 0, pos_y_max = 0;
    {
        FILE* csv = fopen(output_file, "r");
        if (csv) {
            char line[4096];
            fgets(line, sizeof(line), csv);  // skip header
            while (fgets(line, sizeof(line), csv)) {
                float px, py, pz;
                uint32_t ts;
                float dt;
                if (sscanf(line, "%u,%f,%f,%f,%f", &ts, &dt, &px, &py, &pz) >= 5) {
                    if (px < pos_x_min) pos_x_min = px;
                    if (px > pos_x_max) pos_x_max = px;
                    if (py < pos_y_min) pos_y_min = py;
                    if (py > pos_y_max) pos_y_max = py;
                }
            }
            fclose(csv);
        }
    }
    float pos_x_range = pos_x_max - pos_x_min;
    float pos_y_range = pos_y_max - pos_y_min;

    if (quiet) {
        // JSON output for optimization script
        printf("{\"pos_x\":%.6f,\"pos_y\":%.6f,\"pos_z\":%.6f,",
               final_state.position.x, final_state.position.y, final_state.position.z);
        printf("\"vel_x\":%.6f,\"vel_y\":%.6f,\"vel_z\":%.6f,",
               final_state.velocity.x, final_state.velocity.y, final_state.velocity.z);
        printf("\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f,",
               final_state.roll * 180.0f / M_PI,
               final_state.pitch * 180.0f / M_PI,
               final_state.yaw * 180.0f / M_PI);
        printf("\"pos_x_range\":%.6f,\"pos_y_range\":%.6f,",
               pos_x_range, pos_y_range);
        printf("\"final_dist\":%.6f}\n",
               std::sqrt(final_state.position.x * final_state.position.x +
                         final_state.position.y * final_state.position.y));
    } else {
        printf("\n=== Replay Complete ===\n");
        printf("Processed %zu packets (%.2f seconds)\n",
               packets.size(),
               (packets.back().timestamp_ms - packets.front().timestamp_ms) / 1000.0f);

        printf("\nPC ESKF Final state:\n");
        printf("  Position: [%.3f, %.3f, %.3f] m\n",
               final_state.position.x, final_state.position.y, final_state.position.z);
        printf("  Velocity: [%.3f, %.3f, %.3f] m/s\n",
               final_state.velocity.x, final_state.velocity.y, final_state.velocity.z);
        printf("  Attitude: roll=%.2f° pitch=%.2f° yaw=%.2f°\n",
               final_state.roll * 180.0f / M_PI,
               final_state.pitch * 180.0f / M_PI,
               final_state.yaw * 180.0f / M_PI);
        printf("  Gyro bias: [%.6f, %.6f, %.6f] rad/s\n",
               final_state.gyro_bias.x, final_state.gyro_bias.y, final_state.gyro_bias.z);
        printf("  Position range: X=[%.3f, %.3f] Y=[%.3f, %.3f] m\n",
               pos_x_min, pos_x_max, pos_y_min, pos_y_max);

        if (has_device_eskf) {
            printf("\nDevice ESKF Final state:\n");
            printf("  Position: [%.3f, %.3f, %.3f] m\n",
                   last_pkt.dev_pos_x, last_pkt.dev_pos_y, last_pkt.dev_pos_z);
            printf("  Velocity: [%.3f, %.3f, %.3f] m/s\n",
                   last_pkt.dev_vel_x, last_pkt.dev_vel_y, last_pkt.dev_vel_z);
            printf("  Attitude: roll=%.2f° pitch=%.2f° yaw=%.2f°\n",
                   last_pkt.dev_roll * 180.0f / M_PI,
                   last_pkt.dev_pitch * 180.0f / M_PI,
                   last_pkt.dev_yaw * 180.0f / M_PI);

            // Position error
            float pos_err = std::sqrt(
                std::pow(final_state.position.x - last_pkt.dev_pos_x, 2) +
                std::pow(final_state.position.y - last_pkt.dev_pos_y, 2) +
                std::pow(final_state.position.z - last_pkt.dev_pos_z, 2));

            // Yaw error
            float yaw_err = (final_state.yaw - last_pkt.dev_yaw) * 180.0f / M_PI;
            while (yaw_err > 180.0f) yaw_err -= 360.0f;
            while (yaw_err < -180.0f) yaw_err += 360.0f;

            printf("\n=== PC vs Device Comparison ===\n");
            printf("  Position error: %.3f m\n", pos_err);
            printf("  Yaw error: %.2f°\n", yaw_err);
        }

        printf("\nOutput: %s\n", output_file);
    }

    return 0;
}
