/**
 * @file platform_context.hpp
 * @brief Platform service context for application code
 *        アプリケーションコード向けのプラットフォームサービスコンテキスト
 *
 * Aggregates references to key platform services so that application code
 * (controllers, commands, CLI) can access services without globals.
 * Populated in init.cpp and passed to application-level modules.
 *
 * 主要プラットフォームサービスへの参照を集約。
 * アプリケーションコード（コントローラ、コマンド、CLI）が
 * グローバル変数を使わずにサービスにアクセスできるようにする。
 * init.cpp で構築し、アプリケーションレベルのモジュールに渡す。
 */

#pragma once

// Forward declarations to minimize header coupling
// ヘッダの結合を最小化するための前方宣言
namespace sf { class SensorFusion; class HealthMonitor; }

namespace stampfly {
    class BMI270Wrapper;
    class BMM150;
    class MagCalibrator;
    class BMP280;
    class VL53L3CXWrapper;
    class PMW3901;
    class PowerMonitor;
    class MotorDriver;
    class Buzzer;
    class Button;
    class ControllerComm;
    class Logger;
    class LandingHandler;
}

namespace platform {

/**
 * @brief Platform service context
 *        プラットフォームサービスコンテキスト
 *
 * Non-owning references to platform services initialized in init.cpp.
 * Application code receives this context instead of accessing globals directly.
 *
 * init.cpp で初期化されたプラットフォームサービスへの非所有参照。
 * アプリケーションコードはグローバル直接アクセスの代わりにこれを受け取る。
 */
struct PlatformContext {
    // Sensors / センサー
    stampfly::BMI270Wrapper* imu = nullptr;
    stampfly::BMM150* mag = nullptr;
    stampfly::MagCalibrator* mag_cal = nullptr;
    stampfly::BMP280* baro = nullptr;
    stampfly::VL53L3CXWrapper* tof_bottom = nullptr;
    stampfly::VL53L3CXWrapper* tof_front = nullptr;
    stampfly::PMW3901* optflow = nullptr;
    stampfly::PowerMonitor* power = nullptr;

    // Actuators / アクチュエータ
    stampfly::MotorDriver* motor = nullptr;
    stampfly::Buzzer* buzzer = nullptr;
    stampfly::Button* button = nullptr;

    // Estimation / 状態推定
    sf::SensorFusion* fusion = nullptr;
    stampfly::LandingHandler* landing_handler = nullptr;

    // Communication / 通信
    stampfly::ControllerComm* comm = nullptr;
    stampfly::Logger* logger = nullptr;

    // Health monitoring / ヘルスモニタリング
    sf::HealthMonitor* health = nullptr;
};

/**
 * @brief Get the global platform context (set during init)
 *        グローバルプラットフォームコンテキストを取得（init時に設定）
 *
 * Returns the singleton context populated by initPlatformContext().
 * Must be called after init is complete.
 *
 * initPlatformContext() で構築されたシングルトンコンテキストを返す。
 * init 完了後に呼ぶこと。
 */
PlatformContext& getPlatformContext();

/**
 * @brief Initialize platform context from globals
 *        グローバルからプラットフォームコンテキストを初期化
 *
 * Called once during system initialization.
 * Populates the context with pointers to all platform service globals.
 *
 * システム初期化時に1回呼ばれる。
 * 全プラットフォームサービスのグローバルへのポインタでコンテキストを構築。
 */
void initPlatformContext();

}  // namespace platform
