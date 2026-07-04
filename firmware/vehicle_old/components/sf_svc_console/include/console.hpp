/**
 * @file console.hpp
 * @brief Unified Console Interface using ESP-IDF Console
 *
 * ESP-IDF Console を使用した統合コマンドコンソール
 * - esp_console によるコマンド登録・実行
 * - 出力リダイレクト機構（Serial/WiFi両対応）
 * - Thread-Local Storage による複数クライアント対応
 */

#pragma once

#include "esp_console.h"
#include "esp_err.h"
#include <cstdarg>

namespace stampfly {

/**
 * @class Console
 * @brief Singleton class for unified command console
 *
 * 統合コマンドコンソールのシングルトンクラス
 */
class Console {
public:
    /**
     * @brief Get singleton instance
     * @return Reference to Console instance
     */
    static Console& getInstance();

    /**
     * @brief Initialize console system
     * @return ESP_OK on success
     *
     * esp_console の初期化を行う
     */
    esp_err_t init();

    /**
     * @brief Register all built-in commands
     *
     * すべての組み込みコマンドを登録する
     */
    void registerAllCommands();

    /**
     * @brief Execute a command line
     * @param cmdline Command line string
     * @return 0 on success, non-zero on error
     *
     * コマンドラインを実行する
     */
    int run(const char* cmdline);

    // ==========================================================================
    // Output Redirection
    // ==========================================================================

    /**
     * @brief Output function type for redirection
     * @param str String to output
     * @param ctx User context
     */
    using OutputFunc = void (*)(const char* str, void* ctx);

    /**
     * @brief Set output function for current thread
     * @param func Output function (nullptr for default stdout)
     * @param ctx User context passed to func
     *
     * 現在のスレッドの出力関数を設定する
     * Thread-Local Storage を使用するため、各スレッドで独立して設定可能
     */
    void setOutput(OutputFunc func, void* ctx);

    /**
     * @brief Clear output function for current thread (use default stdout)
     *
     * 現在のスレッドの出力関数をクリア（デフォルトのstdoutを使用）
     */
    void clearOutput();

    /**
     * @brief Printf-compatible output function
     * @param fmt Format string
     * @param ... Arguments
     *
     * printf互換の出力関数。setOutput() で設定した関数にリダイレクトされる
     */
    void print(const char* fmt, ...);

    /**
     * @brief Printf-compatible output function (va_list version)
     * @param fmt Format string
     * @param args Arguments
     */
    void vprint(const char* fmt, va_list args);

    // ==========================================================================
    // State Access (for commands)
    // ==========================================================================

    /**
     * @brief Check if console is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

private:
    Console() = default;
    ~Console() = default;
    Console(const Console&) = delete;
    Console& operator=(const Console&) = delete;

    bool initialized_ = false;

    // Internal buffer for formatted output
    // 内部フォーマットバッファ
    static constexpr size_t PRINT_BUF_SIZE = 512;
};

// =============================================================================
// Command Registration Functions
// =============================================================================

/**
 * @brief Register system commands (help, status, reboot, version)
 */
void register_system_commands();

/**
 * @brief Register sensor commands (sensor, teleplot, log, binlog, loglevel)
 */
void register_sensor_commands();

/**
 * @brief Register motor commands (motor)
 */
void register_motor_commands();

/**
 * @brief Register control commands (trim, gain)
 */
void register_control_commands();

/**
 * @brief Register communication commands (comm, pair, unpair)
 */
void register_comm_commands();

/**
 * @brief Register calibration commands (calib, magcal)
 */
void register_calib_commands();

/**
 * @brief Register miscellaneous commands (led, sound, pos, debug, ctrl, attitude)
 */
void register_misc_commands();

}  // namespace stampfly
