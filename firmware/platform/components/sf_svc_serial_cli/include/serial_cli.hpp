/**
 * @file serial_cli.hpp
 * @brief Serial CLI using custom LineEditor for cursor display
 *
 * カスタムLineEditorを使用したSerial CLI（カーソー表示対応）
 *
 * Uses LineEditor instead of ESP-IDF standard linenoise to ensure
 * cursor visibility via ANSI escape sequences.
 */

#pragma once

namespace stampfly {

/**
 * @brief Serial CLI using LineEditor (Singleton)
 *
 * Provides Serial CLI over USB CDC with:
 * - Visible cursor (ANSI escape sequences)
 * - Command history
 * - Line editing (arrows, backspace, delete)
 * - Tab completion
 */
class SerialCLI {
public:
    /**
     * @brief Get singleton instance
     * @return Reference to SerialCLI instance
     */
    static SerialCLI& getInstance();

    /**
     * @brief Initialize Serial CLI
     * @return ESP_OK on success
     *
     * Sets up stdin/stdout buffering and console
     */
    int init();

    /**
     * @brief Run CLI loop (blocking)
     *
     * Runs until disconnection or error
     * CLIループを実行（ブロッキング）
     */
    void run();

private:
    SerialCLI() = default;
    ~SerialCLI() = default;

    // Non-copyable
    SerialCLI(const SerialCLI&) = delete;
    SerialCLI& operator=(const SerialCLI&) = delete;

    bool initialized_ = false;
};

}  // namespace stampfly
