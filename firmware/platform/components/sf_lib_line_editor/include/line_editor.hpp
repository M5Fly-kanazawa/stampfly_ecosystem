/**
 * @file line_editor.hpp
 * @brief Generic line editor with history and completion
 *
 * Provides readline-like line editing:
 * - Command history (up/down arrows)
 * - Cursor movement (left/right arrows, Home/End)
 * - Tab completion
 * - Backspace/Delete
 *
 * I/O is abstracted via callbacks, allowing use with:
 * - TCP sockets (WiFi CLI)
 * - stdin/stdout (Serial CLI)
 *
 * 汎用ラインエディタ（履歴・補完機能付き）
 * I/Oはコールバックで抽象化、ソケット/シリアル両対応
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace stampfly {

/**
 * @brief Completions structure for tab completion
 * Tab補完用の候補構造体
 */
struct LineCompletions {
    static constexpr int MAX_COMPLETIONS = 16;
    char* cvec[MAX_COMPLETIONS] = {};
    size_t len = 0;

    void add(const char* str);
    void clear();
};

/**
 * @brief I/O callbacks for LineEditor
 * LineEditor用のI/Oコールバック
 */
struct LineEditorIO {
    /**
     * @brief Read a single byte (blocking)
     * @param ctx User context
     * @return Byte read, or -1 on error/disconnect
     *
     * 1バイト読み取り（ブロッキング）
     */
    int (*read_byte)(void* ctx);

    /**
     * @brief Write data
     * @param ctx User context
     * @param data Data to write
     * @param len Length of data
     * @return Number of bytes written, or -1 on error
     *
     * データ書き込み
     */
    int (*write_data)(void* ctx, const void* data, size_t len);

    /**
     * @brief User context passed to callbacks
     * コールバックに渡すユーザーコンテキスト
     */
    void* ctx;
};

/**
 * @brief Callback type for tab completion
 * Tab補完コールバック型
 */
using LineCompletionCallback = void (*)(const char* buf, LineCompletions* lc);

/**
 * @brief Configuration for LineEditor
 * LineEditorの設定
 */
struct LineEditorConfig {
    bool enable_telnet_negotiation = false;  // For WiFi CLI
    int history_max_len = 10;
};

/**
 * @brief Generic line editor with history and completion
 *
 * Provides readline-like functionality via I/O callbacks
 */
class LineEditor {
public:
    /**
     * @brief Constructor
     * @param io I/O callbacks
     * @param config Configuration
     */
    LineEditor(const LineEditorIO& io, const LineEditorConfig& config = {});

    /**
     * @brief Destructor - frees history
     */
    ~LineEditor();

    // Non-copyable
    LineEditor(const LineEditor&) = delete;
    LineEditor& operator=(const LineEditor&) = delete;

    /**
     * @brief Get a line of input (blocking)
     * @param prompt Prompt to display
     * @return Line buffer (caller must free with freeLine()) or nullptr on error/disconnect
     *
     * 1行の入力を取得（ブロッキング）
     */
    char* getLine(const char* prompt);

    /**
     * @brief Free a line returned by getLine()
     * @param line Line to free
     */
    void freeLine(char* line);

    /**
     * @brief Add a line to history
     * @param line Line to add
     */
    void addHistory(const char* line);

    /**
     * @brief Set maximum history length
     * @param len Maximum number of history entries
     */
    void setHistoryMaxLen(int len);

    /**
     * @brief Set completion callback
     * @param callback Callback function for tab completion
     */
    void setCompletionCallback(LineCompletionCallback callback);

private:
    LineEditorIO io_;
    LineEditorConfig config_;

    // Line buffer
    static constexpr size_t BUFFER_SIZE = 256;
    char buffer_[BUFFER_SIZE];
    size_t pos_ = 0;  // Cursor position
    size_t len_ = 0;  // Line length

    // History
    static constexpr int MAX_HISTORY = 20;
    char* history_[MAX_HISTORY] = {};
    int history_len_ = 0;
    int history_max_ = 10;
    int history_index_ = -1;

    // Completion callback
    LineCompletionCallback completion_callback_ = nullptr;

    // Escape sequence state machine (also handles Telnet IAC)
    enum class EscState { NONE, ESC, CSI, IAC, IAC_CMD, IAC_SB, IAC_SB_IAC };
    EscState esc_state_ = EscState::NONE;
    char esc_buf_[8];
    int esc_pos_ = 0;

    // Telnet constants
    static constexpr uint8_t TELNET_IAC  = 255;
    static constexpr uint8_t TELNET_WILL = 251;
    static constexpr uint8_t TELNET_WONT = 252;
    static constexpr uint8_t TELNET_DO   = 253;
    static constexpr uint8_t TELNET_DONT = 254;
    static constexpr uint8_t TELNET_SB   = 250;
    static constexpr uint8_t TELNET_SE   = 240;

    // Internal methods
    void writeStr(const char* str);
    void writeChar(char c);
    void writeData(const void* data, size_t len);
    void refreshLine(const char* prompt);

    void sendTelnetNegotiation();
    bool processChar(char c, const char* prompt);
    void processEscape(char c, const char* prompt);
    void processCSI(char c, const char* prompt);

    void handleArrowUp(const char* prompt);
    void handleArrowDown(const char* prompt);
    void handleArrowLeft(const char* prompt);
    void handleArrowRight(const char* prompt);
    void handleBackspace(const char* prompt);
    void handleDelete(const char* prompt);
    void handleTab(const char* prompt);
    void handleHome(const char* prompt);
    void handleEnd(const char* prompt);

    void insertChar(char c, const char* prompt);
    void deleteCharAt(size_t pos, const char* prompt);

    void clearHistory();
};

}  // namespace stampfly
