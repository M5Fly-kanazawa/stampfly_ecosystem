/**
 * @file line_editor.cpp
 * @brief Generic line editor implementation
 *
 * 汎用ラインエディタの実装
 */

#include "line_editor.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace stampfly {

// =============================================================================
// LineCompletions
// =============================================================================

void LineCompletions::add(const char* str)
{
    if (len >= MAX_COMPLETIONS || str == nullptr) {
        return;
    }
    cvec[len] = strdup(str);
    if (cvec[len] != nullptr) {
        len++;
    }
}

void LineCompletions::clear()
{
    for (size_t i = 0; i < len; i++) {
        free(cvec[i]);
        cvec[i] = nullptr;
    }
    len = 0;
}

// =============================================================================
// Constructor / Destructor
// =============================================================================

LineEditor::LineEditor(const LineEditorIO& io, const LineEditorConfig& config)
    : io_(io)
    , config_(config)
    , history_max_(config.history_max_len)
{
    memset(buffer_, 0, sizeof(buffer_));
    memset(history_, 0, sizeof(history_));
    memset(esc_buf_, 0, sizeof(esc_buf_));

    // Send telnet negotiation if enabled
    if (config_.enable_telnet_negotiation) {
        sendTelnetNegotiation();
    }
}

LineEditor::~LineEditor()
{
    clearHistory();
}

void LineEditor::clearHistory()
{
    for (int i = 0; i < history_len_; i++) {
        free(history_[i]);
        history_[i] = nullptr;
    }
    history_len_ = 0;
}

// =============================================================================
// Telnet Negotiation
// =============================================================================

void LineEditor::sendTelnetNegotiation()
{
    // Telnet options
    static constexpr uint8_t TELOPT_ECHO     = 1;
    static constexpr uint8_t TELOPT_SGA      = 3;
    static constexpr uint8_t TELOPT_LINEMODE = 34;

    uint8_t negotiate[] = {
        TELNET_IAC, TELNET_WILL, TELOPT_ECHO,      // Server will echo
        TELNET_IAC, TELNET_WILL, TELOPT_SGA,       // Server will suppress go-ahead
        TELNET_IAC, TELNET_DO,   TELOPT_SGA,       // Client should suppress go-ahead
        TELNET_IAC, TELNET_DONT, TELOPT_LINEMODE,  // Client should not use linemode
    };
    writeData(negotiate, sizeof(negotiate));
}

// =============================================================================
// Public Methods
// =============================================================================

char* LineEditor::getLine(const char* prompt)
{
    // Reset line state
    memset(buffer_, 0, sizeof(buffer_));
    pos_ = 0;
    len_ = 0;
    history_index_ = -1;
    esc_state_ = EscState::NONE;
    esc_pos_ = 0;

    // Send prompt
    if (prompt != nullptr) {
        writeStr(prompt);
    }

    // Read characters
    while (true) {
        int c = io_.read_byte(io_.ctx);

        if (c < 0) {
            // Error or disconnect
            return nullptr;
        }

        // Process character
        if (processChar(static_cast<char>(c), prompt)) {
            // Line complete
            buffer_[len_] = '\0';
            writeStr("\r\n");
            return strdup(buffer_);
        }
    }
}

void LineEditor::freeLine(char* line)
{
    free(line);
}

void LineEditor::addHistory(const char* line)
{
    if (line == nullptr || line[0] == '\0') {
        return;
    }

    // Don't add duplicate of most recent entry
    if (history_len_ > 0 && strcmp(history_[history_len_ - 1], line) == 0) {
        return;
    }

    // If at max, remove oldest entry
    if (history_len_ >= history_max_) {
        free(history_[0]);
        memmove(history_, history_ + 1, (history_max_ - 1) * sizeof(char*));
        history_len_--;
    }

    // Add new entry
    history_[history_len_] = strdup(line);
    if (history_[history_len_] != nullptr) {
        history_len_++;
    }
}

void LineEditor::setHistoryMaxLen(int len)
{
    if (len < 1) len = 1;
    if (len > MAX_HISTORY) len = MAX_HISTORY;
    history_max_ = len;

    // Trim existing history if needed
    while (history_len_ > history_max_) {
        free(history_[0]);
        memmove(history_, history_ + 1, (history_len_ - 1) * sizeof(char*));
        history_len_--;
    }
}

void LineEditor::setCompletionCallback(LineCompletionCallback callback)
{
    completion_callback_ = callback;
}

// =============================================================================
// Character Processing
// =============================================================================

bool LineEditor::processChar(char c, const char* prompt)
{
    uint8_t uc = static_cast<uint8_t>(c);

    // Handle telnet IAC sequences (only if telnet enabled)
    if (config_.enable_telnet_negotiation) {
        if (esc_state_ == EscState::IAC) {
            if (uc == TELNET_IAC) {
                esc_state_ = EscState::NONE;
                // Fall through to insert as character
            } else if (uc == TELNET_WILL || uc == TELNET_WONT ||
                       uc == TELNET_DO || uc == TELNET_DONT) {
                esc_state_ = EscState::IAC_CMD;
                esc_buf_[0] = c;
                return false;
            } else if (uc == TELNET_SB) {
                esc_state_ = EscState::IAC_SB;
                return false;
            } else {
                esc_state_ = EscState::NONE;
                return false;
            }
        }
        if (esc_state_ == EscState::IAC_CMD) {
            esc_state_ = EscState::NONE;
            return false;
        }
        if (esc_state_ == EscState::IAC_SB) {
            if (uc == TELNET_IAC) {
                esc_state_ = EscState::IAC_SB_IAC;
            }
            return false;
        }
        if (esc_state_ == EscState::IAC_SB_IAC) {
            if (uc == TELNET_SE) {
                esc_state_ = EscState::NONE;
            } else {
                esc_state_ = EscState::IAC_SB;
            }
            return false;
        }

        // Check for IAC start
        if (uc == TELNET_IAC) {
            esc_state_ = EscState::IAC;
            return false;
        }
    }

    // Handle ANSI escape sequences
    if (esc_state_ == EscState::ESC) {
        processEscape(c, prompt);
        return false;
    }
    if (esc_state_ == EscState::CSI) {
        processCSI(c, prompt);
        return false;
    }

    // Regular character processing
    switch (c) {
        case '\r':
        case '\n':
            return true;

        case 0x1B:  // ESC
            esc_state_ = EscState::ESC;
            esc_pos_ = 0;
            return false;

        case 0x7F:  // DEL
        case '\b':  // Backspace (Ctrl+H)
            handleBackspace(prompt);
            return false;

        case 0x01:  // Ctrl+A - Home
            handleHome(prompt);
            return false;

        case 0x02:  // Ctrl+B - Left
            handleArrowLeft(prompt);
            return false;

        case 0x03:  // Ctrl+C - Cancel
            writeStr("^C\r\n");
            if (prompt) writeStr(prompt);
            len_ = 0;
            pos_ = 0;
            buffer_[0] = '\0';
            return false;

        case 0x04:  // Ctrl+D - EOF (if empty) or delete
            if (len_ == 0) {
                return true;
            }
            handleDelete(prompt);
            return false;

        case 0x05:  // Ctrl+E - End
            handleEnd(prompt);
            return false;

        case 0x06:  // Ctrl+F - Right
            handleArrowRight(prompt);
            return false;

        case '\t':  // Tab - completion
            handleTab(prompt);
            return false;

        case 0x0B:  // Ctrl+K - Kill to end of line
            if (pos_ < len_) {
                len_ = pos_;
                buffer_[len_] = '\0';
                refreshLine(prompt);
            }
            return false;

        case 0x0C:  // Ctrl+L - Clear screen
            writeStr("\x1B[2J\x1B[H");
            refreshLine(prompt);
            return false;

        case 0x0E:  // Ctrl+N - Down (next history)
            handleArrowDown(prompt);
            return false;

        case 0x10:  // Ctrl+P - Up (previous history)
            handleArrowUp(prompt);
            return false;

        case 0x15:  // Ctrl+U - Kill line
            len_ = 0;
            pos_ = 0;
            buffer_[0] = '\0';
            refreshLine(prompt);
            return false;

        case 0x17:  // Ctrl+W - Kill word
            while (pos_ > 0 && buffer_[pos_ - 1] == ' ') {
                deleteCharAt(pos_ - 1, prompt);
            }
            while (pos_ > 0 && buffer_[pos_ - 1] != ' ') {
                deleteCharAt(pos_ - 1, prompt);
            }
            return false;

        default:
            if (c >= 0x20 && c < 0x7F) {
                insertChar(c, prompt);
            }
            return false;
    }
}

void LineEditor::processEscape(char c, const char* prompt)
{
    if (c == '[') {
        esc_state_ = EscState::CSI;
        return;
    }
    if (c == 'O') {
        esc_state_ = EscState::CSI;
        return;
    }
    esc_state_ = EscState::NONE;
}

void LineEditor::processCSI(char c, const char* prompt)
{
    if (c >= '0' && c <= '9') {
        if (esc_pos_ < (int)sizeof(esc_buf_) - 1) {
            esc_buf_[esc_pos_++] = c;
        }
        return;
    }
    if (c == ';') {
        if (esc_pos_ < (int)sizeof(esc_buf_) - 1) {
            esc_buf_[esc_pos_++] = c;
        }
        return;
    }

    esc_state_ = EscState::NONE;
    esc_buf_[esc_pos_] = '\0';

    switch (c) {
        case 'A': handleArrowUp(prompt); break;
        case 'B': handleArrowDown(prompt); break;
        case 'C': handleArrowRight(prompt); break;
        case 'D': handleArrowLeft(prompt); break;
        case 'H': handleHome(prompt); break;
        case 'F': handleEnd(prompt); break;
        case '~':
            if (esc_pos_ > 0) {
                int code = atoi(esc_buf_);
                switch (code) {
                    case 1: handleHome(prompt); break;
                    case 3: handleDelete(prompt); break;
                    case 4: handleEnd(prompt); break;
                }
            }
            break;
    }

    esc_pos_ = 0;
}

// =============================================================================
// Arrow Key Handlers
// =============================================================================

void LineEditor::handleArrowUp(const char* prompt)
{
    if (history_len_ == 0) return;

    if (history_index_ < 0) {
        history_index_ = history_len_ - 1;
    } else if (history_index_ > 0) {
        history_index_--;
    } else {
        return;
    }

    strncpy(buffer_, history_[history_index_], BUFFER_SIZE - 1);
    buffer_[BUFFER_SIZE - 1] = '\0';
    len_ = strlen(buffer_);
    pos_ = len_;

    refreshLine(prompt);
}

void LineEditor::handleArrowDown(const char* prompt)
{
    if (history_index_ < 0) return;

    if (history_index_ < history_len_ - 1) {
        history_index_++;
        strncpy(buffer_, history_[history_index_], BUFFER_SIZE - 1);
        buffer_[BUFFER_SIZE - 1] = '\0';
    } else {
        history_index_ = -1;
        buffer_[0] = '\0';
    }
    len_ = strlen(buffer_);
    pos_ = len_;

    refreshLine(prompt);
}

void LineEditor::handleArrowLeft(const char* prompt)
{
    (void)prompt;
    if (pos_ > 0) {
        pos_--;
        writeStr("\x1B[D");
    }
}

void LineEditor::handleArrowRight(const char* prompt)
{
    (void)prompt;
    if (pos_ < len_) {
        pos_++;
        writeStr("\x1B[C");
    }
}

void LineEditor::handleHome(const char* prompt)
{
    (void)prompt;
    if (pos_ > 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "\x1B[%dD", (int)pos_);
        writeStr(buf);
        pos_ = 0;
    }
}

void LineEditor::handleEnd(const char* prompt)
{
    (void)prompt;
    if (pos_ < len_) {
        char buf[16];
        snprintf(buf, sizeof(buf), "\x1B[%dC", (int)(len_ - pos_));
        writeStr(buf);
        pos_ = len_;
    }
}

// =============================================================================
// Edit Operations
// =============================================================================

void LineEditor::handleBackspace(const char* prompt)
{
    if (pos_ > 0) {
        deleteCharAt(pos_ - 1, prompt);
    }
}

void LineEditor::handleDelete(const char* prompt)
{
    if (pos_ < len_) {
        memmove(buffer_ + pos_, buffer_ + pos_ + 1, len_ - pos_);
        len_--;
        refreshLine(prompt);
    }
}

void LineEditor::handleTab(const char* prompt)
{
    if (completion_callback_ == nullptr) return;

    LineCompletions completions;
    buffer_[len_] = '\0';
    completion_callback_(buffer_, &completions);

    if (completions.len == 0) {
        writeStr("\x07");  // Beep
    } else if (completions.len == 1) {
        strncpy(buffer_, completions.cvec[0], BUFFER_SIZE - 1);
        buffer_[BUFFER_SIZE - 1] = '\0';
        len_ = strlen(buffer_);
        pos_ = len_;
        refreshLine(prompt);
    } else {
        writeStr("\r\n");
        for (size_t i = 0; i < completions.len; i++) {
            writeStr(completions.cvec[i]);
            writeStr("  ");
        }
        writeStr("\r\n");
        refreshLine(prompt);
    }

    completions.clear();
}

void LineEditor::insertChar(char c, const char* prompt)
{
    if (len_ >= BUFFER_SIZE - 1) return;

    if (pos_ == len_) {
        buffer_[pos_] = c;
        pos_++;
        len_++;
        buffer_[len_] = '\0';
        writeChar(c);
    } else {
        memmove(buffer_ + pos_ + 1, buffer_ + pos_, len_ - pos_);
        buffer_[pos_] = c;
        pos_++;
        len_++;
        buffer_[len_] = '\0';
        refreshLine(prompt);
    }
}

void LineEditor::deleteCharAt(size_t del_pos, const char* prompt)
{
    if (del_pos >= len_) return;

    memmove(buffer_ + del_pos, buffer_ + del_pos + 1, len_ - del_pos);
    len_--;
    if (pos_ > del_pos) {
        pos_--;
    }
    refreshLine(prompt);
}

// =============================================================================
// Output Helpers
// =============================================================================

void LineEditor::writeStr(const char* str)
{
    if (str == nullptr) return;
    size_t len = strlen(str);
    if (len > 0) {
        writeData(str, len);
    }
}

void LineEditor::writeChar(char c)
{
    writeData(&c, 1);
}

void LineEditor::writeData(const void* data, size_t len)
{
    if (io_.write_data && len > 0) {
        io_.write_data(io_.ctx, data, len);
    }
}

void LineEditor::refreshLine(const char* prompt)
{
    // Build the entire output string first, then send at once
    char output[512];
    int out_pos = 0;

    // Hide cursor + CR + clear line
    const char* prefix = "\x1B[?25l\r\x1B[K";
    size_t prefix_len = strlen(prefix);
    if (out_pos + (int)prefix_len < (int)sizeof(output)) {
        memcpy(output + out_pos, prefix, prefix_len);
        out_pos += prefix_len;
    }

    // Add prompt
    if (prompt) {
        size_t plen = strlen(prompt);
        if (out_pos + (int)plen < (int)sizeof(output)) {
            memcpy(output + out_pos, prompt, plen);
            out_pos += plen;
        }
    }

    // Add buffer content
    if (len_ > 0 && out_pos + (int)len_ < (int)sizeof(output)) {
        memcpy(output + out_pos, buffer_, len_);
        out_pos += len_;
    }

    // Move cursor to correct position if not at end
    if (pos_ < len_) {
        char move_buf[16];
        int move_len = snprintf(move_buf, sizeof(move_buf), "\x1B[%dD", (int)(len_ - pos_));
        if (out_pos + move_len < (int)sizeof(output)) {
            memcpy(output + out_pos, move_buf, move_len);
            out_pos += move_len;
        }
    }

    // Show cursor again
    const char* suffix = "\x1B[?25h";
    size_t suffix_len = strlen(suffix);
    if (out_pos + (int)suffix_len < (int)sizeof(output)) {
        memcpy(output + out_pos, suffix, suffix_len);
        out_pos += suffix_len;
    }

    // Send all at once
    if (out_pos > 0) {
        writeData(output, out_pos);
    }
}

}  // namespace stampfly
