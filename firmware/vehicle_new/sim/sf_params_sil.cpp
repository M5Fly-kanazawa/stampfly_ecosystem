/**
 * @file sf_params_sil.cpp
 * @brief SIL implementation of `sf::params` — uses params.def as SSOT, no NVS
 *        SIL用パラメータ実装 — params.defをSSOTとして使用、NVSなし
 *
 * Phase 1.1 of the SIL→Real workflow (development_roadmap.md).
 * The firmware-side `params.cpp` currently triplicates defaults
 * (param_vars + table[] + params.def comments). This SIL impl uses
 * the X-macro pattern on params.def — the originally intended design —
 * so that SIL and firmware draw defaults from the same single source.
 *
 * This file provides the same API surface as the firmware's params.cpp:
 *   init / get_float / get_bool / get_int / set_float / set_bool / set_int /
 *   reset_all / list / count / entry
 * `save()` and `load()` are no-ops here; file-based persistence will be
 * added in Phase 1.3.
 *
 * @design development_roadmap.md §2 — Principle 2: Parameter Identity   [OK]
 * @design detailed_design.md §6 — Single-macro parameter definition     [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 *
 * Phase 1.1 — SIL→実機 ワークフロー（development_roadmap.md）。
 * 現状 firmware 側の params.cpp は defaults を3重に持つ
 * （param_vars + table[] + params.def コメント）。
 * この SIL 実装は params.def を X-macro で展開する本来の設計通りに動作し、
 * SIL と firmware が同じ単一の SSOT から defaults を読む。
 */

#include "params.hpp"
#include "sf_params_file.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace sf::params {

// =============================================================================
// Metadata table — generated at compile time from params.def
// メタデータテーブル — params.def からコンパイル時に生成
// =============================================================================

struct ParamMeta {
    const char* name;
    ParamType   type;
    float       default_val;
    float       min_val;
    float       max_val;
};

// Three X-macro definitions to extract metadata
// 3種のXマクロでメタデータを抽出
#define PARAM_FLOAT(n, def, mn, mx, cb) \
    { n, ParamType::FLOAT, static_cast<float>(def), \
      static_cast<float>(mn), static_cast<float>(mx) },
#define PARAM_BOOL(n, def, mn, mx, cb) \
    { n, ParamType::BOOL, static_cast<float>(def), \
      static_cast<float>(mn), static_cast<float>(mx) },
#define PARAM_INT(n, def, mn, mx, cb) \
    { n, ParamType::INT, static_cast<float>(def), \
      static_cast<float>(mn), static_cast<float>(mx) },

static const ParamMeta meta_table[] = {
    #include "params.def"
};

#undef PARAM_FLOAT
#undef PARAM_BOOL
#undef PARAM_INT

static constexpr int N_PARAMS =
    sizeof(meta_table) / sizeof(meta_table[0]);

// =============================================================================
// Runtime value storage — initialized from defaults
// ランタイム値ストレージ — defaults から初期化
// =============================================================================

// One slot per parameter, indexed by meta_table position. Only the array
// matching the parameter's type holds the live value; the others are unused.
// パラメータごとに1スロット、meta_table のインデックスでアクセス。
// 型に対応する配列だけが有効値を持ち、他は未使用。
static float   float_values[N_PARAMS];
static bool    bool_values[N_PARAMS];
static int32_t int_values[N_PARAMS];

static bool initialized = false;

// =============================================================================
// Helpers
// ヘルパー
// =============================================================================

static int find_index(const char* name)
{
    // Linear search — N_PARAMS is small (~45) and lookups are not in hot path.
    // 線形探索 — N_PARAMS は小さく(~45)、ホットパスでない。
    for (int i = 0; i < N_PARAMS; i++) {
        if (std::strcmp(meta_table[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static void apply_defaults()
{
    for (int i = 0; i < N_PARAMS; i++) {
        switch (meta_table[i].type) {
            case ParamType::FLOAT:
                float_values[i] = meta_table[i].default_val;
                break;
            case ParamType::BOOL:
                bool_values[i] = (meta_table[i].default_val != 0.0f);
                break;
            case ParamType::INT:
                int_values[i] =
                    static_cast<int32_t>(meta_table[i].default_val);
                break;
        }
    }
}

// =============================================================================
// Public API — same surface as firmware-side params.cpp
// 公開 API — firmware 側 params.cpp と同じ表面
// =============================================================================

void init()
{
    apply_defaults();
    initialized = true;
}

bool get_float(const char* name, float& out)
{
    if (!initialized) init();
    int i = find_index(name);
    if (i < 0 || meta_table[i].type != ParamType::FLOAT) return false;
    out = float_values[i];
    return true;
}

bool get_bool(const char* name, bool& out)
{
    if (!initialized) init();
    int i = find_index(name);
    if (i < 0 || meta_table[i].type != ParamType::BOOL) return false;
    out = bool_values[i];
    return true;
}

bool get_int(const char* name, int32_t& out)
{
    if (!initialized) init();
    int i = find_index(name);
    if (i < 0 || meta_table[i].type != ParamType::INT) return false;
    out = int_values[i];
    return true;
}

bool set_float(const char* name, float value)
{
    int i = find_index(name);
    if (i < 0 || meta_table[i].type != ParamType::FLOAT) return false;
    if (std::isnan(value) ||
        value < meta_table[i].min_val || value > meta_table[i].max_val) {
        std::fprintf(stderr,
            "params: set_float('%s', %g) out of range [%g, %g]\n",
            name, value, meta_table[i].min_val, meta_table[i].max_val);
        return false;
    }
    float_values[i] = value;
    return true;
}

bool set_bool(const char* name, bool value)
{
    int i = find_index(name);
    if (i < 0 || meta_table[i].type != ParamType::BOOL) return false;
    bool_values[i] = value;
    return true;
}

bool set_int(const char* name, int32_t value)
{
    int i = find_index(name);
    if (i < 0 || meta_table[i].type != ParamType::INT) return false;
    int32_t mn = static_cast<int32_t>(meta_table[i].min_val);
    int32_t mx = static_cast<int32_t>(meta_table[i].max_val);
    if (value < mn || value > mx) {
        std::fprintf(stderr,
            "params: set_int('%s', %ld) out of range [%ld, %ld]\n",
            name, static_cast<long>(value),
            static_cast<long>(mn), static_cast<long>(mx));
        return false;
    }
    int_values[i] = value;
    return true;
}

void save()
{
    // Phase 1.3 will add file-based persistence
    // Phase 1.3 でファイル永続化を追加
}

void load()
{
    // Phase 1.3 will add file-based persistence
    // Phase 1.3 でファイル永続化を追加
}

// =============================================================================
// File-based override loader (SIL only)
// ファイルベースのオーバーライドローダ（SIL 専用）
// =============================================================================

namespace {

// Trim ASCII whitespace from both ends in place; returns the new start.
// 両端の ASCII 空白をトリムし、新しい開始ポインタを返す。
char* trim(char* s)
{
    while (*s != '\0' && std::isspace(static_cast<unsigned char>(*s))) {
        ++s;
    }
    if (*s == '\0') return s;
    char* end = s + std::strlen(s) - 1;
    while (end > s && std::isspace(static_cast<unsigned char>(*end))) {
        *end-- = '\0';
    }
    return s;
}

// Strip an inline comment: `value  # rest` → `value`.
// Only `#` preceded by whitespace counts; this preserves names that
// might legitimately contain `#` (none today, but cheap insurance).
// インラインコメントを除去する: `value  # rest` → `value`。
// `#` の前に空白がある場合のみコメントとして扱う。
void strip_inline_comment(char* s)
{
    for (char* p = s; *p != '\0'; ++p) {
        if (*p == '#' && p > s &&
            std::isspace(static_cast<unsigned char>(*(p - 1)))) {
            *p = '\0';
            return;
        }
    }
}

// Parse a boolean literal: true/false/1/0 (case-insensitive on words).
// Returns true on success and writes the parsed value to `out`.
// ブールリテラルを解析する: true/false/1/0（単語は大文字小文字無視）。
bool parse_bool(const char* s, bool& out)
{
    if (std::strcmp(s, "1") == 0) { out = true;  return true; }
    if (std::strcmp(s, "0") == 0) { out = false; return true; }
    // Case-insensitive compare for "true" / "false"
    // "true" / "false" は大文字小文字を無視して比較
    char buf[8] = {0};
    std::size_t n = std::strlen(s);
    if (n >= sizeof(buf)) return false;
    for (std::size_t i = 0; i < n; ++i) {
        buf[i] = static_cast<char>(
            std::tolower(static_cast<unsigned char>(s[i])));
    }
    if (std::strcmp(buf, "true")  == 0) { out = true;  return true; }
    if (std::strcmp(buf, "false") == 0) { out = false; return true; }
    return false;
}

}  // anonymous namespace

int load_file(const char* path)
{
    if (!initialized) init();

    std::FILE* fp = std::fopen(path, "r");
    if (fp == nullptr) {
        std::fprintf(stderr,
            "params: load_file('%s') failed to open file\n", path);
        return -1;
    }

    int applied = 0;
    int line_no = 0;
    char line[512];

    while (std::fgets(line, sizeof(line), fp) != nullptr) {
        ++line_no;

        // Strip inline `# ...` comments, then trim whitespace.
        // インラインの `# ...` コメントを除去し、空白をトリムする。
        strip_inline_comment(line);
        char* p = trim(line);

        // Skip blank lines and full-line comments.
        // 空行と行頭コメントをスキップ。
        if (*p == '\0' || *p == '#') continue;

        // Split on '=' — name on the left, value on the right.
        // '=' で分割 — 左側が名前、右側が値。
        char* eq = std::strchr(p, '=');
        if (eq == nullptr) {
            std::fprintf(stderr,
                "params: load_file('%s'):%d missing '=' in: %s\n",
                path, line_no, p);
            continue;
        }
        *eq = '\0';
        char* name  = trim(p);
        char* value = trim(eq + 1);

        if (*name == '\0' || *value == '\0') {
            std::fprintf(stderr,
                "params: load_file('%s'):%d empty name or value\n",
                path, line_no);
            continue;
        }

        // Look up the parameter to determine its type.
        // パラメータを検索して型を判定する。
        int idx = find_index(name);
        if (idx < 0) {
            std::fprintf(stderr,
                "params: load_file('%s'):%d unknown parameter '%s'\n",
                path, line_no, name);
            continue;
        }

        // Dispatch on type, reusing set_* for range validation.
        // 型に応じて分岐し、範囲検証は set_* に委譲する。
        bool ok = false;
        switch (meta_table[idx].type) {
            case ParamType::FLOAT: {
                char* end = nullptr;
                float fv = std::strtof(value, &end);
                if (end == value || *end != '\0') {
                    std::fprintf(stderr,
                        "params: load_file('%s'):%d "
                        "invalid float for '%s': '%s'\n",
                        path, line_no, name, value);
                    break;
                }
                ok = set_float(name, fv);
                break;
            }
            case ParamType::BOOL: {
                bool bv = false;
                if (!parse_bool(value, bv)) {
                    std::fprintf(stderr,
                        "params: load_file('%s'):%d "
                        "invalid bool for '%s': '%s'\n",
                        path, line_no, name, value);
                    break;
                }
                ok = set_bool(name, bv);
                break;
            }
            case ParamType::INT: {
                char* end = nullptr;
                long lv = std::strtol(value, &end, 10);
                if (end == value || *end != '\0') {
                    std::fprintf(stderr,
                        "params: load_file('%s'):%d "
                        "invalid int for '%s': '%s'\n",
                        path, line_no, name, value);
                    break;
                }
                ok = set_int(name, static_cast<int32_t>(lv));
                break;
            }
        }
        if (ok) ++applied;
    }

    std::fclose(fp);
    return applied;
}

void reset_all()
{
    apply_defaults();
}

void list()
{
    if (!initialized) init();
    std::fprintf(stdout, "=== Parameters (%d) ===\n", N_PARAMS);
    for (int i = 0; i < N_PARAMS; i++) {
        const auto& m = meta_table[i];
        switch (m.type) {
            case ParamType::FLOAT:
                std::fprintf(stdout, "  %-32s = %-12g  [%g, %g]\n",
                    m.name, float_values[i], m.min_val, m.max_val);
                break;
            case ParamType::BOOL:
                std::fprintf(stdout, "  %-32s = %s\n",
                    m.name, bool_values[i] ? "true" : "false");
                break;
            case ParamType::INT:
                std::fprintf(stdout, "  %-32s = %ld  [%ld, %ld]\n",
                    m.name, static_cast<long>(int_values[i]),
                    static_cast<long>(m.min_val),
                    static_cast<long>(m.max_val));
                break;
        }
    }
}

int count()
{
    return N_PARAMS;
}

const ParamEntry* entry(int index)
{
    // SIL provides ParamEntry view via a per-call static buffer.
    // The pointer is valid until the next entry() call. Callers that want
    // multiple entries simultaneously should copy the struct.
    // SIL では呼び出しごとの静的バッファで ParamEntry ビューを提供する。
    // ポインタは次の entry() 呼び出しまで有効。複数エントリを同時に扱う場合はコピーすること。
    static ParamEntry buf;
    if (index < 0 || index >= N_PARAMS) return nullptr;
    if (!initialized) init();
    const auto& m = meta_table[index];
    buf.name        = m.name;
    buf.type        = m.type;
    buf.default_val = m.default_val;
    buf.min_val     = m.min_val;
    buf.max_val     = m.max_val;
    buf.callback    = nullptr;
    switch (m.type) {
        case ParamType::FLOAT: buf.value_ptr = &float_values[index]; break;
        case ParamType::BOOL:  buf.value_ptr = &bool_values[index];  break;
        case ParamType::INT:   buf.value_ptr = &int_values[index];   break;
    }
    return &buf;
}

}  // namespace sf::params
