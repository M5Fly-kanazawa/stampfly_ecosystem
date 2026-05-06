/**
 * @file sf_params_file.hpp
 * @brief SIL-only extension to sf::params — file-based override loader.
 *        SIL専用の sf::params 拡張 — ファイルからのオーバーライド読み込み。
 *
 * Phase 1.3 of the SIL→Real workflow (development_roadmap.md).
 * The firmware target uses NVS for persistence; on the host SIL we instead
 * read a `key=value` text file so that tuning experiments can drive the
 * same params API without recompiling.
 *
 * This declaration is intentionally NOT placed in the shared `params.hpp`
 * because firmware-side `params.cpp` does not provide `load_file()`.
 * Including it from a SIL-only header keeps the firmware build untouched.
 *
 * SIL→実機 ワークフロー Phase 1.3（development_roadmap.md）。
 * 実機側は NVS で永続化するが、ホスト SIL では `key=value` 形式の
 * テキストファイルを読むことで、再ビルドせずに同じ params API を
 * チューニング実験から駆動できるようにする。
 *
 * 共有の `params.hpp` には意図的に置かない。実機側 `params.cpp` は
 * `load_file()` を提供しないため、SIL 専用ヘッダにすることで実機ビルド
 * を変更せずに済む。
 *
 * @design development_roadmap.md §2 — Principle 2: Parameter Identity   [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include "params.hpp"

namespace sf {
namespace params {

/// Load parameter overrides from a key=value text file.
/// Defaults from params.def remain in effect for any parameter not
/// mentioned in the file.
///
/// File format:
///   - One `<param.name> = <value>` assignment per line
///   - Whitespace around `=` is ignored
///   - Floats: standard strtof (1.5, 1e-3, ...)
///   - Booleans: `true`/`false`/`1`/`0` (case-insensitive on the words)
///   - Integers: standard strtol
///   - Comment: a full line starting with `#`, OR `#` after the value
///     preceded by whitespace
///   - Blank lines are ignored
///
/// Lines that fail parsing, name lookup, type, or range validation are
/// reported to stderr and skipped — the rest of the file is still
/// processed.
///
/// `key=value` 形式のテキストファイルからパラメータをオーバーライド読み込みする。
/// ファイルに記述のないパラメータは params.def の defaults のままとなる。
///
/// @param path File path to read / 読み込むファイルパス
/// @return Number of parameters successfully overridden; -1 on file open
///         failure. オーバーライドに成功したパラメータ数。
///         ファイルオープン失敗時は -1。
int load_file(const char* path);

}  // namespace params
}  // namespace sf
