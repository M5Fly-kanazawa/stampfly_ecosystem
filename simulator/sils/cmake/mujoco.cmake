# =============================================================================
# Fetch MuJoCo as a build dependency (NOT vendored). License: Apache-2.0.
# MuJoCo を依存として取得する（コピー同梱しない）。ライセンス: Apache-2.0。
#
# Per RESET_PLAN §6: acquire as a dependency and bundle the Apache-2.0 license
# text in THIRD_PARTY_LICENSES. The interactive viewer and examples are
# disabled so the headless core library builds without GLFW/OpenGL.
#
# RESET_PLAN §6 に従う: 依存として取得し、Apache-2.0 全文を
# THIRD_PARTY_LICENSES に同梱する。GUI ビューアと examples を無効化し、
# GLFW/OpenGL なしでヘッドレスのコアライブラリだけをビルドする。
# =============================================================================

include(FetchContent)

# Optionally build MuJoCo's interactive viewer (`simulate`) for eyeballing a
# model in a 3D window. OFF by default (headless core only); turn ON with
# -DSILS_MUJOCO_VIEWER=ON. It pulls GLFW, so it is opt-in.
# MuJoCo の対話ビューア（`simulate`）を任意でビルドする。3D ウィンドウで
# モデルを目視するため。既定 OFF（ヘッドレスのコアのみ）。-DSILS_MUJOCO_VIEWER=ON
# で有効化。GLFW を取得するのでオプトイン。
option(SILS_MUJOCO_VIEWER "Build the MuJoCo interactive viewer (simulate)" OFF)

set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE ${SILS_MUJOCO_VIEWER} CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_PYTHON   OFF CACHE BOOL "" FORCE)

# MinGW note (real stdio calls): mingw-w64's OWN printf/scanf-family
# declarations default to the legacy MSVCRT format archetype, which does not
# understand C99 length modifiers like %zu/%td at RUNTIME (it would print
# garbage, not just warn). __USE_MINGW_ANSI_STDIO=1 switches mingw-w64's
# headers to the C99-compatible archetype for calls to the ACTUAL CRT
# functions. MSVC is unaffected (no MinGW printf shim there), so this is
# MinGW-only.
# MinGW 注記（実際の stdio 呼び出し向け）: mingw-w64 自身の printf/scanf 系宣言は
# 既定でレガシー MSVCRT 書式アーキタイプを使うため、%zu/%td 等の C99 長さ修飾子を
# 実行時に理解しない（警告どころか誤動作する）。__USE_MINGW_ANSI_STDIO=1 で
# 実 CRT 関数呼び出しを C99 互換アーキタイプに切り替える。MSVC には mingw printf
# シムが無いため影響しない（MinGW 限定）。
if(MINGW)
  add_compile_definitions(__USE_MINGW_ANSI_STDIO=1)
endif()

FetchContent_Declare(
  mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
  GIT_TAG 3.9.0
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(mujoco)

# MinGW note (MuJoCo's OWN mju_error/mju_warning %zu use): unlike the real CRT
# printf/scanf above, GCC's mingw target ALWAYS maps a user
# __attribute__((format(printf, ...))) to the "ms_printf" archetype —
# __USE_MINGW_ANSI_STDIO has no effect on that mapping (verified empirically:
# it only reconfigures mingw-w64's OWN stdio.h declarations, not the generic
# "printf" attribute name GCC resolves for third-party code). MuJoCo's
# mjPRINTFLIKE macro (include/mujoco/mjmacro.h) expands to exactly that
# attribute, so its own internal mju_error(..., "%zu", ...) call sites
# (src/user/user_vfs.cc, src/thread/thread_pool.cc) fail -Werror=format under
# MinGW even though the calls are correct C99/C++11. Also on Windows,
# src/xml/mjz/mjz_encoder.cc passes a std::filesystem::path's native
# (wchar_t*) storage to a %s spec — a genuine MuJoCo Windows-path bug, not a
# MinGW-vs-MSVC quirk (MSVC's -Wformat is far weaker so it likely does not
# catch it there). Neither is exercised by this SILS bench: the affected
# functions are unreachable from the .xml model path we drive MuJoCo through
# (the mjz/.mjb compressed-model loader is untouched). Silencing -Wformat for
# the vendored `mujoco` target only (not our own code) unblocks the build
# without patching fetched upstream source.
# MinGW 注記（MuJoCo 自身の mju_error/mju_warning の %zu 使用）: 上記の実 CRT
# printf/scanf と異なり、GCC の mingw ターゲットはユーザー定義の
# __attribute__((format(printf, ...))) を常に "ms_printf" アーキタイプに
# マップする — __USE_MINGW_ANSI_STDIO はこのマッピングに効果がない（実証済み:
# mingw-w64 自身の stdio.h 宣言だけを再設定し、サードパーティコード向けの汎用
# "printf" 属性名の解決には影響しない）。MuJoCo の mjPRINTFLIKE マクロ
# （include/mujoco/mjmacro.h）はまさにこの属性に展開されるため、内部の
# mju_error(..., "%zu", ...) 呼び出し箇所（src/user/user_vfs.cc,
# src/thread/thread_pool.cc）が MinGW では -Werror=format で失敗する（呼び出し
# 自体は正しい C99/C++11）。また Windows では src/xml/mjz/mjz_encoder.cc が
# std::filesystem::path のネイティブ表現（wchar_t*）を %s 指定に渡しており、
# これは MinGW 対 MSVC の差異ではなく MuJoCo 自身の Windows パス処理バグ
# （MSVC の -Wformat は弱く検出しない可能性が高い）。どちらも本 SILS ベンチでは
# 到達しない（.xml モデル経路からは mjz/.mjb 圧縮モデルローダに触れない）。
# 取得元コードにパッチを当てず、ベンダ内 `mujoco` ターゲットに限定して
# -Wformat を抑制することでビルドを通す（自前コードには適用しない）。
if(MINGW AND TARGET mujoco)
  target_compile_options(mujoco PRIVATE -Wno-format)
endif()
