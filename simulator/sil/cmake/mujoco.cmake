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
# -DSIL_MUJOCO_VIEWER=ON. It pulls GLFW, so it is opt-in.
# MuJoCo の対話ビューア（`simulate`）を任意でビルドする。3D ウィンドウで
# モデルを目視するため。既定 OFF（ヘッドレスのコアのみ）。-DSIL_MUJOCO_VIEWER=ON
# で有効化。GLFW を取得するのでオプトイン。
option(SIL_MUJOCO_VIEWER "Build the MuJoCo interactive viewer (simulate)" OFF)

set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE ${SIL_MUJOCO_VIEWER} CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_PYTHON   OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
  GIT_TAG 3.9.0
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(mujoco)
