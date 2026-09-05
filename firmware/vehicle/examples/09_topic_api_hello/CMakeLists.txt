cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS ../../components)

# Restrict component discovery to "main" (and its transitive REQUIRES).
# Without this, EVERY component under ../../components is added to the build
# candidate set and ALL of their REQUIRES get resolved — including unrelated
# ones like sf_hal_led -> led_strip (an IDF Component Manager package this
# standalone example has no manifest for). This is a pre-existing issue that
# also affects examples 01-08 (verified: idf.py set-target fails there too
# with "Failed to resolve component 'led_strip'"); this example works around
# it locally rather than touching the shared examples.
#
# main（とその実際の推移的 REQUIRES）だけに部品発見を絞る。これが無いと
# ../../components 配下の全部品がビルド候補に入り、その全 REQUIRES が解決
# 対象になる — 無関係な sf_hal_led -> led_strip（本サンプルにマニフェストの無い
# IDF Component Manager パッケージ）も含めて。これは examples 01-08 にも
# 影響する既存の問題（idf.py set-target が同じエラーで失敗することを確認済み）
# だが、共有の examples には手を入れず本サンプル側で局所的に回避する。
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(09_topic_api_hello)
