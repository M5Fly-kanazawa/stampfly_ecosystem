cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS ../../components)

# Restrict component discovery to "main" (and its transitive REQUIRES). See
# examples/09_topic_api_hello/CMakeLists.txt for why this is needed: without
# it, EVERY component under ../../components is added to the build and ALL
# of their REQUIRES get resolved, including unrelated ones this standalone
# project has no manifest for (verified pre-existing issue, also affects
# examples 01-08).
#
# main（とその実際の推移的 REQUIRES）だけに部品発見を絞る。理由は
# examples/09_topic_api_hello/CMakeLists.txt 参照: これが無いと ../../components
# 配下の全部品がビルドに加わり、その全 REQUIRES が解決対象になる（本サンプルに
# マニフェストの無い無関係な部品も含めて。examples 01-08 にも影響する既存の問題と
# 確認済み）。
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(10_custom_controller)
