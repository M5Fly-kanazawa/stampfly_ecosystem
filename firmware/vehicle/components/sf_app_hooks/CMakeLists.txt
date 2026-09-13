# sf_app_hooks — L1 application entry point (Topic API), see app_hooks.hpp.
#
# SF_APP_DIR (a cache variable, e.g. `idf.py -D SF_APP_DIR=/abs/path build`)
# selects an application directory whose *.cpp provide sf::app::* instead of
# app_default.cpp. Those application sources are compiled straight into the
# main component (see ../../main/CMakeLists.txt), NOT into this component, so
# that they share the main component's REQUIRES without a component-level
# dependency edge back to it (a component cannot REQUIRES its own consumer).
# app_default.cpp is excluded in that case (the application's own *.cpp
# provide controller()/estimator()/start() instead) but stock_hooks.cpp is
# ALWAYS compiled here: it is the stock controller()/estimator() body that
# both app_default.cpp and an application's own app.cpp may delegate to
# (sf::app::stock::, see stock_hooks.hpp and examples/11_app_controller).
#
# sf_app_hooks — L1（Topic API）アプリの入口。詳細は app_hooks.hpp 参照。
#
# SF_APP_DIR（キャッシュ変数。例: `idf.py -D SF_APP_DIR=/abs/path build`）が
# 与えられたら、そのディレクトリの *.cpp が app_default.cpp の代わりに
# sf::app::* を実装する。それらアプリソースは本コンポーネントではなく main
# コンポーネントへ直接コンパイルする（../../main/CMakeLists.txt 参照）—
# main の REQUIRES をそのまま共有でき、コンポーネントが自分自身の利用者へ逆向きの
# 依存辺を持たずに済む。この場合 app_default.cpp は除外する（アプリ自身の
# *.cpp が controller()/estimator()/start() を提供するため）が、
# stock_hooks.cpp は常にここでコンパイルする — app_default.cpp とアプリ自身の
# app.cpp の両方が委譲できる標準の controller()/estimator() 本体
# （sf::app::stock::。stock_hooks.hpp と examples/11_app_controller 参照）。
if(SF_APP_DIR)
    idf_component_register(
        SRCS "stock_hooks.cpp"
        INCLUDE_DIRS "include"
        REQUIRES sf_controller sf_estimator
        PRIV_REQUIRES
            sf_controller_pid
            sf_estimator_eskf
            sf_estimator_complementary
            sf_core
    )
else()
    idf_component_register(
        SRCS "app_default.cpp" "stock_hooks.cpp"
        INCLUDE_DIRS "include"
        REQUIRES sf_controller sf_estimator
        PRIV_REQUIRES
            sf_controller_pid
            sf_estimator_eskf
            sf_estimator_complementary
            sf_core
    )
endif()
